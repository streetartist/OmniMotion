/**
 * @file arduino_gpio.hpp
 * @brief Arduino GPIO HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 */

#pragma once

#include <omni/hal/gpio.hpp>
#include <Arduino.h>

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Arduino GPIO implementation
 *
 * Uses standard Arduino API: pinMode(), digitalWrite(), digitalRead()
 *
 * Platform differences:
 * - AVR: No PullDown support, OpenDrain requires software emulation
 * - ESP32: Full support for all modes
 * - RP2040: Full support for all modes
 *
 * Usage:
 * @code
 * ArduinoGpio led(13);
 * led.setMode(PinMode::Output);
 * led.write(true);
 * led.toggle();
 * @endcode
 */
class ArduinoGpio : public hal::IGpio {
public:
    /**
     * @brief Constructor
     * @param pin Arduino pin number
     */
    explicit ArduinoGpio(uint8_t pin)
        : pin_(pin)
        , mode_(hal::PinMode::Input)
        , lastValue_(false)
        , openDrainEmulation_(false)
    {
    }

    void setMode(hal::PinMode mode) override {
        mode_ = mode;
        openDrainEmulation_ = false;

        switch (mode) {
            case hal::PinMode::Input:
                pinMode(pin_, INPUT);
                break;

            case hal::PinMode::Output:
                pinMode(pin_, OUTPUT);
                break;

            case hal::PinMode::InputPullUp:
                pinMode(pin_, INPUT_PULLUP);
                break;

            case hal::PinMode::InputPullDown:
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
                pinMode(pin_, INPUT_PULLDOWN);
#else
                // AVR doesn't support pull-down, fall back to regular input
                pinMode(pin_, INPUT);
#endif
                break;

            case hal::PinMode::OutputOpenDrain:
#if defined(ARDUINO_ARCH_ESP32)
                pinMode(pin_, OUTPUT_OPEN_DRAIN);
#elif defined(ARDUINO_ARCH_RP2040)
                // RP2040 doesn't have direct open-drain, emulate it
                openDrainEmulation_ = true;
                pinMode(pin_, INPUT);  // Start in high-Z state
#else
                // AVR: software emulate open-drain
                openDrainEmulation_ = true;
                pinMode(pin_, INPUT);  // Start in high-Z state
#endif
                break;

            case hal::PinMode::Alternate:
                // Alternate function is typically set by peripheral libraries
                break;
        }
    }

    hal::PinMode getMode() const override {
        return mode_;
    }

    void write(bool value) override {
        lastValue_ = value;

        if (openDrainEmulation_) {
            // Open-drain emulation: low = drive low, high = high-Z (input)
            if (value) {
                pinMode(pin_, INPUT);  // High-Z for high
            } else {
                pinMode(pin_, OUTPUT);
                digitalWrite(pin_, LOW);
            }
        } else {
            digitalWrite(pin_, value ? HIGH : LOW);
        }
    }

    bool read() override {
        return digitalRead(pin_) == HIGH;
    }

    void toggle() override {
        write(!lastValue_);
    }

    void enableInterrupt(bool risingEdge, bool fallingEdge) override {
        int mode;
        if (risingEdge && fallingEdge) {
            mode = CHANGE;
        } else if (risingEdge) {
            mode = RISING;
        } else if (fallingEdge) {
            mode = FALLING;
        } else {
            return;  // No edge selected
        }

        // Note: Actual callback must be set using setInterruptCallback
        // Arduino's attachInterrupt requires a function pointer
        int intNum = digitalPinToInterrupt(pin_);
        if (intNum != NOT_AN_INTERRUPT) {
            // We store the mode for later use with setInterruptCallback
            interruptMode_ = mode;
        }
    }

    void disableInterrupt() override {
        int intNum = digitalPinToInterrupt(pin_);
        if (intNum != NOT_AN_INTERRUPT) {
            detachInterrupt(intNum);
        }
    }

    /**
     * @brief Get the pin number
     * @return Arduino pin number
     */
    uint8_t getPin() const { return pin_; }

protected:
    uint8_t pin_;
    hal::PinMode mode_;
    bool lastValue_;
    bool openDrainEmulation_;
    int interruptMode_ = CHANGE;
};

/**
 * @brief Arduino GPIO with interrupt support
 *
 * Provides interrupt callback functionality using Arduino's attachInterrupt.
 *
 * Usage:
 * @code
 * void onButtonPress() {
 *     // Handle interrupt
 * }
 *
 * ArduinoGpioInterrupt button(2);
 * button.setMode(PinMode::InputPullUp);
 * button.attachCallback(onButtonPress, FALLING);
 * @endcode
 */
class ArduinoGpioInterrupt : public ArduinoGpio {
public:
    using Callback = void(*)();

    explicit ArduinoGpioInterrupt(uint8_t pin)
        : ArduinoGpio(pin)
        , callback_(nullptr)
    {
    }

    /**
     * @brief Attach interrupt callback
     * @param callback Function to call on interrupt
     * @param mode Interrupt mode (RISING, FALLING, CHANGE, LOW, HIGH)
     * @return true if successful
     */
    bool attachCallback(Callback callback, int mode) {
        int intNum = digitalPinToInterrupt(pin_);
        if (intNum == NOT_AN_INTERRUPT) {
            return false;
        }

        callback_ = callback;
        attachInterrupt(intNum, callback, mode);
        return true;
    }

    /**
     * @brief Detach interrupt callback
     */
    void detachCallback() {
        int intNum = digitalPinToInterrupt(pin_);
        if (intNum != NOT_AN_INTERRUPT) {
            detachInterrupt(intNum);
        }
        callback_ = nullptr;
    }

    void enableInterrupt(bool risingEdge, bool fallingEdge) override {
        if (callback_ == nullptr) {
            return;  // No callback set
        }

        int mode;
        if (risingEdge && fallingEdge) {
            mode = CHANGE;
        } else if (risingEdge) {
            mode = RISING;
        } else if (fallingEdge) {
            mode = FALLING;
        } else {
            return;
        }

        int intNum = digitalPinToInterrupt(pin_);
        if (intNum != NOT_AN_INTERRUPT) {
            attachInterrupt(intNum, callback_, mode);
        }
    }

    void disableInterrupt() override {
        ArduinoGpio::disableInterrupt();
    }

private:
    Callback callback_;
};

/**
 * @brief GPIO interrupt manager for context-based callbacks
 *
 * Allows using member functions or lambdas with context as interrupt handlers.
 * Limited to a fixed number of interrupt slots.
 */
class GpioInterruptManager {
public:
    static constexpr size_t MAX_INTERRUPTS = 8;

    using ContextCallback = void(*)(void* context);

    struct InterruptSlot {
        uint8_t pin;
        ContextCallback callback;
        void* context;
        bool active;
    };

    static GpioInterruptManager& instance() {
        static GpioInterruptManager inst;
        return inst;
    }

    /**
     * @brief Register an interrupt with context
     * @param pin Arduino pin number
     * @param mode Interrupt mode
     * @param callback Callback function
     * @param context User context pointer
     * @return Slot index, or -1 if no slots available
     */
    int registerInterrupt(uint8_t pin, int mode, ContextCallback callback, void* context) {
        int intNum = digitalPinToInterrupt(pin);
        if (intNum == NOT_AN_INTERRUPT) {
            return -1;
        }

        // Find free slot
        for (size_t i = 0; i < MAX_INTERRUPTS; i++) {
            if (!slots_[i].active) {
                slots_[i].pin = pin;
                slots_[i].callback = callback;
                slots_[i].context = context;
                slots_[i].active = true;

                // Attach the dispatcher for this slot
                attachInterrupt(intNum, getDispatcher(i), mode);
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    /**
     * @brief Unregister an interrupt
     * @param slotIndex Slot index returned by registerInterrupt
     */
    void unregisterInterrupt(int slotIndex) {
        if (slotIndex >= 0 && slotIndex < static_cast<int>(MAX_INTERRUPTS)) {
            if (slots_[slotIndex].active) {
                int intNum = digitalPinToInterrupt(slots_[slotIndex].pin);
                if (intNum != NOT_AN_INTERRUPT) {
                    detachInterrupt(intNum);
                }
                slots_[slotIndex].active = false;
            }
        }
    }

private:
    GpioInterruptManager() {
        for (size_t i = 0; i < MAX_INTERRUPTS; i++) {
            slots_[i].active = false;
        }
    }

    InterruptSlot slots_[MAX_INTERRUPTS];

    // Dispatcher functions for each slot
    static void dispatch0() { dispatchSlot(0); }
    static void dispatch1() { dispatchSlot(1); }
    static void dispatch2() { dispatchSlot(2); }
    static void dispatch3() { dispatchSlot(3); }
    static void dispatch4() { dispatchSlot(4); }
    static void dispatch5() { dispatchSlot(5); }
    static void dispatch6() { dispatchSlot(6); }
    static void dispatch7() { dispatchSlot(7); }

    static void dispatchSlot(size_t slot) {
        auto& mgr = instance();
        if (mgr.slots_[slot].active && mgr.slots_[slot].callback) {
            mgr.slots_[slot].callback(mgr.slots_[slot].context);
        }
    }

    using DispatchFunc = void(*)();

    static DispatchFunc getDispatcher(size_t slot) {
        static const DispatchFunc dispatchers[MAX_INTERRUPTS] = {
            dispatch0, dispatch1, dispatch2, dispatch3,
            dispatch4, dispatch5, dispatch6, dispatch7
        };
        return dispatchers[slot];
    }
};

}  // namespace arduino
}  // namespace platform
}  // namespace omni
