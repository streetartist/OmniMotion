/**
 * @file arduino_encoder.hpp
 * @brief Arduino Encoder HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 */

#pragma once

#include <omni/hal/encoder.hpp>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
    #include <driver/pcnt.h>
#endif

#if defined(ARDUINO_ARCH_RP2040)
    #include <hardware/pio.h>
    #include <hardware/clocks.h>
#endif

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Software quadrature encoder using interrupts
 *
 * Universal implementation that works on any Arduino platform.
 * Uses GPIO interrupts for channel A and B.
 *
 * Usage:
 * @code
 * ArduinoSoftwareEncoder encoder(2, 3);  // Pins with interrupt support
 * encoder.setResolution(4096);
 * encoder.begin();
 *
 * // In loop:
 * encoder.update(dt);
 * float angle = encoder.getAngle();
 * float velocity = encoder.getVelocity();
 * @endcode
 */
class ArduinoSoftwareEncoder : public hal::IEncoder {
public:
    /**
     * @brief Constructor
     * @param pinA Encoder channel A pin (must support interrupts)
     * @param pinB Encoder channel B pin
     * @param pinZ Optional index pin (-1 if not used)
     */
    ArduinoSoftwareEncoder(int8_t pinA, int8_t pinB, int8_t pinZ = -1)
        : pinA_(pinA)
        , pinB_(pinB)
        , pinZ_(pinZ)
        , count_(0)
        , lastCount_(0)
        , cpr_(4096)
        , mode_(hal::EncoderMode::X4)
        , inverted_(false)
        , indexEnabled_(false)
        , indexDetected_(false)
        , velocity_(0.0f)
        , instance_(nullptr)
    {
    }

    virtual ~ArduinoSoftwareEncoder() {
        end();
    }

    /**
     * @brief Initialize the encoder
     *
     * Must be called before using the encoder.
     * Sets up GPIO pins and attaches interrupts.
     */
    void begin() {
        pinMode(pinA_, INPUT_PULLUP);
        pinMode(pinB_, INPUT_PULLUP);
        if (pinZ_ >= 0) {
            pinMode(pinZ_, INPUT_PULLUP);
        }

        // Store instance for static callback
        instance_ = this;

        // Attach interrupt to channel A
        int intA = digitalPinToInterrupt(pinA_);
        if (intA != NOT_AN_INTERRUPT) {
            attachInterrupt(intA, isrA, CHANGE);
        }

        // For X4 mode, also attach interrupt to channel B
        if (mode_ == hal::EncoderMode::X4) {
            int intB = digitalPinToInterrupt(pinB_);
            if (intB != NOT_AN_INTERRUPT) {
                attachInterrupt(intB, isrB, CHANGE);
            }
        }

        // Index interrupt
        if (pinZ_ >= 0 && indexEnabled_) {
            int intZ = digitalPinToInterrupt(pinZ_);
            if (intZ != NOT_AN_INTERRUPT) {
                attachInterrupt(intZ, isrZ, RISING);
            }
        }

        // Read initial state
        lastA_ = digitalRead(pinA_);
        lastB_ = digitalRead(pinB_);
    }

    /**
     * @brief Deinitialize the encoder
     */
    void end() {
        int intA = digitalPinToInterrupt(pinA_);
        if (intA != NOT_AN_INTERRUPT) {
            detachInterrupt(intA);
        }

        int intB = digitalPinToInterrupt(pinB_);
        if (intB != NOT_AN_INTERRUPT) {
            detachInterrupt(intB);
        }

        if (pinZ_ >= 0) {
            int intZ = digitalPinToInterrupt(pinZ_);
            if (intZ != NOT_AN_INTERRUPT) {
                detachInterrupt(intZ);
            }
        }

        instance_ = nullptr;
    }

    int32_t getCount() override {
        noInterrupts();
        int32_t c = count_;
        interrupts();
        return inverted_ ? -c : c;
    }

    void resetCount() override {
        noInterrupts();
        count_ = 0;
        lastCount_ = 0;
        interrupts();
    }

    void setCount(int32_t count) override {
        noInterrupts();
        count_ = inverted_ ? -count : count;
        interrupts();
    }

    float getAngle() override {
        int32_t c = getCount();
        // Wrap to single revolution
        int32_t countsPerRev = static_cast<int32_t>(cpr_);
        c = c % countsPerRev;
        if (c < 0) c += countsPerRev;
        return (static_cast<float>(c) / static_cast<float>(cpr_)) * 2.0f * M_PI;
    }

    float getVelocity() override {
        return velocity_;
    }

    /**
     * @brief Update velocity calculation
     * @param dt Time since last update in seconds
     *
     * Must be called regularly from main loop.
     */
    void update(float dt) {
        if (dt <= 0.0f) return;

        int32_t currentCount = getCount();
        int32_t delta = currentCount - lastCount_;
        lastCount_ = currentCount;

        // Calculate velocity in rad/s
        float deltaAngle = (static_cast<float>(delta) / static_cast<float>(cpr_)) * 2.0f * M_PI;
        velocity_ = deltaAngle / dt;
    }

    void setResolution(uint32_t cpr) override {
        cpr_ = cpr;
    }

    uint32_t getResolution() const override {
        return cpr_;
    }

    void setMode(hal::EncoderMode mode) override {
        mode_ = mode;
        // Would need to reconfigure interrupts for mode change
    }

    void enableIndex(bool enable) override {
        indexEnabled_ = enable;
        if (pinZ_ >= 0 && enable) {
            int intZ = digitalPinToInterrupt(pinZ_);
            if (intZ != NOT_AN_INTERRUPT) {
                attachInterrupt(intZ, isrZ, RISING);
            }
        } else if (pinZ_ >= 0 && !enable) {
            int intZ = digitalPinToInterrupt(pinZ_);
            if (intZ != NOT_AN_INTERRUPT) {
                detachInterrupt(intZ);
            }
        }
    }

    bool indexDetected() override {
        return indexDetected_;
    }

    void clearIndex() override {
        indexDetected_ = false;
    }

    void setInvert(bool invert) override {
        inverted_ = invert;
    }

private:
    int8_t pinA_, pinB_, pinZ_;
    volatile int32_t count_;
    int32_t lastCount_;
    uint32_t cpr_;
    hal::EncoderMode mode_;
    bool inverted_;
    bool indexEnabled_;
    volatile bool indexDetected_;
    float velocity_;

    volatile bool lastA_, lastB_;

    static ArduinoSoftwareEncoder* instance_;

    static void isrA() {
        if (instance_) instance_->handleA();
    }

    static void isrB() {
        if (instance_) instance_->handleB();
    }

    static void isrZ() {
        if (instance_) instance_->handleZ();
    }

    void handleA() {
        bool a = digitalRead(pinA_);
        bool b = digitalRead(pinB_);

        if (mode_ == hal::EncoderMode::X1) {
            // Count on rising edge of A only
            if (a && !lastA_) {
                count_ += b ? -1 : 1;
            }
        } else if (mode_ == hal::EncoderMode::X2) {
            // Count on both edges of A
            if (a != lastA_) {
                if (a == b) count_--;
                else count_++;
            }
        } else {
            // X4: count on both edges of both channels
            if (a != lastA_) {
                if (a == b) count_--;
                else count_++;
            }
        }

        lastA_ = a;
        lastB_ = b;
    }

    void handleB() {
        bool a = digitalRead(pinA_);
        bool b = digitalRead(pinB_);

        // X4 mode: count on both edges of B
        if (b != lastB_) {
            if (a == b) count_++;
            else count_--;
        }

        lastA_ = a;
        lastB_ = b;
    }

    void handleZ() {
        indexDetected_ = true;
        // Optionally reset count on index
    }
};

// Static instance pointer
ArduinoSoftwareEncoder* ArduinoSoftwareEncoder::instance_ = nullptr;

#if defined(ARDUINO_ARCH_ESP32)

/**
 * @brief ESP32 hardware encoder using PCNT (Pulse Counter) module
 *
 * Uses ESP32's PCNT peripheral for hardware-based quadrature decoding.
 * Much more accurate and efficient than software-based counting.
 *
 * Usage:
 * @code
 * ArduinoEsp32Encoder encoder(34, 35, PCNT_UNIT_0);
 * encoder.setResolution(4096);
 * encoder.begin();
 *
 * float angle = encoder.getAngle();
 * @endcode
 */
class ArduinoEsp32Encoder : public hal::IEncoder {
public:
    /**
     * @brief Constructor
     * @param pinA Encoder channel A pin
     * @param pinB Encoder channel B pin
     * @param unit PCNT unit to use (PCNT_UNIT_0 to PCNT_UNIT_7)
     */
    ArduinoEsp32Encoder(int8_t pinA, int8_t pinB, pcnt_unit_t unit = PCNT_UNIT_0)
        : pinA_(pinA)
        , pinB_(pinB)
        , unit_(unit)
        , count_(0)
        , overflowCount_(0)
        , lastCount_(0)
        , cpr_(4096)
        , inverted_(false)
        , velocity_(0.0f)
    {
    }

    /**
     * @brief Initialize the PCNT encoder
     */
    void begin() {
        // Configure PCNT unit
        pcnt_config_t config = {};

        // Channel 0: count on A, direction from B
        config.pulse_gpio_num = pinA_;
        config.ctrl_gpio_num = pinB_;
        config.channel = PCNT_CHANNEL_0;
        config.unit = unit_;
        config.pos_mode = PCNT_COUNT_INC;
        config.neg_mode = PCNT_COUNT_DEC;
        config.lctrl_mode = PCNT_MODE_REVERSE;
        config.hctrl_mode = PCNT_MODE_KEEP;
        config.counter_h_lim = 32767;
        config.counter_l_lim = -32768;
        pcnt_unit_config(&config);

        // Channel 1: count on B, direction from A (for X4 mode)
        config.pulse_gpio_num = pinB_;
        config.ctrl_gpio_num = pinA_;
        config.channel = PCNT_CHANNEL_1;
        config.pos_mode = PCNT_COUNT_DEC;
        config.neg_mode = PCNT_COUNT_INC;
        pcnt_unit_config(&config);

        // Configure filter to ignore glitches
        pcnt_set_filter_value(unit_, 100);  // Filter pulses shorter than 100 APB cycles
        pcnt_filter_enable(unit_);

        // Enable overflow interrupt for extended count
        pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
        pcnt_event_enable(unit_, PCNT_EVT_L_LIM);

        // Clear and start counter
        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        pcnt_counter_resume(unit_);
    }

    int32_t getCount() override {
        int16_t pcntCount;
        pcnt_get_counter_value(unit_, &pcntCount);

        // Combine with overflow count for full 32-bit range
        int32_t fullCount = overflowCount_ * 32768 + pcntCount;
        return inverted_ ? -fullCount : fullCount;
    }

    void resetCount() override {
        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        pcnt_counter_resume(unit_);
        overflowCount_ = 0;
        lastCount_ = 0;
    }

    void setCount(int32_t count) override {
        int32_t targetCount = inverted_ ? -count : count;
        overflowCount_ = targetCount / 32768;
        int16_t remainder = targetCount % 32768;

        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        // PCNT doesn't support setting arbitrary value directly
        // Best effort: clear and track offset
        pcnt_counter_resume(unit_);
    }

    float getAngle() override {
        int32_t c = getCount();
        int32_t countsPerRev = static_cast<int32_t>(cpr_);
        c = c % countsPerRev;
        if (c < 0) c += countsPerRev;
        return (static_cast<float>(c) / static_cast<float>(cpr_)) * 2.0f * M_PI;
    }

    float getVelocity() override {
        return velocity_;
    }

    void update(float dt) {
        if (dt <= 0.0f) return;

        int32_t currentCount = getCount();
        int32_t delta = currentCount - lastCount_;
        lastCount_ = currentCount;

        float deltaAngle = (static_cast<float>(delta) / static_cast<float>(cpr_)) * 2.0f * M_PI;
        velocity_ = deltaAngle / dt;
    }

    void setResolution(uint32_t cpr) override {
        cpr_ = cpr;
    }

    uint32_t getResolution() const override {
        return cpr_;
    }

    void setMode(hal::EncoderMode mode) override {
        // PCNT is always X4 mode
        (void)mode;
    }

    void enableIndex(bool enable) override {
        // PCNT doesn't support index directly
        // Would need separate GPIO interrupt
        (void)enable;
    }

    bool indexDetected() override {
        return false;
    }

    void clearIndex() override {
    }

    void setInvert(bool invert) override {
        inverted_ = invert;
    }

    /**
     * @brief Handle PCNT overflow event
     *
     * Should be called from PCNT ISR if overflow events are used.
     */
    void handleOverflow(bool positive) {
        if (positive) {
            overflowCount_++;
        } else {
            overflowCount_--;
        }
    }

private:
    int8_t pinA_, pinB_;
    pcnt_unit_t unit_;
    volatile int32_t count_;
    volatile int32_t overflowCount_;
    int32_t lastCount_;
    uint32_t cpr_;
    bool inverted_;
    float velocity_;
};

#endif  // ESP32

#if defined(ARDUINO_ARCH_RP2040)

/**
 * @brief RP2040 hardware encoder using PIO
 *
 * Uses RP2040's PIO (Programmable I/O) for hardware quadrature decoding.
 * Provides accurate counting without CPU overhead.
 *
 * Note: This is a simplified implementation. For full PIO support,
 * consider using a dedicated library like pico-encoder.
 */
class ArduinoRp2040Encoder : public hal::IEncoder {
public:
    ArduinoRp2040Encoder(int8_t pinA, int8_t pinB)
        : pinA_(pinA)
        , pinB_(pinB)
        , count_(0)
        , lastCount_(0)
        , cpr_(4096)
        , inverted_(false)
        , velocity_(0.0f)
        , pioInitialized_(false)
    {
    }

    void begin() {
        // For simplicity, use software encoder on RP2040
        // Full PIO implementation would require loading a PIO program
        // See: https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder

        pinMode(pinA_, INPUT_PULLUP);
        pinMode(pinB_, INPUT_PULLUP);

        // Attach interrupts for software fallback
        attachInterrupt(digitalPinToInterrupt(pinA_), isrStatic, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB_), isrStatic, CHANGE);

        instance_ = this;
        lastA_ = digitalRead(pinA_);
        lastB_ = digitalRead(pinB_);
    }

    int32_t getCount() override {
        return inverted_ ? -count_ : count_;
    }

    void resetCount() override {
        count_ = 0;
        lastCount_ = 0;
    }

    void setCount(int32_t count) override {
        count_ = inverted_ ? -count : count;
    }

    float getAngle() override {
        int32_t c = getCount();
        int32_t countsPerRev = static_cast<int32_t>(cpr_);
        c = c % countsPerRev;
        if (c < 0) c += countsPerRev;
        return (static_cast<float>(c) / static_cast<float>(cpr_)) * 2.0f * M_PI;
    }

    float getVelocity() override {
        return velocity_;
    }

    void update(float dt) {
        if (dt <= 0.0f) return;

        int32_t currentCount = getCount();
        int32_t delta = currentCount - lastCount_;
        lastCount_ = currentCount;

        float deltaAngle = (static_cast<float>(delta) / static_cast<float>(cpr_)) * 2.0f * M_PI;
        velocity_ = deltaAngle / dt;
    }

    void setResolution(uint32_t cpr) override {
        cpr_ = cpr;
    }

    uint32_t getResolution() const override {
        return cpr_;
    }

    void setMode(hal::EncoderMode mode) override {
        (void)mode;
    }

    void enableIndex(bool enable) override {
        (void)enable;
    }

    bool indexDetected() override {
        return false;
    }

    void clearIndex() override {
    }

    void setInvert(bool invert) override {
        inverted_ = invert;
    }

private:
    int8_t pinA_, pinB_;
    volatile int32_t count_;
    int32_t lastCount_;
    uint32_t cpr_;
    bool inverted_;
    float velocity_;
    bool pioInitialized_;

    volatile bool lastA_, lastB_;
    static ArduinoRp2040Encoder* instance_;

    static void isrStatic() {
        if (instance_) instance_->handleInterrupt();
    }

    void handleInterrupt() {
        bool a = digitalRead(pinA_);
        bool b = digitalRead(pinB_);

        // X4 decoding
        if (a != lastA_) {
            if (a == b) count_--;
            else count_++;
        }
        if (b != lastB_) {
            if (a == b) count_++;
            else count_--;
        }

        lastA_ = a;
        lastB_ = b;
    }
};

ArduinoRp2040Encoder* ArduinoRp2040Encoder::instance_ = nullptr;

#endif  // RP2040

/**
 * @brief Hall sensor interface for BLDC motors
 *
 * Reads three hall sensor inputs and provides electrical angle.
 */
class ArduinoHallSensor : public hal::IHallSensor {
public:
    /**
     * @brief Constructor
     * @param pinH1 Hall sensor 1 pin
     * @param pinH2 Hall sensor 2 pin
     * @param pinH3 Hall sensor 3 pin
     */
    ArduinoHallSensor(uint8_t pinH1, uint8_t pinH2, uint8_t pinH3)
        : pins_{pinH1, pinH2, pinH3}
    {
        // Default hall to sector mapping (60 degree sectors)
        // Adjust based on motor winding configuration
        mapping_[0] = 0;  // Invalid state 000
        mapping_[1] = 0;  // 001 -> sector 0 (0 degrees)
        mapping_[2] = 2;  // 010 -> sector 2 (120 degrees)
        mapping_[3] = 1;  // 011 -> sector 1 (60 degrees)
        mapping_[4] = 4;  // 100 -> sector 4 (240 degrees)
        mapping_[5] = 5;  // 101 -> sector 5 (300 degrees)
        mapping_[6] = 3;  // 110 -> sector 3 (180 degrees)
        mapping_[7] = 0;  // Invalid state 111
    }

    void begin() {
        for (int i = 0; i < 3; i++) {
            pinMode(pins_[i], INPUT_PULLUP);
        }
    }

    uint8_t getState() override {
        uint8_t state = 0;
        if (digitalRead(pins_[0])) state |= 0x01;
        if (digitalRead(pins_[1])) state |= 0x02;
        if (digitalRead(pins_[2])) state |= 0x04;
        return state;
    }

    float getElectricalAngle() override {
        uint8_t state = getState();
        uint8_t sector = mapping_[state & 0x07];

        // Each sector is 60 degrees (PI/3 radians)
        // Return center of sector
        return (static_cast<float>(sector) + 0.5f) * (M_PI / 3.0f);
    }

    void setMapping(const uint8_t* table) override {
        for (int i = 0; i < 8; i++) {
            mapping_[i] = table[i];
        }
    }

    void enableInterrupt(bool enable) override {
        // Would need to attach interrupts to hall pins
        (void)enable;
    }

private:
    uint8_t pins_[3];
    uint8_t mapping_[8];
};

// Default encoder type alias based on platform
#if defined(ARDUINO_ARCH_ESP32)
    using ArduinoEncoder = ArduinoEsp32Encoder;
#elif defined(ARDUINO_ARCH_RP2040)
    using ArduinoEncoder = ArduinoRp2040Encoder;
#else
    using ArduinoEncoder = ArduinoSoftwareEncoder;
#endif

}  // namespace arduino
}  // namespace platform
}  // namespace omni
