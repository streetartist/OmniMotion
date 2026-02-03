/**
 * @file arduino_timer.hpp
 * @brief Arduino Timer HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 */

#pragma once

#include <omni/hal/timer.hpp>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
    #include <esp_timer.h>
#elif defined(ARDUINO_ARCH_RP2040)
    #include <hardware/timer.h>
    #include <pico/time.h>
#endif

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Arduino timestamp implementation
 *
 * Provides high-resolution timestamps using platform-specific APIs.
 *
 * Usage:
 * @code
 * ArduinoTimestamp ts;
 * uint64_t start = ts.getMicros();
 * // ... do work ...
 * uint64_t elapsed = ts.elapsedMicros(start);
 * @endcode
 */
class ArduinoTimestamp : public hal::ITimestamp {
public:
    ArduinoTimestamp() = default;

    uint64_t getMicros() const override {
#if defined(ARDUINO_ARCH_ESP32)
        return esp_timer_get_time();
#elif defined(ARDUINO_ARCH_RP2040)
        return time_us_64();
#else
        // Standard Arduino micros() - wraps at ~70 minutes
        return micros();
#endif
    }

    uint64_t getNanos() const override {
#if defined(ARDUINO_ARCH_ESP32)
        return esp_timer_get_time() * 1000ULL;
#elif defined(ARDUINO_ARCH_RP2040)
        return time_us_64() * 1000ULL;
#else
        // Arduino doesn't have nanosecond resolution
        return static_cast<uint64_t>(micros()) * 1000ULL;
#endif
    }

    uint64_t elapsedMicros(uint64_t startMicros) const override {
        uint64_t now = getMicros();
        // Handle wraparound for 32-bit micros()
        if (now >= startMicros) {
            return now - startMicros;
        } else {
            // Wraparound occurred
            return (0xFFFFFFFFULL - startMicros) + now + 1;
        }
    }
};

#if defined(ARDUINO_ARCH_ESP32)

/**
 * @brief ESP32 hardware timer implementation
 *
 * Uses ESP32's esp_timer API for precise timing with callbacks.
 *
 * Usage:
 * @code
 * ArduinoEsp32Timer timer;
 * timer.setPeriod(1000);  // 1ms period
 * timer.setCallback([]() { // handle timer }; );
 * timer.enableInterrupt(true);
 * timer.start();
 * @endcode
 */
class ArduinoEsp32Timer : public hal::ITimer {
public:
    ArduinoEsp32Timer()
        : timerHandle_(nullptr)
        , periodUs_(1000)
        , running_(false)
        , oneShot_(false)
        , callback_(nullptr)
    {
    }

    ~ArduinoEsp32Timer() {
        if (timerHandle_) {
            stop();
            esp_timer_delete(timerHandle_);
            timerHandle_ = nullptr;
        }
    }

    void setPeriod(uint32_t periodUs) override {
        periodUs_ = periodUs;

        if (running_) {
            // Restart with new period
            stop();
            start();
        }
    }

    uint32_t getPeriod() const override {
        return periodUs_;
    }

    void start() override {
        if (running_) {
            stop();
        }

        if (!timerHandle_) {
            createTimer();
        }

        if (timerHandle_) {
            if (oneShot_) {
                esp_timer_start_once(timerHandle_, periodUs_);
            } else {
                esp_timer_start_periodic(timerHandle_, periodUs_);
            }
            running_ = true;
        }
    }

    void stop() override {
        if (timerHandle_ && running_) {
            esp_timer_stop(timerHandle_);
            running_ = false;
        }
    }

    bool isRunning() const override {
        return running_;
    }

    void reset() override {
        if (running_) {
            stop();
            start();
        }
    }

    uint32_t getCounter() const override {
        // ESP32 timer doesn't expose counter directly
        // Return elapsed time since start
        return 0;
    }

    void setCallback(hal::TimerCallback callback) override {
        callback_ = callback;

        // Recreate timer with new callback
        if (timerHandle_) {
            bool wasRunning = running_;
            stop();
            esp_timer_delete(timerHandle_);
            timerHandle_ = nullptr;
            createTimer();
            if (wasRunning) {
                start();
            }
        }
    }

    void enableInterrupt(bool enable) override {
        // ESP32 timer callbacks are always interrupt-based
        // Just start/stop the timer
        if (enable && !running_) {
            start();
        } else if (!enable && running_) {
            stop();
        }
    }

    void setOneShot(bool oneShot) override {
        oneShot_ = oneShot;

        if (running_) {
            stop();
            start();
        }
    }

private:
    esp_timer_handle_t timerHandle_;
    uint32_t periodUs_;
    bool running_;
    bool oneShot_;
    hal::TimerCallback callback_;

    void createTimer() {
        esp_timer_create_args_t timerArgs = {};
        timerArgs.callback = &ArduinoEsp32Timer::timerCallbackStatic;
        timerArgs.arg = this;
        timerArgs.dispatch_method = ESP_TIMER_TASK;
        timerArgs.name = "omni_timer";

        esp_timer_create(&timerArgs, &timerHandle_);
    }

    static void timerCallbackStatic(void* arg) {
        auto* self = static_cast<ArduinoEsp32Timer*>(arg);
        if (self->callback_) {
            self->callback_();
        }
        if (self->oneShot_) {
            self->running_ = false;
        }
    }
};

// Use ESP32 timer as default
using ArduinoTimer = ArduinoEsp32Timer;

#elif defined(ARDUINO_ARCH_RP2040)

/**
 * @brief RP2040 hardware timer implementation
 *
 * Uses RP2040's repeating timer API.
 */
class ArduinoRp2040Timer : public hal::ITimer {
public:
    ArduinoRp2040Timer()
        : periodUs_(1000)
        , running_(false)
        , oneShot_(false)
        , callback_(nullptr)
    {
    }

    ~ArduinoRp2040Timer() {
        stop();
    }

    void setPeriod(uint32_t periodUs) override {
        periodUs_ = periodUs;
        if (running_) {
            stop();
            start();
        }
    }

    uint32_t getPeriod() const override {
        return periodUs_;
    }

    void start() override {
        if (running_) {
            stop();
        }

        if (oneShot_) {
            // One-shot using alarm
            add_alarm_in_us(periodUs_, alarmCallbackStatic, this, false);
        } else {
            // Repeating timer
            add_repeating_timer_us(-static_cast<int64_t>(periodUs_),
                                   repeatingCallbackStatic, this, &timer_);
        }
        running_ = true;
    }

    void stop() override {
        if (running_) {
            cancel_repeating_timer(&timer_);
            running_ = false;
        }
    }

    bool isRunning() const override {
        return running_;
    }

    void reset() override {
        if (running_) {
            stop();
            start();
        }
    }

    uint32_t getCounter() const override {
        return 0;  // Not directly available
    }

    void setCallback(hal::TimerCallback callback) override {
        callback_ = callback;
    }

    void enableInterrupt(bool enable) override {
        if (enable && !running_) {
            start();
        } else if (!enable && running_) {
            stop();
        }
    }

    void setOneShot(bool oneShot) override {
        oneShot_ = oneShot;
        if (running_) {
            stop();
            start();
        }
    }

private:
    struct repeating_timer timer_;
    uint32_t periodUs_;
    bool running_;
    bool oneShot_;
    hal::TimerCallback callback_;

    static bool repeatingCallbackStatic(struct repeating_timer* t) {
        auto* self = static_cast<ArduinoRp2040Timer*>(t->user_data);
        if (self->callback_) {
            self->callback_();
        }
        return true;  // Continue repeating
    }

    static int64_t alarmCallbackStatic(alarm_id_t id, void* user_data) {
        (void)id;
        auto* self = static_cast<ArduinoRp2040Timer*>(user_data);
        if (self->callback_) {
            self->callback_();
        }
        self->running_ = false;
        return 0;  // Don't reschedule
    }
};

using ArduinoTimer = ArduinoRp2040Timer;

#else

/**
 * @brief Generic Arduino timer using software polling
 *
 * For platforms without hardware timer access, uses millis()/micros() polling.
 * Less accurate than hardware timers but works on any Arduino platform.
 *
 * Note: This timer requires periodic update() calls from the main loop.
 */
class ArduinoSoftwareTimer : public hal::ITimer {
public:
    ArduinoSoftwareTimer()
        : periodUs_(1000)
        , lastTrigger_(0)
        , running_(false)
        , oneShot_(false)
        , callback_(nullptr)
        , triggered_(false)
    {
    }

    void setPeriod(uint32_t periodUs) override {
        periodUs_ = periodUs;
    }

    uint32_t getPeriod() const override {
        return periodUs_;
    }

    void start() override {
        lastTrigger_ = micros();
        running_ = true;
        triggered_ = false;
    }

    void stop() override {
        running_ = false;
    }

    bool isRunning() const override {
        return running_;
    }

    void reset() override {
        lastTrigger_ = micros();
        triggered_ = false;
    }

    uint32_t getCounter() const override {
        if (!running_) return 0;
        return micros() - lastTrigger_;
    }

    void setCallback(hal::TimerCallback callback) override {
        callback_ = callback;
    }

    void enableInterrupt(bool enable) override {
        // Software timer doesn't support true interrupts
        // Just start/stop
        if (enable) start();
        else stop();
    }

    void setOneShot(bool oneShot) override {
        oneShot_ = oneShot;
    }

    /**
     * @brief Update function - must be called regularly from main loop
     *
     * Call this as frequently as possible for accurate timing.
     */
    void update() {
        if (!running_) return;

        uint32_t now = micros();
        uint32_t elapsed = now - lastTrigger_;

        if (elapsed >= periodUs_) {
            if (callback_) {
                callback_();
            }

            if (oneShot_) {
                running_ = false;
                triggered_ = true;
            } else {
                // Account for timing drift
                lastTrigger_ = now - (elapsed - periodUs_);
            }
        }
    }

    /**
     * @brief Check if one-shot timer has triggered
     */
    bool hasTriggered() const {
        return triggered_;
    }

private:
    uint32_t periodUs_;
    uint32_t lastTrigger_;
    bool running_;
    bool oneShot_;
    hal::TimerCallback callback_;
    bool triggered_;
};

using ArduinoTimer = ArduinoSoftwareTimer;

#endif

/**
 * @brief Simple stopwatch utility
 *
 * Usage:
 * @code
 * ArduinoStopwatch sw;
 * sw.start();
 * // ... do work ...
 * sw.stop();
 * float ms = sw.getElapsedMs();
 * @endcode
 */
class ArduinoStopwatch {
public:
    ArduinoStopwatch()
        : startTime_(0)
        , stopTime_(0)
        , running_(false)
    {
    }

    void start() {
        startTime_ = timestamp_.getMicros();
        running_ = true;
    }

    void stop() {
        if (running_) {
            stopTime_ = timestamp_.getMicros();
            running_ = false;
        }
    }

    void reset() {
        startTime_ = 0;
        stopTime_ = 0;
        running_ = false;
    }

    uint64_t getElapsedUs() const {
        if (running_) {
            return timestamp_.elapsedMicros(startTime_);
        }
        return stopTime_ - startTime_;
    }

    float getElapsedMs() const {
        return static_cast<float>(getElapsedUs()) / 1000.0f;
    }

    float getElapsedSeconds() const {
        return static_cast<float>(getElapsedUs()) / 1000000.0f;
    }

    bool isRunning() const {
        return running_;
    }

private:
    ArduinoTimestamp timestamp_;
    uint64_t startTime_;
    uint64_t stopTime_;
    bool running_;
};

}  // namespace arduino
}  // namespace platform
}  // namespace omni
