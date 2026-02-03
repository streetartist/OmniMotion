/**
 * @file arduino_pwm.hpp
 * @brief Arduino PWM HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 */

#pragma once

#include <omni/hal/pwm.hpp>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
    // ESP32 uses LEDC for PWM
    #include <driver/ledc.h>
    #if __has_include(<driver/mcpwm.h>)
        #include <driver/mcpwm.h>
        #define OMNI_ESP32_MCPWM_AVAILABLE 1
    #endif
#endif

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Arduino single-channel PWM implementation
 *
 * Platform-specific implementations:
 * - AVR: Uses Timer registers directly, limited frequency options
 * - ESP32: Uses LEDC peripheral with configurable frequency/resolution
 * - RP2040: Uses analogWriteFreq() and analogWriteRange()
 *
 * Usage:
 * @code
 * ArduinoPwm pwm(9);  // PWM on pin 9
 * pwm.setFrequency(20000);
 * pwm.setDuty(0.5f);
 * pwm.enable(true);
 * @endcode
 */
class ArduinoPwm : public hal::IPwm {
public:
    /**
     * @brief Constructor
     * @param pin PWM output pin
     * @param channel LEDC channel (ESP32 only, 0-15)
     */
    explicit ArduinoPwm(uint8_t pin, uint8_t channel = 0)
        : pin_(pin)
        , channel_(channel)
        , frequency_(1000)
        , duty_(0.0f)
        , enabled_(false)
        , resolution_(8)
        , deadTimeNs_(0)
    {
    }

    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

#if defined(ARDUINO_ARCH_ESP32)
        // ESP32: Reconfigure LEDC channel
        // Calculate best resolution for frequency
        resolution_ = calculateResolution(freq_hz);
        ledcSetup(channel_, freq_hz, resolution_);
        ledcAttachPin(pin_, channel_);
#elif defined(ARDUINO_ARCH_RP2040)
        // RP2040: Set PWM frequency globally for the slice
        analogWriteFreq(freq_hz);
#else
        // AVR: Limited frequency control through timer prescaler
        // Standard Arduino analogWrite uses ~490Hz or ~980Hz
        // For custom frequencies, direct timer register manipulation is needed
        configureAvrTimer(freq_hz);
#endif

        // Reapply duty cycle with new frequency
        if (enabled_) {
            setDuty(duty_);
        }
    }

    uint32_t getFrequency() const override {
        return frequency_;
    }

    void setDuty(float duty) override {
        duty_ = constrain(duty, 0.0f, 1.0f);

        if (enabled_) {
            applyDuty();
        }
    }

    void setDutyNs(uint32_t ns) override {
        if (frequency_ > 0) {
            uint32_t periodNs = 1000000000UL / frequency_;
            float duty = static_cast<float>(ns) / static_cast<float>(periodNs);
            setDuty(duty);
        }
    }

    float getDuty() const override {
        return duty_;
    }

    void enable(bool en) override {
        enabled_ = en;

        if (en) {
            applyDuty();
        } else {
#if defined(ARDUINO_ARCH_ESP32)
            ledcWrite(channel_, 0);
#else
            analogWrite(pin_, 0);
#endif
        }
    }

    bool isEnabled() const override {
        return enabled_;
    }

    void setDeadTime(uint32_t ns) override {
        deadTimeNs_ = ns;
        // Dead time is primarily for complementary PWM
        // Not directly supported in single-channel mode
        // ESP32 MCPWM supports this, but it's handled in ArduinoPwm3Phase
    }

    uint32_t getDeadTime() const override {
        return deadTimeNs_;
    }

    /**
     * @brief Get the pin number
     */
    uint8_t getPin() const { return pin_; }

    /**
     * @brief Get the LEDC channel (ESP32)
     */
    uint8_t getChannel() const { return channel_; }

private:
    uint8_t pin_;
    uint8_t channel_;
    uint32_t frequency_;
    float duty_;
    bool enabled_;
    uint8_t resolution_;
    uint32_t deadTimeNs_;

    void applyDuty() {
#if defined(ARDUINO_ARCH_ESP32)
        uint32_t maxDuty = (1 << resolution_) - 1;
        uint32_t dutyValue = static_cast<uint32_t>(duty_ * maxDuty);
        ledcWrite(channel_, dutyValue);
#elif defined(ARDUINO_ARCH_RP2040)
        // RP2040 uses 8-bit by default, can be changed with analogWriteResolution
        uint32_t maxDuty = 255;  // Default 8-bit
        uint32_t dutyValue = static_cast<uint32_t>(duty_ * maxDuty);
        analogWrite(pin_, dutyValue);
#else
        // AVR: 8-bit PWM
        uint32_t dutyValue = static_cast<uint32_t>(duty_ * 255.0f);
        analogWrite(pin_, dutyValue);
#endif
    }

#if defined(ARDUINO_ARCH_ESP32)
    static uint8_t calculateResolution(uint32_t freq) {
        // ESP32 LEDC resolution depends on frequency
        // Higher frequencies need lower resolution
        // Max resolution decreases as frequency increases
        // resolution_bits <= log2(80MHz / freq)
        uint32_t maxDuty = 80000000UL / freq;
        uint8_t bits = 0;
        while (maxDuty > 1) {
            maxDuty >>= 1;
            bits++;
        }
        return (bits > 16) ? 16 : ((bits < 1) ? 1 : bits);
    }
#endif

#if defined(ARDUINO_ARCH_AVR)
    void configureAvrTimer(uint32_t freq) {
        // AVR timer configuration for custom frequencies
        // This is pin-specific and complex
        // For simplicity, we use standard analogWrite and accept the default frequency
        // Advanced users can override this for specific timers
        (void)freq;  // Suppress unused warning
    }
#endif
};

#if defined(ARDUINO_ARCH_ESP32) && defined(OMNI_ESP32_MCPWM_AVAILABLE)
/**
 * @brief ESP32 Three-phase PWM using MCPWM module
 *
 * Uses ESP32's Motor Control PWM peripheral for hardware dead-time
 * and synchronized three-phase output.
 *
 * Usage:
 * @code
 * ArduinoEsp32Pwm3Phase pwm(
 *     25, 26,  // Phase A high/low
 *     27, 14,  // Phase B high/low
 *     12, 13   // Phase C high/low
 * );
 * pwm.setFrequency(20000);
 * pwm.setDeadTime(500);  // 500ns dead time
 * pwm.enable(true);
 * pwm.setDuty(0.5f, 0.4f, 0.3f);
 * @endcode
 */
class ArduinoEsp32Pwm3Phase : public hal::IPwm3Phase {
public:
    /**
     * @brief Constructor
     * @param pinAH Phase A high-side pin
     * @param pinAL Phase A low-side pin
     * @param pinBH Phase B high-side pin
     * @param pinBL Phase B low-side pin
     * @param pinCH Phase C high-side pin
     * @param pinCL Phase C low-side pin
     * @param unit MCPWM unit (0 or 1)
     */
    ArduinoEsp32Pwm3Phase(uint8_t pinAH, uint8_t pinAL,
                          uint8_t pinBH, uint8_t pinBL,
                          uint8_t pinCH, uint8_t pinCL,
                          mcpwm_unit_t unit = MCPWM_UNIT_0)
        : unit_(unit)
        , frequency_(20000)
        , dutyA_(0.5f), dutyB_(0.5f), dutyC_(0.5f)
        , enabled_(false)
        , deadTimeNs_(0)
        , enA_(true), enB_(true), enC_(true)
    {
        pins_[0] = pinAH; pins_[1] = pinAL;
        pins_[2] = pinBH; pins_[3] = pinBL;
        pins_[4] = pinCH; pins_[5] = pinCL;

        init();
    }

    void setDuty(float a, float b, float c) override {
        dutyA_ = constrain(a, 0.0f, 1.0f);
        dutyB_ = constrain(b, 0.0f, 1.0f);
        dutyC_ = constrain(c, 0.0f, 1.0f);

        if (enabled_) {
            applyDuty();
        }
    }

    void getDuty(float& a, float& b, float& c) const override {
        a = dutyA_;
        b = dutyB_;
        c = dutyC_;
    }

    void enable(bool en) override {
        enabled_ = en;

        if (en) {
            mcpwm_start(unit_, MCPWM_TIMER_0);
            mcpwm_start(unit_, MCPWM_TIMER_1);
            mcpwm_start(unit_, MCPWM_TIMER_2);
            applyDuty();
        } else {
            mcpwm_stop(unit_, MCPWM_TIMER_0);
            mcpwm_stop(unit_, MCPWM_TIMER_1);
            mcpwm_stop(unit_, MCPWM_TIMER_2);
        }
    }

    bool isEnabled() const override {
        return enabled_;
    }

    void setDeadTime(uint32_t ns) override {
        deadTimeNs_ = ns;

        // Configure dead time for all operators
        mcpwm_deadtime_type_t dt_type = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE;

        // Dead time value depends on MCPWM clock (typically 160MHz)
        // dt_ticks = ns * 160 / 1000
        uint32_t dt_ticks = (ns * 160) / 1000;
        if (dt_ticks > 255) dt_ticks = 255;

        mcpwm_deadtime_enable(unit_, MCPWM_TIMER_0, dt_type, dt_ticks, dt_ticks);
        mcpwm_deadtime_enable(unit_, MCPWM_TIMER_1, dt_type, dt_ticks, dt_ticks);
        mcpwm_deadtime_enable(unit_, MCPWM_TIMER_2, dt_type, dt_ticks, dt_ticks);
    }

    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

        mcpwm_config_t pwm_config = {};
        pwm_config.frequency = freq_hz;
        pwm_config.cmpr_a = 50.0f;
        pwm_config.cmpr_b = 50.0f;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;  // Center-aligned

        mcpwm_init(unit_, MCPWM_TIMER_0, &pwm_config);
        mcpwm_init(unit_, MCPWM_TIMER_1, &pwm_config);
        mcpwm_init(unit_, MCPWM_TIMER_2, &pwm_config);
    }

    void setSvpwm(float alpha, float beta) override {
        // Space Vector PWM implementation
        // Convert alpha-beta to duty cycles

        // Sector determination
        float va = alpha;
        float vb = -0.5f * alpha + 0.866025f * beta;
        float vc = -0.5f * alpha - 0.866025f * beta;

        // Find min and max
        float vmin = min(min(va, vb), vc);
        float vmax = max(max(va, vb), vc);

        // Center the waveform (saddle modulation)
        float voffset = (vmin + vmax) * 0.5f;
        va -= voffset;
        vb -= voffset;
        vc -= voffset;

        // Scale to 0-1 range
        float scale = 1.0f / (vmax - vmin);
        if (scale > 1.0f) scale = 1.0f;

        dutyA_ = constrain(va * scale + 0.5f, 0.0f, 1.0f);
        dutyB_ = constrain(vb * scale + 0.5f, 0.0f, 1.0f);
        dutyC_ = constrain(vc * scale + 0.5f, 0.0f, 1.0f);

        if (enabled_) {
            applyDuty();
        }
    }

    void setPhaseEnable(bool enA, bool enB, bool enC) override {
        enA_ = enA;
        enB_ = enB;
        enC_ = enC;

        // Enable/disable individual phase outputs
        if (enA) {
            mcpwm_set_signal_low(unit_, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_low(unit_, MCPWM_TIMER_0, MCPWM_OPR_B);
        }
        // Note: MCPWM doesn't have direct per-phase enable
        // This would need GPIO manipulation for true tri-state
    }

private:
    mcpwm_unit_t unit_;
    uint8_t pins_[6];
    uint32_t frequency_;
    float dutyA_, dutyB_, dutyC_;
    bool enabled_;
    uint32_t deadTimeNs_;
    bool enA_, enB_, enC_;

    void init() {
        // Initialize MCPWM GPIO
        mcpwm_gpio_init(unit_, MCPWM0A, pins_[0]);
        mcpwm_gpio_init(unit_, MCPWM0B, pins_[1]);
        mcpwm_gpio_init(unit_, MCPWM1A, pins_[2]);
        mcpwm_gpio_init(unit_, MCPWM1B, pins_[3]);
        mcpwm_gpio_init(unit_, MCPWM2A, pins_[4]);
        mcpwm_gpio_init(unit_, MCPWM2B, pins_[5]);

        setFrequency(frequency_);
    }

    void applyDuty() {
        // MCPWM duty is in percentage (0-100)
        if (enA_) mcpwm_set_duty(unit_, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA_ * 100.0f);
        if (enB_) mcpwm_set_duty(unit_, MCPWM_TIMER_1, MCPWM_OPR_A, dutyB_ * 100.0f);
        if (enC_) mcpwm_set_duty(unit_, MCPWM_TIMER_2, MCPWM_OPR_A, dutyC_ * 100.0f);
    }
};

#endif  // ESP32 MCPWM

/**
 * @brief Software-synchronized three-phase PWM
 *
 * Uses three independent PWM channels for platforms without hardware
 * three-phase support. No hardware dead-time generation.
 *
 * Usage:
 * @code
 * ArduinoPwm3Phase pwm(9, 10, 11);  // Three PWM pins
 * pwm.setFrequency(20000);
 * pwm.enable(true);
 * pwm.setDuty(0.5f, 0.4f, 0.3f);
 * @endcode
 */
class ArduinoPwm3Phase : public hal::IPwm3Phase {
public:
    /**
     * @brief Constructor
     * @param pinA Phase A PWM pin
     * @param pinB Phase B PWM pin
     * @param pinC Phase C PWM pin
     */
    ArduinoPwm3Phase(uint8_t pinA, uint8_t pinB, uint8_t pinC)
        : pwmA_(pinA, 0)
        , pwmB_(pinB, 1)
        , pwmC_(pinC, 2)
        , dutyA_(0.5f), dutyB_(0.5f), dutyC_(0.5f)
        , enabled_(false)
        , enA_(true), enB_(true), enC_(true)
    {
    }

    void setDuty(float a, float b, float c) override {
        dutyA_ = constrain(a, 0.0f, 1.0f);
        dutyB_ = constrain(b, 0.0f, 1.0f);
        dutyC_ = constrain(c, 0.0f, 1.0f);

        if (enabled_) {
            if (enA_) pwmA_.setDuty(dutyA_);
            if (enB_) pwmB_.setDuty(dutyB_);
            if (enC_) pwmC_.setDuty(dutyC_);
        }
    }

    void getDuty(float& a, float& b, float& c) const override {
        a = dutyA_;
        b = dutyB_;
        c = dutyC_;
    }

    void enable(bool en) override {
        enabled_ = en;
        pwmA_.enable(en && enA_);
        pwmB_.enable(en && enB_);
        pwmC_.enable(en && enC_);

        if (en) {
            setDuty(dutyA_, dutyB_, dutyC_);
        }
    }

    bool isEnabled() const override {
        return enabled_;
    }

    void setDeadTime(uint32_t ns) override {
        // Software dead-time not implemented for basic 3-phase
        // Would require complementary outputs and precise timing
        (void)ns;
    }

    void setFrequency(uint32_t freq_hz) override {
        pwmA_.setFrequency(freq_hz);
        pwmB_.setFrequency(freq_hz);
        pwmC_.setFrequency(freq_hz);
    }

    void setSvpwm(float alpha, float beta) override {
        // Space Vector PWM implementation
        float va = alpha;
        float vb = -0.5f * alpha + 0.866025f * beta;
        float vc = -0.5f * alpha - 0.866025f * beta;

        float vmin = min(min(va, vb), vc);
        float vmax = max(max(va, vb), vc);

        float voffset = (vmin + vmax) * 0.5f;
        va -= voffset;
        vb -= voffset;
        vc -= voffset;

        float scale = 1.0f / (vmax - vmin);
        if (scale > 1.0f) scale = 1.0f;

        setDuty(
            constrain(va * scale + 0.5f, 0.0f, 1.0f),
            constrain(vb * scale + 0.5f, 0.0f, 1.0f),
            constrain(vc * scale + 0.5f, 0.0f, 1.0f)
        );
    }

    void setPhaseEnable(bool enA, bool enB, bool enC) override {
        enA_ = enA;
        enB_ = enB;
        enC_ = enC;

        if (enabled_) {
            pwmA_.enable(enA);
            pwmB_.enable(enB);
            pwmC_.enable(enC);
        }
    }

private:
    ArduinoPwm pwmA_, pwmB_, pwmC_;
    float dutyA_, dutyB_, dutyC_;
    bool enabled_;
    bool enA_, enB_, enC_;
};

}  // namespace arduino
}  // namespace platform
}  // namespace omni
