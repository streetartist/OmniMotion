/**
 * @file arduino_adc.hpp
 * @brief Arduino ADC HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 */

#pragma once

#include <omni/hal/adc.hpp>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
    #include <driver/adc.h>
#endif

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Arduino single-channel ADC implementation
 *
 * Platform differences:
 * - AVR: 10-bit fixed resolution, 5V reference
 * - ESP32: 12-bit, 3.3V reference, attenuation configurable
 * - RP2040: 12-bit, 3.3V reference
 *
 * Note: Arduino does not support DMA, related methods are no-ops.
 *
 * Usage:
 * @code
 * ArduinoAdc adc(A0);
 * uint16_t raw = adc.read();
 * float voltage = adc.readVoltage();
 * @endcode
 */
class ArduinoAdc : public hal::IAdc {
public:
    /**
     * @brief Constructor
     * @param pin Analog input pin (A0, A1, etc.)
     */
    explicit ArduinoAdc(uint8_t pin)
        : pin_(pin)
        , vref_(getDefaultVref())
        , resolution_(getDefaultResolution())
        , samplingTime_(0)
    {
#if defined(ARDUINO_ARCH_ESP32)
        // Configure ADC for ESP32
        configureEsp32Adc();
#elif defined(ARDUINO_ARCH_RP2040)
        // RP2040 ADC setup
        analogReadResolution(12);
#endif
    }

    uint16_t read() override {
        return analogRead(pin_);
    }

    float readVoltage() override {
        uint16_t raw = read();
        uint16_t maxValue = (1 << resolution_) - 1;
        return (static_cast<float>(raw) / static_cast<float>(maxValue)) * vref_;
    }

    void startDma(uint16_t* buffer, size_t len) override {
        // Arduino does not support DMA ADC
        // Fill buffer with single reads as fallback
        for (size_t i = 0; i < len; i++) {
            buffer[i] = read();
        }
        dmaComplete_ = true;
    }

    void stopDma() override {
        // No-op for Arduino
    }

    bool isDmaComplete() const override {
        return dmaComplete_;
    }

    void setTrigger(hal::AdcTrigger trig) override {
        // Arduino doesn't support hardware ADC triggers
        // Software trigger is always used
        (void)trig;
    }

    uint8_t getResolution() const override {
        return resolution_;
    }

    void setReferenceVoltage(float vref) override {
        vref_ = vref;
    }

    float getReferenceVoltage() const override {
        return vref_;
    }

    void setSamplingTime(uint32_t cycles) override {
        samplingTime_ = cycles;
        // Arduino doesn't expose ADC sampling time configuration
        // ESP32 can adjust attenuation for different input ranges
#if defined(ARDUINO_ARCH_ESP32)
        // Higher cycles = longer sampling = better for high impedance sources
        (void)cycles;
#endif
    }

    /**
     * @brief Get the analog pin number
     */
    uint8_t getPin() const { return pin_; }

    /**
     * @brief Set ADC resolution (if supported by platform)
     * @param bits Resolution in bits (8, 10, 12, etc.)
     */
    void setResolution(uint8_t bits) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAM)
        resolution_ = bits;
        analogReadResolution(bits);
#else
        // AVR has fixed 10-bit resolution
        (void)bits;
#endif
    }

#if defined(ARDUINO_ARCH_ESP32)
    /**
     * @brief Set ESP32 ADC attenuation
     * @param atten Attenuation level (ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11)
     */
    void setAttenuation(adc_atten_t atten) {
        atten_ = atten;
        // Determine ADC channel from pin
        adc1_channel_t channel = getAdc1Channel(pin_);
        if (channel != ADC1_CHANNEL_MAX) {
            adc1_config_channel_atten(channel, atten);
        }

        // Update vref based on attenuation
        switch (atten) {
            case ADC_ATTEN_DB_0:   vref_ = 1.1f; break;
            case ADC_ATTEN_DB_2_5: vref_ = 1.5f; break;
            case ADC_ATTEN_DB_6:   vref_ = 2.2f; break;
            case ADC_ATTEN_DB_11:  vref_ = 3.3f; break;
            default: vref_ = 3.3f;
        }
    }
#endif

private:
    uint8_t pin_;
    float vref_;
    uint8_t resolution_;
    uint32_t samplingTime_;
    mutable bool dmaComplete_ = false;

#if defined(ARDUINO_ARCH_ESP32)
    adc_atten_t atten_ = ADC_ATTEN_DB_11;

    void configureEsp32Adc() {
        resolution_ = 12;
        analogReadResolution(12);

        // Configure ADC1 channel
        adc1_channel_t channel = getAdc1Channel(pin_);
        if (channel != ADC1_CHANNEL_MAX) {
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(channel, atten_);
        }
    }

    static adc1_channel_t getAdc1Channel(uint8_t pin) {
        // ESP32 pin to ADC1 channel mapping
        switch (pin) {
            case 36: return ADC1_CHANNEL_0;
            case 37: return ADC1_CHANNEL_1;
            case 38: return ADC1_CHANNEL_2;
            case 39: return ADC1_CHANNEL_3;
            case 32: return ADC1_CHANNEL_4;
            case 33: return ADC1_CHANNEL_5;
            case 34: return ADC1_CHANNEL_6;
            case 35: return ADC1_CHANNEL_7;
            default: return ADC1_CHANNEL_MAX;
        }
    }
#endif

    static float getDefaultVref() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
        return 3.3f;
#elif defined(ARDUINO_ARCH_AVR)
        return 5.0f;
#else
        return 3.3f;  // Default assumption
#endif
    }

    static uint8_t getDefaultResolution() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
        return 12;
#elif defined(ARDUINO_ARCH_AVR)
        return 10;
#else
        return 10;  // Default assumption
#endif
    }
};

/**
 * @brief Arduino multi-channel ADC implementation
 *
 * Manages multiple ADC channels for synchronized reading.
 * Useful for three-phase current sensing.
 *
 * Usage:
 * @code
 * uint8_t channels[] = {A0, A1, A2};
 * ArduinoAdcMultiChannel adc(channels, 3);
 * uint16_t values[3];
 * adc.readAllChannels(values, 3);
 * @endcode
 */
class ArduinoAdcMultiChannel : public hal::IAdcMultiChannel {
public:
    static constexpr size_t MAX_CHANNELS = 8;

    /**
     * @brief Constructor
     * @param pins Array of analog pin numbers
     * @param numChannels Number of channels
     */
    ArduinoAdcMultiChannel(const uint8_t* pins, uint8_t numChannels)
        : numChannels_(numChannels > MAX_CHANNELS ? MAX_CHANNELS : numChannels)
        , vref_(ArduinoAdc(pins[0]).getReferenceVoltage())
        , resolution_(ArduinoAdc(pins[0]).getResolution())
    {
        for (uint8_t i = 0; i < numChannels_; i++) {
            pins_[i] = pins[i];
        }
    }

    uint16_t readChannel(uint8_t channel) override {
        if (channel < numChannels_) {
            return analogRead(pins_[channel]);
        }
        return 0;
    }

    float readVoltageChannel(uint8_t channel) override {
        if (channel < numChannels_) {
            uint16_t raw = analogRead(pins_[channel]);
            uint16_t maxValue = (1 << resolution_) - 1;
            return (static_cast<float>(raw) / static_cast<float>(maxValue)) * vref_;
        }
        return 0.0f;
    }

    void readAllChannels(uint16_t* values, uint8_t numChannels) override {
        uint8_t count = (numChannels < numChannels_) ? numChannels : numChannels_;
        for (uint8_t i = 0; i < count; i++) {
            values[i] = analogRead(pins_[i]);
        }
    }

    void startSynchronizedDma(uint16_t** buffers, size_t len) override {
        // Arduino doesn't support DMA
        // Perform sequential reads as fallback
        for (size_t sample = 0; sample < len; sample++) {
            for (uint8_t ch = 0; ch < numChannels_; ch++) {
                if (buffers[ch] != nullptr) {
                    buffers[ch][sample] = analogRead(pins_[ch]);
                }
            }
        }
    }

    void configureSynchronizedChannels(const uint8_t* channels, uint8_t numChannels) override {
        // Reconfigure channel mapping
        numChannels_ = (numChannels > MAX_CHANNELS) ? MAX_CHANNELS : numChannels;
        for (uint8_t i = 0; i < numChannels_; i++) {
            pins_[i] = channels[i];
        }
    }

    /**
     * @brief Get number of configured channels
     */
    uint8_t getNumChannels() const { return numChannels_; }

    /**
     * @brief Set reference voltage for all channels
     */
    void setReferenceVoltage(float vref) { vref_ = vref; }

    /**
     * @brief Set resolution for all channels
     */
    void setResolution(uint8_t bits) {
        resolution_ = bits;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAM)
        analogReadResolution(bits);
#endif
    }

private:
    uint8_t pins_[MAX_CHANNELS];
    uint8_t numChannels_;
    float vref_;
    uint8_t resolution_;
};

}  // namespace arduino
}  // namespace platform
}  // namespace omni
