/**
 * @file adc.hpp
 * @brief ADC hardware abstraction interface
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "types.hpp"

namespace omni::hal {

/**
 * @brief Single channel ADC interface
 *
 * Abstract interface for analog-to-digital conversion with support
 * for software and hardware triggered conversions.
 */
class IAdc {
public:
    virtual ~IAdc() = default;

    /**
     * @brief Read raw ADC value
     * @return Raw ADC count
     */
    virtual uint16_t read() = 0;

    /**
     * @brief Read and convert to voltage
     * @return Voltage in volts
     */
    virtual float readVoltage() = 0;

    /**
     * @brief Start DMA-based continuous conversion
     * @param buffer Pointer to buffer for results
     * @param len Number of samples to acquire
     */
    virtual void startDma(uint16_t* buffer, size_t len) = 0;

    /**
     * @brief Stop DMA conversion
     */
    virtual void stopDma() = 0;

    /**
     * @brief Check if DMA transfer is complete
     * @return true if complete
     */
    virtual bool isDmaComplete() const = 0;

    /**
     * @brief Set the conversion trigger source
     * @param trig Trigger source
     */
    virtual void setTrigger(AdcTrigger trig) = 0;

    /**
     * @brief Get the ADC resolution
     * @return Resolution in bits
     */
    virtual uint8_t getResolution() const = 0;

    /**
     * @brief Set the reference voltage for conversion
     * @param vref Reference voltage in volts
     */
    virtual void setReferenceVoltage(float vref) = 0;

    /**
     * @brief Get the reference voltage
     * @return Reference voltage in volts
     */
    virtual float getReferenceVoltage() const = 0;

    /**
     * @brief Set sampling time
     * @param cycles Number of ADC clock cycles
     */
    virtual void setSamplingTime(uint32_t cycles) = 0;
};

/**
 * @brief Multi-channel ADC interface
 *
 * Interface for ADC with multiple channels, commonly used for
 * three-phase current sensing.
 */
class IAdcMultiChannel {
public:
    virtual ~IAdcMultiChannel() = default;

    /**
     * @brief Read raw value from a specific channel
     * @param channel Channel index
     * @return Raw ADC count
     */
    virtual uint16_t readChannel(uint8_t channel) = 0;

    /**
     * @brief Read voltage from a specific channel
     * @param channel Channel index
     * @return Voltage in volts
     */
    virtual float readVoltageChannel(uint8_t channel) = 0;

    /**
     * @brief Read all configured channels simultaneously
     * @param values Array to store results
     * @param numChannels Number of channels to read
     */
    virtual void readAllChannels(uint16_t* values, uint8_t numChannels) = 0;

    /**
     * @brief Start synchronized DMA conversion on all channels
     * @param buffers Array of buffer pointers
     * @param len Number of samples per channel
     */
    virtual void startSynchronizedDma(uint16_t** buffers, size_t len) = 0;

    /**
     * @brief Configure channels for synchronized sampling
     * @param channels Array of channel indices
     * @param numChannels Number of channels
     */
    virtual void configureSynchronizedChannels(const uint8_t* channels, uint8_t numChannels) = 0;
};

}  // namespace omni::hal
