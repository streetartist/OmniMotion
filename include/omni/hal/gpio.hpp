/**
 * @file gpio.hpp
 * @brief GPIO hardware abstraction interface
 */

#pragma once

#include "types.hpp"

namespace omni::hal {

/**
 * @brief GPIO pin interface
 *
 * Abstract interface for GPIO operations including digital read/write
 * and pin mode configuration.
 */
class IGpio {
public:
    virtual ~IGpio() = default;

    /**
     * @brief Set the pin mode
     * @param mode The pin mode to set
     */
    virtual void setMode(PinMode mode) = 0;

    /**
     * @brief Get the current pin mode
     * @return Current pin mode
     */
    virtual PinMode getMode() const = 0;

    /**
     * @brief Write a digital value to the pin
     * @param value true for high, false for low
     */
    virtual void write(bool value) = 0;

    /**
     * @brief Read the digital value from the pin
     * @return true if high, false if low
     */
    virtual bool read() = 0;

    /**
     * @brief Toggle the pin state
     */
    virtual void toggle() = 0;

    /**
     * @brief Enable interrupt on this pin
     * @param risingEdge Trigger on rising edge
     * @param fallingEdge Trigger on falling edge
     */
    virtual void enableInterrupt(bool risingEdge, bool fallingEdge) = 0;

    /**
     * @brief Disable interrupt on this pin
     */
    virtual void disableInterrupt() = 0;
};

}  // namespace omni::hal
