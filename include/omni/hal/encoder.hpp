/**
 * @file encoder.hpp
 * @brief Encoder hardware abstraction interface
 */

#pragma once

#include <cstdint>
#include "types.hpp"

namespace omni::hal {

/**
 * @brief Incremental encoder interface
 *
 * Abstract interface for quadrature encoder reading with support
 * for position, velocity, and index pulse detection.
 */
class IEncoder {
public:
    virtual ~IEncoder() = default;

    /**
     * @brief Get the current count value
     * @return Raw encoder count
     */
    virtual int32_t getCount() = 0;

    /**
     * @brief Reset the counter to zero
     */
    virtual void resetCount() = 0;

    /**
     * @brief Set the counter to a specific value
     * @param count Value to set
     */
    virtual void setCount(int32_t count) = 0;

    /**
     * @brief Get angle within one revolution
     * @return Angle in radians (0 to 2*pi)
     */
    virtual float getAngle() = 0;

    /**
     * @brief Get angular velocity
     * @return Velocity in rad/s
     */
    virtual float getVelocity() = 0;

    /**
     * @brief Set the encoder resolution
     * @param cpr Counts per revolution (after quadrature decoding)
     */
    virtual void setResolution(uint32_t cpr) = 0;

    /**
     * @brief Get the encoder resolution
     * @return Counts per revolution
     */
    virtual uint32_t getResolution() const = 0;

    /**
     * @brief Set the counting mode
     * @param mode Counting mode (X1, X2, X4)
     */
    virtual void setMode(EncoderMode mode) = 0;

    /**
     * @brief Enable or disable index pulse detection
     * @param enable true to enable
     */
    virtual void enableIndex(bool enable) = 0;

    /**
     * @brief Check if index pulse was detected
     * @return true if index was detected
     */
    virtual bool indexDetected() = 0;

    /**
     * @brief Clear the index detection flag
     */
    virtual void clearIndex() = 0;

    /**
     * @brief Set direction inversion
     * @param invert true to invert direction
     */
    virtual void setInvert(bool invert) = 0;
};

/**
 * @brief Absolute encoder interface
 *
 * Extended interface for absolute encoders supporting multi-turn
 * counting and validity checking.
 */
class IAbsoluteEncoder : public IEncoder {
public:
    /**
     * @brief Get the absolute angle (not affected by resets)
     * @return Absolute angle in radians
     */
    virtual float getAbsoluteAngle() = 0;

    /**
     * @brief Get the multi-turn count
     * @return Number of complete revolutions
     */
    virtual int32_t getMultiTurnCount() = 0;

    /**
     * @brief Check if the current reading is valid
     * @return true if data is valid
     */
    virtual bool isValid() = 0;

    /**
     * @brief Get the raw absolute position
     * @return Raw position value from encoder
     */
    virtual uint32_t getRawPosition() = 0;

    /**
     * @brief Set the zero offset
     * @param offset Offset in radians
     */
    virtual void setZeroOffset(float offset) = 0;

    /**
     * @brief Get error status
     * @return Error flags
     */
    virtual uint32_t getErrorStatus() = 0;

    /**
     * @brief Clear error flags
     */
    virtual void clearErrors() = 0;
};

/**
 * @brief Hall sensor interface for BLDC commutation
 */
class IHallSensor {
public:
    virtual ~IHallSensor() = default;

    /**
     * @brief Get the current hall state (3-bit pattern)
     * @return Hall state (0-7)
     */
    virtual uint8_t getState() = 0;

    /**
     * @brief Get the electrical angle from hall sensors
     * @return Electrical angle in radians
     */
    virtual float getElectricalAngle() = 0;

    /**
     * @brief Set the hall sensor mapping table
     * @param table Mapping from hall state to sector (6 entries)
     */
    virtual void setMapping(const uint8_t* table) = 0;

    /**
     * @brief Enable interrupt on state change
     * @param enable true to enable
     */
    virtual void enableInterrupt(bool enable) = 0;
};

}  // namespace omni::hal
