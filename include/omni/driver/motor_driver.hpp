/**
 * @file motor_driver.hpp
 * @brief Base motor driver interface
 */

#pragma once

#include "types.hpp"

namespace omni::driver {

/**
 * @brief Motor driver interface
 *
 * Abstract base class defining the interface for all motor drivers.
 * Provides a unified API for controlling various motor types.
 */
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    // ===== Lifecycle =====

    /**
     * @brief Initialize the motor driver
     * @return true if initialization successful
     */
    virtual bool init() = 0;

    /**
     * @brief Deinitialize the motor driver
     */
    virtual void deinit() = 0;

    // ===== Enable/Disable =====

    /**
     * @brief Enable the motor
     */
    virtual void enable() = 0;

    /**
     * @brief Disable the motor
     */
    virtual void disable() = 0;

    /**
     * @brief Check if motor is enabled
     * @return true if enabled
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Emergency stop - immediately stop with maximum deceleration
     */
    virtual void emergencyStop() = 0;

    // ===== Control Mode =====

    /**
     * @brief Set the control mode
     * @param mode Control mode to use
     */
    virtual void setControlMode(ControlMode mode) = 0;

    /**
     * @brief Get the current control mode
     * @return Current control mode
     */
    virtual ControlMode getControlMode() const = 0;

    // ===== Target Setting =====

    /**
     * @brief Set target position (for position mode)
     * @param pos Target position (rad or m)
     */
    virtual void setPosition(float pos) = 0;

    /**
     * @brief Set target velocity (for velocity mode)
     * @param vel Target velocity (rad/s or m/s)
     */
    virtual void setVelocity(float vel) = 0;

    /**
     * @brief Set target torque (for torque mode)
     * @param torque Target torque (Nm or N)
     */
    virtual void setTorque(float torque) = 0;

    /**
     * @brief Set target current (for current mode)
     * @param current Target current (A)
     */
    virtual void setCurrent(float current) = 0;

    /**
     * @brief Set target voltage (for voltage mode)
     * @param voltage Target voltage (V)
     */
    virtual void setVoltage(float voltage) = 0;

    /**
     * @brief Set duty cycle (for open-loop duty mode)
     * @param duty Duty cycle (-1.0 to 1.0)
     */
    virtual void setDuty(float duty) = 0;

    // ===== State Reading =====

    /**
     * @brief Get the complete motor state
     * @return Current motor state
     */
    virtual MotorState getState() const = 0;

    /**
     * @brief Get current position
     * @return Position (rad or m)
     */
    virtual float getPosition() const = 0;

    /**
     * @brief Get current velocity
     * @return Velocity (rad/s or m/s)
     */
    virtual float getVelocity() const = 0;

    /**
     * @brief Get current torque (estimated or measured)
     * @return Torque (Nm or N)
     */
    virtual float getTorque() const = 0;

    /**
     * @brief Get current (measured)
     * @return Current (A)
     */
    virtual float getCurrent() const = 0;

    /**
     * @brief Get temperature
     * @return Temperature (C)
     */
    virtual float getTemperature() const = 0;

    // ===== Parameters =====

    /**
     * @brief Set motor parameters
     * @param params Motor parameters
     */
    virtual void setParams(const MotorParams& params) = 0;

    /**
     * @brief Get motor parameters
     * @return Current motor parameters
     */
    virtual MotorParams getParams() const = 0;

    // ===== Update =====

    /**
     * @brief Update the motor control loop
     *
     * Must be called periodically at the control rate.
     */
    virtual void update() = 0;

    // ===== Type Information =====

    /**
     * @brief Get the motor type
     * @return Motor type enum
     */
    virtual MotorType getType() const = 0;

    /**
     * @brief Get the motor name/description
     * @return Motor name string
     */
    virtual const char* getName() const = 0;

    // ===== Error Handling =====

    /**
     * @brief Get error code
     * @return Current error code
     */
    virtual uint32_t getErrorCode() const = 0;

    /**
     * @brief Clear errors
     */
    virtual void clearErrors() = 0;

    /**
     * @brief Check if there is a fault
     * @return true if fault present
     */
    virtual bool hasFault() const = 0;
};

}  // namespace omni::driver
