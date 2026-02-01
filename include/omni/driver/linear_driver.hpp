/**
 * @file linear_driver.hpp
 * @brief Linear motor driver base class
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"
#include "omni/control/pid_controller.hpp"

namespace omni::driver {

/**
 * @brief Linear motor driver
 *
 * Base driver for linear motors providing position in meters
 * rather than radians. Supports various linear motor types.
 */
class LinearMotorDriver : public IMotorDriver {
public:
    /**
     * @brief Construct linear motor driver
     */
    LinearMotorDriver();

    ~LinearMotorDriver() override = default;

    // IMotorDriver interface (partial - to be completed by derived classes)
    MotorType getType() const override { return MotorType::LinearSynchronous; }
    const char* getName() const override { return "Linear Motor Driver"; }

    // ===== Linear Motor Specific =====

    /**
     * @brief Get linear position
     * @return Position in meters
     */
    virtual float getLinearPosition() const = 0;

    /**
     * @brief Get linear velocity
     * @return Velocity in m/s
     */
    virtual float getLinearVelocity() const = 0;

    /**
     * @brief Set linear position target
     * @param pos Position in meters
     */
    virtual void setLinearPosition(float pos) = 0;

    /**
     * @brief Set linear velocity target
     * @param vel Velocity in m/s
     */
    virtual void setLinearVelocity(float vel) = 0;

    /**
     * @brief Set force target
     * @param force Force in Newtons
     */
    virtual void setForce(float force) = 0;

    /**
     * @brief Get current force
     * @return Force in Newtons
     */
    virtual float getForce() const = 0;

    /**
     * @brief Set linear scale (for encoder conversion)
     * @param metersPerCount Conversion factor
     */
    void setLinearScale(float metersPerCount) { linearScale_ = metersPerCount; }

    /**
     * @brief Get linear scale
     * @return Meters per encoder count
     */
    float getLinearScale() const { return linearScale_; }

protected:
    float linearScale_ = 1.0f;  // m/count
};

/**
 * @brief Linear synchronous motor driver (iron-core or ironless)
 *
 * Uses FOC control adapted for linear motion.
 */
class LinearSyncMotorDriver : public LinearMotorDriver {
public:
    /**
     * @brief Construct linear synchronous motor driver
     * @param pwm Three-phase PWM
     * @param currentAdcA Phase A current ADC
     * @param currentAdcB Phase B current ADC
     * @param linearEncoder Linear encoder for position
     */
    LinearSyncMotorDriver(hal::IPwm3Phase* pwm,
                          hal::IAdc* currentAdcA,
                          hal::IAdc* currentAdcB,
                          hal::IAbsoluteEncoder* linearEncoder);

    ~LinearSyncMotorDriver() override = default;

    // IMotorDriver interface
    bool init() override;
    void deinit() override;
    void enable() override;
    void disable() override;
    bool isEnabled() const override;
    void emergencyStop() override;

    void setControlMode(ControlMode mode) override;
    ControlMode getControlMode() const override;

    void setPosition(float pos) override;
    void setVelocity(float vel) override;
    void setTorque(float torque) override;
    void setCurrent(float current) override;
    void setVoltage(float voltage) override;
    void setDuty(float duty) override;

    MotorState getState() const override;
    float getPosition() const override;
    float getVelocity() const override;
    float getTorque() const override;
    float getCurrent() const override;
    float getTemperature() const override;

    void setParams(const MotorParams& params) override;
    MotorParams getParams() const override;

    void update(float dt) override;

    MotorType getType() const override { return MotorType::LinearSynchronous; }
    const char* getName() const override { return "Linear Synchronous Motor"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // Linear specific
    float getLinearPosition() const override;
    float getLinearVelocity() const override;
    void setLinearPosition(float pos) override;
    void setLinearVelocity(float vel) override;
    void setForce(float force) override;
    float getForce() const override;

    // ===== LSM Specific =====

    /**
     * @brief Set pole pitch (electrical period)
     * @param pitch Pole pitch in meters
     */
    void setPolePitch(float pitch);

    /**
     * @brief Get pole pitch
     * @return Pole pitch in meters
     */
    float getPolePitch() const { return polePitch_; }

    /**
     * @brief Set force constant
     * @param kf Force constant (N/A)
     */
    void setForceConstant(float kf);

    /**
     * @brief Calibrate commutation offset
     * @return true if successful
     */
    bool calibrateCommutation();

private:
    // Hardware
    hal::IPwm3Phase* pwm_;
    hal::IAdc* adcA_;
    hal::IAdc* adcB_;
    hal::IAbsoluteEncoder* linearEncoder_;

    // Motor parameters
    float polePitch_;       // m
    float forceConstant_;   // N/A

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Position/velocity
    float linearPosition_;
    float linearVelocity_;
    float electricalAngle_;
    float commutationOffset_;

    // Current
    float ia_, ib_;
    float id_, iq_;
    float idRef_, iqRef_;

    // Control targets
    float targetPosition_;
    float targetVelocity_;
    float targetForce_;
    float targetId_;
    float targetIq_;

    // Controllers
    control::PidController idPid_;
    control::PidController iqPid_;
    control::PidController velocityPid_;
    control::PidController positionPid_;

    // Internal methods
    void readFeedback();
    void runFocLoop(float dt);
    float positionToElectricalAngle(float pos) const;
};

}  // namespace omni::driver
