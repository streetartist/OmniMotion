/**
 * @file vcm_driver.hpp
 * @brief Voice Coil Motor (VCM) driver
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"

namespace omni::driver {

/**
 * @brief Voice Coil Motor driver
 *
 * Drives voice coil actuators for precise linear positioning.
 * Commonly used in hard drives, camera autofocus, and precision stages.
 */
class VcmDriver : public IMotorDriver {
public:
    /**
     * @brief Construct VCM driver
     * @param pwm PWM for current control
     * @param positionAdc ADC for position feedback
     */
    VcmDriver(hal::IPwm* pwm, hal::IAdc* positionAdc);

    /**
     * @brief Construct VCM driver with current sensing
     * @param pwm PWM for current control
     * @param positionAdc ADC for position feedback
     * @param currentAdc ADC for current feedback
     */
    VcmDriver(hal::IPwm* pwm, hal::IAdc* positionAdc, hal::IAdc* currentAdc);

    ~VcmDriver() override = default;

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

    void update() override;

    MotorType getType() const override { return MotorType::VoiceCoil; }
    const char* getName() const override { return "Voice Coil Motor Driver"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== VCM Specific =====

    /**
     * @brief Set output force
     * @param force Force (N)
     */
    void setForce(float force);

    /**
     * @brief Get current force
     * @return Force (N)
     */
    float getForce() const;

    /**
     * @brief Set force constant
     * @param kf Force constant (N/A)
     */
    void setForceConstant(float kf);

    /**
     * @brief Get force constant
     * @return Force constant (N/A)
     */
    float getForceConstant() const { return forceConstant_; }

    /**
     * @brief Set position sensor range
     * @param minPos Minimum position (m)
     * @param maxPos Maximum position (m)
     */
    void setPositionRange(float minPos, float maxPos);

    /**
     * @brief Set position sensor calibration
     * @param gain Position gain (m/V or m/count)
     * @param offset Position offset (m)
     */
    void setPositionSensorCalibration(float gain, float offset);

    /**
     * @brief Set current sensor calibration
     * @param gain Current gain (A/V or A/count)
     * @param offset Current offset (A)
     */
    void setCurrentSensorCalibration(float gain, float offset);

    /**
     * @brief Set position PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPositionPid(float kp, float ki, float kd);

    /**
     * @brief Set current PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setCurrentPid(float kp, float ki, float kd);

    /**
     * @brief Enable/disable bipolar operation
     * @param bipolar true for bipolar (H-bridge), false for unipolar
     */
    void setBipolar(bool bipolar);

    /**
     * @brief Set H-bridge direction pin
     * @param dirPin Direction GPIO for H-bridge mode
     */
    void setDirectionPin(hal::IGpio* dirPin);

private:
    // Hardware
    hal::IPwm* pwm_;
    hal::IAdc* positionAdc_;
    hal::IAdc* currentAdc_;
    hal::IGpio* dirPin_;

    // Configuration
    float forceConstant_;
    float positionGain_;
    float positionOffset_;
    float currentGain_;
    float currentOffset_;
    float minPosition_;
    float maxPosition_;
    bool bipolar_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Measurements
    float position_;
    float velocity_;
    float prevPosition_;
    float current_;
    float force_;

    // Control targets
    float positionRef_;
    float forceRef_;
    float currentRef_;

    // Controllers
    struct Controllers;
    Controllers* ctrl_;

    // Internal methods
    void readPosition();
    void readCurrent();
    void runPositionLoop();
    void runCurrentLoop();
    void applyOutput(float output);
};

}  // namespace omni::driver
