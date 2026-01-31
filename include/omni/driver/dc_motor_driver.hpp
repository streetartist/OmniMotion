/**
 * @file dc_motor_driver.hpp
 * @brief Brushed DC motor driver
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"

namespace omni::driver {

/**
 * @brief Brushed DC motor driver
 *
 * Supports both H-bridge (dual PWM) and PWM+Direction configurations.
 * Provides open-loop voltage/duty control and closed-loop velocity/position
 * control when encoder feedback is available.
 */
class DcMotorDriver : public IMotorDriver {
public:
    /**
     * @brief Construct DC motor driver with H-bridge (dual PWM)
     * @param pwmA PWM for forward direction
     * @param pwmB PWM for reverse direction
     */
    DcMotorDriver(hal::IPwm* pwmA, hal::IPwm* pwmB);

    /**
     * @brief Construct DC motor driver with PWM + direction pin
     * @param pwm PWM for speed control
     * @param dir Direction GPIO
     */
    DcMotorDriver(hal::IPwm* pwm, hal::IGpio* dir);

    /**
     * @brief Construct DC motor driver with encoder feedback
     * @param pwmA PWM A (or main PWM)
     * @param pwmB PWM B (or nullptr for PWM+dir mode)
     * @param dir Direction pin (for PWM+dir mode)
     * @param encoder Encoder for feedback
     */
    DcMotorDriver(hal::IPwm* pwmA, hal::IPwm* pwmB,
                  hal::IGpio* dir, hal::IEncoder* encoder);

    ~DcMotorDriver() override = default;

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

    MotorType getType() const override { return MotorType::BrushedDC; }
    const char* getName() const override { return "Brushed DC Motor Driver"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== DC Motor Specific =====

    /**
     * @brief Set brake mode
     * @param mode Brake mode (Coast, Brake, Hold)
     */
    void setBrakeMode(BrakeMode mode);

    /**
     * @brief Get current brake mode
     * @return Brake mode
     */
    BrakeMode getBrakeMode() const { return brakeMode_; }

    /**
     * @brief Set PWM frequency
     * @param freqHz PWM frequency (Hz)
     */
    void setPwmFrequency(uint32_t freqHz);

    /**
     * @brief Set velocity PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setVelocityPid(float kp, float ki, float kd);

    /**
     * @brief Set position PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPositionPid(float kp, float ki, float kd);

    /**
     * @brief Set current sense ADC
     * @param currentAdc ADC for current sensing
     * @param gain Current sensor gain (A/V)
     * @param offset Current sensor offset (A)
     */
    void setCurrentSensor(hal::IAdc* currentAdc, float gain, float offset);

    /**
     * @brief Invert motor direction
     * @param invert true to invert
     */
    void setDirectionInvert(bool invert);

private:
    // Hardware interfaces
    hal::IPwm* pwmA_;
    hal::IPwm* pwmB_;
    hal::IGpio* dirPin_;
    hal::IEncoder* encoder_;
    hal::IAdc* currentAdc_;

    // Configuration
    bool hBridgeMode_;
    bool directionInvert_;
    BrakeMode brakeMode_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Measurements
    float position_;
    float velocity_;
    float current_;
    float currentGain_;
    float currentOffset_;

    // Control targets
    float positionRef_;
    float velocityRef_;
    float dutyRef_;

    // Controllers - forward declaration
    struct Controllers;
    Controllers* ctrl_;

    // Internal methods
    void readFeedback();
    void runVelocityLoop();
    void runPositionLoop();
    void applyDuty(float duty);
};

}  // namespace omni::driver
