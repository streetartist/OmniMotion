/**
 * @file servo_driver.hpp
 * @brief RC servo motor driver
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"

namespace omni::driver {

/**
 * @brief RC Servo motor driver
 *
 * Controls RC hobby servos using PWM pulse width modulation.
 * Standard servos use 1-2ms pulses at 50Hz, with 1.5ms being center.
 */
class ServoDriver : public IMotorDriver {
public:
    /**
     * @brief Construct servo driver
     * @param pwm PWM interface for servo control
     */
    explicit ServoDriver(hal::IPwm* pwm);

    ~ServoDriver() override = default;

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

    MotorType getType() const override { return MotorType::RCServo; }
    const char* getName() const override { return "RC Servo Driver"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== Servo Specific =====

    /**
     * @brief Set angle in degrees
     * @param angle Angle in degrees
     */
    void setAngle(float angle);

    /**
     * @brief Get current angle in degrees
     * @return Angle in degrees
     */
    float getAngle() const;

    /**
     * @brief Set pulse width range
     * @param minUs Minimum pulse width (microseconds)
     * @param maxUs Maximum pulse width (microseconds)
     */
    void setPulseRange(uint16_t minUs, uint16_t maxUs);

    /**
     * @brief Get pulse range
     * @param minUs Output minimum pulse
     * @param maxUs Output maximum pulse
     */
    void getPulseRange(uint16_t& minUs, uint16_t& maxUs) const;

    /**
     * @brief Set angle range
     * @param minDeg Minimum angle (degrees)
     * @param maxDeg Maximum angle (degrees)
     */
    void setAngleRange(float minDeg, float maxDeg);

    /**
     * @brief Get angle range
     * @param minDeg Output minimum angle
     * @param maxDeg Output maximum angle
     */
    void getAngleRange(float& minDeg, float& maxDeg) const;

    /**
     * @brief Set PWM frequency
     * @param freqHz PWM frequency (typically 50Hz for standard servos)
     */
    void setPwmFrequency(uint32_t freqHz);

    /**
     * @brief Set pulse width directly
     * @param us Pulse width in microseconds
     */
    void setPulseWidth(uint16_t us);

    /**
     * @brief Get current pulse width
     * @return Pulse width in microseconds
     */
    uint16_t getPulseWidth() const;

    /**
     * @brief Set center trim offset
     * @param us Trim offset in microseconds
     */
    void setTrim(int16_t us);

    /**
     * @brief Invert servo direction
     * @param invert true to invert
     */
    void setInvert(bool invert);

    /**
     * @brief Enable position feedback (for digital servos with feedback)
     * @param feedbackAdc ADC for position feedback
     */
    void setFeedback(hal::IAdc* feedbackAdc);

private:
    // Hardware
    hal::IPwm* pwm_;
    hal::IAdc* feedbackAdc_;

    // Configuration
    uint16_t minPulseUs_;
    uint16_t maxPulseUs_;
    float minAngleDeg_;
    float maxAngleDeg_;
    int16_t trimUs_;
    bool inverted_;
    uint32_t pwmFreqHz_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Current values
    float currentAngle_;
    uint16_t currentPulseUs_;

    // Internal methods
    uint16_t angleToPulse(float angle) const;
    float pulseToAngle(uint16_t pulse) const;
    void updatePwm();
};

}  // namespace omni::driver
