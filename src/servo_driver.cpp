/**
 * @file servo_driver.cpp
 * @brief RC servo motor driver implementation
 */

#include "omni/driver/servo_driver.hpp"
#include <cmath>

namespace omni::driver {

static constexpr float PI = 3.14159265f;

ServoDriver::ServoDriver(hal::IPwm* pwm)
    : pwm_(pwm), enabled_(false), errorCode_(0)
    , minPulseUs_(1000), maxPulseUs_(2000), frequency_(50)
    , minAngle_(-PI/2), maxAngle_(PI/2), currentAngle_(0)
{
}

bool ServoDriver::init() {
    if (pwm_) {
        pwm_->setFrequency(frequency_);
        pwm_->enable(false);
    }
    return true;
}

void ServoDriver::deinit() { disable(); }

void ServoDriver::enable() {
    if (pwm_) pwm_->enable(true);
    enabled_ = true;
}

void ServoDriver::disable() {
    if (pwm_) pwm_->enable(false);
    enabled_ = false;
}

bool ServoDriver::isEnabled() const { return enabled_; }
void ServoDriver::emergencyStop() { disable(); }

void ServoDriver::setControlMode(ControlMode mode) { controlMode_ = mode; }
ControlMode ServoDriver::getControlMode() const { return controlMode_; }

void ServoDriver::setPosition(float pos) {
    targetAngle_ = std::clamp(pos, minAngle_, maxAngle_);
}

void ServoDriver::setVelocity(float) { }
void ServoDriver::setTorque(float) { }
void ServoDriver::setCurrent(float) { }
void ServoDriver::setVoltage(float) { }
void ServoDriver::setDuty(float) { }

MotorState ServoDriver::getState() const { return state_; }
float ServoDriver::getPosition() const { return currentAngle_; }
float ServoDriver::getVelocity() const { return 0; }
float ServoDriver::getTorque() const { return 0; }
float ServoDriver::getCurrent() const { return 0; }
float ServoDriver::getTemperature() const { return 0; }

void ServoDriver::setParams(const MotorParams& params) { params_ = params; }
MotorParams ServoDriver::getParams() const { return params_; }

uint32_t ServoDriver::getErrorCode() const { return errorCode_; }
void ServoDriver::clearErrors() { errorCode_ = 0; }
bool ServoDriver::hasFault() const { return errorCode_ != 0; }

void ServoDriver::update() {
    if (!enabled_) return;

    currentAngle_ = targetAngle_;

    // 角度转脉宽
    float normalized = (currentAngle_ - minAngle_) / (maxAngle_ - minAngle_);
    float pulseUs = minPulseUs_ + normalized * (maxPulseUs_ - minPulseUs_);

    // 脉宽转占空比
    float periodUs = 1000000.0f / frequency_;
    float duty = pulseUs / periodUs;

    if (pwm_) pwm_->setDuty(duty);
}

void ServoDriver::setPulseRange(uint32_t minUs, uint32_t maxUs) {
    minPulseUs_ = minUs;
    maxPulseUs_ = maxUs;
}

void ServoDriver::setAngleRange(float minRad, float maxRad) {
    minAngle_ = minRad;
    maxAngle_ = maxRad;
}

void ServoDriver::setAngleDeg(float degrees) {
    setPosition(degrees * PI / 180.0f);
}

}  // namespace omni::driver
