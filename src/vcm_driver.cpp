/**
 * @file vcm_driver.cpp
 * @brief Voice Coil Motor driver implementation
 */

#include "omni/driver/vcm_driver.hpp"
#include <cmath>

namespace omni::driver {

VcmDriver::VcmDriver(hal::IPwm* pwm, hal::IAdc* positionAdc)
    : pwm_(pwm), positionAdc_(positionAdc), currentAdc_(nullptr)
    , enabled_(false), errorCode_(0), controlMode_(ControlMode::Position)
{
}

VcmDriver::VcmDriver(hal::IPwm* pwm, hal::IAdc* positionAdc, hal::IAdc* currentAdc)
    : pwm_(pwm), positionAdc_(positionAdc), currentAdc_(currentAdc)
    , enabled_(false), errorCode_(0), controlMode_(ControlMode::Position)
{
}

bool VcmDriver::init() {
    if (pwm_) { pwm_->setDuty(0.5f); pwm_->enable(false); }
    return true;
}

void VcmDriver::deinit() { disable(); }

void VcmDriver::enable() {
    if (pwm_) pwm_->enable(true);
    enabled_ = true;
}

void VcmDriver::disable() {
    if (pwm_) { pwm_->setDuty(0.5f); pwm_->enable(false); }
    enabled_ = false;
}

bool VcmDriver::isEnabled() const { return enabled_; }
void VcmDriver::emergencyStop() { disable(); }

void VcmDriver::setControlMode(ControlMode mode) { controlMode_ = mode; }
ControlMode VcmDriver::getControlMode() const { return controlMode_; }

void VcmDriver::setPosition(float pos) { targetPosition_ = pos; }
void VcmDriver::setVelocity(float vel) { targetVelocity_ = vel; }
void VcmDriver::setTorque(float force) { targetForce_ = force; }
void VcmDriver::setCurrent(float current) { targetCurrent_ = current; }
void VcmDriver::setVoltage(float voltage) { targetVoltage_ = voltage; }
void VcmDriver::setDuty(float duty) { targetDuty_ = duty; }

MotorState VcmDriver::getState() const { return state_; }
float VcmDriver::getPosition() const {
    return positionAdc_ ? positionAdc_->read() * positionScale_ : 0;
}
float VcmDriver::getVelocity() const { return velocity_; }
float VcmDriver::getTorque() const { return 0; }
float VcmDriver::getCurrent() const {
    return currentAdc_ ? currentAdc_->read() * currentScale_ : 0;
}
float VcmDriver::getTemperature() const { return 0; }

void VcmDriver::setParams(const MotorParams& params) { params_ = params; }
MotorParams VcmDriver::getParams() const { return params_; }

uint32_t VcmDriver::getErrorCode() const { return errorCode_; }
void VcmDriver::clearErrors() { errorCode_ = 0; }
bool VcmDriver::hasFault() const { return errorCode_ != 0; }

void VcmDriver::update(float dt) {
    if (!enabled_) return;

    float output = 0.5f;  // Middle point

    switch (controlMode_) {
        case ControlMode::Position: {
            float pos = getPosition();
            output = 0.5f + positionPid_.update(targetPosition_, pos, dt);
            break;
        }
        case ControlMode::Current: {
            float current = getCurrent();
            output = 0.5f + currentPid_.update(targetCurrent_, current, dt);
            break;
        }
        case ControlMode::Duty:
            output = 0.5f + targetDuty_ * 0.5f;
            break;
        default:
            break;
    }

    output = std::clamp(output, 0.0f, 1.0f);
    if (pwm_) pwm_->setDuty(output);
}

void VcmDriver::setPositionPid(float kp, float ki, float kd) {
    positionPid_.setGains(kp, ki, kd);
}

void VcmDriver::setCurrentPid(float kp, float ki, float kd) {
    currentPid_.setGains(kp, ki, kd);
}

}  // namespace omni::driver
