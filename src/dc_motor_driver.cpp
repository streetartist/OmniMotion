/**
 * @file dc_motor_driver.cpp
 * @brief Brushed DC motor driver implementation
 */

#include "omni/driver/dc_motor_driver.hpp"
#include <cmath>

namespace omni::driver {

DcMotorDriver::DcMotorDriver(hal::IPwm* pwmA, hal::IPwm* pwmB)
    : pwmA_(pwmA), pwmB_(pwmB), dirPin_(nullptr), encoder_(nullptr)
    , currentAdc_(nullptr), hBridgeMode_(true), directionInvert_(false)
    , brakeMode_(BrakeMode::Coast), controlMode_(ControlMode::Duty)
    , enabled_(false), errorCode_(0)
{
}

DcMotorDriver::DcMotorDriver(hal::IPwm* pwm, hal::IGpio* dir)
    : pwmA_(pwm), pwmB_(nullptr), dirPin_(dir), encoder_(nullptr)
    , currentAdc_(nullptr), hBridgeMode_(false), directionInvert_(false)
    , brakeMode_(BrakeMode::Coast), controlMode_(ControlMode::Duty)
    , enabled_(false), errorCode_(0)
{
}

DcMotorDriver::DcMotorDriver(hal::IPwm* pwmA, hal::IPwm* pwmB,
                             hal::IGpio* dir, hal::IEncoder* encoder)
    : pwmA_(pwmA), pwmB_(pwmB), dirPin_(dir), encoder_(encoder)
    , currentAdc_(nullptr), hBridgeMode_(pwmB != nullptr)
    , directionInvert_(false), brakeMode_(BrakeMode::Coast)
    , controlMode_(ControlMode::Duty), enabled_(false), errorCode_(0)
{
}

bool DcMotorDriver::init() {
    if (pwmA_) { pwmA_->setDuty(0); pwmA_->enable(false); }
    if (pwmB_) { pwmB_->setDuty(0); pwmB_->enable(false); }
    if (dirPin_) { dirPin_->setMode(hal::PinMode::Output); dirPin_->write(false); }
    return true;
}

void DcMotorDriver::deinit() { disable(); }

void DcMotorDriver::enable() {
    if (pwmA_) pwmA_->enable(true);
    if (pwmB_) pwmB_->enable(true);
    enabled_ = true;
}

void DcMotorDriver::disable() {
    applyDuty(0);
    if (pwmA_) pwmA_->enable(false);
    if (pwmB_) pwmB_->enable(false);
    enabled_ = false;
}

bool DcMotorDriver::isEnabled() const { return enabled_; }

void DcMotorDriver::emergencyStop() {
    if (hBridgeMode_) { pwmA_->setDuty(1.0f); pwmB_->setDuty(1.0f); }
    else { pwmA_->setDuty(0); }
    enabled_ = false;
}

void DcMotorDriver::setControlMode(ControlMode mode) { controlMode_ = mode; }
ControlMode DcMotorDriver::getControlMode() const { return controlMode_; }

void DcMotorDriver::setPosition(float pos) { targetPosition_ = pos; }
void DcMotorDriver::setVelocity(float vel) { targetVelocity_ = vel; }
void DcMotorDriver::setTorque(float) { }
void DcMotorDriver::setCurrent(float) { }
void DcMotorDriver::setVoltage(float voltage) { setDuty(voltage / params_.nominalVoltage); }
void DcMotorDriver::setDuty(float duty) { targetDuty_ = std::clamp(duty, -1.0f, 1.0f); }

MotorState DcMotorDriver::getState() const { return state_; }
float DcMotorDriver::getPosition() const { return encoder_ ? encoder_->getAngle() : position_; }
float DcMotorDriver::getVelocity() const { return encoder_ ? encoder_->getVelocity() : velocity_; }
float DcMotorDriver::getTorque() const { return 0; }
float DcMotorDriver::getCurrent() const { return current_; }
float DcMotorDriver::getTemperature() const { return 0; }

void DcMotorDriver::setParams(const MotorParams& params) { params_ = params; }
MotorParams DcMotorDriver::getParams() const { return params_; }

uint32_t DcMotorDriver::getErrorCode() const { return errorCode_; }
void DcMotorDriver::clearErrors() { errorCode_ = 0; }
bool DcMotorDriver::hasFault() const { return errorCode_ != 0; }

void DcMotorDriver::update() {
    if (!enabled_) return;

    float duty = targetDuty_;

    switch (controlMode_) {
        case ControlMode::Position:
            if (encoder_) {
                float posError = targetPosition_ - encoder_->getAngle();
                duty = positionPid_.compute(posError);
            }
            break;
        case ControlMode::Velocity:
            if (encoder_) {
                float velError = targetVelocity_ - encoder_->getVelocity();
                duty = velocityPid_.compute(velError);
            }
            break;
        default:
            duty = targetDuty_;
            break;
    }

    applyDuty(duty);
}

void DcMotorDriver::applyDuty(float duty) {
    if (directionInvert_) duty = -duty;
    duty = std::clamp(duty, -1.0f, 1.0f);

    if (hBridgeMode_) {
        if (duty >= 0) {
            pwmA_->setDuty(duty);
            pwmB_->setDuty(0);
        } else {
            pwmA_->setDuty(0);
            pwmB_->setDuty(-duty);
        }
    } else {
        dirPin_->write(duty >= 0);
        pwmA_->setDuty(std::fabs(duty));
    }
}

void DcMotorDriver::setBrakeMode(BrakeMode mode) { brakeMode_ = mode; }
void DcMotorDriver::setPwmFrequency(uint32_t freq) { if (pwmA_) pwmA_->setFrequency(freq); }
void DcMotorDriver::setVelocityPid(float kp, float ki, float kd) { velocityPid_.setGains(kp, ki, kd); }
void DcMotorDriver::setPositionPid(float kp, float ki, float kd) { positionPid_.setGains(kp, ki, kd); }
void DcMotorDriver::setCurrentSensor(hal::IAdc* adc, float gain, float offset) {
    currentAdc_ = adc; currentGain_ = gain; currentOffset_ = offset;
}
void DcMotorDriver::setDirectionInvert(bool invert) { directionInvert_ = invert; }

}  // namespace omni::driver
