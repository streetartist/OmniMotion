/**
 * @file bldc_driver.cpp
 * @brief BLDC/PMSM motor driver implementation with FOC
 */

#include "omni/driver/bldc_driver.hpp"
#include <cmath>

namespace omni::driver {

static constexpr float PI = 3.14159265f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float SQRT3 = 1.732050808f;

BldcDriver::BldcDriver(hal::IPwm3Phase* pwm,
                       hal::IAdc* currentAdcA,
                       hal::IAdc* currentAdcB,
                       hal::IAdc* currentAdcC,
                       hal::IEncoder* encoder)
    : pwm_(pwm), currentAdcA_(currentAdcA), currentAdcB_(currentAdcB)
    , currentAdcC_(currentAdcC), encoder_(encoder), hall_(nullptr)
    , controlMode_(ControlMode::Voltage), enabled_(false), errorCode_(0)
    , polePairs_(7), electricalAngle_(0), currentGain_(1.0f)
    , id_(0), iq_(0)
    , targetPosition_(0), targetVelocity_(0), targetIq_(0), targetId_(0), targetVoltage_(0), targetDuty_(0)
{
}

BldcDriver::BldcDriver(hal::IPwm3Phase* pwm,
                       hal::IAdc* currentAdcA,
                       hal::IAdc* currentAdcB,
                       hal::IHallSensor* hall)
    : pwm_(pwm), currentAdcA_(currentAdcA), currentAdcB_(currentAdcB)
    , currentAdcC_(nullptr), encoder_(nullptr), hall_(hall)
    , controlMode_(ControlMode::Voltage), enabled_(false), errorCode_(0)
    , polePairs_(7), electricalAngle_(0), currentGain_(1.0f)
    , id_(0), iq_(0)
    , targetPosition_(0), targetVelocity_(0), targetIq_(0), targetId_(0), targetVoltage_(0), targetDuty_(0)
{
}

bool BldcDriver::init() {
    if (pwm_) pwm_->enable(false);
    return true;
}

void BldcDriver::deinit() { disable(); }

void BldcDriver::enable() {
    if (pwm_) pwm_->enable(true);
    enabled_ = true;
    // Reset controllers
    idPid_.reset();
    iqPid_.reset();
    velocityPid_.reset();
    positionPid_.reset();
}

void BldcDriver::disable() {
    if (pwm_) { pwm_->setDuty(0, 0, 0); pwm_->enable(false); }
    enabled_ = false;
}

bool BldcDriver::isEnabled() const { return enabled_; }

void BldcDriver::emergencyStop() {
    if (pwm_) pwm_->setDuty(0, 0, 0);
    enabled_ = false;
}

void BldcDriver::setControlMode(ControlMode mode) { controlMode_ = mode; }
ControlMode BldcDriver::getControlMode() const { return controlMode_; }

void BldcDriver::setPosition(float pos) { targetPosition_ = pos; }
void BldcDriver::setVelocity(float vel) { targetVelocity_ = vel; }
void BldcDriver::setTorque(float torque) { 
    float kt = params_.torqueConstant > 0 ? params_.torqueConstant : 1.0f;
    targetIq_ = torque / kt; 
}
void BldcDriver::setCurrent(float current) { targetIq_ = current; }
void BldcDriver::setVoltage(float voltage) { targetVoltage_ = voltage; }
void BldcDriver::setDuty(float duty) { targetDuty_ = duty; }

MotorState BldcDriver::getState() const { return state_; }
float BldcDriver::getPosition() const { return encoder_ ? encoder_->getAngle() : 0; }
float BldcDriver::getVelocity() const { return encoder_ ? encoder_->getVelocity() : 0; }
float BldcDriver::getTorque() const { return iq_ * params_.torqueConstant; }
float BldcDriver::getCurrent() const { return std::sqrt(id_*id_ + iq_*iq_); }
float BldcDriver::getTemperature() const { return 0; }

void BldcDriver::setParams(const MotorParams& params) { 
    params_ = params; 
    polePairs_ = params.polePairs;
}
MotorParams BldcDriver::getParams() const { return params_; }

uint32_t BldcDriver::getErrorCode() const { return errorCode_; }
void BldcDriver::clearErrors() { errorCode_ = 0; }
bool BldcDriver::hasFault() const { return errorCode_ != 0; }

void BldcDriver::update(float dt) {
    if (!enabled_) return;

    // 1. Read Feedback
    float mechanicalAngle = 0;
    float velocity = 0;
    if (encoder_) {
        // Encoder update should probably be done outside or by HAL, 
        // but if the interface requires driving it:
        encoder_->update(dt);
        mechanicalAngle = encoder_->getAngle();
        velocity = encoder_->getVelocity();
    }
    
    // Update electrical angle
    electricalAngle_ = std::fmod(mechanicalAngle * polePairs_, TWO_PI);
    if (electricalAngle_ < 0) electricalAngle_ += TWO_PI;

    // 2. Read Currents
    float ia = currentAdcA_ ? currentAdcA_->read() * currentGain_ : 0;
    float ib = currentAdcB_ ? currentAdcB_->read() * currentGain_ : 0;
    // float ic = -ia - ib;  // Kirchhoff

    // 3. Clarke Transform (abc -> alpha/beta)
    float ialpha = ia;
    float ibeta = (ia + 2*ib) / SQRT3;

    // 4. Park Transform (alpha/beta -> dq)
    float cosTheta = std::cos(electricalAngle_);
    float sinTheta = std::sin(electricalAngle_);
    id_ = ialpha * cosTheta + ibeta * sinTheta;
    iq_ = -ialpha * sinTheta + ibeta * cosTheta;

    // 5. Cascade Control
    float targetIq = targetIq_;
    float targetId = targetId_;

    // Handle upper loops
    switch (controlMode_) {
        case ControlMode::Position:
            targetVelocity_ = positionPid_.update(targetPosition_, mechanicalAngle, dt);
            [[fallthrough]];
        case ControlMode::Velocity:
            targetIq = velocityPid_.update(targetVelocity_, velocity, dt);
            [[fallthrough]];
        case ControlMode::Torque:
        case ControlMode::Current:
            // targetIq is set
            break;
        default:
            break;
    }

    // 6. Current Loop or Voltage Generation
    float vd = 0, vq = 0;

    if (controlMode_ == ControlMode::Voltage) {
        vd = 0;
        vq = targetVoltage_;
    } else if (controlMode_ == ControlMode::Duty) {
        float vBus = params_.maxVoltage > 0 ? params_.maxVoltage : 12.0f;
        vd = 0;
        vq = targetDuty_ * vBus;
    } else {
        // Current Control
        vd = idPid_.update(targetId, id_, dt);
        vq = iqPid_.update(targetIq, iq_, dt);
    }

    // 7. Inverse Park (dq -> alpha/beta)
    float valpha = vd * cosTheta - vq * sinTheta;
    float vbeta = vd * sinTheta + vq * cosTheta;

    // 8. SVPWM
    setSvpwm(valpha, vbeta);
}

void BldcDriver::setSvpwm(float valpha, float vbeta) {
    // Inverse Clarke
    float va = valpha;
    float vb = -0.5f * valpha + SQRT3 * 0.5f * vbeta;
    float vc = -0.5f * valpha - SQRT3 * 0.5f * vbeta;

    // SVPWM centering
    float vmin = std::min({va, vb, vc});
    float vmax = std::max({va, vb, vc});
    float voffset = -0.5f * (vmin + vmax);

    float da = va + voffset;
    float db = vb + voffset;
    float dc = vc + voffset;

    // Normalize to 0-1 (Duty Cycle)
    float vBus = params_.maxVoltage > 0 ? params_.maxVoltage : 24.0f;
    
    da = da / vBus + 0.5f;
    db = db / vBus + 0.5f;
    dc = dc / vBus + 0.5f;

    // Clamp
    da = std::clamp(da, 0.0f, 1.0f);
    db = std::clamp(db, 0.0f, 1.0f);
    dc = std::clamp(dc, 0.0f, 1.0f);

    if (pwm_) pwm_->setDuty(da, db, dc);
}

void BldcDriver::setFocMode(FocMode mode) { focMode_ = mode; } // Not heavily used yet

void BldcDriver::setDeadTime(uint32_t ns) { (void)ns; }

void BldcDriver::setCurrentPid(float kp, float ki, float kd) {
    idPid_.setGains(kp, ki, kd);
    iqPid_.setGains(kp, ki, kd);
}
void BldcDriver::setVelocityPid(float kp, float ki, float kd) {
    velocityPid_.setGains(kp, ki, kd);
}
void BldcDriver::setPositionPid(float kp, float ki, float kd) {
    positionPid_.setGains(kp, ki, kd);
}

}  // namespace omni::driver
