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
    , polePairs_(7), electricalAngle_(0)
{
}

BldcDriver::BldcDriver(hal::IPwm3Phase* pwm,
                       hal::IAdc* currentAdcA,
                       hal::IAdc* currentAdcB,
                       hal::IHallSensor* hall)
    : pwm_(pwm), currentAdcA_(currentAdcA), currentAdcB_(currentAdcB)
    , currentAdcC_(nullptr), encoder_(nullptr), hall_(hall)
    , controlMode_(ControlMode::Voltage), enabled_(false), errorCode_(0)
    , polePairs_(7), electricalAngle_(0)
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
void BldcDriver::setTorque(float torque) { targetIq_ = torque / params_.torqueConstant; }
void BldcDriver::setCurrent(float current) { targetIq_ = current; }
void BldcDriver::setVoltage(float voltage) { targetVoltage_ = voltage; }
void BldcDriver::setDuty(float duty) { targetDuty_ = duty; }

MotorState BldcDriver::getState() const { return state_; }
float BldcDriver::getPosition() const { return encoder_ ? encoder_->getAngle() : 0; }
float BldcDriver::getVelocity() const { return encoder_ ? encoder_->getVelocity() : 0; }
float BldcDriver::getTorque() const { return iq_ * params_.torqueConstant; }
float BldcDriver::getCurrent() const { return std::sqrt(id_*id_ + iq_*iq_); }
float BldcDriver::getTemperature() const { return 0; }

void BldcDriver::setParams(const MotorParams& params) { params_ = params; }
MotorParams BldcDriver::getParams() const { return params_; }

uint32_t BldcDriver::getErrorCode() const { return errorCode_; }
void BldcDriver::clearErrors() { errorCode_ = 0; }
bool BldcDriver::hasFault() const { return errorCode_ != 0; }

void BldcDriver::update() {
    if (!enabled_) return;

    // 1. 读取电角度
    if (encoder_) {
        electricalAngle_ = std::fmod(encoder_->getAngle() * polePairs_, TWO_PI);
    }

    // 2. 读取相电流
    float ia = currentAdcA_ ? currentAdcA_->read() * currentGain_ : 0;
    float ib = currentAdcB_ ? currentAdcB_->read() * currentGain_ : 0;
    float ic = -ia - ib;  // Kirchhoff

    // 3. Clarke 变换 (abc -> αβ)
    float ialpha = ia;
    float ibeta = (ia + 2*ib) / SQRT3;

    // 4. Park 变换 (αβ -> dq)
    float cosTheta = std::cos(electricalAngle_);
    float sinTheta = std::sin(electricalAngle_);
    id_ = ialpha * cosTheta + ibeta * sinTheta;
    iq_ = -ialpha * sinTheta + ibeta * cosTheta;

    // 5. 计算目标电流
    float vd = 0, vq = 0;
    switch (controlMode_) {
        case ControlMode::Current:
            vd = idPid_.compute(targetId_ - id_);
            vq = iqPid_.compute(targetIq_ - iq_);
            break;
        case ControlMode::Voltage:
            vd = 0;
            vq = targetVoltage_;
            break;
        default:
            vd = 0;
            vq = targetDuty_;
            break;
    }

    // 6. 逆 Park 变换 (dq -> αβ)
    float valpha = vd * cosTheta - vq * sinTheta;
    float vbeta = vd * sinTheta + vq * cosTheta;

    // 7. SVPWM
    setSvpwm(valpha, vbeta);
}

void BldcDriver::setSvpwm(float valpha, float vbeta) {
    // 逆 Clarke 变换
    float va = valpha;
    float vb = -0.5f * valpha + SQRT3 * 0.5f * vbeta;
    float vc = -0.5f * valpha - SQRT3 * 0.5f * vbeta;

    // 归一化到 0-1
    float vmin = std::min({va, vb, vc});
    float vmax = std::max({va, vb, vc});
    float voffset = (vmin + vmax) * 0.5f;

    float da = (va - voffset + 0.5f);
    float db = (vb - voffset + 0.5f);
    float dc = (vc - voffset + 0.5f);

    // 限幅
    da = std::clamp(da, 0.0f, 1.0f);
    db = std::clamp(db, 0.0f, 1.0f);
    dc = std::clamp(dc, 0.0f, 1.0f);

    if (pwm_) pwm_->setDuty(da, db, dc);
}

void BldcDriver::setPolePairs(uint8_t pp) { polePairs_ = pp; }
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
