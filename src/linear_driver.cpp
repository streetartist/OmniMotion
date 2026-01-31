/**
 * @file linear_driver.cpp
 * @brief Linear motor driver implementation
 */

#include "omni/driver/linear_driver.hpp"
#include "omni/control/pid_controller.hpp"
#include <cmath>

namespace omni::driver {

static constexpr float PI = 3.14159265f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float SQRT3 = 1.732050808f;

// ============================================================================
// LinearMotorDriver 基类
// ============================================================================

LinearMotorDriver::LinearMotorDriver()
    : linearScale_(1.0f)
{
}

// ============================================================================
// LinearSyncMotorDriver 实现
// ============================================================================

struct LinearSyncMotorDriver::Controllers {
    control::PIDController idPid;
    control::PIDController iqPid;
    control::PIDController velocityPid;
    control::PIDController positionPid;
};

LinearSyncMotorDriver::LinearSyncMotorDriver(hal::IPwm3Phase* pwm,
                                             hal::IAdc* currentAdcA,
                                             hal::IAdc* currentAdcB,
                                             hal::IAbsoluteEncoder* linearEncoder)
    : pwm_(pwm), adcA_(currentAdcA), adcB_(currentAdcB)
    , linearEncoder_(linearEncoder)
    , polePitch_(0.032f), forceConstant_(50.0f)
    , controlMode_(ControlMode::Position)
    , enabled_(false), errorCode_(0)
    , commutationOffset_(0)
{
    ctrl_ = new Controllers();
}

bool LinearSyncMotorDriver::init() {
    if (pwm_) pwm_->enable(false);
    ctrl_->idPid.setGains(0.5f, 10.0f, 0);
    ctrl_->iqPid.setGains(0.5f, 10.0f, 0);
    ctrl_->velocityPid.setGains(10.0f, 1.0f, 0);
    ctrl_->positionPid.setGains(50.0f, 0, 5.0f);
    return true;
}

void LinearSyncMotorDriver::deinit() { disable(); delete ctrl_; }

void LinearSyncMotorDriver::enable() {
    if (pwm_) pwm_->enable(true);
    enabled_ = true;
}

void LinearSyncMotorDriver::disable() {
    if (pwm_) { pwm_->setDuty(0, 0, 0); pwm_->enable(false); }
    enabled_ = false;
}

bool LinearSyncMotorDriver::isEnabled() const { return enabled_; }

void LinearSyncMotorDriver::emergencyStop() {
    if (pwm_) pwm_->setDuty(0, 0, 0);
    enabled_ = false;
}

void LinearSyncMotorDriver::setControlMode(ControlMode mode) { controlMode_ = mode; }
ControlMode LinearSyncMotorDriver::getControlMode() const { return controlMode_; }

void LinearSyncMotorDriver::setPosition(float pos) { positionRef_ = pos; }
void LinearSyncMotorDriver::setVelocity(float vel) { velocityRef_ = vel; }
void LinearSyncMotorDriver::setTorque(float force) { forceRef_ = force; }
void LinearSyncMotorDriver::setCurrent(float current) { iqRef_ = current; }
void LinearSyncMotorDriver::setVoltage(float) { }
void LinearSyncMotorDriver::setDuty(float) { }

void LinearSyncMotorDriver::setLinearPosition(float pos) { positionRef_ = pos; }
void LinearSyncMotorDriver::setLinearVelocity(float vel) { velocityRef_ = vel; }
void LinearSyncMotorDriver::setForce(float force) { forceRef_ = force; }

MotorState LinearSyncMotorDriver::getState() const { return state_; }
float LinearSyncMotorDriver::getPosition() const { return linearPosition_; }
float LinearSyncMotorDriver::getVelocity() const { return linearVelocity_; }
float LinearSyncMotorDriver::getTorque() const { return iq_ * forceConstant_; }
float LinearSyncMotorDriver::getCurrent() const { return std::sqrt(id_*id_ + iq_*iq_); }
float LinearSyncMotorDriver::getTemperature() const { return 0; }

float LinearSyncMotorDriver::getLinearPosition() const { return linearPosition_; }
float LinearSyncMotorDriver::getLinearVelocity() const { return linearVelocity_; }
float LinearSyncMotorDriver::getForce() const { return iq_ * forceConstant_; }

void LinearSyncMotorDriver::setParams(const MotorParams& params) { params_ = params; }
MotorParams LinearSyncMotorDriver::getParams() const { return params_; }

uint32_t LinearSyncMotorDriver::getErrorCode() const { return errorCode_; }
void LinearSyncMotorDriver::clearErrors() { errorCode_ = 0; }
bool LinearSyncMotorDriver::hasFault() const { return errorCode_ != 0; }

void LinearSyncMotorDriver::update() {
    if (!enabled_) return;

    readFeedback();
    runFocLoop();
}

void LinearSyncMotorDriver::readFeedback() {
    // 读取位置
    if (linearEncoder_) {
        linearPosition_ = linearEncoder_->getAngle() * linearScale_;
        linearVelocity_ = linearEncoder_->getVelocity() * linearScale_;
    }

    // 计算电角度
    electricalAngle_ = positionToElectricalAngle(linearPosition_) + commutationOffset_;

    // 读取电流
    ia_ = adcA_ ? adcA_->read() : 0;
    ib_ = adcB_ ? adcB_->read() : 0;
}

void LinearSyncMotorDriver::runFocLoop() {
    // Clarke 变换
    float ialpha = ia_;
    float ibeta = (ia_ + 2*ib_) / SQRT3;

    // Park 变换
    float cosTheta = std::cos(electricalAngle_);
    float sinTheta = std::sin(electricalAngle_);
    id_ = ialpha * cosTheta + ibeta * sinTheta;
    iq_ = -ialpha * sinTheta + ibeta * cosTheta;

    // 计算目标电流
    float targetIq = 0;
    switch (controlMode_) {
        case ControlMode::Position:
            targetIq = ctrl_->positionPid.compute(positionRef_ - linearPosition_);
            break;
        case ControlMode::Velocity:
            targetIq = ctrl_->velocityPid.compute(velocityRef_ - linearVelocity_);
            break;
        case ControlMode::Torque:
            targetIq = forceRef_ / forceConstant_;
            break;
        case ControlMode::Current:
            targetIq = iqRef_;
            break;
        default:
            break;
    }

    // 电流环
    float vd = ctrl_->idPid.compute(0 - id_);
    float vq = ctrl_->iqPid.compute(targetIq - iq_);

    // 逆 Park 变换
    float valpha = vd * cosTheta - vq * sinTheta;
    float vbeta = vd * sinTheta + vq * cosTheta;

    // SVPWM
    float va = valpha;
    float vb = -0.5f * valpha + SQRT3 * 0.5f * vbeta;
    float vc = -0.5f * valpha - SQRT3 * 0.5f * vbeta;

    float vmin = std::min({va, vb, vc});
    float vmax = std::max({va, vb, vc});
    float voffset = (vmin + vmax) * 0.5f;

    float da = std::clamp(va - voffset + 0.5f, 0.0f, 1.0f);
    float db = std::clamp(vb - voffset + 0.5f, 0.0f, 1.0f);
    float dc = std::clamp(vc - voffset + 0.5f, 0.0f, 1.0f);

    if (pwm_) pwm_->setDuty(da, db, dc);
}

float LinearSyncMotorDriver::positionToElectricalAngle(float pos) const {
    return std::fmod(pos / polePitch_ * TWO_PI, TWO_PI);
}

void LinearSyncMotorDriver::setPolePitch(float pitch) { polePitch_ = pitch; }
void LinearSyncMotorDriver::setForceConstant(float kf) { forceConstant_ = kf; }

bool LinearSyncMotorDriver::calibrateCommutation() {
    // 简单校准：施加 d 轴电流，记录位置
    // 实际应用需要更复杂的校准流程
    commutationOffset_ = 0;
    return true;
}

}  // namespace omni::driver
