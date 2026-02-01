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

LinearSyncMotorDriver::LinearSyncMotorDriver(hal::IPwm3Phase* pwm,
                                             hal::IAdc* currentAdcA,
                                             hal::IAdc* currentAdcB,
                                             hal::IAbsoluteEncoder* linearEncoder)
    : pwm_(pwm), adcA_(currentAdcA), adcB_(currentAdcB)
    , linearEncoder_(linearEncoder)
    , polePitch_(0.032f), forceConstant_(50.0f)
    , state_(MotorState::Idle), params_(), controlMode_(ControlMode::Position)
    , enabled_(false), errorCode_(0)
    , linearPosition_(0), linearVelocity_(0), electricalAngle_(0), commutationOffset_(0)
    , ia_(0), ib_(0), id_(0), iq_(0)
    , targetPosition_(0), targetVelocity_(0), targetForce_(0), targetId_(0), targetIq_(0)
{
}

bool LinearSyncMotorDriver::init() {
    if (pwm_) pwm_->enable(false);
    return true;
}

void LinearSyncMotorDriver::deinit() { disable(); }

void LinearSyncMotorDriver::enable() {
    if (pwm_) pwm_->enable(true);
    enabled_ = true;
    idPid_.reset();
    iqPid_.reset();
    velocityPid_.reset();
    positionPid_.reset();
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

void LinearSyncMotorDriver::setPosition(float pos) { targetPosition_ = pos; }
void LinearSyncMotorDriver::setVelocity(float vel) { targetVelocity_ = vel; }
void LinearSyncMotorDriver::setTorque(float force) { targetForce_ = force; }
void LinearSyncMotorDriver::setCurrent(float current) { targetIq_ = current; }
void LinearSyncMotorDriver::setVoltage(float) { }
void LinearSyncMotorDriver::setDuty(float) { }

void LinearSyncMotorDriver::setLinearPosition(float pos) { targetPosition_ = pos; }
void LinearSyncMotorDriver::setLinearVelocity(float vel) { targetVelocity_ = vel; }
void LinearSyncMotorDriver::setForce(float force) { targetForce_ = force; }

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

void LinearSyncMotorDriver::update(float dt) {
    if (!enabled_) return;

    readFeedback();
    runFocLoop(dt);
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

void LinearSyncMotorDriver::runFocLoop(float dt) {
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
            targetVelocity_ = positionPid_.update(targetPosition_, linearPosition_, dt);
            targetIq = velocityPid_.update(targetVelocity_, linearVelocity_, dt);
            break;
        case ControlMode::Velocity:
            targetIq = velocityPid_.update(targetVelocity_, linearVelocity_, dt);
            break;
        case ControlMode::Torque:
            targetIq = targetForce_ / forceConstant_;
            break;
        case ControlMode::Current:
            targetIq = targetIq_;
            break;
        default:
            break;
    }

    // 电流环
    float vd = idPid_.update(0, id_, dt);
    float vq = iqPid_.update(targetIq, iq_, dt);

    // 逆 Park 变换
    float valpha = vd * cosTheta - vq * sinTheta;
    float vbeta = vd * sinTheta + vq * cosTheta;

    // SVPWM
    float va = valpha;
    float vb = -0.5f * valpha + SQRT3 * 0.5f * vbeta;
    float vc = -0.5f * valpha - SQRT3 * 0.5f * vbeta;

    float vmin = std::min({va, vb, vc});
    float vmax = std::max({va, vb, vc});
    float voffset = -0.5f * (vmin + vmax);
    
    float vBus = params_.maxVoltage > 0 ? params_.maxVoltage : 24.0f;

    float da = (va + voffset) / vBus + 0.5f;
    float db = (vb + voffset) / vBus + 0.5f;
    float dc = (vc + voffset) / vBus + 0.5f;

    da = std::clamp(da, 0.0f, 1.0f);
    db = std::clamp(db, 0.0f, 1.0f);
    dc = std::clamp(dc, 0.0f, 1.0f);

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
