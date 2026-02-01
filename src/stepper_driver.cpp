/**
 * @file stepper_driver.cpp
 * @brief Stepper motor driver implementation
 */

#include "omni/driver/stepper_driver.hpp"
#include <cmath>

namespace omni::driver {

StepperDriver::StepperDriver(hal::IGpio* stepPin,
                             hal::IGpio* dirPin,
                             hal::IGpio* enablePin)
    : stepPin_(stepPin)
    , dirPin_(dirPin)
    , enablePin_(enablePin)
    , encoder_(nullptr)
    , controlMode_(ControlMode::Position)
    , enabled_(false)
    , errorCode_(0)
    , stepPosition_(0)
    , targetStepPosition_(0)
    , velocity_(0)
    , targetVelocity_(0)
    , stepsPerRev_(200)
    , microstep_(1)
    , directionInvert_(false)
    , stepPulseWidthNs_(1000)
    , maxStepRate_(100000)
    , lastStepTime_(0)
    , stepAccumulator_(0)
    , stepsRemaining_(0)
{
}

StepperDriver::StepperDriver(hal::IGpio* stepPin,
                             hal::IGpio* dirPin,
                             hal::IEncoder* encoder,
                             hal::IGpio* enablePin)
    : stepPin_(stepPin)
    , dirPin_(dirPin)
    , enablePin_(enablePin)
    , encoder_(encoder)
    , controlMode_(ControlMode::Position)
    , enabled_(false)
    , errorCode_(0)
    , stepPosition_(0)
    , targetStepPosition_(0)
    , velocity_(0)
    , targetVelocity_(0)
    , stepsPerRev_(200)
    , microstep_(1)
    , directionInvert_(false)
    , stepPulseWidthNs_(1000)
    , maxStepRate_(100000)
    , lastStepTime_(0)
    , stepAccumulator_(0)
    , stepsRemaining_(0)
{
}

bool StepperDriver::init() {
    if (enablePin_) {
        enablePin_->setMode(hal::PinMode::Output);
        enablePin_->write(true);  // Disabled (active low)
    }
    if (dirPin_) {
        dirPin_->setMode(hal::PinMode::Output);
        dirPin_->write(false);
    }
    if (stepPin_) {
        stepPin_->setMode(hal::PinMode::Output);
        stepPin_->write(false);
    }
    return true;
}

void StepperDriver::deinit() {
    disable();
}

void StepperDriver::enable() {
    if (enablePin_) {
        enablePin_->write(false);  // Active low
    }
    enabled_ = true;
}

void StepperDriver::disable() {
    if (enablePin_) {
        enablePin_->write(true);
    }
    enabled_ = false;
    velocity_ = 0;
    targetVelocity_ = 0;
}

bool StepperDriver::isEnabled() const {
    return enabled_;
}

void StepperDriver::emergencyStop() {
    disable();
    stepsRemaining_ = 0;
    stepAccumulator_ = 0;
}

void StepperDriver::setControlMode(ControlMode mode) {
    controlMode_ = mode;
}

ControlMode StepperDriver::getControlMode() const {
    return controlMode_;
}

void StepperDriver::setPosition(float pos) {
    targetStepPosition_ = radiansToSteps(pos);
}

void StepperDriver::setVelocity(float vel) {
    targetVelocity_ = vel;
}

void StepperDriver::setTorque(float) {
    // Not supported for stepper
}

void StepperDriver::setCurrent(float) {
    // Not directly supported
}

void StepperDriver::setVoltage(float) {
    // Not supported
}

void StepperDriver::setDuty(float) {
    // Not supported
}

MotorState StepperDriver::getState() const {
    return state_;
}

float StepperDriver::getPosition() const {
    if (encoder_) {
        return encoder_->getAngle();
    }
    return stepsToRadians(stepPosition_);
}

float StepperDriver::getVelocity() const {
    if (encoder_) {
        return encoder_->getVelocity();
    }
    return velocity_;
}

float StepperDriver::getTorque() const {
    return 0;
}

float StepperDriver::getCurrent() const {
    return 0;
}

float StepperDriver::getTemperature() const {
    return 0;
}

void StepperDriver::setParams(const MotorParams& params) {
    params_ = params;
}

MotorParams StepperDriver::getParams() const {
    return params_;
}

void StepperDriver::update(float dt) {
    if (!enabled_) return;

    // Update velocity based on target
    velocity_ = targetVelocity_;

    // Calculate step rate from velocity
    float stepsPerSec = std::fabs(velocity_) / (2.0f * 3.14159265f) *
                        stepsPerRev_ * microstep_;

    // Set direction
    bool dir = velocity_ >= 0;
    if (directionInvert_) dir = !dir;
    if (dirPin_) {
        dirPin_->write(dir);
    }

    // Accumulate steps
    if (stepsPerSec > 0) {
        stepAccumulator_ += stepsPerSec * dt;

        while (stepAccumulator_ >= 1.0f) {
            generateStep();
            stepAccumulator_ -= 1.0f;

            if (velocity_ >= 0) {
                stepPosition_++;
            } else {
                stepPosition_--;
            }
        }
    }
}

uint32_t StepperDriver::getErrorCode() const {
    return errorCode_;
}

void StepperDriver::clearErrors() {
    errorCode_ = 0;
}

bool StepperDriver::hasFault() const {
    return errorCode_ != 0;
}

void StepperDriver::setMicrostep(uint16_t microstep) {
    microstep_ = microstep;
}

void StepperDriver::setStepMode(StepMode) {
    // Implementation depends on driver
}

void StepperDriver::step(int32_t steps) {
    stepsRemaining_ = steps;
}

void StepperDriver::setStepsPerRev(uint32_t steps) {
    stepsPerRev_ = steps;
}

void StepperDriver::resetStepPosition() {
    stepPosition_ = 0;
}

void StepperDriver::setStepPosition(int32_t steps) {
    stepPosition_ = steps;
}

void StepperDriver::setMaxStepRate(uint32_t stepsPerSec) {
    maxStepRate_ = stepsPerSec;
}

void StepperDriver::setStepPulseWidth(uint32_t ns) {
    stepPulseWidthNs_ = ns;
}

void StepperDriver::setDirectionInvert(bool invert) {
    directionInvert_ = invert;
}

void StepperDriver::setStealthChop(bool enable) {
    stealthChopEnabled_ = enable;
}

void StepperDriver::setSpreadCycle(bool enable) {
    spreadCycleEnabled_ = enable;
}

void StepperDriver::setStallGuard(uint8_t threshold) {
    stallGuardThreshold_ = threshold;
}

bool StepperDriver::isStalled() const {
    return stallDetected_;
}

void StepperDriver::setRunCurrent(uint8_t percent) {
    runCurrent_ = percent;
}

void StepperDriver::setHoldCurrent(uint8_t percent) {
    holdCurrent_ = percent;
}

void StepperDriver::generateStep() {
    if (stepPin_) {
        stepPin_->write(true);
        // Brief delay (in real implementation, use timer)
        for (volatile int i = 0; i < 10; i++);
        stepPin_->write(false);
    }
}

void StepperDriver::updatePosition() {
    if (encoder_) {
        // Use encoder position
    }
}

float StepperDriver::stepsToRadians(int32_t steps) const {
    return static_cast<float>(steps) / (stepsPerRev_ * microstep_) * 2.0f * 3.14159265f;
}

int32_t StepperDriver::radiansToSteps(float radians) const {
    return static_cast<int32_t>(radians / (2.0f * 3.14159265f) * stepsPerRev_ * microstep_);
}

}  // namespace omni::driver
