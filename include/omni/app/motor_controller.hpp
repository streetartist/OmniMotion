/**
 * @file motor_controller.hpp
 * @brief High-level motor controller
 */

#pragma once

#include "omni/driver/motor_driver.hpp"
#include "omni/motion/online_planner.hpp"
#include "omni/motion/scurve_profile.hpp"
#include "omni/motion/trapezoidal_profile.hpp"
#include "omni/control/cascade_pid.hpp"
#include "omni/control/feedforward.hpp"
#include <memory>

namespace omni::app {

/**
 * @brief Homing mode
 */
enum class HomingMode {
    SensorBased,    // Use limit sensor
    CurrentBased,   // Detect current spike
    IndexPulse,     // Use encoder index pulse
    Hardstop,       // Run until mechanical stop
    AbsoluteEncoder, // Use absolute encoder (no motion needed)
    Manual          // User sets home position
};

/**
 * @brief High-level motor controller
 *
 * Provides a user-friendly interface for motor control with
 * trajectory planning, multiple control modes, and safety features.
 */
class MotorController {
public:
    /**
     * @brief Construct motor controller
     * @param driver Motor driver to control
     */
    explicit MotorController(driver::IMotorDriver* driver)
        : driver_(driver)
        , constraints_{10.0f, 100.0f, 0, 1000.0f, 0}
        , planner_(constraints_)
        , enabled_(false)
        , homed_(false)
        , homeOffset_(0)
        , softLimitEnabled_(false)
        , softLimitMin_(-1e6f)
        , softLimitMax_(1e6f) {
        // Configure default PID
        control::PidController::Params posParams;
        posParams.kp = 50.0f;
        posParams.ki = 0.1f;
        posParams.kd = 0.5f;
        posParams.outputLimit = constraints_.maxVelocity;

        control::PidController::Params velParams;
        velParams.kp = 0.5f;
        velParams.ki = 0.01f;
        velParams.outputLimit = 10.0f;

        cascadePid_.setPositionPid(posParams);
        cascadePid_.setVelocityPid(velParams);
    }

    // === Basic Operations ===

    /**
     * @brief Enable the motor
     */
    void enable() {
        if (driver_) {
            driver_->enable();
            enabled_ = true;
        }
    }

    /**
     * @brief Disable the motor
     */
    void disable() {
        if (driver_) {
            driver_->disable();
            enabled_ = false;
        }
    }

    /**
     * @brief Check if enabled
     */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Emergency stop
     */
    void emergencyStop() {
        if (driver_) {
            driver_->emergencyStop();
            planner_.reset(driver_->getPosition());
        }
    }

    // === Position Control ===

    /**
     * @brief Move to absolute position
     * @param position Target position
     */
    void moveTo(float position) {
        if (!enabled_) return;
        if (!checkSoftLimits(position)) return;

        planner_.setTarget(position);
        driver_->setControlMode(driver::ControlMode::Position);
    }

    /**
     * @brief Move to position with specified velocity
     * @param position Target position
     * @param velocity Maximum velocity
     */
    void moveTo(float position, float velocity) {
        motion::MotionConstraints temp = constraints_;
        temp.maxVelocity = velocity;
        planner_.setConstraints(temp);
        moveTo(position);
        planner_.setConstraints(constraints_);  // Restore
    }

    /**
     * @brief Move to position with specified velocity and acceleration
     */
    void moveTo(float position, float velocity, float accel) {
        motion::MotionConstraints temp = constraints_;
        temp.maxVelocity = velocity;
        temp.maxAcceleration = accel;
        planner_.setConstraints(temp);
        moveTo(position);
        planner_.setConstraints(constraints_);
    }

    /**
     * @brief Move relative to current position
     * @param distance Distance to move
     */
    void moveBy(float distance) {
        moveTo(getPosition() + distance);
    }

    // === Profile Motion ===

    /**
     * @brief Move with custom trajectory profile
     * @param position Target position
     * @param profile Trajectory generator
     */
    void moveWithProfile(float position, motion::ITrajectoryGenerator* profile) {
        (void)profile;  // TODO: Implement custom profile support
        moveTo(position);
    }

    /**
     * @brief Move with S-curve profile
     * @param position Target position
     * @param maxVel Maximum velocity
     * @param maxAccel Maximum acceleration
     * @param maxJerk Maximum jerk
     */
    void moveWithSCurve(float position, float maxVel, float maxAccel, float maxJerk) {
        motion::MotionConstraints c{maxVel, maxAccel, 0, maxJerk, 0};
        planner_.setConstraints(c);
        moveTo(position);
    }

    /**
     * @brief Move with trapezoidal profile
     * @param position Target position
     * @param maxVel Maximum velocity
     * @param maxAccel Maximum acceleration
     */
    void moveWithTrapezoid(float position, float maxVel, float maxAccel) {
        // Use high jerk for near-trapezoidal profile
        moveWithSCurve(position, maxVel, maxAccel, maxAccel * 100.0f);
    }

    // === Velocity Control ===

    /**
     * @brief Set velocity
     * @param velocity Target velocity
     */
    void setVelocity(float velocity) {
        if (!enabled_) return;
        driver_->setControlMode(driver::ControlMode::Velocity);
        driver_->setVelocity(velocity);
    }

    /**
     * @brief Set velocity with acceleration limit
     */
    void setVelocity(float velocity, float accel) {
        (void)accel;  // TODO: Implement velocity ramping
        setVelocity(velocity);
    }

    /**
     * @brief Ramp to target velocity
     * @param targetVel Target velocity
     * @param rampTime Time to reach target
     */
    void rampVelocity(float targetVel, float rampTime) {
        (void)rampTime;  // TODO: Implement
        setVelocity(targetVel);
    }

    // === Torque/Force Control ===

    /**
     * @brief Set torque (rotary) or force (linear)
     * @param torque Torque in Nm or Force in N
     */
    void setTorque(float torque) {
        if (!enabled_) return;
        driver_->setControlMode(driver::ControlMode::Current);
        driver_->setTorque(torque);
    }

    /**
     * @brief Set force (alias for setTorque for linear motors)
     */
    void setForce(float force) {
        setTorque(force);
    }

    // === Impedance Control ===

    /**
     * @brief Set impedance parameters
     * @param stiffness Stiffness (Nm/rad or N/m)
     * @param damping Damping (Nm*s/rad or N*s/m)
     */
    void setImpedance(float stiffness, float damping) {
        impedanceKp_ = stiffness;
        impedanceKd_ = damping;
        impedanceEnabled_ = true;
    }

    /**
     * @brief Set impedance with inertia shaping
     */
    void setImpedance(float stiffness, float damping, float inertia) {
        setImpedance(stiffness, damping);
        (void)inertia;  // TODO: Implement inertia shaping
    }

    // === State Query ===

    /**
     * @brief Get current position
     */
    float getPosition() const {
        return driver_ ? driver_->getPosition() - homeOffset_ : 0;
    }

    /**
     * @brief Get current velocity
     */
    float getVelocity() const {
        return driver_ ? driver_->getVelocity() : 0;
    }

    /**
     * @brief Get current torque
     */
    float getTorque() const {
        return driver_ ? driver_->getTorque() : 0;
    }

    /**
     * @brief Check if motor is moving
     */
    bool isMoving() const {
        return !planner_.isSettled();
    }

    /**
     * @brief Check if motion is complete
     */
    bool isSettled() const {
        return planner_.isSettled();
    }

    /**
     * @brief Check for errors
     */
    bool hasError() const {
        return driver_ ? driver_->hasFault() : true;
    }

    /**
     * @brief Get error code
     */
    uint32_t getErrorCode() const {
        return driver_ ? driver_->getErrorCode() : 0xFFFFFFFF;
    }

    // === Homing ===

    /**
     * @brief Execute homing sequence
     * @param mode Homing mode
     */
    void home(HomingMode mode = HomingMode::SensorBased) {
        (void)mode;  // TODO: Implement full homing
        homeOffset_ = driver_ ? driver_->getPosition() : 0;
        homed_ = true;
        planner_.reset(0);
    }

    /**
     * @brief Set home offset
     * @param offset Position offset
     */
    void setHomeOffset(float offset) {
        homeOffset_ = offset;
    }

    /**
     * @brief Check if homed
     */
    bool isHomed() const { return homed_; }

    // === Limits ===

    /**
     * @brief Set software position limits
     */
    void setSoftLimits(float minPos, float maxPos) {
        softLimitMin_ = minPos;
        softLimitMax_ = maxPos;
    }

    /**
     * @brief Enable/disable soft limits
     */
    void enableSoftLimits(bool enable) {
        softLimitEnabled_ = enable;
    }

    /**
     * @brief Set hardware limit switch pins
     */
    void setHardLimitPins(hal::IGpio* negLimit, hal::IGpio* posLimit) {
        negLimitPin_ = negLimit;
        posLimitPin_ = posLimit;
    }

    // === Control Parameters ===

    /**
     * @brief Set position PID gains
     */
    void setPositionPid(float kp, float ki, float kd) {
        cascadePid_.setPositionGains(kp, ki, kd);
    }

    /**
     * @brief Set velocity PID gains
     */
    void setVelocityPid(float kp, float ki, float kd) {
        cascadePid_.setVelocityGains(kp, ki, kd);
    }

    /**
     * @brief Set feedforward gains
     */
    void setFeedforward(float kv, float ka) {
        control::FeedforwardController::Params ff;
        ff.kv = kv;
        ff.ka = ka;
        feedforward_.setParams(ff);
    }

    /**
     * @brief Set motion constraints
     */
    void setConstraints(const motion::MotionConstraints& constraints) {
        constraints_ = constraints;
        planner_.setConstraints(constraints);
    }

    // === Update ===

    /**
     * @brief Update control loop
     * @param dt Time step in seconds
     */
    void update(float dt) {
        if (!enabled_ || !driver_) return;

        // Get trajectory command
        motion::TrajectoryPoint cmd = planner_.update(dt);

        // Get feedback
        float posFb = driver_->getPosition() - homeOffset_;
        float velFb = driver_->getVelocity();
        float curFb = driver_->getCurrent();

        // Calculate feedforward
        float ff = feedforward_.calculate(cmd.velocity, cmd.acceleration, cmd.jerk);

        // Run cascade controller
        float output;
        if (impedanceEnabled_) {
            // Impedance control
            float posError = cmd.position - posFb;
            float velError = cmd.velocity - velFb;
            output = impedanceKp_ * posError + impedanceKd_ * velError + ff;
        } else {
            // Cascade PID control
            output = cascadePid_.update(cmd.position, cmd.velocity, ff,
                                        posFb, velFb, curFb, dt);
        }

        // Send command to driver
        driver_->setPosition(cmd.position);

        // Update driver
        driver_->update();
    }

    /**
     * @brief Get the motor driver
     */
    driver::IMotorDriver* getDriver() { return driver_; }

private:
    driver::IMotorDriver* driver_;

    // Motion planning
    motion::MotionConstraints constraints_;
    motion::OnlineTrajectoryPlanner planner_;

    // Control
    control::CascadePidController cascadePid_;
    control::FeedforwardController feedforward_;

    // Impedance control
    bool impedanceEnabled_ = false;
    float impedanceKp_ = 0;
    float impedanceKd_ = 0;

    // State
    bool enabled_;
    bool homed_;
    float homeOffset_;

    // Limits
    bool softLimitEnabled_;
    float softLimitMin_, softLimitMax_;
    hal::IGpio* negLimitPin_ = nullptr;
    hal::IGpio* posLimitPin_ = nullptr;

    bool checkSoftLimits(float position) const {
        if (!softLimitEnabled_) return true;
        return position >= softLimitMin_ && position <= softLimitMax_;
    }
};

}  // namespace omni::app
