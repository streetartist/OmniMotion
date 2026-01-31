/**
 * @file online_planner.hpp
 * @brief Real-time trajectory planner with target modification support
 */

#pragma once

#include "trajectory.hpp"
#include "scurve_profile.hpp"
#include <cmath>
#include <algorithm>

namespace omni::motion {

/**
 * @brief Online (real-time) trajectory planner
 *
 * Generates trajectories that can be modified on-the-fly while
 * maintaining smooth motion. Uses time-optimal S-curve planning.
 */
class OnlineTrajectoryPlanner {
public:
    /**
     * @brief Planner state
     */
    enum class State {
        Idle,
        Accelerating,
        ConstantVelocity,
        Decelerating,
        Settling
    };

    /**
     * @brief Construct planner with constraints
     * @param constraints Motion constraints
     */
    explicit OnlineTrajectoryPlanner(const MotionConstraints& constraints)
        : constraints_(constraints)
        , currentPos_(0), currentVel_(0), currentAcc_(0)
        , targetPos_(0), targetVel_(0)
        , state_(State::Idle)
        , settled_(true) {}

    /**
     * @brief Set new target position
     * @param targetPos Target position
     */
    void setTarget(float targetPos) {
        setTarget(targetPos, 0);
    }

    /**
     * @brief Set new target position and velocity
     * @param targetPos Target position
     * @param targetVel Target velocity at position (usually 0)
     */
    void setTarget(float targetPos, float targetVel) {
        targetPos_ = targetPos;
        targetVel_ = targetVel;
        settled_ = false;
        replan();
    }

    /**
     * @brief Update planner and get current command
     * @param dt Time step
     * @return Current trajectory point
     */
    TrajectoryPoint update(float dt) {
        if (settled_) {
            return TrajectoryPoint(0, currentPos_, 0, 0, 0);
        }

        // Time in current profile
        profileTime_ += dt;

        // Evaluate current profile
        TrajectoryPoint point = profile_.evaluate(profileTime_);

        // Update state
        currentPos_ = point.position;
        currentVel_ = point.velocity;
        currentAcc_ = point.acceleration;

        // Check if settled
        float posError = std::abs(currentPos_ - targetPos_);
        float velError = std::abs(currentVel_ - targetVel_);

        if (posError < settlePositionTolerance_ &&
            velError < settleVelocityTolerance_) {
            settled_ = true;
            currentPos_ = targetPos_;
            currentVel_ = targetVel_;
            currentAcc_ = 0;
            state_ = State::Idle;
        }

        // Update state based on profile phase
        if (profileTime_ < profile_.getDuration() * 0.3f) {
            state_ = State::Accelerating;
        } else if (profileTime_ < profile_.getDuration() * 0.7f) {
            state_ = State::ConstantVelocity;
        } else {
            state_ = State::Decelerating;
        }

        return point;
    }

    /**
     * @brief Get current position
     */
    float getCurrentPosition() const { return currentPos_; }

    /**
     * @brief Get current velocity
     */
    float getCurrentVelocity() const { return currentVel_; }

    /**
     * @brief Get current acceleration
     */
    float getCurrentAcceleration() const { return currentAcc_; }

    /**
     * @brief Get target position
     */
    float getTargetPosition() const { return targetPos_; }

    /**
     * @brief Check if motion is complete
     */
    bool isSettled() const { return settled_; }

    /**
     * @brief Get current state
     */
    State getState() const { return state_; }

    /**
     * @brief Emergency stop
     *
     * Decelerates at maximum rate to stop as quickly as possible.
     */
    void emergencyStop() {
        // Plan stop from current state
        float stopDist = 0.5f * currentVel_ * currentVel_ / constraints_.maxAcceleration;
        if (currentVel_ < 0) stopDist = -stopDist;

        targetPos_ = currentPos_ + stopDist;
        targetVel_ = 0;
        replan();
    }

    /**
     * @brief Reset to position
     * @param position Position to reset to
     */
    void reset(float position = 0) {
        currentPos_ = position;
        currentVel_ = 0;
        currentAcc_ = 0;
        targetPos_ = position;
        targetVel_ = 0;
        profileTime_ = 0;
        settled_ = true;
        state_ = State::Idle;
    }

    /**
     * @brief Set motion constraints
     */
    void setConstraints(const MotionConstraints& constraints) {
        constraints_ = constraints;
        profile_.setConstraints(constraints);
        if (!settled_) {
            replan();
        }
    }

    /**
     * @brief Get motion constraints
     */
    const MotionConstraints& getConstraints() const { return constraints_; }

    /**
     * @brief Set settling tolerances
     * @param posTol Position tolerance
     * @param velTol Velocity tolerance
     */
    void setSettleTolerance(float posTol, float velTol) {
        settlePositionTolerance_ = posTol;
        settleVelocityTolerance_ = velTol;
    }

    /**
     * @brief Get estimated time to target
     */
    float getTimeToTarget() const {
        if (settled_) return 0;
        float remaining = profile_.getDuration() - profileTime_;
        return (remaining > 0) ? remaining : 0;
    }

private:
    MotionConstraints constraints_;
    SCurveProfile profile_{constraints_};

    // Current state
    float currentPos_;
    float currentVel_;
    float currentAcc_;

    // Target
    float targetPos_;
    float targetVel_;

    // Profile tracking
    float profileTime_ = 0;

    // State
    State state_;
    bool settled_;

    // Tolerances
    float settlePositionTolerance_ = 0.001f;
    float settleVelocityTolerance_ = 0.01f;

    void replan() {
        // Plan new trajectory from current state to target
        profile_.plan(currentPos_, targetPos_, currentVel_, targetVel_);
        profileTime_ = 0;
    }
};

}  // namespace omni::motion
