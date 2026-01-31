/**
 * @file trapezoidal_profile.hpp
 * @brief Trapezoidal velocity profile generator
 */

#pragma once

#include "trajectory.hpp"
#include <cmath>
#include <algorithm>

namespace omni::motion {

/**
 * @brief Trapezoidal velocity profile (T-curve)
 *
 * Generates a velocity profile with constant acceleration,
 * constant velocity, and constant deceleration phases.
 */
class TrapezoidalProfile : public ITrajectoryGenerator {
public:
    /**
     * @brief Construct with constraints
     * @param constraints Motion constraints
     */
    explicit TrapezoidalProfile(const MotionConstraints& constraints)
        : constraints_(constraints)
        , startPos_(0), endPos_(0)
        , startVel_(0), endVel_(0)
        , peakVel_(0)
        , accelTime_(0), constTime_(0), decelTime_(0)
        , totalTime_(0)
        , valid_(false) {}

    /**
     * @brief Construct with individual limits
     * @param maxVel Maximum velocity
     * @param maxAccel Maximum acceleration
     * @param maxDecel Maximum deceleration (0 = same as accel)
     */
    TrapezoidalProfile(float maxVel, float maxAccel, float maxDecel = 0)
        : TrapezoidalProfile(MotionConstraints{maxVel, maxAccel, maxDecel, 0, 0}) {}

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override {
        startPos_ = startPos;
        endPos_ = endPos;
        startVel_ = startVel;
        endVel_ = endVel;

        float distance = endPos - startPos;
        float direction = (distance >= 0) ? 1.0f : -1.0f;
        distance = std::abs(distance);

        float maxVel = constraints_.maxVelocity;
        float accel = constraints_.maxAcceleration;
        float decel = constraints_.getDeceleration();

        // Adjust for direction
        startVel = startVel * direction;
        endVel = endVel * direction;

        // Calculate acceleration and deceleration times to reach max velocity
        float accelTimeToMax = (maxVel - startVel) / accel;
        float decelTimeFromMax = (maxVel - endVel) / decel;

        // Distance covered during acceleration and deceleration
        float accelDist = startVel * accelTimeToMax + 0.5f * accel * accelTimeToMax * accelTimeToMax;
        float decelDist = maxVel * decelTimeFromMax - 0.5f * decel * decelTimeFromMax * decelTimeFromMax;

        if (accelDist + decelDist <= distance) {
            // Full trapezoidal profile (3 phases)
            accelTime_ = accelTimeToMax;
            decelTime_ = decelTimeFromMax;
            float constDist = distance - accelDist - decelDist;
            constTime_ = constDist / maxVel;
            peakVel_ = maxVel * direction;
        } else {
            // Triangular profile (no constant velocity phase)
            constTime_ = 0;

            // Solve for peak velocity
            // v_peak^2 = (2*accel*decel*distance + decel*v_start^2 + accel*v_end^2) / (accel + decel)
            float v2 = (2.0f * accel * decel * distance +
                       decel * startVel * startVel +
                       accel * endVel * endVel) / (accel + decel);

            if (v2 < 0) {
                valid_ = false;
                return false;
            }

            peakVel_ = std::sqrt(v2) * direction;
            accelTime_ = (std::abs(peakVel_) - std::abs(startVel_)) / accel;
            decelTime_ = (std::abs(peakVel_) - std::abs(endVel_)) / decel;

            // Clamp negative times
            if (accelTime_ < 0) accelTime_ = 0;
            if (decelTime_ < 0) decelTime_ = 0;
        }

        totalTime_ = accelTime_ + constTime_ + decelTime_;
        valid_ = true;
        return true;
    }

    TrajectoryPoint evaluate(float time) const override {
        TrajectoryPoint point;
        point.time = time;

        if (!valid_ || time < 0) {
            point.position = startPos_;
            point.velocity = startVel_;
            point.acceleration = 0;
            return point;
        }

        if (time >= totalTime_) {
            point.position = endPos_;
            point.velocity = endVel_;
            point.acceleration = 0;
            return point;
        }

        float direction = (endPos_ >= startPos_) ? 1.0f : -1.0f;
        float accel = constraints_.maxAcceleration * direction;
        float decel = -constraints_.getDeceleration() * direction;

        if (time < accelTime_) {
            // Acceleration phase
            point.acceleration = accel;
            point.velocity = startVel_ + accel * time;
            point.position = startPos_ + startVel_ * time + 0.5f * accel * time * time;
        } else if (time < accelTime_ + constTime_) {
            // Constant velocity phase
            float t = time - accelTime_;
            point.acceleration = 0;
            point.velocity = peakVel_;

            float posAfterAccel = startPos_ + startVel_ * accelTime_ +
                                  0.5f * accel * accelTime_ * accelTime_;
            point.position = posAfterAccel + peakVel_ * t;
        } else {
            // Deceleration phase
            float t = time - accelTime_ - constTime_;
            point.acceleration = decel;
            point.velocity = peakVel_ + decel * t;

            float posAfterAccel = startPos_ + startVel_ * accelTime_ +
                                  0.5f * accel * accelTime_ * accelTime_;
            float posAfterConst = posAfterAccel + peakVel_ * constTime_;
            point.position = posAfterConst + peakVel_ * t + 0.5f * decel * t * t;
        }

        point.jerk = 0;  // Trapezoidal has infinite jerk at transitions
        return point;
    }

    float getDuration() const override { return totalTime_; }
    bool isValid() const override { return valid_; }
    float getStartPosition() const override { return startPos_; }
    float getEndPosition() const override { return endPos_; }

    /**
     * @brief Get acceleration phase time
     */
    float getAccelTime() const { return accelTime_; }

    /**
     * @brief Get constant velocity phase time
     */
    float getConstTime() const { return constTime_; }

    /**
     * @brief Get deceleration phase time
     */
    float getDecelTime() const { return decelTime_; }

    /**
     * @brief Get peak velocity reached
     */
    float getPeakVelocity() const { return peakVel_; }

    /**
     * @brief Set constraints
     */
    void setConstraints(const MotionConstraints& constraints) {
        constraints_ = constraints;
        valid_ = false;
    }

    /**
     * @brief Get constraints
     */
    const MotionConstraints& getConstraints() const { return constraints_; }

private:
    MotionConstraints constraints_;

    float startPos_, endPos_;
    float startVel_, endVel_;
    float peakVel_;
    float accelTime_, constTime_, decelTime_;
    float totalTime_;
    bool valid_;
};

}  // namespace omni::motion
