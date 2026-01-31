/**
 * @file scurve_profile.hpp
 * @brief S-curve (7-segment) velocity profile generator
 */

#pragma once

#include "trajectory.hpp"
#include <cmath>
#include <algorithm>
#include <array>

namespace omni::motion {

/**
 * @brief S-curve (7-segment) velocity profile
 *
 * Generates smooth motion profiles with limited jerk for reduced
 * mechanical vibration and wear. The profile consists of 7 segments:
 * 1. Increasing acceleration (jerk)
 * 2. Constant acceleration
 * 3. Decreasing acceleration (negative jerk)
 * 4. Constant velocity
 * 5. Increasing deceleration (negative jerk)
 * 6. Constant deceleration
 * 7. Decreasing deceleration (jerk)
 */
class SCurveProfile : public ITrajectoryGenerator {
public:
    /**
     * @brief Construct with constraints
     * @param constraints Motion constraints including jerk limit
     */
    explicit SCurveProfile(const MotionConstraints& constraints)
        : constraints_(constraints)
        , startPos_(0), endPos_(0)
        , startVel_(0), endVel_(0)
        , peakVel_(0), peakAccel_(0)
        , totalTime_(0)
        , valid_(false) {
        segmentTimes_.fill(0);
    }

    /**
     * @brief Construct with individual limits
     * @param maxVel Maximum velocity
     * @param maxAccel Maximum acceleration
     * @param maxJerk Maximum jerk
     */
    SCurveProfile(float maxVel, float maxAccel, float maxJerk)
        : SCurveProfile(MotionConstraints{maxVel, maxAccel, 0, maxJerk, 0}) {}

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override {
        startPos_ = startPos;
        endPos_ = endPos;
        startVel_ = startVel;
        endVel_ = endVel;

        float distance = endPos - startPos;
        float direction = (distance >= 0) ? 1.0f : -1.0f;
        distance = std::abs(distance);

        // Get limits (adjusted for direction in internal calculations)
        float vMax = constraints_.maxVelocity;
        float aMax = constraints_.maxAcceleration;
        float jMax = constraints_.maxJerk;

        // Time to reach max acceleration at max jerk
        float tj = aMax / jMax;

        // Velocity change during jerk phase
        float vj = 0.5f * jMax * tj * tj;

        // Velocity change during constant acceleration
        // Full acceleration phase: 2*vj (two jerk phases) + aMax*ta (constant accel)

        // Calculate if we can reach max velocity
        float vAccelPhase = 2.0f * vj;  // Min velocity change in accel phase

        // Distance covered during minimum accel phase (triangle)
        float minAccelDist = 2.0f * (startVel * tj + jMax * tj * tj * tj / 6.0f);

        // Simplified planning - aim for symmetric profile
        // Full S-curve times: tj1, ta, tj2, tv, tj3, td, tj4

        // Try full profile first
        peakVel_ = vMax;
        peakAccel_ = aMax;

        // Time at constant acceleration
        float ta = (vMax - startVel - 2.0f * vj) / aMax;
        if (ta < 0) {
            // Cannot reach max velocity with max acceleration
            ta = 0;
            // Recalculate peak velocity
            peakVel_ = startVel + 2.0f * vj;
        }

        // Time at constant deceleration (symmetric for now)
        float td = ta;

        // Calculate distances
        float distAccel = calculateAccelDistance(startVel, tj, ta, jMax, aMax);
        float distDecel = calculateDecelDistance(peakVel_, tj, td, jMax, aMax);

        float constVelDist = distance - distAccel - distDecel;

        if (constVelDist >= 0) {
            // Full profile with constant velocity phase
            float tv = constVelDist / peakVel_;

            segmentTimes_[0] = tj;       // Jerk up
            segmentTimes_[1] = ta;       // Const accel
            segmentTimes_[2] = tj;       // Jerk down
            segmentTimes_[3] = tv;       // Const vel
            segmentTimes_[4] = tj;       // Jerk up (decel start)
            segmentTimes_[5] = td;       // Const decel
            segmentTimes_[6] = tj;       // Jerk down (decel end)
        } else {
            // Reduced profile - no constant velocity phase
            // Need to solve for reduced peak velocity

            // Iterative solution
            float vPeakLow = startVel;
            float vPeakHigh = vMax;

            for (int i = 0; i < 20; i++) {
                peakVel_ = (vPeakLow + vPeakHigh) / 2.0f;

                // Recalculate with new peak velocity
                float accelNeeded = peakVel_ - startVel;
                if (accelNeeded <= 2.0f * vj) {
                    // Only jerk phases
                    float tjActual = std::sqrt(accelNeeded / jMax);
                    segmentTimes_[0] = tjActual;
                    segmentTimes_[1] = 0;
                    segmentTimes_[2] = tjActual;
                } else {
                    segmentTimes_[0] = tj;
                    segmentTimes_[1] = (accelNeeded - 2.0f * vj) / aMax;
                    segmentTimes_[2] = tj;
                }

                float decelNeeded = peakVel_ - endVel;
                if (decelNeeded <= 2.0f * vj) {
                    float tjActual = std::sqrt(decelNeeded / jMax);
                    segmentTimes_[4] = tjActual;
                    segmentTimes_[5] = 0;
                    segmentTimes_[6] = tjActual;
                } else {
                    segmentTimes_[4] = tj;
                    segmentTimes_[5] = (decelNeeded - 2.0f * vj) / aMax;
                    segmentTimes_[6] = tj;
                }

                segmentTimes_[3] = 0;

                float totalDist = calculateTotalDistance();
                if (std::abs(totalDist - distance) < 1e-6f) {
                    break;
                }

                if (totalDist < distance) {
                    vPeakLow = peakVel_;
                } else {
                    vPeakHigh = peakVel_;
                }
            }
        }

        peakVel_ *= direction;
        totalTime_ = 0;
        for (float t : segmentTimes_) {
            totalTime_ += t;
        }

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
            point.jerk = 0;
            return point;
        }

        if (time >= totalTime_) {
            point.position = endPos_;
            point.velocity = endVel_;
            point.acceleration = 0;
            point.jerk = 0;
            return point;
        }

        float direction = (endPos_ >= startPos_) ? 1.0f : -1.0f;
        float jMax = constraints_.maxJerk * direction;
        float aMax = constraints_.maxAcceleration * direction;

        // Find current segment
        float t = 0;
        int segment = 0;
        for (int i = 0; i < 7; i++) {
            if (time < t + segmentTimes_[i]) {
                segment = i;
                break;
            }
            t += segmentTimes_[i];
            if (i == 6) segment = 6;
        }

        float dt = time - t;  // Time within segment

        // Calculate cumulative state at start of segment
        float pos = startPos_;
        float vel = startVel_;
        float acc = 0;

        // Accumulate through previous segments
        float tCum = 0;
        for (int i = 0; i < segment; i++) {
            float segT = segmentTimes_[i];
            float segJ = getSegmentJerk(i) * direction;
            float segA = acc;

            pos += vel * segT + 0.5f * segA * segT * segT +
                   segJ * segT * segT * segT / 6.0f;
            vel += segA * segT + 0.5f * segJ * segT * segT;
            acc += segJ * segT;

            tCum += segT;
        }

        // Calculate state within current segment
        float segJ = getSegmentJerk(segment) * direction;
        point.jerk = segJ;
        point.acceleration = acc + segJ * dt;
        point.velocity = vel + acc * dt + 0.5f * segJ * dt * dt;
        point.position = pos + vel * dt + 0.5f * acc * dt * dt +
                        segJ * dt * dt * dt / 6.0f;

        return point;
    }

    float getDuration() const override { return totalTime_; }
    bool isValid() const override { return valid_; }
    float getStartPosition() const override { return startPos_; }
    float getEndPosition() const override { return endPos_; }

    /**
     * @brief Get segment times
     * @param times Output array of 7 segment times
     */
    void getSegmentTimes(float times[7]) const {
        for (int i = 0; i < 7; i++) {
            times[i] = segmentTimes_[i];
        }
    }

    /**
     * @brief Get peak velocity reached
     */
    float getPeakVelocity() const { return peakVel_; }

    /**
     * @brief Get peak acceleration reached
     */
    float getPeakAcceleration() const { return peakAccel_; }

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
    float peakVel_, peakAccel_;
    std::array<float, 7> segmentTimes_;
    float totalTime_;
    bool valid_;

    /**
     * @brief Get jerk value for segment (normalized, before direction)
     */
    float getSegmentJerk(int segment) const {
        switch (segment) {
            case 0: return constraints_.maxJerk;   // Accel ramp up
            case 1: return 0;                       // Const accel
            case 2: return -constraints_.maxJerk;  // Accel ramp down
            case 3: return 0;                       // Const vel
            case 4: return -constraints_.maxJerk;  // Decel ramp up
            case 5: return 0;                       // Const decel
            case 6: return constraints_.maxJerk;   // Decel ramp down
            default: return 0;
        }
    }

    float calculateAccelDistance(float v0, float tj, float ta, float j, float a) const {
        // Distance during jerk-up, const-accel, jerk-down
        float d1 = v0 * tj + j * tj * tj * tj / 6.0f;
        float v1 = v0 + 0.5f * j * tj * tj;
        float d2 = v1 * ta + 0.5f * a * ta * ta;
        float v2 = v1 + a * ta;
        float d3 = v2 * tj + 0.5f * a * tj * tj - j * tj * tj * tj / 6.0f;
        return d1 + d2 + d3;
    }

    float calculateDecelDistance(float v0, float tj, float td, float j, float a) const {
        // Similar to accel but with negative acceleration
        float d1 = v0 * tj - j * tj * tj * tj / 6.0f;
        float v1 = v0 - 0.5f * j * tj * tj;
        float d2 = v1 * td - 0.5f * a * td * td;
        float v2 = v1 - a * td;
        float d3 = v2 * tj - 0.5f * a * tj * tj + j * tj * tj * tj / 6.0f;
        return d1 + d2 + d3;
    }

    float calculateTotalDistance() const {
        TrajectoryPoint end = evaluate(totalTime_ - 0.0001f);
        return std::abs(end.position - startPos_);
    }
};

}  // namespace omni::motion
