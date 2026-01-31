/**
 * @file trajectory.hpp
 * @brief Base trajectory types and interfaces
 */

#pragma once

#include <cstdint>
#include <vector>

namespace omni::motion {

/**
 * @brief Trajectory point containing all motion states
 */
struct TrajectoryPoint {
    float time = 0.0f;          // Time (seconds)
    float position = 0.0f;      // Position (rad or m)
    float velocity = 0.0f;      // Velocity (rad/s or m/s)
    float acceleration = 0.0f;  // Acceleration
    float jerk = 0.0f;          // Jerk (derivative of acceleration)

    TrajectoryPoint() = default;
    TrajectoryPoint(float t, float p, float v = 0, float a = 0, float j = 0)
        : time(t), position(p), velocity(v), acceleration(a), jerk(j) {}
};

/**
 * @brief Motion constraints
 */
struct MotionConstraints {
    float maxVelocity = 1.0f;
    float maxAcceleration = 10.0f;
    float maxDeceleration = 0.0f;     // 0 = same as acceleration
    float maxJerk = 100.0f;
    float maxDecelerationJerk = 0.0f; // 0 = same as jerk

    /**
     * @brief Get deceleration (uses maxAcceleration if not set)
     */
    float getDeceleration() const {
        return (maxDeceleration > 0) ? maxDeceleration : maxAcceleration;
    }

    /**
     * @brief Get deceleration jerk (uses maxJerk if not set)
     */
    float getDecelerationJerk() const {
        return (maxDecelerationJerk > 0) ? maxDecelerationJerk : maxJerk;
    }
};

/**
 * @brief Trajectory generator interface
 */
class ITrajectoryGenerator {
public:
    virtual ~ITrajectoryGenerator() = default;

    /**
     * @brief Plan trajectory between two points
     * @param startPos Starting position
     * @param endPos Ending position
     * @param startVel Starting velocity (default 0)
     * @param endVel Ending velocity (default 0)
     * @return true if planning successful
     */
    virtual bool plan(float startPos, float endPos,
                      float startVel = 0, float endVel = 0) = 0;

    /**
     * @brief Evaluate trajectory at given time
     * @param time Time since start (seconds)
     * @return Trajectory point at given time
     */
    virtual TrajectoryPoint evaluate(float time) const = 0;

    /**
     * @brief Get total trajectory duration
     * @return Duration in seconds
     */
    virtual float getDuration() const = 0;

    /**
     * @brief Check if trajectory is valid
     * @return true if valid trajectory exists
     */
    virtual bool isValid() const = 0;

    /**
     * @brief Get starting position
     * @return Start position
     */
    virtual float getStartPosition() const = 0;

    /**
     * @brief Get ending position
     * @return End position
     */
    virtual float getEndPosition() const = 0;
};

}  // namespace omni::motion
