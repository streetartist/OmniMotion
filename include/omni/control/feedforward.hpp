/**
 * @file feedforward.hpp
 * @brief Feedforward controller for motion control
 */

#pragma once

#include <cmath>

namespace omni::control {

/**
 * @brief Feedforward controller
 *
 * Computes feedforward compensation based on velocity, acceleration,
 * jerk, friction, and gravity.
 */
class FeedforwardController {
public:
    /**
     * @brief Feedforward parameters
     */
    struct Params {
        float kv = 0.0f;    // Velocity feedforward gain (torque/velocity)
        float ka = 0.0f;    // Acceleration feedforward gain (inertia)
        float kj = 0.0f;    // Jerk feedforward gain
        float ks = 0.0f;    // Static friction compensation
        float kf = 0.0f;    // Viscous (dynamic) friction coefficient
        float kg = 0.0f;    // Gravity compensation gain
    };

    FeedforwardController() : gravityAngle_(0) {}
    explicit FeedforwardController(const Params& params)
        : params_(params), gravityAngle_(0) {}

    /**
     * @brief Calculate feedforward output
     * @param velocity Commanded velocity
     * @param acceleration Commanded acceleration
     * @param jerk Commanded jerk (optional)
     * @return Feedforward torque/force
     */
    float calculate(float velocity, float acceleration, float jerk = 0.0f) const {
        float output = 0.0f;

        // Velocity feedforward (back-EMF compensation)
        output += params_.kv * velocity;

        // Acceleration feedforward (inertia compensation)
        output += params_.ka * acceleration;

        // Jerk feedforward (for S-curve profiles)
        output += params_.kj * jerk;

        // Friction compensation
        if (std::abs(velocity) > 0.001f) {
            // Dynamic friction
            output += params_.kf * velocity;
            // Static friction (Coulomb)
            output += (velocity > 0) ? params_.ks : -params_.ks;
        }

        // Gravity compensation
        output += params_.kg * std::sin(gravityAngle_);

        return output;
    }

    /**
     * @brief Calculate with separate static friction threshold
     * @param velocity Commanded velocity
     * @param acceleration Commanded acceleration
     * @param staticThreshold Velocity threshold for static friction
     * @return Feedforward output
     */
    float calculate(float velocity, float acceleration, float jerk,
                    float staticThreshold) const {
        float output = params_.kv * velocity + params_.ka * acceleration + params_.kj * jerk;

        if (std::abs(velocity) > staticThreshold) {
            output += params_.kf * velocity;
            output += (velocity > 0) ? params_.ks : -params_.ks;
        } else if (std::abs(acceleration) > 0.001f) {
            // Static friction breakaway
            output += (acceleration > 0) ? params_.ks : -params_.ks;
        }

        output += params_.kg * std::sin(gravityAngle_);

        return output;
    }

    /**
     * @brief Set gravity compensation angle
     * @param angle Angle from horizontal (radians)
     */
    void setGravityAngle(float angle) { gravityAngle_ = angle; }

    /**
     * @brief Get gravity compensation angle
     * @return Angle in radians
     */
    float getGravityAngle() const { return gravityAngle_; }

    /**
     * @brief Set parameters
     * @param params Feedforward parameters
     */
    void setParams(const Params& params) { params_ = params; }

    /**
     * @brief Get parameters
     * @return Current parameters
     */
    const Params& getParams() const { return params_; }

    // Individual setters
    void setKv(float kv) { params_.kv = kv; }
    void setKa(float ka) { params_.ka = ka; }
    void setKj(float kj) { params_.kj = kj; }
    void setKs(float ks) { params_.ks = ks; }
    void setKf(float kf) { params_.kf = kf; }
    void setKg(float kg) { params_.kg = kg; }

    // Getters
    float getKv() const { return params_.kv; }
    float getKa() const { return params_.ka; }
    float getKj() const { return params_.kj; }
    float getKs() const { return params_.ks; }
    float getKf() const { return params_.kf; }
    float getKg() const { return params_.kg; }

private:
    Params params_;
    float gravityAngle_;
};

}  // namespace omni::control
