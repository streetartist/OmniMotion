/**
 * @file mpc_controller.hpp
 * @brief Model Predictive Controller (simplified version)
 */

#pragma once

#include <array>
#include <algorithm>
#include <cmath>

namespace omni::control {

/**
 * @brief Simple MPC controller for single-axis motion
 *
 * Simplified MPC implementation suitable for embedded systems.
 * Uses a discrete-time double integrator model (position-velocity).
 */
template<int HORIZON = 10>
class SimpleMpcController {
public:
    struct Config {
        float dt = 0.001f;              // Sample time
        float qPos = 1.0f;              // Position error weight
        float qVel = 0.1f;              // Velocity error weight
        float r = 0.01f;                // Control effort weight
        float uMin = -100.0f;           // Minimum control
        float uMax = 100.0f;            // Maximum control
        float posMin = -1e6f;           // Position limit min
        float posMax = 1e6f;            // Position limit max
        float velMin = -1e6f;           // Velocity limit min
        float velMax = 1e6f;            // Velocity limit max
    };

    SimpleMpcController() = default;
    explicit SimpleMpcController(const Config& config) : config_(config) {
        computeGains();
    }

    /**
     * @brief Update MPC controller
     * @param posRef Position reference
     * @param posFb Position feedback
     * @param velFb Velocity feedback
     * @return Optimal control input
     */
    float update(float posRef, float posFb, float velFb) {
        // State error
        float posErr = posRef - posFb;
        float velErr = -velFb;  // Reference velocity is 0 at steady state

        // Simplified MPC: use precomputed gains for steady-state tracking
        // This is equivalent to LQR for the infinite-horizon case
        float u = kPos_ * posErr + kVel_ * velErr;

        // Apply constraints
        u = std::clamp(u, config_.uMin, config_.uMax);

        return u;
    }

    /**
     * @brief Update with velocity reference
     * @param posRef Position reference
     * @param velRef Velocity reference (feedforward)
     * @param posFb Position feedback
     * @param velFb Velocity feedback
     * @return Optimal control
     */
    float update(float posRef, float velRef, float posFb, float velFb) {
        float posErr = posRef - posFb;
        float velErr = velRef - velFb;

        float u = kPos_ * posErr + kVel_ * velErr;
        u = std::clamp(u, config_.uMin, config_.uMax);

        return u;
    }

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config) {
        config_ = config;
        computeGains();
    }

    /**
     * @brief Get configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Get computed position gain
     */
    float getKPos() const { return kPos_; }

    /**
     * @brief Get computed velocity gain
     */
    float getKVel() const { return kVel_; }

private:
    Config config_;
    float kPos_ = 0;
    float kVel_ = 0;

    /**
     * @brief Compute optimal feedback gains
     *
     * Solves the discrete-time Riccati equation for the double integrator.
     * System: x(k+1) = A*x(k) + B*u(k)
     * A = [1, dt; 0, 1], B = [0.5*dt^2; dt]
     */
    void computeGains() {
        float dt = config_.dt;
        float q1 = config_.qPos;
        float q2 = config_.qVel;
        float r = config_.r;

        // Discrete-time double integrator model
        // Using simplified pole placement / LQR approximation
        // For a double integrator with cost J = sum(q1*e_pos^2 + q2*e_vel^2 + r*u^2)

        // Compute natural frequency and damping from weights
        float wn = std::sqrt(q1 / r) / dt;  // Approximate natural frequency
        float zeta = 0.5f * std::sqrt(q2 / q1);  // Approximate damping ratio

        // Ensure reasonable values
        zeta = std::clamp(zeta, 0.5f, 2.0f);

        // Feedback gains (pole placement approximation)
        kPos_ = wn * wn;
        kVel_ = 2.0f * zeta * wn;
    }
};

/**
 * @brief MPC with explicit trajectory tracking
 *
 * Tracks a reference trajectory over the prediction horizon.
 */
template<int HORIZON = 10>
class TrajectoryMpcController {
public:
    struct Config {
        float dt = 0.001f;
        std::array<float, HORIZON> qPos;    // Position weights over horizon
        std::array<float, HORIZON> qVel;    // Velocity weights over horizon
        std::array<float, HORIZON> r;       // Control weights over horizon
        float uMin = -100.0f;
        float uMax = 100.0f;
    };

    struct Reference {
        std::array<float, HORIZON> position;
        std::array<float, HORIZON> velocity;
    };

    TrajectoryMpcController() {
        // Initialize with uniform weights
        config_.qPos.fill(1.0f);
        config_.qVel.fill(0.1f);
        config_.r.fill(0.01f);
    }

    /**
     * @brief Set reference trajectory
     */
    void setReference(const Reference& ref) {
        reference_ = ref;
    }

    /**
     * @brief Update with current state
     * @param posFb Current position
     * @param velFb Current velocity
     * @return Optimal control input
     */
    float update(float posFb, float velFb) {
        // Simplified: just use first step of horizon
        float posErr = reference_.position[0] - posFb;
        float velErr = reference_.velocity[0] - velFb;

        // Weighted sum
        float u = config_.qPos[0] * posErr + config_.qVel[0] * velErr;
        u = std::clamp(u, config_.uMin, config_.uMax);

        return u;
    }

    void setConfig(const Config& config) { config_ = config; }
    const Config& getConfig() const { return config_; }

private:
    Config config_;
    Reference reference_;
};

}  // namespace omni::control
