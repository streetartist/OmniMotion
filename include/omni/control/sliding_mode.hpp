/**
 * @file sliding_mode.hpp
 * @brief Sliding mode controller
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace omni::control {

/**
 * @brief Sliding mode controller
 *
 * Implements sliding mode control with boundary layer to reduce chattering.
 * Provides robust control against disturbances and model uncertainties.
 */
class SlidingModeController {
public:
    /**
     * @brief Controller parameters
     */
    struct Params {
        float lambda = 1.0f;    // Sliding surface slope
        float eta = 1.0f;       // Reaching law gain (switching gain)
        float phi = 0.1f;       // Boundary layer thickness (anti-chattering)
        float kp = 0.0f;        // Optional proportional gain for equivalent control
        bool adaptiveEta = false;  // Enable adaptive switching gain
    };

    SlidingModeController() = default;
    explicit SlidingModeController(const Params& params) : params_(params) {}

    /**
     * @brief Update controller
     * @param posError Position error
     * @param velError Velocity error (can be derived from posError)
     * @param dt Time step
     * @return Control output
     */
    float update(float posError, float velError, float dt) {
        (void)dt;  // dt not used in basic sliding mode

        // Sliding surface: s = velError + lambda * posError
        float s = velError + params_.lambda * posError;

        // Equivalent control (optional proportional term)
        float ueq = params_.kp * posError;

        // Switching control with saturation (boundary layer)
        float usw;
        if (params_.phi > 0) {
            // Continuous approximation (saturation function)
            usw = params_.eta * sat(s / params_.phi);
        } else {
            // Pure switching (sign function) - causes chattering
            usw = params_.eta * sign(s);
        }

        // Adaptive gain (increases when far from surface)
        if (params_.adaptiveEta) {
            float adaptiveFactor = 1.0f + std::abs(s);
            usw *= adaptiveFactor;
        }

        slidingSurface_ = s;
        return ueq + usw;
    }

    /**
     * @brief Update with velocity derived from position
     * @param posError Position error
     * @param prevPosError Previous position error
     * @param dt Time step
     * @return Control output
     */
    float update(float posError, float prevPosError, float dt, bool) {
        float velError = (posError - prevPosError) / dt;
        return update(posError, velError, dt);
    }

    /**
     * @brief Get sliding surface value
     * @return Current s value
     */
    float getSlidingSurface() const { return slidingSurface_; }

    /**
     * @brief Check if on sliding surface
     * @return true if within boundary layer
     */
    bool isOnSurface() const {
        return std::abs(slidingSurface_) <= params_.phi;
    }

    /**
     * @brief Enable adaptive switching gain
     * @param enable true to enable
     */
    void enableAdaptive(bool enable) { params_.adaptiveEta = enable; }

    /**
     * @brief Set parameters
     */
    void setParams(const Params& params) { params_ = params; }

    /**
     * @brief Get parameters
     */
    const Params& getParams() const { return params_; }

    // Individual setters
    void setLambda(float lambda) { params_.lambda = lambda; }
    void setEta(float eta) { params_.eta = eta; }
    void setPhi(float phi) { params_.phi = phi; }
    void setKp(float kp) { params_.kp = kp; }

    /**
     * @brief Reset controller state
     */
    void reset() {
        slidingSurface_ = 0;
    }

private:
    Params params_;
    float slidingSurface_ = 0;

    // Sign function
    static float sign(float x) {
        if (x > 0) return 1.0f;
        if (x < 0) return -1.0f;
        return 0.0f;
    }

    // Saturation function
    static float sat(float x) {
        return std::clamp(x, -1.0f, 1.0f);
    }
};

/**
 * @brief Super-twisting sliding mode controller
 *
 * Second-order sliding mode controller that provides continuous
 * control output while maintaining robustness.
 */
class SuperTwistingController {
public:
    struct Params {
        float alpha = 1.0f;     // First gain
        float beta = 1.0f;      // Second gain (integral term)
    };

    SuperTwistingController() : integral_(0) {}
    explicit SuperTwistingController(const Params& params)
        : params_(params), integral_(0) {}

    /**
     * @brief Update controller
     * @param error Sliding variable (s)
     * @param dt Time step
     * @return Control output
     */
    float update(float error, float dt) {
        // Super-twisting algorithm
        float sqrtAbsS = std::sqrt(std::abs(error));
        float u1 = -params_.alpha * sqrtAbsS * sign(error);

        integral_ += -params_.beta * sign(error) * dt;

        return u1 + integral_;
    }

    void reset() {
        integral_ = 0;
    }

    void setParams(const Params& params) { params_ = params; }
    const Params& getParams() const { return params_; }

private:
    Params params_;
    float integral_;

    static float sign(float x) {
        if (x > 0) return 1.0f;
        if (x < 0) return -1.0f;
        return 0.0f;
    }
};

}  // namespace omni::control
