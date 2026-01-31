/**
 * @file adaptive_controller.hpp
 * @brief Adaptive control algorithms
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace omni::control {

/**
 * @brief Model Reference Adaptive Controller (MRAC)
 *
 * Implements direct MRAC using MIT rule for parameter adaptation.
 * Adjusts controller gains to make the plant follow a reference model.
 */
class MracController {
public:
    struct Params {
        // Reference model parameters (first-order)
        float modelGain = 1.0f;         // Reference model gain
        float modelTimeConstant = 0.1f; // Reference model time constant

        // Adaptation gains
        float gamma1 = 0.1f;    // Feedforward adaptation gain
        float gamma2 = 0.1f;    // Feedback adaptation gain

        // Limits
        float thetaMin = 0.1f;  // Minimum gain
        float thetaMax = 100.0f; // Maximum gain
    };

    MracController() : theta1_(1.0f), theta2_(1.0f), modelState_(0) {}
    explicit MracController(const Params& params)
        : params_(params), theta1_(1.0f), theta2_(1.0f), modelState_(0) {}

    /**
     * @brief Update adaptive controller
     * @param reference Reference input
     * @param output Plant output (measurement)
     * @param dt Time step
     * @return Control signal
     */
    float update(float reference, float output, float dt) {
        // Update reference model: tau * dm/dt + m = gain * r
        // Discrete: m(k+1) = alpha * m(k) + (1-alpha) * gain * r
        float alpha = std::exp(-dt / params_.modelTimeConstant);
        modelState_ = alpha * modelState_ +
                     (1.0f - alpha) * params_.modelGain * reference;

        // Tracking error
        float error = output - modelState_;

        // MIT rule for parameter adaptation
        // dtheta1/dt = -gamma1 * e * r
        // dtheta2/dt = -gamma2 * e * y
        theta1_ -= params_.gamma1 * error * reference * dt;
        theta2_ -= params_.gamma2 * error * output * dt;

        // Apply limits
        theta1_ = std::clamp(theta1_, params_.thetaMin, params_.thetaMax);
        theta2_ = std::clamp(theta2_, params_.thetaMin, params_.thetaMax);

        // Control law: u = theta1 * r - theta2 * y
        float control = theta1_ * reference - theta2_ * output;

        return control;
    }

    /**
     * @brief Get estimated feedforward parameter
     */
    float getTheta1() const { return theta1_; }

    /**
     * @brief Get estimated feedback parameter
     */
    float getTheta2() const { return theta2_; }

    /**
     * @brief Get reference model output
     */
    float getModelOutput() const { return modelState_; }

    /**
     * @brief Reset controller
     */
    void reset() {
        theta1_ = 1.0f;
        theta2_ = 1.0f;
        modelState_ = 0;
    }

    void setParams(const Params& params) { params_ = params; }
    const Params& getParams() const { return params_; }

private:
    Params params_;
    float theta1_;      // Feedforward parameter
    float theta2_;      // Feedback parameter
    float modelState_;  // Reference model state
};

/**
 * @brief Self-Tuning Regulator (STR)
 *
 * Uses recursive least squares (RLS) for online parameter estimation
 * and pole placement for controller design.
 */
class SelfTuningRegulator {
public:
    struct Params {
        float forgettingFactor = 0.98f;  // RLS forgetting factor (0.9-1.0)
        float initialP = 1000.0f;        // Initial covariance
        float desiredPole = 0.8f;        // Desired closed-loop pole (0-1)
    };

    SelfTuningRegulator()
        : a_(0), b_(1.0f), P_(1000.0f), prevOutput_(0), prevControl_(0) {}

    explicit SelfTuningRegulator(const Params& params)
        : params_(params), a_(0), b_(1.0f), P_(params.initialP)
        , prevOutput_(0), prevControl_(0) {}

    /**
     * @brief Update self-tuning regulator
     * @param reference Reference input
     * @param output Current plant output
     * @param dt Time step (for discretization reference)
     * @return Control signal
     */
    float update(float reference, float output, float dt) {
        (void)dt;  // Using discrete model directly

        // Parameter estimation using RLS
        // Plant model: y(k) = a*y(k-1) + b*u(k-1)
        // Regressor: phi = [y(k-1), u(k-1)]'
        // Parameters: theta = [a, b]'

        // Prediction error
        float yPred = a_ * prevOutput_ + b_ * prevControl_;
        float error = output - yPred;

        // RLS update (scalar version for 2 parameters)
        float phi1 = prevOutput_;
        float phi2 = prevControl_;
        float phiTPphi = P_ * (phi1 * phi1 + phi2 * phi2);
        float gain = P_ / (params_.forgettingFactor + phiTPphi);

        // Update parameters
        a_ += gain * phi1 * error;
        b_ += gain * phi2 * error;

        // Update covariance
        P_ = (P_ - gain * phiTPphi) / params_.forgettingFactor;
        P_ = std::max(P_, 0.001f);  // Prevent numerical issues

        // Ensure b is not zero (for control calculation)
        if (std::abs(b_) < 0.01f) {
            b_ = (b_ >= 0) ? 0.01f : -0.01f;
        }

        // Pole placement controller design
        // Desired closed-loop: y(k) = p * y(k-1) + (1-p) * r(k)
        // Controller: u = (1/b) * ((p-a)*y + (1-p)*r)
        float p = params_.desiredPole;
        float control = ((p - a_) * output + (1.0f - p) * reference) / b_;

        // Store for next iteration
        prevOutput_ = output;
        prevControl_ = control;

        return control;
    }

    /**
     * @brief Get estimated 'a' parameter
     */
    float getEstimatedA() const { return a_; }

    /**
     * @brief Get estimated 'b' parameter
     */
    float getEstimatedB() const { return b_; }

    /**
     * @brief Reset estimator
     */
    void reset() {
        a_ = 0;
        b_ = 1.0f;
        P_ = params_.initialP;
        prevOutput_ = 0;
        prevControl_ = 0;
    }

    void setParams(const Params& params) { params_ = params; }
    const Params& getParams() const { return params_; }

private:
    Params params_;
    float a_, b_;       // Estimated plant parameters
    float P_;           // Covariance (scalar approximation)
    float prevOutput_;
    float prevControl_;
};

}  // namespace omni::control
