/**
 * @file disturbance_observer.hpp
 * @brief Disturbance Observer (DOB) for motor control
 */

#pragma once

#include "filters.hpp"
#include <cmath>

namespace omni::control {

/**
 * @brief Disturbance Observer
 *
 * Estimates external disturbances and load variations for compensation.
 * Based on the inverse nominal model approach.
 */
class DisturbanceObserver {
public:
    /**
     * @brief Construct DOB
     * @param bandwidth Observer bandwidth (Hz)
     * @param inertia Nominal inertia (kg*m^2 or kg)
     */
    DisturbanceObserver(float bandwidth, float inertia)
        : bandwidth_(bandwidth)
        , inertia_(inertia)
        , ktConstant_(1.0f)
        , disturbance_(0)
        , compensation_(0)
        , prevVelocity_(0)
        , enabled_(true) {
        updateFilter();
    }

    /**
     * @brief Estimate disturbance from current and velocity
     * @param current Motor current (A)
     * @param velocity Motor velocity (rad/s or m/s)
     * @param dt Time step
     * @return Estimated disturbance torque/force
     */
    float estimate(float current, float velocity, float dt) {
        if (!enabled_) {
            disturbance_ = 0;
            compensation_ = 0;
            return 0;
        }

        // Estimated torque from current
        float torqueFromCurrent = ktConstant_ * current;

        // Estimated acceleration (differentiated velocity)
        float acceleration = (velocity - prevVelocity_) / dt;
        prevVelocity_ = velocity;

        // Inverse model: torque needed for this acceleration
        float torqueFromModel = inertia_ * acceleration;

        // Raw disturbance estimate
        float rawDisturbance = torqueFromCurrent - torqueFromModel;

        // Low-pass filter the disturbance estimate
        disturbance_ = lpf_.filter(rawDisturbance);

        // Compensation is negative of disturbance
        compensation_ = -disturbance_;

        return disturbance_;
    }

    /**
     * @brief Get disturbance compensation value
     * @return Compensation torque/force to add to control output
     */
    float getCompensation() const { return compensation_; }

    /**
     * @brief Get estimated disturbance
     * @return Estimated disturbance
     */
    float getDisturbance() const { return disturbance_; }

    /**
     * @brief Set observer bandwidth
     * @param bandwidth Bandwidth in Hz
     */
    void setBandwidth(float bandwidth) {
        bandwidth_ = bandwidth;
        updateFilter();
    }

    /**
     * @brief Get observer bandwidth
     * @return Bandwidth in Hz
     */
    float getBandwidth() const { return bandwidth_; }

    /**
     * @brief Set nominal inertia
     * @param inertia Inertia (kg*m^2 or kg)
     */
    void setInertia(float inertia) { inertia_ = inertia; }

    /**
     * @brief Get nominal inertia
     * @return Inertia
     */
    float getInertia() const { return inertia_; }

    /**
     * @brief Set torque constant
     * @param kt Torque constant (Nm/A)
     */
    void setTorqueConstant(float kt) { ktConstant_ = kt; }

    /**
     * @brief Get torque constant
     * @return Torque constant
     */
    float getTorqueConstant() const { return ktConstant_; }

    /**
     * @brief Enable/disable observer
     * @param enable true to enable
     */
    void enable(bool enable) { enabled_ = enable; }

    /**
     * @brief Check if enabled
     * @return true if enabled
     */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Reset observer state
     */
    void reset() {
        disturbance_ = 0;
        compensation_ = 0;
        prevVelocity_ = 0;
        lpf_.reset();
    }

private:
    float bandwidth_;
    float inertia_;
    float ktConstant_;
    float disturbance_;
    float compensation_;
    float prevVelocity_;
    bool enabled_;
    LowPassFilter lpf_;

    void updateFilter() {
        // Assuming 1kHz default sample rate
        lpf_ = LowPassFilter(bandwidth_, 1000.0f);
    }
};

/**
 * @brief Extended State Observer (ESO)
 *
 * Part of Active Disturbance Rejection Control (ADRC).
 * Estimates both the state and lumped disturbance.
 */
class ExtendedStateObserver {
public:
    /**
     * @brief Construct ESO
     * @param bandwidth Observer bandwidth (Hz)
     * @param b0 Control gain estimate
     */
    ExtendedStateObserver(float bandwidth, float b0)
        : bandwidth_(bandwidth)
        , b0_(b0)
        , x1_(0), x2_(0), x3_(0)  // position, velocity, disturbance
        , enabled_(true) {
        updateGains();
    }

    /**
     * @brief Update observer
     * @param control Control input (u)
     * @param position Position measurement
     * @param dt Time step
     */
    void update(float control, float position, float dt) {
        if (!enabled_) return;

        // Observer error
        float e = position - x1_;

        // ESO dynamics (second-order plant + disturbance)
        float dx1 = x2_ + beta1_ * e;
        float dx2 = x3_ + b0_ * control + beta2_ * e;
        float dx3 = beta3_ * e;

        // Integrate
        x1_ += dx1 * dt;
        x2_ += dx2 * dt;
        x3_ += dx3 * dt;
    }

    /**
     * @brief Get estimated position
     */
    float getPosition() const { return x1_; }

    /**
     * @brief Get estimated velocity
     */
    float getVelocity() const { return x2_; }

    /**
     * @brief Get estimated disturbance (lumped uncertainty)
     */
    float getDisturbance() const { return x3_; }

    /**
     * @brief Get disturbance compensation
     * @return Compensation value (-disturbance/b0)
     */
    float getCompensation() const {
        return (std::abs(b0_) > 1e-6f) ? -x3_ / b0_ : 0;
    }

    /**
     * @brief Set observer bandwidth
     * @param bandwidth Bandwidth (Hz)
     */
    void setBandwidth(float bandwidth) {
        bandwidth_ = bandwidth;
        updateGains();
    }

    /**
     * @brief Set control gain estimate
     * @param b0 Control gain
     */
    void setB0(float b0) { b0_ = b0; }

    /**
     * @brief Reset observer
     */
    void reset() {
        x1_ = x2_ = x3_ = 0;
    }

    /**
     * @brief Reset with initial position
     * @param position Initial position
     */
    void reset(float position) {
        x1_ = position;
        x2_ = 0;
        x3_ = 0;
    }

    void enable(bool en) { enabled_ = en; }
    bool isEnabled() const { return enabled_; }

private:
    float bandwidth_;
    float b0_;
    float x1_, x2_, x3_;
    float beta1_, beta2_, beta3_;
    bool enabled_;

    void updateGains() {
        // Place observer poles at -bandwidth (rad/s)
        float wo = 2.0f * 3.14159f * bandwidth_;
        beta1_ = 3.0f * wo;
        beta2_ = 3.0f * wo * wo;
        beta3_ = wo * wo * wo;
    }
};

}  // namespace omni::control
