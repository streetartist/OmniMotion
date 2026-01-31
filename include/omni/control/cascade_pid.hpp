/**
 * @file cascade_pid.hpp
 * @brief Cascade PID controller for motor control (position-velocity-current)
 */

#pragma once

#include "pid_controller.hpp"

namespace omni::control {

/**
 * @brief Cascade control mode
 */
enum class CascadeMode {
    Position,   // Position -> Velocity -> Current
    Velocity,   // Velocity -> Current
    Current     // Current only
};

/**
 * @brief Cascade PID controller
 *
 * Implements a three-loop cascade controller typically used in
 * servo motor control: position -> velocity -> current/torque.
 */
class CascadePidController {
public:
    CascadePidController()
        : mode_(CascadeMode::Position)
        , positionOutput_(0)
        , velocityOutput_(0)
        , currentOutput_(0) {}

    /**
     * @brief Set position loop PID parameters
     * @param params PID parameters
     */
    void setPositionPid(const PidController::Params& params) {
        positionPid_.setParams(params);
    }

    /**
     * @brief Set velocity loop PID parameters
     * @param params PID parameters
     */
    void setVelocityPid(const PidController::Params& params) {
        velocityPid_.setParams(params);
    }

    /**
     * @brief Set current loop PID parameters
     * @param params PID parameters
     */
    void setCurrentPid(const PidController::Params& params) {
        currentPid_.setParams(params);
    }

    /**
     * @brief Set position PID gains
     */
    void setPositionGains(float kp, float ki, float kd) {
        positionPid_.setGains(kp, ki, kd);
    }

    /**
     * @brief Set velocity PID gains
     */
    void setVelocityGains(float kp, float ki, float kd) {
        velocityPid_.setGains(kp, ki, kd);
    }

    /**
     * @brief Set current PID gains
     */
    void setCurrentGains(float kp, float ki, float kd) {
        currentPid_.setGains(kp, ki, kd);
    }

    /**
     * @brief Set control mode
     * @param mode Cascade mode
     */
    void setMode(CascadeMode mode) {
        if (mode != mode_) {
            mode_ = mode;
            reset();
        }
    }

    /**
     * @brief Get current mode
     * @return Cascade mode
     */
    CascadeMode getMode() const { return mode_; }

    /**
     * @brief Update cascade controller
     * @param posRef Position reference (used in Position mode)
     * @param velRef Velocity reference (used in Velocity mode, or feedforward)
     * @param curRef Current reference (used in Current mode, or feedforward)
     * @param posFb Position feedback
     * @param velFb Velocity feedback
     * @param curFb Current feedback
     * @param dt Time step
     * @return Current/torque command
     */
    float update(float posRef, float velRef, float curRef,
                 float posFb, float velFb, float curFb, float dt) {
        switch (mode_) {
            case CascadeMode::Position: {
                // Position loop outputs velocity reference
                positionOutput_ = positionPid_.update(posRef, posFb, dt);
                // Add velocity feedforward
                float velCommand = positionOutput_ + velRef;
                // Velocity loop outputs current reference
                velocityOutput_ = velocityPid_.update(velCommand, velFb, dt);
                // Add current feedforward
                float curCommand = velocityOutput_ + curRef;
                // Current loop
                currentOutput_ = currentPid_.update(curCommand, curFb, dt);
                break;
            }

            case CascadeMode::Velocity: {
                positionOutput_ = 0;
                // Velocity loop outputs current reference
                velocityOutput_ = velocityPid_.update(velRef, velFb, dt);
                // Add current feedforward
                float curCommand = velocityOutput_ + curRef;
                // Current loop
                currentOutput_ = currentPid_.update(curCommand, curFb, dt);
                break;
            }

            case CascadeMode::Current: {
                positionOutput_ = 0;
                velocityOutput_ = 0;
                // Current loop only
                currentOutput_ = currentPid_.update(curRef, curFb, dt);
                break;
            }
        }

        return currentOutput_;
    }

    /**
     * @brief Simplified update for position control
     * @param posRef Position reference
     * @param posFb Position feedback
     * @param velFb Velocity feedback
     * @param curFb Current feedback
     * @param dt Time step
     * @return Current command
     */
    float update(float posRef, float posFb, float velFb, float curFb, float dt) {
        return update(posRef, 0, 0, posFb, velFb, curFb, dt);
    }

    /**
     * @brief Reset all controllers
     */
    void reset() {
        positionPid_.reset();
        velocityPid_.reset();
        currentPid_.reset();
        positionOutput_ = 0;
        velocityOutput_ = 0;
        currentOutput_ = 0;
    }

    /**
     * @brief Reset with initial values
     */
    void reset(float position, float velocity, float current) {
        positionPid_.reset(position);
        velocityPid_.reset(velocity);
        currentPid_.reset(current);
        positionOutput_ = 0;
        velocityOutput_ = 0;
        currentOutput_ = 0;
    }

    // === Getters ===

    float getPositionOutput() const { return positionOutput_; }
    float getVelocityOutput() const { return velocityOutput_; }
    float getCurrentOutput() const { return currentOutput_; }

    PidController& getPositionPid() { return positionPid_; }
    PidController& getVelocityPid() { return velocityPid_; }
    PidController& getCurrentPid() { return currentPid_; }

    const PidController& getPositionPid() const { return positionPid_; }
    const PidController& getVelocityPid() const { return velocityPid_; }
    const PidController& getCurrentPid() const { return currentPid_; }

    // === Output limits ===

    void setPositionOutputLimit(float limit) {
        positionPid_.setOutputLimit(limit);
    }

    void setVelocityOutputLimit(float limit) {
        velocityPid_.setOutputLimit(limit);
    }

    void setCurrentOutputLimit(float limit) {
        currentPid_.setOutputLimit(limit);
    }

private:
    CascadeMode mode_;

    PidController positionPid_;
    PidController velocityPid_;
    PidController currentPid_;

    float positionOutput_;
    float velocityOutput_;
    float currentOutput_;
};

}  // namespace omni::control
