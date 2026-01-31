/**
 * @file homing_manager.hpp
 * @brief Motor homing sequence manager
 */

#pragma once

#include "motor_controller.hpp"
#include "omni/hal/gpio.hpp"
#include <functional>

namespace omni::app {

/**
 * @brief Homing error codes
 */
enum class HomingError {
    None,
    Timeout,
    SensorNotFound,
    MotorFault,
    NotEnabled,
    AlreadyHomed,
    Aborted
};

/**
 * @brief Homing sequence configuration
 */
struct HomingConfig {
    HomingMode mode = HomingMode::SensorBased;
    float searchVelocity = 1.0f;       // Velocity for initial search
    float approachVelocity = 0.1f;     // Velocity for final approach
    float offset = 0.0f;               // Offset after finding home
    float currentThreshold = 5.0f;     // Current threshold (for CurrentBased mode)
    bool reverseDirection = false;      // Search in negative direction
    float timeout = 30.0f;             // Timeout in seconds
    float backoffDistance = 0.1f;      // Distance to back off after finding home
};

/**
 * @brief Homing manager
 *
 * Manages automatic homing sequences for motor axes.
 */
class HomingManager {
public:
    /**
     * @brief Homing state
     */
    enum class State {
        Idle,
        SearchingFast,
        BackingOff,
        SearchingSlow,
        MovingToOffset,
        Complete,
        Error
    };

    /**
     * @brief Construct homing manager
     * @param controller Motor controller
     * @param config Homing configuration
     */
    HomingManager(MotorController* controller, const HomingConfig& config)
        : controller_(controller)
        , config_(config)
        , state_(State::Idle)
        , error_(HomingError::None)
        , elapsedTime_(0)
        , homePosition_(0) {}

    /**
     * @brief Set limit switch GPIO
     * @param limitSwitch GPIO for home/limit sensor
     */
    void setLimitSwitch(hal::IGpio* limitSwitch) {
        limitSwitch_ = limitSwitch;
    }

    /**
     * @brief Start homing sequence
     */
    void start() {
        if (state_ != State::Idle && state_ != State::Complete &&
            state_ != State::Error) {
            return;
        }

        if (!controller_ || !controller_->isEnabled()) {
            error_ = HomingError::NotEnabled;
            state_ = State::Error;
            return;
        }

        // Check for AbsoluteEncoder mode (no motion needed)
        if (config_.mode == HomingMode::AbsoluteEncoder) {
            homePosition_ = 0;  // Use current position as reference
            controller_->setHomeOffset(controller_->getPosition());
            state_ = State::Complete;
            return;
        }

        // Check for Manual mode
        if (config_.mode == HomingMode::Manual) {
            state_ = State::Complete;
            return;
        }

        // Start search
        error_ = HomingError::None;
        elapsedTime_ = 0;
        state_ = State::SearchingFast;

        float searchDir = config_.reverseDirection ? -1.0f : 1.0f;
        controller_->setVelocity(config_.searchVelocity * searchDir);
    }

    /**
     * @brief Abort homing sequence
     */
    void abort() {
        if (state_ != State::Idle && state_ != State::Complete) {
            controller_->setVelocity(0);
            error_ = HomingError::Aborted;
            state_ = State::Error;
        }
    }

    /**
     * @brief Check if homing is running
     */
    bool isRunning() const {
        return state_ != State::Idle && state_ != State::Complete &&
               state_ != State::Error;
    }

    /**
     * @brief Check if homing is complete
     */
    bool isCompleted() const {
        return state_ == State::Complete;
    }

    /**
     * @brief Get homing error
     */
    HomingError getError() const { return error_; }

    /**
     * @brief Get current state
     */
    State getState() const { return state_; }

    /**
     * @brief Update homing sequence
     * @param dt Time step
     */
    void update(float dt) {
        if (state_ == State::Idle || state_ == State::Complete ||
            state_ == State::Error) {
            return;
        }

        elapsedTime_ += dt;

        // Check timeout
        if (elapsedTime_ > config_.timeout) {
            controller_->setVelocity(0);
            error_ = HomingError::Timeout;
            state_ = State::Error;
            return;
        }

        // Check motor fault
        if (controller_->hasError()) {
            error_ = HomingError::MotorFault;
            state_ = State::Error;
            return;
        }

        switch (state_) {
            case State::SearchingFast:
                updateSearchingFast();
                break;

            case State::BackingOff:
                updateBackingOff();
                break;

            case State::SearchingSlow:
                updateSearchingSlow();
                break;

            case State::MovingToOffset:
                updateMovingToOffset();
                break;

            default:
                break;
        }
    }

    /**
     * @brief Set homing configuration
     */
    void setConfig(const HomingConfig& config) {
        config_ = config;
    }

    /**
     * @brief Get homing configuration
     */
    const HomingConfig& getConfig() const { return config_; }

private:
    MotorController* controller_;
    HomingConfig config_;
    hal::IGpio* limitSwitch_ = nullptr;

    State state_;
    HomingError error_;
    float elapsedTime_;
    float homePosition_;
    float backoffStartPos_;

    bool isHomeSensorTriggered() const {
        switch (config_.mode) {
            case HomingMode::SensorBased:
                return limitSwitch_ && limitSwitch_->read();

            case HomingMode::CurrentBased:
                return std::abs(controller_->getTorque()) > config_.currentThreshold;

            case HomingMode::Hardstop:
                // Detect by velocity dropping near zero while commanding movement
                return std::abs(controller_->getVelocity()) < 0.01f;

            case HomingMode::IndexPulse:
                // Would need encoder index signal
                return false;

            default:
                return false;
        }
    }

    void updateSearchingFast() {
        if (isHomeSensorTriggered()) {
            // Found home sensor, back off
            controller_->setVelocity(0);
            backoffStartPos_ = controller_->getPosition();
            state_ = State::BackingOff;

            float backoffDir = config_.reverseDirection ? 1.0f : -1.0f;
            controller_->moveBy(config_.backoffDistance * backoffDir);
        }
    }

    void updateBackingOff() {
        if (controller_->isSettled()) {
            // Start slow approach
            state_ = State::SearchingSlow;
            float searchDir = config_.reverseDirection ? -1.0f : 1.0f;
            controller_->setVelocity(config_.approachVelocity * searchDir);
        }
    }

    void updateSearchingSlow() {
        if (isHomeSensorTriggered()) {
            // Found home precisely
            controller_->setVelocity(0);
            homePosition_ = controller_->getPosition();

            // Move to offset
            if (std::abs(config_.offset) > 0.0001f) {
                state_ = State::MovingToOffset;
                controller_->moveTo(homePosition_ + config_.offset);
            } else {
                completeHoming();
            }
        }
    }

    void updateMovingToOffset() {
        if (controller_->isSettled()) {
            completeHoming();
        }
    }

    void completeHoming() {
        // Set current position as home
        controller_->setHomeOffset(controller_->getPosition());
        state_ = State::Complete;
    }
};

}  // namespace omni::app
