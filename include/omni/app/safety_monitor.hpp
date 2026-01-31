/**
 * @file safety_monitor.hpp
 * @brief Motor safety monitoring and protection
 */

#pragma once

#include "omni/driver/motor_driver.hpp"
#include <functional>
#include <cmath>

namespace omni::app {

/**
 * @brief Safety fault types
 */
enum class SafetyFault : uint32_t {
    None = 0,
    PositionLimitMin = 1 << 0,
    PositionLimitMax = 1 << 1,
    VelocityLimit = 1 << 2,
    CurrentLimit = 1 << 3,
    TemperatureLimit = 1 << 4,
    FollowingError = 1 << 5,
    DriverFault = 1 << 6,
    CommunicationLoss = 1 << 7,
    EmergencyStop = 1 << 8
};

inline SafetyFault operator|(SafetyFault a, SafetyFault b) {
    return static_cast<SafetyFault>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline SafetyFault operator&(SafetyFault a, SafetyFault b) {
    return static_cast<SafetyFault>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline SafetyFault& operator|=(SafetyFault& a, SafetyFault b) {
    a = a | b;
    return a;
}

/**
 * @brief Safety monitor
 *
 * Monitors motor operation and triggers faults when limits are exceeded.
 */
class SafetyMonitor {
public:
    /**
     * @brief Fault callback type
     */
    using FaultCallback = std::function<void(SafetyFault)>;

    /**
     * @brief Construct safety monitor
     * @param driver Motor driver to monitor
     */
    explicit SafetyMonitor(driver::IMotorDriver* driver)
        : driver_(driver)
        , fault_(SafetyFault::None)
        , positionMin_(-1e6f)
        , positionMax_(1e6f)
        , maxVelocity_(1e6f)
        , maxCurrent_(100.0f)
        , maxTemperature_(80.0f)
        , followingErrorThreshold_(0.1f)
        , positionTarget_(0)
        , positionMonitorEnabled_(false)
        , velocityMonitorEnabled_(false)
        , currentMonitorEnabled_(false)
        , temperatureMonitorEnabled_(false)
        , followingErrorMonitorEnabled_(false) {}

    // === Limit Configuration ===

    /**
     * @brief Set position limits
     * @param min Minimum position
     * @param max Maximum position
     */
    void setPositionLimits(float min, float max) {
        positionMin_ = min;
        positionMax_ = max;
    }

    /**
     * @brief Set velocity limit
     * @param max Maximum velocity magnitude
     */
    void setVelocityLimit(float max) {
        maxVelocity_ = max;
    }

    /**
     * @brief Set current limit
     * @param max Maximum current magnitude
     */
    void setCurrentLimit(float max) {
        maxCurrent_ = max;
    }

    /**
     * @brief Set temperature limit
     * @param max Maximum temperature
     */
    void setTemperatureLimit(float max) {
        maxTemperature_ = max;
    }

    // === Monitor Enable ===

    /**
     * @brief Enable position monitoring
     * @param enable true to enable
     */
    void enablePositionMonitor(bool enable) {
        positionMonitorEnabled_ = enable;
    }

    /**
     * @brief Enable velocity monitoring
     * @param enable true to enable
     */
    void enableVelocityMonitor(bool enable) {
        velocityMonitorEnabled_ = enable;
    }

    /**
     * @brief Enable current monitoring
     * @param enable true to enable
     */
    void enableCurrentMonitor(bool enable) {
        currentMonitorEnabled_ = enable;
    }

    /**
     * @brief Enable temperature monitoring
     * @param enable true to enable
     */
    void enableTemperatureMonitor(bool enable) {
        temperatureMonitorEnabled_ = enable;
    }

    /**
     * @brief Enable following error monitoring
     * @param enable true to enable
     * @param threshold Maximum allowed error
     */
    void enableFollowingErrorMonitor(bool enable, float threshold = 0.1f) {
        followingErrorMonitorEnabled_ = enable;
        followingErrorThreshold_ = threshold;
    }

    /**
     * @brief Set position target for following error calculation
     * @param target Target position
     */
    void setPositionTarget(float target) {
        positionTarget_ = target;
    }

    // === Status ===

    /**
     * @brief Check if any fault is active
     */
    bool hasFault() const {
        return fault_ != SafetyFault::None;
    }

    /**
     * @brief Get current fault flags
     */
    SafetyFault getFault() const {
        return fault_;
    }

    /**
     * @brief Clear all faults
     */
    void clearFault() {
        fault_ = SafetyFault::None;
    }

    /**
     * @brief Set fault callback
     * @param callback Function to call when fault occurs
     */
    void setFaultCallback(FaultCallback callback) {
        faultCallback_ = callback;
    }

    // === Update ===

    /**
     * @brief Update safety checks
     *
     * Should be called regularly in control loop.
     */
    void update() {
        if (!driver_) return;

        SafetyFault newFault = SafetyFault::None;

        // Position limits
        if (positionMonitorEnabled_) {
            float pos = driver_->getPosition();
            if (pos < positionMin_) {
                newFault |= SafetyFault::PositionLimitMin;
            }
            if (pos > positionMax_) {
                newFault |= SafetyFault::PositionLimitMax;
            }
        }

        // Velocity limit
        if (velocityMonitorEnabled_) {
            float vel = std::abs(driver_->getVelocity());
            if (vel > maxVelocity_) {
                newFault |= SafetyFault::VelocityLimit;
            }
        }

        // Current limit
        if (currentMonitorEnabled_) {
            float current = std::abs(driver_->getCurrent());
            if (current > maxCurrent_) {
                newFault |= SafetyFault::CurrentLimit;
            }
        }

        // Temperature limit
        if (temperatureMonitorEnabled_) {
            float temp = driver_->getTemperature();
            if (temp > maxTemperature_) {
                newFault |= SafetyFault::TemperatureLimit;
            }
        }

        // Following error
        if (followingErrorMonitorEnabled_) {
            float error = std::abs(driver_->getPosition() - positionTarget_);
            if (error > followingErrorThreshold_) {
                newFault |= SafetyFault::FollowingError;
            }
        }

        // Driver fault
        if (driver_->hasFault()) {
            newFault |= SafetyFault::DriverFault;
        }

        // Check for new faults
        SafetyFault justTriggered = static_cast<SafetyFault>(
            static_cast<uint32_t>(newFault) &
            ~static_cast<uint32_t>(fault_));

        fault_ = newFault;

        // Call callback for new faults
        if (justTriggered != SafetyFault::None && faultCallback_) {
            faultCallback_(justTriggered);
        }
    }

    /**
     * @brief Trigger emergency stop fault
     */
    void triggerEmergencyStop() {
        fault_ |= SafetyFault::EmergencyStop;
        if (faultCallback_) {
            faultCallback_(SafetyFault::EmergencyStop);
        }
    }

private:
    driver::IMotorDriver* driver_;

    SafetyFault fault_;
    FaultCallback faultCallback_;

    // Limits
    float positionMin_, positionMax_;
    float maxVelocity_;
    float maxCurrent_;
    float maxTemperature_;
    float followingErrorThreshold_;
    float positionTarget_;

    // Enable flags
    bool positionMonitorEnabled_;
    bool velocityMonitorEnabled_;
    bool currentMonitorEnabled_;
    bool temperatureMonitorEnabled_;
    bool followingErrorMonitorEnabled_;
};

}  // namespace omni::app
