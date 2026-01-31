/**
 * @file timer.hpp
 * @brief Software timer utilities
 */

#pragma once

#include <cstdint>
#include <functional>
#include <chrono>

namespace omni::utils {

/**
 * @brief Simple elapsed time tracker
 */
class ElapsedTimer {
public:
    ElapsedTimer() : running_(false), elapsed_(0) {}

    /**
     * @brief Start/restart the timer
     */
    void start() {
        startTime_ = std::chrono::steady_clock::now();
        running_ = true;
    }

    /**
     * @brief Stop the timer
     */
    void stop() {
        if (running_) {
            elapsed_ = getElapsedUs();
            running_ = false;
        }
    }

    /**
     * @brief Reset the timer
     */
    void reset() {
        elapsed_ = 0;
        running_ = false;
    }

    /**
     * @brief Restart (reset + start)
     */
    void restart() {
        reset();
        start();
    }

    /**
     * @brief Check if timer is running
     */
    bool isRunning() const { return running_; }

    /**
     * @brief Get elapsed time in microseconds
     */
    uint64_t getElapsedUs() const {
        if (!running_) return elapsed_;

        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - startTime_);
        return static_cast<uint64_t>(duration.count());
    }

    /**
     * @brief Get elapsed time in milliseconds
     */
    uint64_t getElapsedMs() const {
        return getElapsedUs() / 1000;
    }

    /**
     * @brief Get elapsed time in seconds
     */
    float getElapsedSec() const {
        return getElapsedUs() / 1000000.0f;
    }

    /**
     * @brief Check if time has elapsed
     * @param us Microseconds to check
     */
    bool hasElapsed(uint64_t us) const {
        return getElapsedUs() >= us;
    }

private:
    std::chrono::steady_clock::time_point startTime_;
    bool running_;
    uint64_t elapsed_;
};

/**
 * @brief Periodic timer for regular callbacks
 */
class PeriodicTimer {
public:
    using Callback = std::function<void()>;

    PeriodicTimer() : periodUs_(1000), enabled_(false), lastTrigger_(0) {}

    /**
     * @brief Set timer period
     * @param periodUs Period in microseconds
     */
    void setPeriod(uint32_t periodUs) {
        periodUs_ = periodUs;
    }

    /**
     * @brief Set timer period in milliseconds
     */
    void setPeriodMs(uint32_t periodMs) {
        periodUs_ = periodMs * 1000;
    }

    /**
     * @brief Set callback function
     */
    void setCallback(Callback callback) {
        callback_ = callback;
    }

    /**
     * @brief Enable/disable timer
     */
    void enable(bool en) {
        if (en && !enabled_) {
            timer_.restart();
            lastTrigger_ = 0;
        }
        enabled_ = en;
    }

    /**
     * @brief Check and trigger callback if period elapsed
     *
     * Call this regularly from main loop.
     * @return true if callback was triggered
     */
    bool update() {
        if (!enabled_) return false;

        uint64_t elapsed = timer_.getElapsedUs();
        if (elapsed - lastTrigger_ >= periodUs_) {
            lastTrigger_ = elapsed;
            if (callback_) {
                callback_();
            }
            return true;
        }
        return false;
    }

    /**
     * @brief Get time until next trigger
     */
    uint64_t getTimeToNextUs() const {
        if (!enabled_) return 0;
        uint64_t elapsed = timer_.getElapsedUs();
        uint64_t sinceLastTrigger = elapsed - lastTrigger_;
        if (sinceLastTrigger >= periodUs_) return 0;
        return periodUs_ - sinceLastTrigger;
    }

private:
    uint32_t periodUs_;
    bool enabled_;
    uint64_t lastTrigger_;
    ElapsedTimer timer_;
    Callback callback_;
};

/**
 * @brief One-shot timer
 */
class OneShotTimer {
public:
    using Callback = std::function<void()>;

    OneShotTimer() : delayUs_(0), armed_(false) {}

    /**
     * @brief Arm the timer
     * @param delayUs Delay in microseconds
     * @param callback Function to call when timer fires
     */
    void arm(uint64_t delayUs, Callback callback = nullptr) {
        delayUs_ = delayUs;
        callback_ = callback;
        timer_.restart();
        armed_ = true;
    }

    /**
     * @brief Arm with delay in milliseconds
     */
    void armMs(uint32_t delayMs, Callback callback = nullptr) {
        arm(delayMs * 1000, callback);
    }

    /**
     * @brief Disarm the timer
     */
    void disarm() {
        armed_ = false;
    }

    /**
     * @brief Check if timer is armed
     */
    bool isArmed() const { return armed_; }

    /**
     * @brief Check if timer has fired
     */
    bool hasFired() const {
        return armed_ && timer_.hasElapsed(delayUs_);
    }

    /**
     * @brief Check and trigger callback if time elapsed
     * @return true if timer fired
     */
    bool update() {
        if (!armed_) return false;

        if (timer_.hasElapsed(delayUs_)) {
            armed_ = false;
            if (callback_) {
                callback_();
            }
            return true;
        }
        return false;
    }

    /**
     * @brief Get remaining time
     */
    uint64_t getRemainingUs() const {
        if (!armed_) return 0;
        uint64_t elapsed = timer_.getElapsedUs();
        if (elapsed >= delayUs_) return 0;
        return delayUs_ - elapsed;
    }

private:
    uint64_t delayUs_;
    bool armed_;
    ElapsedTimer timer_;
    Callback callback_;
};

/**
 * @brief Rate limiter to throttle operations
 */
class RateLimiter {
public:
    /**
     * @brief Construct rate limiter
     * @param minIntervalUs Minimum interval between operations
     */
    explicit RateLimiter(uint64_t minIntervalUs = 1000)
        : minIntervalUs_(minIntervalUs) {
        timer_.start();
        lastOperation_ = 0;
    }

    /**
     * @brief Check if operation is allowed
     * @return true if operation can proceed
     */
    bool check() {
        uint64_t now = timer_.getElapsedUs();
        if (now - lastOperation_ >= minIntervalUs_) {
            lastOperation_ = now;
            return true;
        }
        return false;
    }

    /**
     * @brief Set minimum interval
     */
    void setMinInterval(uint64_t us) {
        minIntervalUs_ = us;
    }

    /**
     * @brief Reset the limiter
     */
    void reset() {
        lastOperation_ = 0;
        timer_.restart();
    }

private:
    uint64_t minIntervalUs_;
    uint64_t lastOperation_;
    ElapsedTimer timer_;
};

}  // namespace omni::utils
