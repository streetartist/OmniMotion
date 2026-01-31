/**
 * @file timer.hpp
 * @brief Timer hardware abstraction interface
 */

#pragma once

#include <cstdint>
#include <functional>

namespace omni::hal {

/**
 * @brief Timer callback type
 */
using TimerCallback = std::function<void()>;

/**
 * @brief Hardware timer interface
 *
 * Abstract interface for hardware timers with support for
 * periodic callbacks and one-shot operation.
 */
class ITimer {
public:
    virtual ~ITimer() = default;

    /**
     * @brief Set timer period
     * @param periodUs Period in microseconds
     */
    virtual void setPeriod(uint32_t periodUs) = 0;

    /**
     * @brief Get timer period
     * @return Period in microseconds
     */
    virtual uint32_t getPeriod() const = 0;

    /**
     * @brief Start the timer
     */
    virtual void start() = 0;

    /**
     * @brief Stop the timer
     */
    virtual void stop() = 0;

    /**
     * @brief Check if timer is running
     * @return true if running
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Reset timer counter
     */
    virtual void reset() = 0;

    /**
     * @brief Get current counter value
     * @return Counter value in microseconds
     */
    virtual uint32_t getCounter() const = 0;

    /**
     * @brief Set callback for timer overflow/match
     * @param callback Function to call
     */
    virtual void setCallback(TimerCallback callback) = 0;

    /**
     * @brief Enable/disable timer interrupt
     * @param enable true to enable
     */
    virtual void enableInterrupt(bool enable) = 0;

    /**
     * @brief Set one-shot mode
     * @param oneShot true for one-shot, false for periodic
     */
    virtual void setOneShot(bool oneShot) = 0;
};

/**
 * @brief High resolution timestamp interface
 */
class ITimestamp {
public:
    virtual ~ITimestamp() = default;

    /**
     * @brief Get current timestamp in microseconds
     * @return Timestamp in microseconds
     */
    virtual uint64_t getMicros() const = 0;

    /**
     * @brief Get current timestamp in nanoseconds
     * @return Timestamp in nanoseconds
     */
    virtual uint64_t getNanos() const = 0;

    /**
     * @brief Get elapsed time since a previous timestamp
     * @param startMicros Previous timestamp
     * @return Elapsed microseconds
     */
    virtual uint64_t elapsedMicros(uint64_t startMicros) const = 0;
};

}  // namespace omni::hal
