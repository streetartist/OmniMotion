/**
 * @file pwm.hpp
 * @brief PWM hardware abstraction interface
 */

#pragma once

#include <cstdint>

namespace omni::hal {

/**
 * @brief Single channel PWM interface
 *
 * Abstract interface for PWM output control including frequency,
 * duty cycle, and dead time configuration.
 */
class IPwm {
public:
    virtual ~IPwm() = default;

    /**
     * @brief Set the PWM frequency
     * @param freq_hz Frequency in Hz
     */
    virtual void setFrequency(uint32_t freq_hz) = 0;

    /**
     * @brief Get the current PWM frequency
     * @return Frequency in Hz
     */
    virtual uint32_t getFrequency() const = 0;

    /**
     * @brief Set duty cycle as a ratio
     * @param duty Duty cycle from 0.0 to 1.0
     */
    virtual void setDuty(float duty) = 0;

    /**
     * @brief Set duty cycle in nanoseconds
     * @param ns High time in nanoseconds
     */
    virtual void setDutyNs(uint32_t ns) = 0;

    /**
     * @brief Get the current duty cycle
     * @return Duty cycle from 0.0 to 1.0
     */
    virtual float getDuty() const = 0;

    /**
     * @brief Enable or disable the PWM output
     * @param en true to enable, false to disable
     */
    virtual void enable(bool en) = 0;

    /**
     * @brief Check if PWM is enabled
     * @return true if enabled
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Set dead time for complementary outputs
     * @param ns Dead time in nanoseconds
     */
    virtual void setDeadTime(uint32_t ns) = 0;

    /**
     * @brief Get configured dead time
     * @return Dead time in nanoseconds
     */
    virtual uint32_t getDeadTime() const = 0;
};

/**
 * @brief Three-phase PWM interface for motor control
 *
 * Specialized interface for controlling three-phase motor drives,
 * supporting both individual phase control and space vector modulation.
 */
class IPwm3Phase {
public:
    virtual ~IPwm3Phase() = default;

    /**
     * @brief Set duty cycles for all three phases
     * @param a Phase A duty (0.0 to 1.0)
     * @param b Phase B duty (0.0 to 1.0)
     * @param c Phase C duty (0.0 to 1.0)
     */
    virtual void setDuty(float a, float b, float c) = 0;

    /**
     * @brief Get current duty cycles
     * @param a Output for phase A duty
     * @param b Output for phase B duty
     * @param c Output for phase C duty
     */
    virtual void getDuty(float& a, float& b, float& c) const = 0;

    /**
     * @brief Enable or disable all phases
     * @param en true to enable, false to disable
     */
    virtual void enable(bool en) = 0;

    /**
     * @brief Check if PWM is enabled
     * @return true if enabled
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Set dead time for all phases
     * @param ns Dead time in nanoseconds
     */
    virtual void setDeadTime(uint32_t ns) = 0;

    /**
     * @brief Set PWM frequency
     * @param freq_hz Frequency in Hz
     */
    virtual void setFrequency(uint32_t freq_hz) = 0;

    /**
     * @brief Apply space vector PWM directly from alpha-beta components
     * @param alpha Alpha component (-1.0 to 1.0)
     * @param beta Beta component (-1.0 to 1.0)
     */
    virtual void setSvpwm(float alpha, float beta) = 0;

    /**
     * @brief Enable/disable individual phases (for six-step commutation)
     * @param enA Enable phase A
     * @param enB Enable phase B
     * @param enC Enable phase C
     */
    virtual void setPhaseEnable(bool enA, bool enB, bool enC) = 0;
};

}  // namespace omni::hal
