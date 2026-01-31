/**
 * @file stepper_driver.hpp
 * @brief Stepper motor driver
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"

namespace omni::driver {

/**
 * @brief Stepper motor driver
 *
 * Supports step/direction interface with various microstepping modes.
 * Compatible with common stepper drivers like A4988, DRV8825, TMC2209, etc.
 */
class StepperDriver : public IMotorDriver {
public:
    /**
     * @brief Construct stepper driver with step/dir interface
     * @param stepPin Step pulse GPIO
     * @param dirPin Direction GPIO
     * @param enablePin Enable GPIO (optional, active low)
     */
    StepperDriver(hal::IGpio* stepPin,
                  hal::IGpio* dirPin,
                  hal::IGpio* enablePin = nullptr);

    /**
     * @brief Construct stepper driver with step/dir and encoder feedback
     * @param stepPin Step pulse GPIO
     * @param dirPin Direction GPIO
     * @param encoder Encoder for position feedback
     * @param enablePin Enable GPIO (optional)
     */
    StepperDriver(hal::IGpio* stepPin,
                  hal::IGpio* dirPin,
                  hal::IEncoder* encoder,
                  hal::IGpio* enablePin = nullptr);

    ~StepperDriver() override = default;

    // IMotorDriver interface
    bool init() override;
    void deinit() override;
    void enable() override;
    void disable() override;
    bool isEnabled() const override;
    void emergencyStop() override;

    void setControlMode(ControlMode mode) override;
    ControlMode getControlMode() const override;

    void setPosition(float pos) override;
    void setVelocity(float vel) override;
    void setTorque(float torque) override;
    void setCurrent(float current) override;
    void setVoltage(float voltage) override;
    void setDuty(float duty) override;

    MotorState getState() const override;
    float getPosition() const override;
    float getVelocity() const override;
    float getTorque() const override;
    float getCurrent() const override;
    float getTemperature() const override;

    void setParams(const MotorParams& params) override;
    MotorParams getParams() const override;

    void update() override;

    MotorType getType() const override { return MotorType::Stepper; }
    const char* getName() const override { return "Stepper Motor Driver"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== Stepper Specific =====

    /**
     * @brief Set microstepping mode
     * @param microstep Microstep divisor (1, 2, 4, 8, 16, 32, etc.)
     */
    void setMicrostep(uint16_t microstep);

    /**
     * @brief Get current microstepping mode
     * @return Microstep divisor
     */
    uint16_t getMicrostep() const { return microstep_; }

    /**
     * @brief Set step mode
     * @param mode Step mode enum
     */
    void setStepMode(StepMode mode);

    /**
     * @brief Step a specific number of steps
     * @param steps Number of steps (negative for reverse)
     */
    void step(int32_t steps);

    /**
     * @brief Set full steps per revolution (before microstepping)
     * @param steps Steps per revolution
     */
    void setStepsPerRev(uint32_t steps);

    /**
     * @brief Get steps per revolution
     * @return Steps per revolution
     */
    uint32_t getStepsPerRev() const { return stepsPerRev_; }

    /**
     * @brief Get current step position
     * @return Step count
     */
    int32_t getStepPosition() const { return stepPosition_; }

    /**
     * @brief Reset step position to zero
     */
    void resetStepPosition();

    /**
     * @brief Set step position
     * @param steps Step position
     */
    void setStepPosition(int32_t steps);

    /**
     * @brief Set maximum step rate
     * @param stepsPerSec Maximum steps per second
     */
    void setMaxStepRate(uint32_t stepsPerSec);

    /**
     * @brief Set step pulse width
     * @param ns Pulse width in nanoseconds
     */
    void setStepPulseWidth(uint32_t ns);

    /**
     * @brief Invert direction
     * @param invert true to invert
     */
    void setDirectionInvert(bool invert);

    // ===== Advanced Driver Features (TMC series) =====

    /**
     * @brief Enable StealthChop mode (silent)
     * @param enable true to enable
     */
    void setStealthChop(bool enable);

    /**
     * @brief Enable SpreadCycle mode (high torque)
     * @param enable true to enable
     */
    void setSpreadCycle(bool enable);

    /**
     * @brief Set StallGuard threshold
     * @param threshold Stall detection threshold
     */
    void setStallGuard(uint8_t threshold);

    /**
     * @brief Check if motor is stalled
     * @return true if stall detected
     */
    bool isStalled() const;

    /**
     * @brief Set run current (% of max)
     * @param percent Current percentage (0-100)
     */
    void setRunCurrent(uint8_t percent);

    /**
     * @brief Set hold current (% of max)
     * @param percent Current percentage (0-100)
     */
    void setHoldCurrent(uint8_t percent);

private:
    // Hardware interfaces
    hal::IGpio* stepPin_;
    hal::IGpio* dirPin_;
    hal::IGpio* enablePin_;
    hal::IEncoder* encoder_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Stepper state
    int32_t stepPosition_;
    int32_t targetStepPosition_;
    float velocity_;
    float targetVelocity_;
    uint32_t stepsPerRev_;
    uint16_t microstep_;
    bool directionInvert_;

    // Timing
    uint32_t stepPulseWidthNs_;
    uint32_t maxStepRate_;
    uint64_t lastStepTime_;

    // Step generation
    float stepAccumulator_;
    int32_t stepsRemaining_;

    // Advanced features
    bool stealthChopEnabled_;
    bool spreadCycleEnabled_;
    uint8_t stallGuardThreshold_;
    bool stallDetected_;
    uint8_t runCurrent_;
    uint8_t holdCurrent_;

    // Internal methods
    void generateStep();
    void updatePosition();
    float stepsToRadians(int32_t steps) const;
    int32_t radiansToSteps(float radians) const;
};

}  // namespace omni::driver
