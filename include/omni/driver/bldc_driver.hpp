/**
 * @file bldc_driver.hpp
 * @brief BLDC/PMSM motor driver with FOC support
 */

#pragma once

#include "motor_driver.hpp"
#include "omni/hal/hal.hpp"
#include "omni/control/pid_controller.hpp"

namespace omni::driver {

/**
 * @brief BLDC/PMSM motor driver
 *
 * Implements Field Oriented Control (FOC) for brushless DC and
 * permanent magnet synchronous motors. Supports various control
 * modes including position, velocity, current, and voltage control.
 */
class BldcDriver : public IMotorDriver {
public:
    /**
     * @brief Construct BLDC driver with hardware interfaces
     * @param pwm Three-phase PWM interface
     * @param currentAdc Array of 3 ADC interfaces for phase currents
     * @param encoder Encoder interface for position feedback
     */
    BldcDriver(hal::IPwm3Phase* pwm,
               hal::IAdc* currentAdcA,
               hal::IAdc* currentAdcB,
               hal::IAdc* currentAdcC,
               hal::IEncoder* encoder);

    /**
     * @brief Construct BLDC driver with hall sensors (sensorless FOC)
     * @param pwm Three-phase PWM interface
     * @param currentAdcA Phase A current ADC
     * @param currentAdcB Phase B current ADC
     * @param hall Hall sensor interface
     */
    BldcDriver(hal::IPwm3Phase* pwm,
               hal::IAdc* currentAdcA,
               hal::IAdc* currentAdcB,
               hal::IHallSensor* hall);

    ~BldcDriver() override = default;

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

    void update(float dt) override;

    MotorType getType() const override { return MotorType::BLDC; }
    const char* getName() const override { return "BLDC/PMSM Driver"; }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== BLDC/FOC Specific =====

    /**
     * @brief Set FOC mode
     * @param mode FOC mode (SVPWM, SinePWM, SixStep)
     */
    void setFocMode(FocMode mode);

    /**
     * @brief Get current FOC mode
     * @return FOC mode
     */
    FocMode getFocMode() const { return focMode_; }

    /**
     * @brief Set current loop PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setCurrentPid(float kp, float ki, float kd);

    /**
     * @brief Set velocity loop PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setVelocityPid(float kp, float ki, float kd);

    /**
     * @brief Set position loop PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPositionPid(float kp, float ki, float kd);

    /**
     * @brief Set d-axis current reference (for field weakening)
     * @param id D-axis current reference (A)
     */
    void setIdRef(float id);

    /**
     * @brief Set q-axis current reference (torque producing)
     * @param iq Q-axis current reference (A)
     */
    void setIqRef(float iq);

    /**
     * @brief Get d-axis current
     * @return D-axis current (A)
     */
    float getId() const { return id_; }

    /**
     * @brief Get q-axis current
     * @return Q-axis current (A)
     */
    float getIq() const { return iq_; }

    /**
     * @brief Get electrical angle
     * @return Electrical angle (rad)
     */
    float getElectricalAngle() const { return electricalAngle_; }

    /**
     * @brief Calibrate encoder alignment
     * @return true if calibration successful
     */
    bool calibrateEncoder();

    /**
     * @brief Calibrate current sensors
     * @return true if calibration successful
     */
    bool calibrateCurrentSensor();

    /**
     * @brief Enable field weakening
     * @param enable true to enable
     * @param maxNegativeId Maximum negative Id current (A)
     */
    void enableFieldWeakening(bool enable, float maxNegativeId = 0.0f);

    /**
     * @brief Enable MTPA (Maximum Torque Per Ampere)
     * @param enable true to enable
     */
    void enableMtpa(bool enable);

    /**
     * @brief Set current sensor gains
     * @param gainA Gain for phase A (A/V or A/count)
     * @param gainB Gain for phase B
     * @param gainC Gain for phase C
     */
    void setCurrentSensorGains(float gainA, float gainB, float gainC);

    /**
     * @brief Set current sensor offsets
     * @param offsetA Offset for phase A
     * @param offsetB Offset for phase B
     * @param offsetC Offset for phase C
     */
    void setCurrentSensorOffsets(float offsetA, float offsetB, float offsetC);

    /**
     * @brief Set PWM frequency
     * @param freqHz PWM frequency (Hz)
     */
    void setPwmFrequency(uint32_t freqHz);

    /**
     * @brief Set dead time
     * @param ns Dead time (ns)
     */
    void setDeadTime(uint32_t ns);

private:
    // Hardware interfaces
    hal::IPwm3Phase* pwm_;
    hal::IAdc* currentAdcA_;
    hal::IAdc* currentAdcB_;
    hal::IAdc* currentAdcC_;
    hal::IEncoder* encoder_;
    hal::IHallSensor* hall_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    FocMode focMode_;
    bool enabled_;
    uint32_t errorCode_;
    
    // Internal state
    int polePairs_;
    float electricalAngle_;
    float currentGain_;

    // Current measurements
    float id_;
    float iq_;

    // Control targets
    float targetPosition_;
    float targetVelocity_;
    float targetIq_;
    float targetId_;
    float targetVoltage_;
    float targetDuty_;

    // Controllers
    control::PidController idPid_;
    control::PidController iqPid_;
    control::PidController velocityPid_;
    control::PidController positionPid_;

    // Internal methods
    void setSvpwm(float valpha, float vbeta);
};

}  // namespace omni::driver
