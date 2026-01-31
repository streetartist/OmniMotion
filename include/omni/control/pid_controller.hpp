/**
 * @file pid_controller.hpp
 * @brief Enhanced PID controller
 */

#pragma once

#include <cmath>
#include <cfloat>
#include <algorithm>

namespace omni::control {

/**
 * @brief PID controller with advanced features
 *
 * Features include:
 * - Anti-windup with back-calculation
 * - Derivative filtering
 * - Setpoint ramping
 * - Deadband
 * - Output limiting
 */
class PidController {
public:
    /**
     * @brief PID parameters
     */
    struct Params {
        float kp = 0.0f;                // Proportional gain
        float ki = 0.0f;                // Integral gain
        float kd = 0.0f;                // Derivative gain
        float outputLimit = FLT_MAX;    // Output saturation limit
        float integralLimit = FLT_MAX;  // Integral term limit
        float deadband = 0.0f;          // Error deadband
        float setpointRamp = 0.0f;      // Setpoint rate limit (units/s)
        float derivativeFilter = 0.0f;  // Derivative low-pass filter coefficient (0-1)
        bool enableAntiWindup = true;   // Enable anti-windup
        bool derivativeOnMeasurement = true;  // Derivative on measurement (vs error)
    };

    /**
     * @brief Construct PID controller with default parameters
     */
    PidController() : PidController(Params{}) {}

    /**
     * @brief Construct PID controller with parameters
     * @param params PID parameters
     */
    explicit PidController(const Params& params)
        : params_(params)
        , error_(0)
        , prevError_(0)
        , prevMeasurement_(0)
        , integral_(0)
        , derivative_(0)
        , filteredDerivative_(0)
        , output_(0)
        , rampedSetpoint_(0)
        , saturated_(false)
        , firstUpdate_(true) {}

    /**
     * @brief Update the controller
     * @param setpoint Desired value
     * @param measurement Current value
     * @param dt Time step (seconds)
     * @return Controller output
     */
    float update(float setpoint, float measurement, float dt) {
        if (dt <= 0) return output_;

        // Setpoint ramping
        float targetSetpoint = setpoint;
        if (params_.setpointRamp > 0 && !firstUpdate_) {
            float maxChange = params_.setpointRamp * dt;
            float diff = targetSetpoint - rampedSetpoint_;
            if (std::abs(diff) > maxChange) {
                rampedSetpoint_ += (diff > 0) ? maxChange : -maxChange;
            } else {
                rampedSetpoint_ = targetSetpoint;
            }
        } else {
            rampedSetpoint_ = targetSetpoint;
        }

        // Calculate error
        error_ = rampedSetpoint_ - measurement;

        // Apply deadband
        if (std::abs(error_) < params_.deadband) {
            error_ = 0;
        }

        // Proportional term
        float pTerm = params_.kp * error_;

        // Integral term with anti-windup
        if (!saturated_ || params_.enableAntiWindup) {
            integral_ += params_.ki * error_ * dt;
            integral_ = std::clamp(integral_, -params_.integralLimit, params_.integralLimit);
        }

        // Derivative term
        if (!firstUpdate_) {
            if (params_.derivativeOnMeasurement) {
                // Derivative on measurement (avoids derivative kick)
                derivative_ = -(measurement - prevMeasurement_) / dt;
            } else {
                // Derivative on error
                derivative_ = (error_ - prevError_) / dt;
            }

            // Low-pass filter on derivative
            if (params_.derivativeFilter > 0) {
                filteredDerivative_ = params_.derivativeFilter * filteredDerivative_ +
                                     (1.0f - params_.derivativeFilter) * derivative_;
            } else {
                filteredDerivative_ = derivative_;
            }
        }
        float dTerm = params_.kd * filteredDerivative_;

        // Calculate output
        float unsaturatedOutput = pTerm + integral_ + dTerm;
        output_ = std::clamp(unsaturatedOutput, -params_.outputLimit, params_.outputLimit);

        // Track saturation for anti-windup
        saturated_ = (std::abs(unsaturatedOutput) > params_.outputLimit);

        // Update state
        prevError_ = error_;
        prevMeasurement_ = measurement;
        firstUpdate_ = false;

        return output_;
    }

    /**
     * @brief Reset controller state
     */
    void reset() {
        error_ = 0;
        prevError_ = 0;
        prevMeasurement_ = 0;
        integral_ = 0;
        derivative_ = 0;
        filteredDerivative_ = 0;
        output_ = 0;
        saturated_ = false;
        firstUpdate_ = true;
    }

    /**
     * @brief Reset with initial measurement
     * @param measurement Initial measurement value
     */
    void reset(float measurement) {
        reset();
        prevMeasurement_ = measurement;
        rampedSetpoint_ = measurement;
    }

    // === Parameter Setters ===

    void setGains(float kp, float ki, float kd) {
        params_.kp = kp;
        params_.ki = ki;
        params_.kd = kd;
    }

    void setKp(float kp) { params_.kp = kp; }
    void setKi(float ki) { params_.ki = ki; }
    void setKd(float kd) { params_.kd = kd; }

    void setOutputLimit(float limit) { params_.outputLimit = limit; }
    void setIntegralLimit(float limit) { params_.integralLimit = limit; }
    void setDeadband(float deadband) { params_.deadband = deadband; }
    void setSetpointRamp(float ramp) { params_.setpointRamp = ramp; }
    void setDerivativeFilter(float filter) { params_.derivativeFilter = filter; }
    void enableAntiWindup(bool enable) { params_.enableAntiWindup = enable; }
    void setDerivativeOnMeasurement(bool onMeasurement) {
        params_.derivativeOnMeasurement = onMeasurement;
    }

    void setParams(const Params& params) { params_ = params; }

    // === Getters ===

    float getError() const { return error_; }
    float getIntegral() const { return integral_; }
    float getDerivative() const { return filteredDerivative_; }
    float getOutput() const { return output_; }
    bool isSaturated() const { return saturated_; }
    const Params& getParams() const { return params_; }

    float getKp() const { return params_.kp; }
    float getKi() const { return params_.ki; }
    float getKd() const { return params_.kd; }

private:
    Params params_;
    float error_;
    float prevError_;
    float prevMeasurement_;
    float integral_;
    float derivative_;
    float filteredDerivative_;
    float output_;
    float rampedSetpoint_;
    bool saturated_;
    bool firstUpdate_;
};

}  // namespace omni::control
