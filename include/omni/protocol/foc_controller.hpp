/**
 * @file foc_controller.hpp
 * @brief Field Oriented Control (FOC) implementation
 */

#pragma once

#include "omni/control/pid_controller.hpp"
#include <cmath>
#include <algorithm>

namespace omni::protocol {

/**
 * @brief FOC operating mode
 */
enum class FocMode {
    SinePWM,    // Sinusoidal PWM
    SVPWM,      // Space Vector PWM
    SixStep     // Six-step/trapezoidal commutation
};

/**
 * @brief FOC controller configuration
 */
struct FocConfig {
    float currentBandwidth = 1000.0f;   // Current loop bandwidth (Hz)
    float velocityBandwidth = 100.0f;   // Velocity loop bandwidth (Hz)
    float positionBandwidth = 10.0f;    // Position loop bandwidth (Hz)
    float pwmFrequency = 20000.0f;      // PWM frequency (Hz)
    FocMode mode = FocMode::SVPWM;
    float maxModulation = 0.95f;        // Maximum modulation index
    float deadTimeNs = 500.0f;          // Dead time (ns)
};

/**
 * @brief Clarke transform (abc -> alpha-beta)
 */
struct ClarkeTransform {
    /**
     * @brief Forward Clarke transform
     * @param ia Phase A current
     * @param ib Phase B current
     * @param ic Phase C current (optional, assumes ia + ib + ic = 0)
     * @param alpha Output alpha component
     * @param beta Output beta component
     */
    static void forward(float ia, float ib, float ic,
                        float& alpha, float& beta) {
        (void)ic;  // For balanced systems
        // Clarke transform (amplitude invariant)
        alpha = ia;
        beta = (ia + 2.0f * ib) / std::sqrt(3.0f);
    }

    /**
     * @brief Forward Clarke transform (2-phase)
     */
    static void forward(float ia, float ib, float& alpha, float& beta) {
        alpha = ia;
        beta = (ia + 2.0f * ib) / std::sqrt(3.0f);
    }

    /**
     * @brief Inverse Clarke transform
     * @param alpha Alpha component
     * @param beta Beta component
     * @param a Output phase A
     * @param b Output phase B
     * @param c Output phase C
     */
    static void inverse(float alpha, float beta,
                        float& a, float& b, float& c) {
        a = alpha;
        b = (-alpha + std::sqrt(3.0f) * beta) / 2.0f;
        c = (-alpha - std::sqrt(3.0f) * beta) / 2.0f;
    }
};

/**
 * @brief Park transform (alpha-beta -> d-q)
 */
struct ParkTransform {
    /**
     * @brief Forward Park transform
     * @param alpha Alpha component
     * @param beta Beta component
     * @param theta Electrical angle (radians)
     * @param d Output d-axis component
     * @param q Output q-axis component
     */
    static void forward(float alpha, float beta, float theta,
                        float& d, float& q) {
        float sinTheta = std::sin(theta);
        float cosTheta = std::cos(theta);
        d = alpha * cosTheta + beta * sinTheta;
        q = -alpha * sinTheta + beta * cosTheta;
    }

    /**
     * @brief Inverse Park transform
     * @param d D-axis component
     * @param q Q-axis component
     * @param theta Electrical angle (radians)
     * @param alpha Output alpha component
     * @param beta Output beta component
     */
    static void inverse(float d, float q, float theta,
                        float& alpha, float& beta) {
        float sinTheta = std::sin(theta);
        float cosTheta = std::cos(theta);
        alpha = d * cosTheta - q * sinTheta;
        beta = d * sinTheta + q * cosTheta;
    }
};

/**
 * @brief Space Vector PWM modulator
 */
class SvpwmModulator {
public:
    /**
     * @brief Calculate SVPWM duty cycles
     * @param alpha Alpha voltage component
     * @param beta Beta voltage component
     * @param vdc DC bus voltage
     * @param ta Output duty cycle phase A (0-1)
     * @param tb Output duty cycle phase B (0-1)
     * @param tc Output duty cycle phase C (0-1)
     * @param maxModulation Maximum modulation index (default 1.0)
     */
    static void calculate(float alpha, float beta, float vdc,
                          float& ta, float& tb, float& tc,
                          float maxModulation = 1.0f) {
        if (vdc < 0.1f) {
            ta = tb = tc = 0.5f;
            return;
        }

        // Normalize to DC bus
        float scale = std::sqrt(3.0f) / vdc;
        alpha *= scale;
        beta *= scale;

        // Limit modulation
        float mag = std::sqrt(alpha * alpha + beta * beta);
        if (mag > maxModulation) {
            float factor = maxModulation / mag;
            alpha *= factor;
            beta *= factor;
        }

        // Inverse Clarke to get phase voltages
        float va = alpha;
        float vb = (-alpha + std::sqrt(3.0f) * beta) / 2.0f;
        float vc = (-alpha - std::sqrt(3.0f) * beta) / 2.0f;

        // Find min and max
        float vmin = std::min({va, vb, vc});
        float vmax = std::max({va, vb, vc});

        // Center-aligned SVPWM (add common mode offset)
        float voffset = -(vmax + vmin) / 2.0f;

        // Convert to duty cycles (0-1)
        ta = (va + voffset) + 0.5f;
        tb = (vb + voffset) + 0.5f;
        tc = (vc + voffset) + 0.5f;

        // Clamp to valid range
        ta = std::clamp(ta, 0.0f, 1.0f);
        tb = std::clamp(tb, 0.0f, 1.0f);
        tc = std::clamp(tc, 0.0f, 1.0f);
    }

    /**
     * @brief Calculate SVPWM from d-q voltages
     */
    static void calculateFromDQ(float vd, float vq, float theta, float vdc,
                                float& ta, float& tb, float& tc,
                                float maxModulation = 1.0f) {
        float alpha, beta;
        ParkTransform::inverse(vd, vq, theta, alpha, beta);
        calculate(alpha, beta, vdc, ta, tb, tc, maxModulation);
    }
};

/**
 * @brief Complete FOC controller
 */
class FocController {
public:
    FocController() : FocController(FocConfig{}) {}

    explicit FocController(const FocConfig& config)
        : config_(config)
        , electricalAngle_(0)
        , id_(0), iq_(0)
        , idRef_(0), iqRef_(0)
        , vd_(0), vq_(0)
        , valpha_(0), vbeta_(0)
        , vdc_(24.0f)
        , fieldWeakeningEnabled_(false)
        , maxNegativeId_(0) {
        configureCurrentPid();
    }

    /**
     * @brief Set current feedback
     * @param ia Phase A current
     * @param ib Phase B current
     * @param ic Phase C current
     */
    void setCurrentFeedback(float ia, float ib, float ic) {
        // Clarke transform
        float alpha, beta;
        ClarkeTransform::forward(ia, ib, ic, alpha, beta);

        // Park transform
        ParkTransform::forward(alpha, beta, electricalAngle_, id_, iq_);
    }

    /**
     * @brief Set current feedback (2-phase)
     */
    void setCurrentFeedback(float ia, float ib) {
        float ic = -(ia + ib);
        setCurrentFeedback(ia, ib, ic);
    }

    /**
     * @brief Set electrical angle
     * @param angle Electrical angle in radians
     */
    void setElectricalAngle(float angle) {
        electricalAngle_ = angle;
    }

    /**
     * @brief Set DC bus voltage
     * @param vdc Bus voltage
     */
    void setDcBusVoltage(float vdc) {
        vdc_ = vdc;
    }

    /**
     * @brief Set d-axis current reference
     * @param idRef D-axis current reference (A)
     */
    void setIdRef(float idRef) {
        idRef_ = idRef;
    }

    /**
     * @brief Set q-axis current reference
     * @param iqRef Q-axis current reference (A)
     */
    void setIqRef(float iqRef) {
        iqRef_ = iqRef;
    }

    /**
     * @brief Set torque reference (converts to Iq)
     * @param torque Torque reference
     * @param kt Torque constant (Nm/A)
     */
    void setTorqueRef(float torque, float kt) {
        iqRef_ = torque / kt;
    }

    /**
     * @brief Run current control loop
     * @param dt Time step
     */
    void updateCurrentLoop(float dt) {
        // Field weakening
        float idCommand = idRef_;
        if (fieldWeakeningEnabled_) {
            // Simple field weakening based on modulation
            float modulation = std::sqrt(vd_ * vd_ + vq_ * vq_) / vdc_;
            if (modulation > 0.9f) {
                idCommand = std::max(idRef_, maxNegativeId_ * (modulation - 0.9f) / 0.1f);
            }
        }

        // D-axis current control
        vd_ = idPid_.update(idCommand, id_, dt);

        // Q-axis current control
        vq_ = iqPid_.update(iqRef_, iq_, dt);

        // Inverse Park transform
        ParkTransform::inverse(vd_, vq_, electricalAngle_, valpha_, vbeta_);
    }

    /**
     * @brief Get PWM duty cycles
     * @param ta Output phase A duty
     * @param tb Output phase B duty
     * @param tc Output phase C duty
     */
    void getPwmDuty(float& ta, float& tb, float& tc) const {
        SvpwmModulator::calculate(valpha_, vbeta_, vdc_, ta, tb, tc,
                                  config_.maxModulation);
    }

    /**
     * @brief Get d-axis current
     */
    float getId() const { return id_; }

    /**
     * @brief Get q-axis current
     */
    float getIq() const { return iq_; }

    /**
     * @brief Get d-axis voltage
     */
    float getVd() const { return vd_; }

    /**
     * @brief Get q-axis voltage
     */
    float getVq() const { return vq_; }

    /**
     * @brief Enable field weakening
     * @param enable true to enable
     * @param maxNegativeId Maximum negative Id (A)
     */
    void enableFieldWeakening(bool enable, float maxNegativeId = 0.0f) {
        fieldWeakeningEnabled_ = enable;
        maxNegativeId_ = maxNegativeId;
    }

    /**
     * @brief Set current loop PID parameters
     * @param kp Proportional gain
     * @param ki Integral gain
     */
    void setCurrentPidGains(float kp, float ki) {
        idPid_.setGains(kp, ki, 0);
        iqPid_.setGains(kp, ki, 0);
    }

    /**
     * @brief Reset controller
     */
    void reset() {
        idPid_.reset();
        iqPid_.reset();
        id_ = iq_ = 0;
        vd_ = vq_ = 0;
        valpha_ = vbeta_ = 0;
    }

    /**
     * @brief Get configuration
     */
    const FocConfig& getConfig() const { return config_; }

    /**
     * @brief Set configuration
     */
    void setConfig(const FocConfig& config) {
        config_ = config;
        configureCurrentPid();
    }

private:
    FocConfig config_;

    // State
    float electricalAngle_;
    float id_, iq_;
    float idRef_, iqRef_;
    float vd_, vq_;
    float valpha_, vbeta_;
    float vdc_;

    // Controllers
    omni::control::PidController idPid_;
    omni::control::PidController iqPid_;

    // Features
    bool fieldWeakeningEnabled_;
    float maxNegativeId_;

    void configureCurrentPid() {
        // Configure PID based on bandwidth
        // Simplified: Kp = bandwidth * L, Ki = bandwidth * R
        // Using typical values
        float kp = 2.0f * 3.14159f * config_.currentBandwidth * 0.001f;  // L ~ 1mH
        float ki = 2.0f * 3.14159f * config_.currentBandwidth * 0.5f;   // R ~ 0.5 ohm

        omni::control::PidController::Params params;
        params.kp = kp;
        params.ki = ki;
        params.kd = 0;
        params.outputLimit = config_.maxModulation * 0.577f;  // Max phase voltage
        params.integralLimit = params.outputLimit;

        idPid_.setParams(params);
        iqPid_.setParams(params);
    }
};

}  // namespace omni::protocol
