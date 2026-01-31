/**
 * @file omnimotion.hpp
 * @brief Main OmniMotion library header - includes all components
 */

#pragma once

// Hardware Abstraction Layer
#include "omni/hal/hal.hpp"

// Motor Drivers
#include "omni/driver/driver.hpp"

// Communication Protocols
#include "omni/protocol/protocol.hpp"

// Control Algorithms
#include "omni/control/control.hpp"

// Motion Planning
#include "omni/motion/motion.hpp"

// Application Layer
#include "omni/app/app.hpp"

// Utilities
#include "omni/utils/utils.hpp"

/**
 * @namespace omni
 * @brief OmniMotion - Universal Motor Control Library
 *
 * OmniMotion provides a comprehensive framework for controlling
 * various types of motors with support for:
 * - Multiple motor types (BLDC, Stepper, DC, Servo, etc.)
 * - Various control algorithms (PID, FOC, MPC, etc.)
 * - Motion planning (S-curve, trapezoidal, spline)
 * - Communication protocols (CAN, Modbus, MIT)
 * - Application features (homing, teach/playback, safety)
 *
 * @example Basic Usage:
 * @code
 * #include <omnimotion.hpp>
 *
 * // Create hardware interfaces (platform specific)
 * auto pwm = createPwm3Phase(TIM1);
 * auto encoder = createEncoder(TIM2);
 *
 * // Create BLDC driver
 * omni::driver::BldcDriver motor(pwm, adcA, adcB, adcC, encoder);
 * motor.init();
 *
 * // Create high-level controller
 * omni::app::MotorController ctrl(&motor);
 * ctrl.enable();
 *
 * // Execute motion
 * ctrl.moveWithSCurve(3.14159f, 10.0f, 100.0f, 1000.0f);
 *
 * // Main loop
 * while (true) {
 *     ctrl.update(0.001f);  // 1ms control period
 * }
 * @endcode
 */
namespace omni {

/**
 * @brief Library version information
 */
struct Version {
    static constexpr int MAJOR = 1;
    static constexpr int MINOR = 0;
    static constexpr int PATCH = 0;

    static const char* getString() {
        return "1.0.0";
    }
};

}  // namespace omni
