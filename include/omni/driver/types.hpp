/**
 * @file types.hpp
 * @brief Common types for motor drivers
 */

#pragma once

#include <cstdint>
#include <cfloat>

namespace omni::driver {

/**
 * @brief Motor types
 */
enum class MotorType {
    // Rotary motors
    BrushedDC,
    BLDC,
    PMSM,
    Stepper,
    ACInduction,
    SRM,
    Servo,
    Coreless,
    Outrunner,

    // Linear motors
    LinearInduction,
    LinearSynchronous,
    VoiceCoil,
    LinearStepper,

    // Special motors
    TorqueMotor,
    Ultrasonic,
    Piezo,
    RCServo,
    HubMotor,

    Custom
};

/**
 * @brief Control modes
 */
enum class ControlMode {
    Off,            // Motor disabled
    Voltage,        // Open-loop voltage control
    Duty,           // Open-loop PWM duty cycle
    Current,        // Current/torque control
    Velocity,       // Velocity control
    Position,       // Position control
    Impedance,      // Impedance control
    Mit             // MIT mode (Kp, Kd, pos, vel, torque)
};

/**
 * @brief Motor state structure
 */
struct MotorState {
    float position = 0.0f;        // Position (rad or m)
    float velocity = 0.0f;        // Velocity (rad/s or m/s)
    float acceleration = 0.0f;    // Acceleration
    float torque = 0.0f;          // Torque/force (Nm or N)
    float current = 0.0f;         // Current (A)
    float temperature = 0.0f;     // Temperature (C)
    float voltage = 0.0f;         // Bus voltage (V)
    uint32_t errorCode = 0;       // Error flags
    bool enabled = false;         // Enable state
    bool fault = false;           // Fault flag
    bool moving = false;          // In motion flag
    bool homed = false;           // Homing complete flag
};

/**
 * @brief Motor parameters
 */
struct MotorParams {
    // Electrical parameters
    float polePairs = 1.0f;       // Number of pole pairs
    float resistance = 0.0f;      // Phase resistance (ohms)
    float inductance = 0.0f;      // Phase inductance (H)
    float fluxLinkage = 0.0f;     // Flux linkage (Wb)
    float ktConstant = 0.0f;      // Torque constant (Nm/A)
    float keConstant = 0.0f;      // Back-EMF constant (V/rad/s)

    // Mechanical parameters
    float inertia = 0.0f;         // Rotor inertia (kg*m^2)
    float friction = 0.0f;        // Viscous friction (Nm*s/rad)
    float staticFriction = 0.0f;  // Static friction (Nm)

    // Limits
    float maxCurrent = FLT_MAX;   // Maximum current (A)
    float maxVelocity = FLT_MAX;  // Maximum velocity (rad/s or m/s)
    float maxTorque = FLT_MAX;    // Maximum torque (Nm or N)
    float maxTemperature = 100.0f; // Maximum temperature (C)
    float minPosition = -FLT_MAX; // Minimum position
    float maxPosition = FLT_MAX;  // Maximum position

    // Encoder parameters
    uint32_t encoderCpr = 0;      // Counts per revolution
    float encoderOffset = 0.0f;   // Encoder zero offset (rad)
    bool encoderInvert = false;   // Invert encoder direction

    // For linear motors
    float pitch = 0.0f;           // Linear pitch (m/rev) or (m/pole)
};

/**
 * @brief Motor command structure
 */
struct MotorCommand {
    ControlMode mode = ControlMode::Off;
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float current = 0.0f;
    float voltage = 0.0f;
    float duty = 0.0f;

    // For MIT mode
    float kp = 0.0f;
    float kd = 0.0f;
    float feedforwardTorque = 0.0f;
};

/**
 * @brief Brake modes
 */
enum class BrakeMode {
    Coast,      // Coast to stop
    Brake,      // Active braking (short windings)
    Hold        // Hold position
};

/**
 * @brief Step modes for stepper motors
 */
enum class StepMode {
    Full,
    Half,
    Quarter,
    Eighth,
    Sixteenth,
    ThirtySecond
};

/**
 * @brief FOC modes
 */
enum class FocMode {
    SinePWM,    // Sinusoidal PWM
    SVPWM,      // Space Vector PWM
    SixStep     // Six-step/trapezoidal commutation
};

/**
 * @brief Error codes
 */
enum class MotorError : uint32_t {
    None = 0,
    Overcurrent = 1 << 0,
    Overvoltage = 1 << 1,
    Undervoltage = 1 << 2,
    Overtemperature = 1 << 3,
    HallSensorFault = 1 << 4,
    EncoderFault = 1 << 5,
    CommunicationFault = 1 << 6,
    PositionLimitExceeded = 1 << 7,
    VelocityLimitExceeded = 1 << 8,
    FollowingError = 1 << 9,
    DriverFault = 1 << 10,
    PhaseOpenCircuit = 1 << 11,
    PhaseShortCircuit = 1 << 12,
    CalibrationFault = 1 << 13
};

inline MotorError operator|(MotorError a, MotorError b) {
    return static_cast<MotorError>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline MotorError operator&(MotorError a, MotorError b) {
    return static_cast<MotorError>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

}  // namespace omni::driver
