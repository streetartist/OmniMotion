/**
 * @file main.cpp
 * @brief Stepper motor S-curve motion example
 */

#include <omni/omnimotion.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Simulated GPIO for step/dir interface
class SimulatedGpio : public omni::hal::IGpio {
public:
    void setMode(omni::hal::PinMode) override {}
    omni::hal::PinMode getMode() const override {
        return omni::hal::PinMode::Output;
    }
    void write(bool value) override { state_ = value; }
    bool read() override { return state_; }
    void toggle() override { state_ = !state_; }
    void enableInterrupt(bool, bool) override {}
    void disableInterrupt() override {}
private:
    bool state_ = false;
};

int main() {
    std::cout << "OmniMotion Stepper S-Curve Example" << std::endl;
    std::cout << "Version: " << omni::Version::getString() << std::endl;
    std::cout << std::endl;

    // Create simulated GPIO pins
    SimulatedGpio stepPin;
    SimulatedGpio dirPin;
    SimulatedGpio enablePin;

    // Create stepper driver
    omni::driver::StepperDriver stepper(&stepPin, &dirPin, &enablePin);

    // Configure stepper parameters
    stepper.setMicrostep(16);
    stepper.setStepsPerRev(200);  // 1.8 degree motor

    // Initialize
    stepper.init();

    // Create high-level controller
    omni::app::MotorController motor(&stepper);

    // Set motion constraints
    omni::motion::MotionConstraints constraints;
    constraints.maxVelocity = 10.0f * 2.0f * 3.14159f;      // 10 rev/s
    constraints.maxAcceleration = 50.0f * 2.0f * 3.14159f;  // 50 rev/s^2
    constraints.maxJerk = 500.0f * 2.0f * 3.14159f;         // 500 rev/s^3
    motor.setConstraints(constraints);

    // Enable motor
    motor.enable();
    std::cout << "Stepper enabled" << std::endl;

    // Move using S-curve profile
    float targetPosition = 10.0f * 2.0f * 3.14159f;  // 10 revolutions
    std::cout << "Moving " << targetPosition / (2.0f * 3.14159f) << " revolutions..." << std::endl;

    motor.moveTo(targetPosition);

    // Monitor motion
    const float dt = 0.001f;
    float simTime = 0;
    float lastPrintTime = 0;

    while (!motor.isSettled() && simTime < 10.0f) {
        motor.update(dt);
        // stepper.update(dt); // Called by motor.update()

        // Print status every 100ms
        if (simTime - lastPrintTime >= 0.1f) {
            float posRevs = motor.getPosition() / (2.0f * 3.14159f);
            float velRps = motor.getVelocity() / (2.0f * 3.14159f);
            std::cout << "t=" << simTime << "s"
                      << " pos=" << posRevs << " rev"
                      << " vel=" << velRps << " rev/s"
                      << std::endl;
            lastPrintTime = simTime;
        }

        simTime += dt;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "Motion complete!" << std::endl;
    std::cout << "Final position: " << motor.getPosition() / (2.0f * 3.14159f) << " rev" << std::endl;

    // Disable motor
    motor.disable();
    std::cout << "Stepper disabled" << std::endl;

    return 0;
}
