/**
 * @file main.cpp
 * @brief Trajectory tracking example
 */

#include <omni/omnimotion.hpp>
#include <iostream>
#include <cmath>

// Simulated motor driver
class SimulatedMotor : public omni::driver::IMotorDriver {
public:
    bool init() override { return true; }
    void deinit() override {}
    void enable() override { enabled_ = true; }
    void disable() override { enabled_ = false; }
    bool isEnabled() const override { return enabled_; }
    void emergencyStop() override { velocity_ = 0; }

    void setControlMode(omni::driver::ControlMode mode) override { mode_ = mode; }
    omni::driver::ControlMode getControlMode() const override { return mode_; }

    void setPosition(float pos) override { targetPos_ = pos; }
    void setVelocity(float vel) override { (void)vel; }
    void setTorque(float torque) override { (void)torque; }
    void setCurrent(float current) override { (void)current; }
    void setVoltage(float voltage) override { (void)voltage; }
    void setDuty(float duty) override { (void)duty; }

    omni::driver::MotorState getState() const override {
        omni::driver::MotorState state;
        state.position = position_;
        state.velocity = velocity_;
        state.enabled = enabled_;
        return state;
    }

    float getPosition() const override { return position_; }
    float getVelocity() const override { return velocity_; }
    float getTorque() const override { return 0; }
    float getCurrent() const override { return 0; }
    float getTemperature() const override { return 25.0f; }

    void setParams(const omni::driver::MotorParams&) override {}
    omni::driver::MotorParams getParams() const override { return {}; }

    void update() override {
        float error = targetPos_ - position_;
        velocity_ = error * 20.0f;
        position_ += velocity_ * 0.001f;
    }

    omni::driver::MotorType getType() const override {
        return omni::driver::MotorType::Custom;
    }
    const char* getName() const override { return "SimMotor"; }
    uint32_t getErrorCode() const override { return 0; }
    void clearErrors() override {}
    bool hasFault() const override { return false; }

private:
    bool enabled_ = false;
    omni::driver::ControlMode mode_ = omni::driver::ControlMode::Off;
    float position_ = 0;
    float velocity_ = 0;
    float targetPos_ = 0;
};

int main() {
    std::cout << "OmniMotion Trajectory Tracking Example" << std::endl;
    std::cout << "Version: " << omni::Version::getString() << std::endl;
    std::cout << std::endl;

    // Create motor and controller
    SimulatedMotor motor;
    motor.init();

    omni::app::MotorController controller(&motor);
    controller.enable();

    // Create trajectory tracker
    omni::app::TrajectoryTracker tracker(&controller);

    // Generate sine wave trajectory
    std::vector<omni::motion::TrajectoryPoint> trajectory;
    const float duration = 5.0f;
    const float amplitude = 100.0f;
    const float frequency = 0.5f;

    std::cout << "Generating sine wave trajectory..." << std::endl;
    for (float t = 0; t <= duration; t += 0.01f) {
        omni::motion::TrajectoryPoint point;
        point.time = t;
        point.position = amplitude * std::sin(2.0f * 3.14159f * frequency * t);
        point.velocity = amplitude * 2.0f * 3.14159f * frequency *
                        std::cos(2.0f * 3.14159f * frequency * t);
        trajectory.push_back(point);
    }

    // Load and start trajectory
    tracker.loadTrajectory(trajectory);
    tracker.start();

    std::cout << "Tracking trajectory..." << std::endl;
    std::cout << std::endl;

    // Track the trajectory
    const float dt = 0.001f;
    float lastPrintTime = -1;

    while (!tracker.isFinished()) {
        tracker.update(dt);
        controller.update(dt);
        motor.update();

        float elapsed = tracker.getElapsedTime();
        if (elapsed - lastPrintTime >= 0.5f) {
            float target = amplitude * std::sin(2.0f * 3.14159f * frequency * elapsed);
            float actual = controller.getPosition();
            float error = target - actual;

            std::cout << "t=" << elapsed << "s"
                      << " target=" << target
                      << " actual=" << actual
                      << " error=" << error
                      << " progress=" << (tracker.getProgress() * 100) << "%"
                      << std::endl;
            lastPrintTime = elapsed;
        }
    }

    std::cout << std::endl;
    std::cout << "Trajectory tracking complete!" << std::endl;

    // Test speed override
    std::cout << std::endl;
    std::cout << "Testing 0.5x speed playback..." << std::endl;

    tracker.setSpeedOverride(0.5f);
    tracker.start();

    while (!tracker.isFinished() && tracker.getElapsedTime() < 3.0f) {
        tracker.update(dt);
        controller.update(dt);
        motor.update();
    }

    std::cout << "Speed override test complete." << std::endl;

    controller.disable();
    return 0;
}
