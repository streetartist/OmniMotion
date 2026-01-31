/**
 * @file main.cpp
 * @brief Multi-axis synchronized motion example
 */

#include <omni/omnimotion.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Simulated motor driver for testing
class SimulatedMotorDriver : public omni::driver::IMotorDriver {
public:
    SimulatedMotorDriver(const char* name) : name_(name) {}

    bool init() override { return true; }
    void deinit() override {}
    void enable() override { enabled_ = true; }
    void disable() override { enabled_ = false; }
    bool isEnabled() const override { return enabled_; }
    void emergencyStop() override { velocity_ = 0; enabled_ = false; }

    void setControlMode(omni::driver::ControlMode mode) override { mode_ = mode; }
    omni::driver::ControlMode getControlMode() const override { return mode_; }

    void setPosition(float pos) override { targetPos_ = pos; }
    void setVelocity(float vel) override { velocity_ = vel; }
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

    void setParams(const omni::driver::MotorParams& params) override {
        params_ = params;
    }
    omni::driver::MotorParams getParams() const override { return params_; }

    void update() override {
        // Simplified position tracking
        float error = targetPos_ - position_;
        velocity_ = error * 10.0f;  // Simple P control
        if (std::abs(velocity_) > 10.0f) {
            velocity_ = (velocity_ > 0) ? 10.0f : -10.0f;
        }
        position_ += velocity_ * 0.001f;  // Assume 1ms update
    }

    omni::driver::MotorType getType() const override {
        return omni::driver::MotorType::Custom;
    }
    const char* getName() const override { return name_; }

    uint32_t getErrorCode() const override { return 0; }
    void clearErrors() override {}
    bool hasFault() const override { return false; }

private:
    const char* name_;
    bool enabled_ = false;
    omni::driver::ControlMode mode_ = omni::driver::ControlMode::Off;
    omni::driver::MotorParams params_;
    float position_ = 0;
    float velocity_ = 0;
    float targetPos_ = 0;
};

int main() {
    std::cout << "OmniMotion Multi-Axis Sync Example" << std::endl;
    std::cout << "Version: " << omni::Version::getString() << std::endl;
    std::cout << std::endl;

    // Create simulated motor drivers
    SimulatedMotorDriver motorX("X-Axis");
    SimulatedMotorDriver motorY("Y-Axis");
    SimulatedMotorDriver motorZ("Z-Axis");

    // Create multi-axis controller
    omni::app::MultiAxisController robot(3);
    robot.addAxis(0, &motorX);
    robot.addAxis(1, &motorY);
    robot.addAxis(2, &motorZ);

    // Enable all axes
    robot.startSynchronized();
    std::cout << "All axes enabled" << std::endl;

    // Define waypoints
    std::vector<std::vector<float>> waypoints = {
        {0.0f, 0.0f, 0.0f},
        {100.0f, 50.0f, 25.0f},
        {100.0f, 100.0f, 50.0f},
        {0.0f, 100.0f, 50.0f},
        {0.0f, 0.0f, 0.0f}
    };

    // Execute motion through waypoints
    const float dt = 0.001f;

    for (size_t i = 0; i < waypoints.size(); i++) {
        std::cout << "\nMoving to waypoint " << i << ": ("
                  << waypoints[i][0] << ", "
                  << waypoints[i][1] << ", "
                  << waypoints[i][2] << ")" << std::endl;

        robot.moveTo(waypoints[i]);

        float simTime = 0;
        float lastPrintTime = -1;

        while (robot.isMoving() && simTime < 5.0f) {
            robot.update(dt);

            if (simTime - lastPrintTime >= 0.2f) {
                auto* axisX = robot.getAxis(0);
                auto* axisY = robot.getAxis(1);
                auto* axisZ = robot.getAxis(2);

                std::cout << "  X=" << (axisX ? axisX->getPosition() : 0)
                          << " Y=" << (axisY ? axisY->getPosition() : 0)
                          << " Z=" << (axisZ ? axisZ->getPosition() : 0)
                          << std::endl;
                lastPrintTime = simTime;
            }

            simTime += dt;
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    std::cout << "\nMotion sequence complete!" << std::endl;

    // Emergency stop demonstration
    std::cout << "\nTesting emergency stop..." << std::endl;
    robot.moveTo({200.0f, 200.0f, 100.0f});

    // Let it move a bit then stop
    for (int i = 0; i < 100; i++) {
        robot.update(dt);
    }
    robot.emergencyStopAll();
    std::cout << "Emergency stop triggered!" << std::endl;

    return 0;
}
