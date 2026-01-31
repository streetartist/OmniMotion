/**
 * @file main.cpp
 * @brief Teach and playback example
 */

#include <omni/omnimotion.hpp>
#include <iostream>
#include <chrono>
#include <thread>

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
    void setVelocity(float) override {}
    void setTorque(float) override {}
    void setCurrent(float) override {}
    void setVoltage(float) override {}
    void setDuty(float) override {}

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

    // For simulation: manually set position
    void setSimulatedPosition(float pos) { position_ = pos; }

private:
    bool enabled_ = false;
    omni::driver::ControlMode mode_ = omni::driver::ControlMode::Off;
    float position_ = 0;
    float velocity_ = 0;
    float targetPos_ = 0;
};

int main() {
    std::cout << "OmniMotion Teach & Playback Example" << std::endl;
    std::cout << "Version: " << omni::Version::getString() << std::endl;
    std::cout << std::endl;

    // Create motor and controller
    SimulatedMotor motor;
    motor.init();

    omni::app::MotorController controller(&motor);
    controller.enable();

    // Create teach recorder
    omni::app::TeachRecorder teacher(&controller);

    // ===== Phase 1: Teaching =====
    std::cout << "=== Teaching Phase ===" << std::endl;
    std::cout << "Recording waypoints..." << std::endl;

    teacher.startTeaching();

    // Simulate moving to positions and recording
    float teachPositions[] = {0.0f, 25.0f, 50.0f, 75.0f, 100.0f, 75.0f, 50.0f, 25.0f, 0.0f};

    for (float pos : teachPositions) {
        motor.setSimulatedPosition(pos);
        teacher.recordPoint();
        std::cout << "  Recorded position: " << pos << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    teacher.stopTeaching();
    std::cout << "Teaching complete. Recorded " << teacher.getPointCount() << " points." << std::endl;

    // ===== Phase 2: Save to file =====
    std::cout << std::endl;
    std::cout << "=== Saving to File ===" << std::endl;

    if (teacher.saveToFile("teach_data.json")) {
        std::cout << "Saved to teach_data.json" << std::endl;
    } else {
        std::cout << "Failed to save (expected in this example)" << std::endl;
    }

    // ===== Phase 3: Playback =====
    std::cout << std::endl;
    std::cout << "=== Playback Phase ===" << std::endl;

    // Reset motor position
    motor.setSimulatedPosition(0);

    // Play back at normal speed
    std::cout << "Playing back at 1x speed..." << std::endl;
    teacher.playback(1.0f);

    const float dt = 0.01f;
    float playbackTime = 0;

    while (teacher.isPlaying() && playbackTime < 20.0f) {
        teacher.update(dt);
        controller.update(dt);
        motor.update();

        if (static_cast<int>(playbackTime * 10) % 5 == 0) {
            std::cout << "  Position: " << controller.getPosition() << std::endl;
        }

        playbackTime += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "Playback complete!" << std::endl;

    // ===== Phase 4: Loop playback =====
    std::cout << std::endl;
    std::cout << "=== Loop Playback (2x speed, 2 loops) ===" << std::endl;

    motor.setSimulatedPosition(0);
    teacher.playbackLoop(2);

    playbackTime = 0;
    int lastLoop = 0;

    while (teacher.isPlaying() && playbackTime < 30.0f) {
        teacher.update(dt * 2);  // 2x speed
        controller.update(dt);
        motor.update();

        playbackTime += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "Loop playback complete!" << std::endl;

    // ===== Cleanup =====
    teacher.clear();
    controller.disable();

    std::cout << std::endl;
    std::cout << "Example complete!" << std::endl;

    return 0;
}
