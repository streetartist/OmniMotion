/**
 * @file main.cpp
 * @brief BLDC motor FOC control example
 */

#include <omni/omnimotion.hpp>
#include <iostream>
#include <thread>
#include <chrono>

// Simulated hardware interfaces for demonstration
class SimulatedPwm3Phase : public omni::hal::IPwm3Phase {
public:
    void setDuty(float a, float b, float c) override {
        dutyA_ = a; dutyB_ = b; dutyC_ = c;
    }
    void getDuty(float& a, float& b, float& c) const override {
        a = dutyA_; b = dutyB_; c = dutyC_;
    }
    void enable(bool en) override { enabled_ = en; }
    bool isEnabled() const override { return enabled_; }
    void setDeadTime(uint32_t ns) override { (void)ns; }
    void setFrequency(uint32_t freq) override { (void)freq; }
    void setSvpwm(float alpha, float beta) override {
        (void)alpha; (void)beta;
    }
    void setPhaseEnable(bool a, bool b, bool c) override {
        (void)a; (void)b; (void)c;
    }
private:
    float dutyA_ = 0, dutyB_ = 0, dutyC_ = 0;
    bool enabled_ = false;
};

class SimulatedAdc : public omni::hal::IAdc {
public:
    uint16_t read() override { return value_; }
    float readVoltage() override { return value_ / 4096.0f * 3.3f; }
    void startDma(uint16_t*, size_t) override {}
    void stopDma() override {}
    bool isDmaComplete() const override { return true; }
    void setTrigger(omni::hal::AdcTrigger) override {}
    uint8_t getResolution() const override { return 12; }
    void setReferenceVoltage(float) override {}
    float getReferenceVoltage() const override { return 3.3f; }
    void setSamplingTime(uint32_t) override {}

    void setValue(uint16_t v) { value_ = v; }
private:
    uint16_t value_ = 2048;
};

class SimulatedEncoder : public omni::hal::IEncoder {
public:
    int32_t getCount() override { return count_; }
    void resetCount() override { count_ = 0; }
    void setCount(int32_t c) override { count_ = c; }
    float getAngle() override {
        return (count_ % cpr_) * 2.0f * 3.14159f / cpr_;
    }
    float getVelocity() override { return velocity_; }
    void setResolution(uint32_t cpr) override { cpr_ = cpr; }
    uint32_t getResolution() const override { return cpr_; }
    void setMode(omni::hal::EncoderMode) override {}
    void enableIndex(bool) override {}
    bool indexDetected() override { return false; }
    void clearIndex() override {}
    void setInvert(bool) override {}

    void simulate(float velocity, float dt) {
        velocity_ = velocity;
        count_ += static_cast<int32_t>(velocity * cpr_ / (2.0f * 3.14159f) * dt);
    }
private:
    int32_t count_ = 0;
    uint32_t cpr_ = 4096;
    float velocity_ = 0;
};

int main() {
    std::cout << "OmniMotion BLDC FOC Example" << std::endl;
    std::cout << "Version: " << omni::Version::getString() << std::endl;
    std::cout << std::endl;

    // Create simulated hardware
    SimulatedPwm3Phase pwm;
    SimulatedAdc adcA, adcB, adcC;
    SimulatedEncoder encoder;

    // Create BLDC driver
    omni::driver::BldcDriver bldc(&pwm, &adcA, &adcB, &adcC, &encoder);

    // Configure motor parameters
    omni::driver::MotorParams params;
    params.polePairs = 7;
    params.resistance = 0.5f;
    params.inductance = 0.001f;
    params.maxCurrent = 20.0f;
    params.encoderCpr = 4096;
    bldc.setParams(params);

    // Initialize
    if (!bldc.init()) {
        std::cerr << "Failed to initialize BLDC driver" << std::endl;
        return 1;
    }

    // Create high-level controller
    omni::app::MotorController motor(&bldc);

    // Configure control parameters
    motor.setPositionPid(50.0f, 0.1f, 0.5f);
    motor.setVelocityPid(0.5f, 0.01f, 0.0f);

    // Enable motor
    motor.enable();
    std::cout << "Motor enabled" << std::endl;

    // Command motion using S-curve profile
    float targetPosition = 3.14159f;  // 1/2 revolution
    motor.moveWithSCurve(targetPosition, 10.0f, 100.0f, 1000.0f);
    std::cout << "Moving to position: " << targetPosition << " rad" << std::endl;

    // Simulation loop
    const float dt = 0.001f;  // 1ms control period
    float simTime = 0;

    while (simTime < 2.0f) {
        // Simulate encoder movement (simplified)
        float cmdVel = 0;  // Would come from controller output in real system
        encoder.simulate(cmdVel, dt);

        // Update controller
        motor.update(dt);

        // Print status periodically
        if (static_cast<int>(simTime * 100) % 10 == 0) {
            std::cout << "t=" << simTime << "s"
                      << " pos=" << motor.getPosition()
                      << " vel=" << motor.getVelocity()
                      << " settled=" << motor.isSettled()
                      << std::endl;
        }

        simTime += dt;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // Disable motor
    motor.disable();
    std::cout << "Motor disabled" << std::endl;

    return 0;
}
