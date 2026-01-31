/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for PID controller
 */

#include <gtest/gtest.h>
#include <omni/control/pid_controller.hpp>
#include <cmath>

using namespace omni::control;

class PidControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default PID parameters
        PidController::Params params;
        params.kp = 1.0f;
        params.ki = 0.1f;
        params.kd = 0.01f;
        params.outputLimit = 100.0f;
        pid_ = PidController(params);
    }

    PidController pid_;
};

TEST_F(PidControllerTest, ProportionalResponse) {
    PidController::Params params;
    params.kp = 2.0f;
    params.ki = 0.0f;
    params.kd = 0.0f;
    PidController pid(params);

    float output = pid.update(10.0f, 5.0f, 0.001f);
    EXPECT_FLOAT_EQ(output, 10.0f);  // kp * error = 2 * 5 = 10
}

TEST_F(PidControllerTest, IntegralAccumulation) {
    PidController::Params params;
    params.kp = 0.0f;
    params.ki = 10.0f;
    params.kd = 0.0f;
    PidController pid(params);

    float dt = 0.001f;
    float setpoint = 10.0f;
    float measurement = 5.0f;
    float error = setpoint - measurement;

    // First update
    float output1 = pid.update(setpoint, measurement, dt);
    EXPECT_FLOAT_EQ(output1, error * 10.0f * dt);

    // Second update - integral should accumulate
    float output2 = pid.update(setpoint, measurement, dt);
    EXPECT_FLOAT_EQ(output2, 2.0f * error * 10.0f * dt);
}

TEST_F(PidControllerTest, DerivativeResponse) {
    PidController::Params params;
    params.kp = 0.0f;
    params.ki = 0.0f;
    params.kd = 1.0f;
    params.derivativeOnMeasurement = false;
    PidController pid(params);

    float dt = 0.01f;

    // First update (no derivative yet)
    pid.update(0.0f, 0.0f, dt);

    // Second update with error change
    float output = pid.update(10.0f, 0.0f, dt);
    EXPECT_NEAR(output, 1000.0f, 1.0f);  // kd * d(error)/dt = 1 * 10/0.01
}

TEST_F(PidControllerTest, OutputSaturation) {
    PidController::Params params;
    params.kp = 100.0f;
    params.ki = 0.0f;
    params.kd = 0.0f;
    params.outputLimit = 50.0f;
    PidController pid(params);

    float output = pid.update(100.0f, 0.0f, 0.001f);
    EXPECT_FLOAT_EQ(output, 50.0f);  // Should be clamped to outputLimit

    output = pid.update(-100.0f, 0.0f, 0.001f);
    EXPECT_FLOAT_EQ(output, -50.0f);  // Should be clamped to -outputLimit
}

TEST_F(PidControllerTest, IntegralWindup) {
    PidController::Params params;
    params.kp = 0.0f;
    params.ki = 100.0f;
    params.kd = 0.0f;
    params.integralLimit = 10.0f;
    PidController pid(params);

    // Accumulate integral
    for (int i = 0; i < 1000; i++) {
        pid.update(100.0f, 0.0f, 0.001f);
    }

    EXPECT_LE(std::abs(pid.getIntegral()), 10.0f);
}

TEST_F(PidControllerTest, Deadband) {
    PidController::Params params;
    params.kp = 1.0f;
    params.ki = 0.0f;
    params.kd = 0.0f;
    params.deadband = 5.0f;
    PidController pid(params);

    // Error within deadband
    float output = pid.update(10.0f, 7.0f, 0.001f);
    EXPECT_FLOAT_EQ(output, 0.0f);

    // Error outside deadband
    output = pid.update(10.0f, 0.0f, 0.001f);
    EXPECT_GT(std::abs(output), 0.0f);
}

TEST_F(PidControllerTest, Reset) {
    pid_.update(100.0f, 0.0f, 0.001f);
    pid_.update(100.0f, 0.0f, 0.001f);

    EXPECT_NE(pid_.getIntegral(), 0.0f);

    pid_.reset();

    EXPECT_FLOAT_EQ(pid_.getIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(pid_.getError(), 0.0f);
}

TEST_F(PidControllerTest, ZeroSetpoint) {
    PidController::Params params;
    params.kp = 1.0f;
    PidController pid(params);

    float output = pid.update(0.0f, 10.0f, 0.001f);
    EXPECT_FLOAT_EQ(output, -10.0f);
}
