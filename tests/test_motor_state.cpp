/**
 * @file test_motor_state.cpp
 * @brief Unit tests for motor state and types
 */

#include <gtest/gtest.h>
#include <omni/driver/types.hpp>

using namespace omni::driver;

TEST(MotorStateTest, DefaultConstruction) {
    MotorState state;

    EXPECT_FLOAT_EQ(state.position, 0.0f);
    EXPECT_FLOAT_EQ(state.velocity, 0.0f);
    EXPECT_FLOAT_EQ(state.torque, 0.0f);
    EXPECT_EQ(state.errorCode, 0u);
    EXPECT_FALSE(state.enabled);
    EXPECT_FALSE(state.fault);
}

TEST(MotorParamsTest, DefaultConstruction) {
    MotorParams params;

    EXPECT_FLOAT_EQ(params.polePairs, 1.0f);
    EXPECT_FLOAT_EQ(params.resistance, 0.0f);
    EXPECT_FLOAT_EQ(params.inductance, 0.0f);
    EXPECT_FALSE(params.encoderInvert);
}

TEST(MotorErrorTest, ErrorCombination) {
    MotorError err1 = MotorError::Overcurrent;
    MotorError err2 = MotorError::Overtemperature;

    MotorError combined = err1 | err2;

    EXPECT_TRUE((combined & MotorError::Overcurrent) != MotorError::None);
    EXPECT_TRUE((combined & MotorError::Overtemperature) != MotorError::None);
    EXPECT_TRUE((combined & MotorError::EncoderFault) == MotorError::None);
}

TEST(MotorErrorTest, NoError) {
    MotorError err = MotorError::None;
    EXPECT_EQ(static_cast<uint32_t>(err), 0u);
}

TEST(MotorCommandTest, DefaultConstruction) {
    MotorCommand cmd;

    EXPECT_EQ(cmd.mode, ControlMode::Off);
    EXPECT_FLOAT_EQ(cmd.position, 0.0f);
    EXPECT_FLOAT_EQ(cmd.velocity, 0.0f);
    EXPECT_FLOAT_EQ(cmd.torque, 0.0f);
}

TEST(ControlModeTest, ModeValues) {
    EXPECT_NE(ControlMode::Off, ControlMode::Position);
    EXPECT_NE(ControlMode::Position, ControlMode::Velocity);
    EXPECT_NE(ControlMode::Velocity, ControlMode::Current);
}

TEST(MotorTypeTest, TypeValues) {
    // Just ensure types are distinct
    EXPECT_NE(MotorType::BLDC, MotorType::Stepper);
    EXPECT_NE(MotorType::Stepper, MotorType::BrushedDC);
    EXPECT_NE(MotorType::VoiceCoil, MotorType::LinearSynchronous);
}

TEST(BrakeModeTest, ModeValues) {
    EXPECT_NE(BrakeMode::Coast, BrakeMode::Brake);
    EXPECT_NE(BrakeMode::Brake, BrakeMode::Hold);
}

TEST(StepModeTest, ModeValues) {
    EXPECT_NE(StepMode::Full, StepMode::Half);
    EXPECT_NE(StepMode::Quarter, StepMode::Sixteenth);
}
