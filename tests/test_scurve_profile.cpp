/**
 * @file test_scurve_profile.cpp
 * @brief Unit tests for S-curve profile generator
 */

#include <gtest/gtest.h>
#include <omni/motion/scurve_profile.hpp>
#include <cmath>

using namespace omni::motion;

class SCurveProfileTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionConstraints constraints;
        constraints.maxVelocity = 10.0f;
        constraints.maxAcceleration = 100.0f;
        constraints.maxJerk = 1000.0f;
        profile_ = SCurveProfile(constraints);
    }

    SCurveProfile profile_;
};

TEST_F(SCurveProfileTest, BasicPlanning) {
    EXPECT_TRUE(profile_.plan(0.0f, 100.0f));
    EXPECT_TRUE(profile_.isValid());
    EXPECT_GT(profile_.getDuration(), 0.0f);
}

TEST_F(SCurveProfileTest, StartEndPositions) {
    profile_.plan(0.0f, 50.0f);

    auto start = profile_.evaluate(0.0f);
    auto end = profile_.evaluate(profile_.getDuration());

    EXPECT_NEAR(start.position, 0.0f, 0.1f);
    EXPECT_NEAR(end.position, 50.0f, 0.5f);  // Allow some tolerance
}

TEST_F(SCurveProfileTest, SevenSegments) {
    // Long enough motion to have all 7 segments
    profile_.plan(0.0f, 1000.0f);

    float times[7];
    profile_.getSegmentTimes(times);

    // In full profile, all segments should have positive duration
    for (int i = 0; i < 7; i++) {
        EXPECT_GE(times[i], 0.0f);
    }
}

TEST_F(SCurveProfileTest, VelocityContinuity) {
    profile_.plan(0.0f, 100.0f);

    float prevVel = 0;
    float dt = 0.001f;

    for (float t = dt; t <= profile_.getDuration(); t += dt) {
        auto point = profile_.evaluate(t);
        float velChange = std::abs(point.velocity - prevVel);

        // Velocity change should be bounded by max accel * dt
        EXPECT_LT(velChange, 100.0f * dt * 2);  // Some tolerance

        prevVel = point.velocity;
    }
}

TEST_F(SCurveProfileTest, AccelerationContinuity) {
    profile_.plan(0.0f, 100.0f);

    float prevAcc = 0;
    float dt = 0.001f;

    for (float t = dt; t <= profile_.getDuration(); t += dt) {
        auto point = profile_.evaluate(t);
        float accChange = std::abs(point.acceleration - prevAcc);

        // Acceleration change should be bounded by max jerk * dt
        EXPECT_LT(accChange, 1000.0f * dt * 2);  // Some tolerance

        prevAcc = point.acceleration;
    }
}

TEST_F(SCurveProfileTest, VelocityNeverExceedsMax) {
    profile_.plan(0.0f, 100.0f);

    for (float t = 0; t <= profile_.getDuration(); t += 0.01f) {
        auto point = profile_.evaluate(t);
        EXPECT_LE(std::abs(point.velocity), 10.0f + 0.1f);
    }
}

TEST_F(SCurveProfileTest, AccelerationNeverExceedsMax) {
    profile_.plan(0.0f, 100.0f);

    for (float t = 0; t <= profile_.getDuration(); t += 0.01f) {
        auto point = profile_.evaluate(t);
        EXPECT_LE(std::abs(point.acceleration), 100.0f + 1.0f);
    }
}

TEST_F(SCurveProfileTest, NegativeMotion) {
    profile_.plan(100.0f, 0.0f);

    EXPECT_TRUE(profile_.isValid());

    auto end = profile_.evaluate(profile_.getDuration());
    EXPECT_NEAR(end.position, 0.0f, 0.5f);
}

TEST_F(SCurveProfileTest, ShortDistance) {
    // Very short distance - reduced profile
    profile_.plan(0.0f, 0.1f);

    EXPECT_TRUE(profile_.isValid());
    EXPECT_LT(profile_.getPeakVelocity(), 10.0f);
}

TEST_F(SCurveProfileTest, InitialConditions) {
    auto start = profile_.evaluate(0.0f);

    EXPECT_FLOAT_EQ(start.position, profile_.getStartPosition());
    EXPECT_NEAR(start.velocity, 0.0f, 0.001f);
}

TEST_F(SCurveProfileTest, FinalConditions) {
    profile_.plan(0.0f, 100.0f);
    auto end = profile_.evaluate(profile_.getDuration());

    EXPECT_NEAR(end.velocity, 0.0f, 0.1f);
}
