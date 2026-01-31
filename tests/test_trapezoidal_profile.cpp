/**
 * @file test_trapezoidal_profile.cpp
 * @brief Unit tests for trapezoidal profile generator
 */

#include <gtest/gtest.h>
#include <omni/motion/trapezoidal_profile.hpp>
#include <cmath>

using namespace omni::motion;

class TrapezoidalProfileTest : public ::testing::Test {
protected:
    void SetUp() override {
        MotionConstraints constraints;
        constraints.maxVelocity = 10.0f;
        constraints.maxAcceleration = 100.0f;
        profile_ = TrapezoidalProfile(constraints);
    }

    TrapezoidalProfile profile_;
};

TEST_F(TrapezoidalProfileTest, BasicPlanning) {
    EXPECT_TRUE(profile_.plan(0.0f, 100.0f));
    EXPECT_TRUE(profile_.isValid());
    EXPECT_GT(profile_.getDuration(), 0.0f);
}

TEST_F(TrapezoidalProfileTest, StartEndPositions) {
    profile_.plan(10.0f, 50.0f);

    auto start = profile_.evaluate(0.0f);
    auto end = profile_.evaluate(profile_.getDuration());

    EXPECT_NEAR(start.position, 10.0f, 0.001f);
    EXPECT_NEAR(end.position, 50.0f, 0.001f);
}

TEST_F(TrapezoidalProfileTest, ZeroMotion) {
    profile_.plan(0.0f, 0.0f);

    EXPECT_NEAR(profile_.getDuration(), 0.0f, 0.001f);

    auto point = profile_.evaluate(0.0f);
    EXPECT_FLOAT_EQ(point.position, 0.0f);
}

TEST_F(TrapezoidalProfileTest, NegativeMotion) {
    profile_.plan(100.0f, 0.0f);

    EXPECT_TRUE(profile_.isValid());

    auto end = profile_.evaluate(profile_.getDuration());
    EXPECT_NEAR(end.position, 0.0f, 0.001f);
}

TEST_F(TrapezoidalProfileTest, VelocityLimits) {
    profile_.plan(0.0f, 1000.0f);

    // Check that velocity never exceeds limit
    for (float t = 0; t <= profile_.getDuration(); t += 0.01f) {
        auto point = profile_.evaluate(t);
        EXPECT_LE(std::abs(point.velocity), 10.0f + 0.01f);
    }
}

TEST_F(TrapezoidalProfileTest, ShortDistance) {
    // Short distance that can't reach max velocity
    MotionConstraints constraints;
    constraints.maxVelocity = 100.0f;
    constraints.maxAcceleration = 10.0f;
    TrapezoidalProfile shortProfile(constraints);

    shortProfile.plan(0.0f, 1.0f);

    // Should be triangular (no constant velocity phase)
    EXPECT_NEAR(shortProfile.getConstTime(), 0.0f, 0.001f);
    EXPECT_LT(shortProfile.getPeakVelocity(), constraints.maxVelocity);
}

TEST_F(TrapezoidalProfileTest, FullTrapezoidal) {
    // Long distance that reaches max velocity
    profile_.plan(0.0f, 100.0f);

    // Should have constant velocity phase
    EXPECT_GT(profile_.getConstTime(), 0.0f);
    EXPECT_NEAR(std::abs(profile_.getPeakVelocity()), 10.0f, 0.01f);
}

TEST_F(TrapezoidalProfileTest, InitialVelocity) {
    profile_.plan(0.0f, 100.0f, 5.0f, 0.0f);

    auto start = profile_.evaluate(0.0f);
    EXPECT_NEAR(start.velocity, 5.0f, 0.1f);
}

TEST_F(TrapezoidalProfileTest, Continuity) {
    profile_.plan(0.0f, 100.0f);

    float prevPos = 0;
    float prevVel = 0;

    for (float t = 0.01f; t <= profile_.getDuration(); t += 0.01f) {
        auto point = profile_.evaluate(t);

        // Position should be continuous
        EXPECT_LT(std::abs(point.position - prevPos), 1.0f);

        prevPos = point.position;
        prevVel = point.velocity;
    }
}

TEST_F(TrapezoidalProfileTest, AsymmetricAccelDecel) {
    MotionConstraints constraints;
    constraints.maxVelocity = 10.0f;
    constraints.maxAcceleration = 100.0f;
    constraints.maxDeceleration = 50.0f;  // Different decel
    TrapezoidalProfile asymProfile(constraints);

    asymProfile.plan(0.0f, 100.0f);

    EXPECT_TRUE(asymProfile.isValid());
    EXPECT_GT(asymProfile.getDecelTime(), asymProfile.getAccelTime());
}
