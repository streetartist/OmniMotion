/**
 * @file test_filters.cpp
 * @brief Unit tests for digital filters
 */

#include <gtest/gtest.h>
#include <omni/control/filters.hpp>
#include <cmath>

using namespace omni::control;

class LowPassFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = LowPassFilter(10.0f, 1000.0f);  // 10Hz cutoff, 1kHz sample
    }

    LowPassFilter filter_;
};

TEST_F(LowPassFilterTest, FirstSamplePassthrough) {
    float input = 100.0f;
    float output = filter_.filter(input);
    EXPECT_FLOAT_EQ(output, input);
}

TEST_F(LowPassFilterTest, StepResponse) {
    // Apply step input
    for (int i = 0; i < 1000; i++) {
        filter_.filter(100.0f);
    }

    // Should converge to step value
    EXPECT_NEAR(filter_.getOutput(), 100.0f, 0.1f);
}

TEST_F(LowPassFilterTest, HighFrequencyAttenuation) {
    LowPassFilter filter(10.0f, 1000.0f);

    // Generate high frequency signal (100Hz)
    float maxOutput = 0;
    for (int i = 0; i < 1000; i++) {
        float input = std::sin(2.0f * 3.14159f * 100.0f * i / 1000.0f);
        float output = filter.filter(input);
        if (i > 100) {  // After settling
            maxOutput = std::max(maxOutput, std::abs(output));
        }
    }

    // High frequency should be attenuated significantly
    EXPECT_LT(maxOutput, 0.2f);
}

TEST_F(LowPassFilterTest, Reset) {
    filter_.filter(100.0f);
    filter_.filter(100.0f);

    filter_.reset();
    float output = filter_.filter(50.0f);

    EXPECT_FLOAT_EQ(output, 50.0f);  // First sample after reset
}

class BiquadFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = BiquadFilter(BiquadFilter::Type::LowPass, 100.0f, 1000.0f, 0.707f);
    }

    BiquadFilter filter_;
};

TEST_F(BiquadFilterTest, StepResponse) {
    for (int i = 0; i < 1000; i++) {
        filter_.filter(1.0f);
    }

    // Should converge (low-pass passes DC)
    float output = filter_.filter(1.0f);
    EXPECT_NEAR(output, 1.0f, 0.01f);
}

TEST_F(BiquadFilterTest, NotchAttenuation) {
    BiquadFilter notch(BiquadFilter::Type::Notch, 50.0f, 1000.0f, 10.0f);

    // Generate signal at notch frequency
    float maxOutput = 0;
    for (int i = 0; i < 1000; i++) {
        float input = std::sin(2.0f * 3.14159f * 50.0f * i / 1000.0f);
        float output = notch.filter(input);
        if (i > 200) {
            maxOutput = std::max(maxOutput, std::abs(output));
        }
    }

    EXPECT_LT(maxOutput, 0.1f);
}

class NotchFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = NotchFilter(60.0f, 5.0f, 1000.0f);  // 60Hz notch
    }

    NotchFilter filter_;
};

TEST_F(NotchFilterTest, PassesDC) {
    for (int i = 0; i < 1000; i++) {
        filter_.filter(1.0f);
    }
    EXPECT_NEAR(filter_.filter(1.0f), 1.0f, 0.01f);
}

TEST_F(NotchFilterTest, AttenuatesNotchFrequency) {
    float maxOutput = 0;
    for (int i = 0; i < 2000; i++) {
        float input = std::sin(2.0f * 3.14159f * 60.0f * i / 1000.0f);
        float output = filter_.filter(input);
        if (i > 500) {
            maxOutput = std::max(maxOutput, std::abs(output));
        }
    }
    EXPECT_LT(maxOutput, 0.15f);
}

class MovingAverageTest : public ::testing::Test {
protected:
    MovingAverageFilter<float, 10> filter_;
};

TEST_F(MovingAverageTest, AveragesCorrectly) {
    // Fill with 10 samples of 5.0
    for (int i = 0; i < 10; i++) {
        filter_.filter(5.0f);
    }

    EXPECT_NEAR(filter_.getAverage(), 5.0f, 0.001f);
}

TEST_F(MovingAverageTest, SlidingWindow) {
    // Fill with values 1-10
    for (int i = 1; i <= 10; i++) {
        filter_.filter(static_cast<float>(i));
    }

    // Average of 1-10 = 5.5
    EXPECT_NEAR(filter_.getAverage(), 5.5f, 0.001f);

    // Add 11, should now average 2-11 = 6.5
    filter_.filter(11.0f);
    EXPECT_NEAR(filter_.getAverage(), 6.5f, 0.001f);
}

TEST_F(MovingAverageTest, Reset) {
    filter_.filter(100.0f);
    filter_.reset();
    float result = filter_.filter(50.0f);
    EXPECT_NEAR(result, 50.0f, 0.001f);
}
