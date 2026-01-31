/**
 * @file filters.hpp
 * @brief Digital filters for signal processing
 */

#pragma once

#include <cmath>
#include <array>
#include <algorithm>

namespace omni::control {

/**
 * @brief First-order low-pass filter
 */
class LowPassFilter {
public:
    /**
     * @brief Construct low-pass filter
     * @param cutoffFreq Cutoff frequency (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    LowPassFilter(float cutoffFreq = 100.0f, float sampleFreq = 1000.0f)
        : output_(0), firstSample_(true) {
        setCutoff(cutoffFreq, sampleFreq);
    }

    /**
     * @brief Filter input value
     * @param input New input sample
     * @return Filtered output
     */
    float filter(float input) {
        if (firstSample_) {
            output_ = input;
            firstSample_ = false;
        } else {
            output_ = alpha_ * input + (1.0f - alpha_) * output_;
        }
        return output_;
    }

    /**
     * @brief Reset filter state
     * @param value Initial value (default 0)
     */
    void reset(float value = 0.0f) {
        output_ = value;
        firstSample_ = (value == 0.0f);
    }

    /**
     * @brief Set cutoff frequency
     * @param cutoffFreq Cutoff frequency (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    void setCutoff(float cutoffFreq, float sampleFreq) {
        cutoffFreq_ = cutoffFreq;
        sampleFreq_ = sampleFreq;

        // Calculate filter coefficient
        float dt = 1.0f / sampleFreq;
        float rc = 1.0f / (2.0f * 3.14159265f * cutoffFreq);
        alpha_ = dt / (rc + dt);
    }

    /**
     * @brief Get cutoff frequency
     * @return Cutoff frequency (Hz)
     */
    float getCutoff() const { return cutoffFreq_; }

    /**
     * @brief Get current output
     * @return Last filtered value
     */
    float getOutput() const { return output_; }

private:
    float alpha_;
    float output_;
    float cutoffFreq_;
    float sampleFreq_;
    bool firstSample_;
};

/**
 * @brief Second-order (biquad) filter
 */
class BiquadFilter {
public:
    enum class Type {
        LowPass,
        HighPass,
        BandPass,
        Notch,
        Peak,
        LowShelf,
        HighShelf
    };

    /**
     * @brief Construct biquad filter
     * @param type Filter type
     * @param freq Center/cutoff frequency (Hz)
     * @param sampleFreq Sample frequency (Hz)
     * @param Q Quality factor
     * @param gainDb Gain in dB (for Peak/Shelf types)
     */
    BiquadFilter(Type type = Type::LowPass, float freq = 100.0f,
                 float sampleFreq = 1000.0f, float Q = 0.707f, float gainDb = 0.0f) {
        setParams(type, freq, sampleFreq, Q, gainDb);
        reset();
    }

    /**
     * @brief Set filter parameters
     */
    void setParams(Type type, float freq, float sampleFreq, float Q, float gainDb = 0.0f) {
        float w0 = 2.0f * 3.14159265f * freq / sampleFreq;
        float sinW0 = std::sin(w0);
        float cosW0 = std::cos(w0);
        float alpha = sinW0 / (2.0f * Q);
        float A = std::pow(10.0f, gainDb / 40.0f);

        switch (type) {
            case Type::LowPass:
                b0_ = (1.0f - cosW0) / 2.0f;
                b1_ = 1.0f - cosW0;
                b2_ = (1.0f - cosW0) / 2.0f;
                a0_ = 1.0f + alpha;
                a1_ = -2.0f * cosW0;
                a2_ = 1.0f - alpha;
                break;

            case Type::HighPass:
                b0_ = (1.0f + cosW0) / 2.0f;
                b1_ = -(1.0f + cosW0);
                b2_ = (1.0f + cosW0) / 2.0f;
                a0_ = 1.0f + alpha;
                a1_ = -2.0f * cosW0;
                a2_ = 1.0f - alpha;
                break;

            case Type::BandPass:
                b0_ = alpha;
                b1_ = 0.0f;
                b2_ = -alpha;
                a0_ = 1.0f + alpha;
                a1_ = -2.0f * cosW0;
                a2_ = 1.0f - alpha;
                break;

            case Type::Notch:
                b0_ = 1.0f;
                b1_ = -2.0f * cosW0;
                b2_ = 1.0f;
                a0_ = 1.0f + alpha;
                a1_ = -2.0f * cosW0;
                a2_ = 1.0f - alpha;
                break;

            case Type::Peak:
                b0_ = 1.0f + alpha * A;
                b1_ = -2.0f * cosW0;
                b2_ = 1.0f - alpha * A;
                a0_ = 1.0f + alpha / A;
                a1_ = -2.0f * cosW0;
                a2_ = 1.0f - alpha / A;
                break;

            case Type::LowShelf:
                b0_ = A * ((A + 1.0f) - (A - 1.0f) * cosW0 + 2.0f * std::sqrt(A) * alpha);
                b1_ = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosW0);
                b2_ = A * ((A + 1.0f) - (A - 1.0f) * cosW0 - 2.0f * std::sqrt(A) * alpha);
                a0_ = (A + 1.0f) + (A - 1.0f) * cosW0 + 2.0f * std::sqrt(A) * alpha;
                a1_ = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosW0);
                a2_ = (A + 1.0f) + (A - 1.0f) * cosW0 - 2.0f * std::sqrt(A) * alpha;
                break;

            case Type::HighShelf:
                b0_ = A * ((A + 1.0f) + (A - 1.0f) * cosW0 + 2.0f * std::sqrt(A) * alpha);
                b1_ = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosW0);
                b2_ = A * ((A + 1.0f) + (A - 1.0f) * cosW0 - 2.0f * std::sqrt(A) * alpha);
                a0_ = (A + 1.0f) - (A - 1.0f) * cosW0 + 2.0f * std::sqrt(A) * alpha;
                a1_ = 2.0f * ((A - 1.0f) - (A + 1.0f) * cosW0);
                a2_ = (A + 1.0f) - (A - 1.0f) * cosW0 - 2.0f * std::sqrt(A) * alpha;
                break;
        }

        // Normalize
        b0_ /= a0_;
        b1_ /= a0_;
        b2_ /= a0_;
        a1_ /= a0_;
        a2_ /= a0_;
    }

    /**
     * @brief Filter input sample
     * @param input Input sample
     * @return Filtered output
     */
    float filter(float input) {
        float output = b0_ * input + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

        x2_ = x1_;
        x1_ = input;
        y2_ = y1_;
        y1_ = output;

        return output;
    }

    /**
     * @brief Reset filter state
     */
    void reset() {
        x1_ = x2_ = y1_ = y2_ = 0.0f;
    }

private:
    float b0_, b1_, b2_;
    float a0_, a1_, a2_;
    float x1_, x2_;  // Input history
    float y1_, y2_;  // Output history
};

/**
 * @brief Notch filter (specialized biquad)
 */
class NotchFilter {
public:
    /**
     * @brief Construct notch filter
     * @param centerFreq Center frequency (Hz)
     * @param bandwidth Bandwidth (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    NotchFilter(float centerFreq = 50.0f, float bandwidth = 10.0f,
                float sampleFreq = 1000.0f) {
        setParams(centerFreq, bandwidth, sampleFreq);
    }

    /**
     * @brief Set filter parameters
     * @param centerFreq Center frequency (Hz)
     * @param bandwidth Bandwidth (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    void setParams(float centerFreq, float bandwidth, float sampleFreq) {
        centerFreq_ = centerFreq;
        bandwidth_ = bandwidth;
        float Q = centerFreq / bandwidth;
        biquad_.setParams(BiquadFilter::Type::Notch, centerFreq, sampleFreq, Q);
    }

    /**
     * @brief Filter input
     * @param input Input sample
     * @return Filtered output
     */
    float filter(float input) {
        return biquad_.filter(input);
    }

    /**
     * @brief Reset filter
     */
    void reset() {
        biquad_.reset();
    }

    float getCenterFreq() const { return centerFreq_; }
    float getBandwidth() const { return bandwidth_; }

private:
    BiquadFilter biquad_;
    float centerFreq_;
    float bandwidth_;
};

/**
 * @brief Moving average filter
 */
template<int N>
class MovingAverageFilter {
public:
    MovingAverageFilter() : index_(0), sum_(0), filled_(false) {
        buffer_.fill(0);
    }

    /**
     * @brief Add sample and get average
     * @param input Input sample
     * @return Moving average
     */
    float filter(float input) {
        sum_ -= buffer_[index_];
        buffer_[index_] = input;
        sum_ += input;

        index_ = (index_ + 1) % N;
        if (index_ == 0) filled_ = true;

        int count = filled_ ? N : (index_ == 0 ? N : index_);
        return sum_ / static_cast<float>(count);
    }

    /**
     * @brief Reset filter
     */
    void reset() {
        buffer_.fill(0);
        index_ = 0;
        sum_ = 0;
        filled_ = false;
    }

    /**
     * @brief Get current average
     */
    float getAverage() const {
        int count = filled_ ? N : (index_ == 0 ? N : index_);
        return (count > 0) ? sum_ / static_cast<float>(count) : 0;
    }

private:
    std::array<float, N> buffer_;
    int index_;
    float sum_;
    bool filled_;
};

/**
 * @brief Derivative filter with low-pass
 *
 * Computes derivative of signal while filtering high-frequency noise.
 */
class DerivativeFilter {
public:
    /**
     * @brief Construct derivative filter
     * @param cutoffFreq Low-pass cutoff frequency (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    DerivativeFilter(float cutoffFreq = 100.0f, float sampleFreq = 1000.0f)
        : lpf_(cutoffFreq, sampleFreq)
        , samplePeriod_(1.0f / sampleFreq)
        , prevInput_(0)
        , firstSample_(true) {}

    /**
     * @brief Compute filtered derivative
     * @param input Input sample
     * @return Filtered derivative
     */
    float filter(float input) {
        if (firstSample_) {
            prevInput_ = input;
            firstSample_ = false;
            return 0;
        }

        float rawDerivative = (input - prevInput_) / samplePeriod_;
        prevInput_ = input;

        return lpf_.filter(rawDerivative);
    }

    /**
     * @brief Reset filter
     */
    void reset() {
        lpf_.reset();
        prevInput_ = 0;
        firstSample_ = true;
    }

    /**
     * @brief Set cutoff frequency
     * @param cutoffFreq Cutoff frequency (Hz)
     * @param sampleFreq Sample frequency (Hz)
     */
    void setCutoff(float cutoffFreq, float sampleFreq) {
        lpf_.setCutoff(cutoffFreq, sampleFreq);
        samplePeriod_ = 1.0f / sampleFreq;
    }

private:
    LowPassFilter lpf_;
    float samplePeriod_;
    float prevInput_;
    bool firstSample_;
};

}  // namespace omni::control
