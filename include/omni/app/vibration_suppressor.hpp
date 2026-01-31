/**
 * @file vibration_suppressor.hpp
 * @brief Vibration suppression using input shaping and filtering
 */

#pragma once

#include "motor_controller.hpp"
#include "omni/control/filters.hpp"
#include <vector>
#include <cmath>

namespace omni::app {

/**
 * @brief Input shaper types
 */
enum class InputShaperType {
    ZV,         // Zero Vibration
    ZVD,        // Zero Vibration and Derivative
    ZVDD,       // Zero Vibration and Double Derivative
    EI          // Extra Insensitive
};

/**
 * @brief Input shaper impulse
 */
struct ShaperImpulse {
    float amplitude;
    float time;
};

/**
 * @brief Vibration suppressor
 *
 * Reduces mechanical vibration using input shaping and notch filtering.
 */
class VibrationSuppressor {
public:
    /**
     * @brief Construct vibration suppressor
     * @param controller Motor controller
     */
    explicit VibrationSuppressor(MotorController* controller)
        : controller_(controller)
        , enabled_(false)
        , inputShaperEnabled_(false)
        , resonantFreq_(50.0f)
        , dampingRatio_(0.05f) {
        calculateShaper();
    }

    // === Input Shaper ===

    /**
     * @brief Set input shaper
     * @param type Shaper type
     * @param frequency Resonant frequency (Hz)
     * @param damping Damping ratio (0-1)
     */
    void setInputShaper(InputShaperType type, float frequency, float damping = 0.0f) {
        shaperType_ = type;
        resonantFreq_ = frequency;
        dampingRatio_ = damping;
        calculateShaper();
        inputShaperEnabled_ = true;
    }

    /**
     * @brief Get input shaper impulses
     * @return Vector of impulses
     */
    const std::vector<ShaperImpulse>& getImpulses() const {
        return impulses_;
    }

    // === Notch Filters ===

    /**
     * @brief Add notch filter at frequency
     * @param frequency Center frequency (Hz)
     * @param bandwidth Notch bandwidth (Hz)
     */
    void addNotchFilter(float frequency, float bandwidth) {
        notchFilters_.emplace_back(frequency, bandwidth, sampleFreq_);
    }

    /**
     * @brief Remove notch filter
     * @param frequency Frequency of filter to remove
     */
    void removeNotchFilter(float frequency) {
        for (auto it = notchFilters_.begin(); it != notchFilters_.end(); ) {
            if (std::abs(it->getCenterFreq() - frequency) < 1.0f) {
                it = notchFilters_.erase(it);
            } else {
                ++it;
            }
        }
    }

    /**
     * @brief Clear all notch filters
     */
    void clearNotchFilters() {
        notchFilters_.clear();
    }

    // === Auto Detection ===

    /**
     * @brief Start automatic frequency detection
     *
     * Performs a chirp test to identify resonant frequencies.
     */
    void autoDetectFrequency() {
        detecting_ = true;
        detectionPhase_ = 0;
        detectedFreq_ = 0;
        // Would implement FFT-based detection in a real system
    }

    /**
     * @brief Get detected frequency
     * @return Detected resonant frequency (Hz)
     */
    float getDetectedFrequency() const {
        return detectedFreq_;
    }

    /**
     * @brief Check if detection is complete
     */
    bool isDetectionComplete() const {
        return !detecting_;
    }

    // === Enable ===

    /**
     * @brief Enable/disable suppression
     * @param enable true to enable
     */
    void enable(bool enable) {
        enabled_ = enable;
    }

    /**
     * @brief Check if enabled
     */
    bool isEnabled() const {
        return enabled_;
    }

    // === Processing ===

    /**
     * @brief Apply input shaping to command
     * @param command Input command
     * @return Shaped command
     */
    float shapeCommand(float command) {
        if (!enabled_ || !inputShaperEnabled_) {
            return command;
        }

        // Add new command to history
        commandHistory_.push_back({command, historyTime_});

        // Remove old commands
        float maxDelay = impulses_.empty() ? 0 : impulses_.back().time;
        while (!commandHistory_.empty() &&
               (historyTime_ - commandHistory_.front().time) > maxDelay + 0.01f) {
            commandHistory_.erase(commandHistory_.begin());
        }

        // Convolve with impulse response
        float shaped = 0;
        for (const auto& cmd : commandHistory_) {
            float cmdAge = historyTime_ - cmd.time;
            for (const auto& imp : impulses_) {
                if (std::abs(cmdAge - imp.time) < 0.0001f) {
                    shaped += cmd.value * imp.amplitude;
                }
            }
        }

        historyTime_ += 0.001f;  // Assume 1kHz
        return shaped;
    }

    /**
     * @brief Apply notch filtering to signal
     * @param signal Input signal
     * @return Filtered signal
     */
    float filterSignal(float signal) {
        if (!enabled_) {
            return signal;
        }

        float filtered = signal;
        for (auto& filter : notchFilters_) {
            filtered = filter.filter(filtered);
        }
        return filtered;
    }

    /**
     * @brief Set sample frequency
     * @param freq Sample frequency (Hz)
     */
    void setSampleFrequency(float freq) {
        sampleFreq_ = freq;
        // Recreate notch filters with new frequency
        std::vector<std::pair<float, float>> filterParams;
        for (const auto& f : notchFilters_) {
            filterParams.push_back({f.getCenterFreq(), f.getBandwidth()});
        }
        notchFilters_.clear();
        for (const auto& p : filterParams) {
            addNotchFilter(p.first, p.second);
        }
    }

private:
    MotorController* controller_;

    bool enabled_;
    bool inputShaperEnabled_;

    // Input shaper
    InputShaperType shaperType_ = InputShaperType::ZV;
    float resonantFreq_;
    float dampingRatio_;
    std::vector<ShaperImpulse> impulses_;

    // Command history for convolution
    struct CommandEntry {
        float value;
        float time;
    };
    std::vector<CommandEntry> commandHistory_;
    float historyTime_ = 0;

    // Notch filters
    std::vector<control::NotchFilter> notchFilters_;
    float sampleFreq_ = 1000.0f;

    // Auto detection
    bool detecting_ = false;
    int detectionPhase_ = 0;
    float detectedFreq_ = 0;

    void calculateShaper() {
        impulses_.clear();

        float omega = 2.0f * 3.14159265f * resonantFreq_;
        float K = std::exp(-dampingRatio_ * 3.14159265f /
                          std::sqrt(1.0f - dampingRatio_ * dampingRatio_));
        float T = 1.0f / resonantFreq_;  // Period

        switch (shaperType_) {
            case InputShaperType::ZV: {
                float A1 = 1.0f / (1.0f + K);
                float A2 = K / (1.0f + K);
                impulses_.push_back({A1, 0});
                impulses_.push_back({A2, T / 2.0f});
                break;
            }

            case InputShaperType::ZVD: {
                float K2 = K * K;
                float denom = 1.0f + 2.0f * K + K2;
                impulses_.push_back({1.0f / denom, 0});
                impulses_.push_back({2.0f * K / denom, T / 2.0f});
                impulses_.push_back({K2 / denom, T});
                break;
            }

            case InputShaperType::ZVDD: {
                float K2 = K * K;
                float K3 = K2 * K;
                float denom = 1.0f + 3.0f * K + 3.0f * K2 + K3;
                impulses_.push_back({1.0f / denom, 0});
                impulses_.push_back({3.0f * K / denom, T / 2.0f});
                impulses_.push_back({3.0f * K2 / denom, T});
                impulses_.push_back({K3 / denom, 1.5f * T});
                break;
            }

            case InputShaperType::EI: {
                // Extra-insensitive shaper (more robust but slower)
                float V = 0.05f;  // Vibration tolerance
                float A1 = 0.25f * (1.0f + V);
                float A2 = 0.5f * (1.0f - V);
                float A3 = 0.25f * (1.0f + V);
                impulses_.push_back({A1, 0});
                impulses_.push_back({A2, T / 2.0f});
                impulses_.push_back({A3, T});
                break;
            }
        }
    }
};

}  // namespace omni::app
