/**
 * @file mit_protocol.hpp
 * @brief MIT/Cheetah protocol for CAN motor control
 */

#pragma once

#include "omni/hal/types.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace omni::protocol {

/**
 * @brief MIT protocol command structure
 */
struct MitCommand {
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};

/**
 * @brief MIT protocol feedback structure
 */
struct MitFeedback {
    uint8_t motorId = 0;
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
};

/**
 * @brief MIT protocol encoder/decoder
 *
 * Implements the MIT Mini Cheetah motor protocol commonly used
 * with quadruped robots and similar applications.
 */
class MitProtocol {
public:
    /**
     * @brief Parameter ranges
     */
    struct Ranges {
        float posMin = -12.5f;
        float posMax = 12.5f;
        float velMin = -50.0f;
        float velMax = 50.0f;
        float torqueMin = -18.0f;
        float torqueMax = 18.0f;
        float kpMin = 0.0f;
        float kpMax = 500.0f;
        float kdMin = 0.0f;
        float kdMax = 5.0f;
    };

    MitProtocol() = default;
    explicit MitProtocol(const Ranges& ranges) : ranges_(ranges) {}

    /**
     * @brief Encode command to CAN frame
     * @param motorId Motor ID (1-based)
     * @param cmd Command to encode
     * @param frame Output CAN frame
     */
    void encodeCommand(uint8_t motorId, const MitCommand& cmd,
                       hal::CanFrame& frame) const {
        // Convert floats to integers
        uint16_t posInt = floatToUint(cmd.position, ranges_.posMin, ranges_.posMax, 16);
        uint16_t velInt = floatToUint(cmd.velocity, ranges_.velMin, ranges_.velMax, 12);
        uint16_t kpInt = floatToUint(cmd.kp, ranges_.kpMin, ranges_.kpMax, 12);
        uint16_t kdInt = floatToUint(cmd.kd, ranges_.kdMin, ranges_.kdMax, 12);
        uint16_t torqueInt = floatToUint(cmd.torque, ranges_.torqueMin, ranges_.torqueMax, 12);

        // Pack into 8 bytes
        frame.id = motorId;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        frame.data[0] = posInt >> 8;
        frame.data[1] = posInt & 0xFF;
        frame.data[2] = velInt >> 4;
        frame.data[3] = ((velInt & 0x0F) << 4) | (kpInt >> 8);
        frame.data[4] = kpInt & 0xFF;
        frame.data[5] = kdInt >> 4;
        frame.data[6] = ((kdInt & 0x0F) << 4) | (torqueInt >> 8);
        frame.data[7] = torqueInt & 0xFF;
    }

    /**
     * @brief Decode feedback from CAN frame
     * @param frame CAN frame to decode
     * @param fb Output feedback structure
     * @return true if decode successful
     */
    bool decodeFeedback(const hal::CanFrame& frame, MitFeedback& fb) const {
        if (frame.len < 6) {
            return false;
        }

        fb.motorId = frame.id & 0xFF;

        // Unpack bytes
        uint16_t posInt = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
        uint16_t velInt = (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
        uint16_t torqueInt = (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];

        // Convert to floats
        fb.position = uintToFloat(posInt, ranges_.posMin, ranges_.posMax, 16);
        fb.velocity = uintToFloat(velInt, ranges_.velMin, ranges_.velMax, 12);
        fb.torque = uintToFloat(torqueInt, ranges_.torqueMin, ranges_.torqueMax, 12);

        return true;
    }

    /**
     * @brief Encode enter motor mode command
     * @param motorId Motor ID
     * @param frame Output CAN frame
     */
    static void encodeEnterMotorMode(uint8_t motorId, hal::CanFrame& frame) {
        frame.id = motorId;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFC;
    }

    /**
     * @brief Encode exit motor mode command
     * @param motorId Motor ID
     * @param frame Output CAN frame
     */
    static void encodeExitMotorMode(uint8_t motorId, hal::CanFrame& frame) {
        frame.id = motorId;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFD;
    }

    /**
     * @brief Encode set zero position command
     * @param motorId Motor ID
     * @param frame Output CAN frame
     */
    static void encodeSetZero(uint8_t motorId, hal::CanFrame& frame) {
        frame.id = motorId;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xFF;
        frame.data[3] = 0xFF;
        frame.data[4] = 0xFF;
        frame.data[5] = 0xFF;
        frame.data[6] = 0xFF;
        frame.data[7] = 0xFE;
    }

    // === Range Setters ===

    void setPosRange(float min, float max) {
        ranges_.posMin = min;
        ranges_.posMax = max;
    }

    void setVelRange(float min, float max) {
        ranges_.velMin = min;
        ranges_.velMax = max;
    }

    void setTorqueRange(float min, float max) {
        ranges_.torqueMin = min;
        ranges_.torqueMax = max;
    }

    void setKpRange(float min, float max) {
        ranges_.kpMin = min;
        ranges_.kpMax = max;
    }

    void setKdRange(float min, float max) {
        ranges_.kdMin = min;
        ranges_.kdMax = max;
    }

    void setRanges(const Ranges& ranges) { ranges_ = ranges; }
    const Ranges& getRanges() const { return ranges_; }

private:
    Ranges ranges_;

    /**
     * @brief Convert float to unsigned integer with scaling
     */
    static uint16_t floatToUint(float value, float min, float max, int bits) {
        float span = max - min;
        float normalized = (value - min) / span;
        normalized = std::clamp(normalized, 0.0f, 1.0f);
        return static_cast<uint16_t>(normalized * ((1 << bits) - 1));
    }

    /**
     * @brief Convert unsigned integer to float with scaling
     */
    static float uintToFloat(uint16_t value, float min, float max, int bits) {
        float span = max - min;
        float normalized = static_cast<float>(value) / ((1 << bits) - 1);
        return normalized * span + min;
    }
};

}  // namespace omni::protocol
