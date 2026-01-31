/**
 * @file can_protocol.hpp
 * @brief CAN protocol interfaces and common implementations
 */

#pragma once

#include "omni/hal/types.hpp"
#include "omni/driver/types.hpp"
#include <cstdint>

namespace omni::protocol {

/**
 * @brief CAN protocol interface
 *
 * Base interface for implementing motor CAN protocols.
 */
class ICanProtocol {
public:
    virtual ~ICanProtocol() = default;

    /**
     * @brief Encode a motor command to CAN frame
     * @param cmd Motor command
     * @param frame Output CAN frame
     */
    virtual void encodeCommand(const driver::MotorCommand& cmd,
                               hal::CanFrame& frame) = 0;

    /**
     * @brief Decode feedback from CAN frame
     * @param frame CAN frame
     * @param state Output motor state
     * @return true if decode successful
     */
    virtual bool decodeFeedback(const hal::CanFrame& frame,
                                driver::MotorState& state) = 0;

    /**
     * @brief Get command CAN ID for motor
     * @param motorId Motor ID
     * @return CAN ID for commands
     */
    virtual uint32_t getCommandId(uint8_t motorId) = 0;

    /**
     * @brief Get feedback CAN ID for motor
     * @param motorId Motor ID
     * @return CAN ID for feedback
     */
    virtual uint32_t getFeedbackId(uint8_t motorId) = 0;

    /**
     * @brief Set motor ID
     * @param id Motor ID
     */
    virtual void setMotorId(uint8_t id) { motorId_ = id; }

    /**
     * @brief Get motor ID
     * @return Motor ID
     */
    virtual uint8_t getMotorId() const { return motorId_; }

protected:
    uint8_t motorId_ = 1;
};

/**
 * @brief DJI motor protocol (C610/C620 ESC)
 */
class DjiCanProtocol : public ICanProtocol {
public:
    explicit DjiCanProtocol(uint8_t motorId = 1) {
        motorId_ = motorId;
    }

    void encodeCommand(const driver::MotorCommand& cmd,
                       hal::CanFrame& frame) override {
        // DJI uses 0x200 for motors 1-4, 0x1FF for motors 5-8
        frame.id = (motorId_ <= 4) ? 0x200 : 0x1FF;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        // Clear frame
        for (int i = 0; i < 8; i++) frame.data[i] = 0;

        // Current command in mA, big-endian
        int16_t currentMa = static_cast<int16_t>(cmd.current * 1000.0f);
        int idx = ((motorId_ - 1) % 4) * 2;
        frame.data[idx] = (currentMa >> 8) & 0xFF;
        frame.data[idx + 1] = currentMa & 0xFF;
    }

    bool decodeFeedback(const hal::CanFrame& frame,
                        driver::MotorState& state) override {
        // Feedback ID is 0x201 + (motorId - 1)
        uint32_t expectedId = 0x200 + motorId_;
        if (frame.id != expectedId || frame.len < 8) {
            return false;
        }

        // Decode angle (0-8191 for 0-360 degrees)
        uint16_t angle = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        state.position = (angle / 8191.0f) * 2.0f * 3.14159265f;

        // Decode velocity (RPM)
        int16_t rpm = (static_cast<int16_t>(frame.data[2]) << 8) | frame.data[3];
        state.velocity = rpm * 2.0f * 3.14159265f / 60.0f;

        // Decode torque current (mA)
        int16_t currentMa = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];
        state.current = currentMa / 1000.0f;

        // Temperature
        state.temperature = static_cast<float>(frame.data[6]);

        return true;
    }

    uint32_t getCommandId(uint8_t motorId) override {
        return (motorId <= 4) ? 0x200 : 0x1FF;
    }

    uint32_t getFeedbackId(uint8_t motorId) override {
        return 0x200 + motorId;
    }
};

/**
 * @brief ODrive CAN protocol
 */
class OdriveCanProtocol : public ICanProtocol {
public:
    enum class CommandId : uint8_t {
        Heartbeat = 0x01,
        GetError = 0x03,
        SetAxisState = 0x07,
        GetEncoderEstimates = 0x09,
        SetControllerMode = 0x0B,
        SetInputPos = 0x0C,
        SetInputVel = 0x0D,
        SetInputTorque = 0x0E,
        GetIq = 0x14,
        GetBusVoltage = 0x17
    };

    explicit OdriveCanProtocol(uint8_t nodeId = 0) {
        motorId_ = nodeId;
    }

    void encodeCommand(const driver::MotorCommand& cmd,
                       hal::CanFrame& frame) override {
        // ODrive CAN ID = nodeId << 5 | cmdId
        uint32_t cmdId;
        switch (cmd.mode) {
            case driver::ControlMode::Position:
                cmdId = static_cast<uint8_t>(CommandId::SetInputPos);
                break;
            case driver::ControlMode::Velocity:
                cmdId = static_cast<uint8_t>(CommandId::SetInputVel);
                break;
            case driver::ControlMode::Current:
                cmdId = static_cast<uint8_t>(CommandId::SetInputTorque);
                break;
            default:
                cmdId = static_cast<uint8_t>(CommandId::SetInputTorque);
        }

        frame.id = (motorId_ << 5) | cmdId;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        // Pack float values (little-endian)
        float value = 0;
        switch (cmd.mode) {
            case driver::ControlMode::Position:
                value = cmd.position;
                break;
            case driver::ControlMode::Velocity:
                value = cmd.velocity;
                break;
            case driver::ControlMode::Current:
                value = cmd.torque;
                break;
            default:
                value = 0;
        }

        // Pack as 2 floats (position command includes velocity FF)
        packFloat(frame.data, value);
        packFloat(frame.data + 4, 0);  // Feedforward
    }

    bool decodeFeedback(const hal::CanFrame& frame,
                        driver::MotorState& state) override {
        uint8_t nodeId = (frame.id >> 5) & 0x3F;
        uint8_t cmdId = frame.id & 0x1F;

        if (nodeId != motorId_) {
            return false;
        }

        if (cmdId == static_cast<uint8_t>(CommandId::GetEncoderEstimates)) {
            state.position = unpackFloat(frame.data);
            state.velocity = unpackFloat(frame.data + 4);
            return true;
        }

        if (cmdId == static_cast<uint8_t>(CommandId::GetIq)) {
            state.current = unpackFloat(frame.data);
            return true;
        }

        return false;
    }

    uint32_t getCommandId(uint8_t motorId) override {
        return (motorId << 5) | static_cast<uint8_t>(CommandId::SetInputPos);
    }

    uint32_t getFeedbackId(uint8_t motorId) override {
        return (motorId << 5) | static_cast<uint8_t>(CommandId::GetEncoderEstimates);
    }

    /**
     * @brief Create request encoder estimates frame
     */
    void encodeGetEncoderEstimates(hal::CanFrame& frame) {
        frame.id = (motorId_ << 5) | static_cast<uint8_t>(CommandId::GetEncoderEstimates);
        frame.len = 0;
        frame.extendedId = false;
        frame.remoteRequest = true;
    }

private:
    static void packFloat(uint8_t* data, float value) {
        uint32_t bits;
        std::memcpy(&bits, &value, sizeof(float));
        data[0] = bits & 0xFF;
        data[1] = (bits >> 8) & 0xFF;
        data[2] = (bits >> 16) & 0xFF;
        data[3] = (bits >> 24) & 0xFF;
    }

    static float unpackFloat(const uint8_t* data) {
        uint32_t bits = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
        float value;
        std::memcpy(&value, &bits, sizeof(float));
        return value;
    }
};

/**
 * @brief CyberGear (Xiaomi) protocol
 */
class CybergearProtocol : public ICanProtocol {
public:
    enum class CommandType : uint8_t {
        GetMotorId = 0x00,
        SetRunMode = 0x12,
        SetIqRef = 0x14,
        SetSpdRef = 0x15,
        SetLimitSpdRef = 0x16,
        SetCurFilterGain = 0x17,
        SetPosRef = 0x18,
        SetLimitCurRef = 0x19,
        EnableMotor = 0x03,
        StopMotor = 0x04,
        SetMechPosAsZero = 0x06,
        MitCtrl = 0x01
    };

    explicit CybergearProtocol(uint8_t motorId = 127, uint8_t hostId = 0)
        : hostId_(hostId) {
        motorId_ = motorId;
    }

    void encodeCommand(const driver::MotorCommand& cmd,
                       hal::CanFrame& frame) override {
        // MIT-style control mode
        frame.id = buildExtId(motorId_, static_cast<uint8_t>(CommandType::MitCtrl), hostId_);
        frame.len = 8;
        frame.extendedId = true;
        frame.remoteRequest = false;

        // Pack MIT-style command
        uint16_t posInt = floatToUint(cmd.position, -12.5f, 12.5f, 16);
        uint16_t velInt = floatToUint(cmd.velocity, -30.0f, 30.0f, 16);
        uint16_t kpInt = floatToUint(cmd.kp, 0.0f, 500.0f, 16);
        uint16_t kdInt = floatToUint(cmd.kd, 0.0f, 5.0f, 16);
        uint16_t torqueInt = floatToUint(cmd.torque, -12.0f, 12.0f, 16);

        frame.data[0] = posInt >> 8;
        frame.data[1] = posInt & 0xFF;
        frame.data[2] = velInt >> 8;
        frame.data[3] = velInt & 0xFF;
        frame.data[4] = kpInt >> 8;
        frame.data[5] = kdInt >> 8;
        frame.data[6] = torqueInt >> 8;
        frame.data[7] = torqueInt & 0xFF;
    }

    bool decodeFeedback(const hal::CanFrame& frame,
                        driver::MotorState& state) override {
        if (!frame.extendedId) return false;

        // Extract motor ID from extended ID
        uint8_t srcId = (frame.id >> 8) & 0xFF;
        if (srcId != motorId_) return false;

        // Decode feedback
        uint16_t posInt = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        uint16_t velInt = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
        uint16_t torqueInt = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

        state.position = uintToFloat(posInt, -12.5f, 12.5f, 16);
        state.velocity = uintToFloat(velInt, -30.0f, 30.0f, 16);
        state.torque = uintToFloat(torqueInt, -12.0f, 12.0f, 16);
        state.temperature = static_cast<float>(frame.data[6]);

        return true;
    }

    uint32_t getCommandId(uint8_t motorId) override {
        return buildExtId(motorId, static_cast<uint8_t>(CommandType::MitCtrl), hostId_);
    }

    uint32_t getFeedbackId(uint8_t motorId) override {
        return buildExtId(hostId_, 0, motorId);
    }

    void setHostId(uint8_t hostId) { hostId_ = hostId; }
    uint8_t getHostId() const { return hostId_; }

private:
    uint8_t hostId_;

    static uint32_t buildExtId(uint8_t destId, uint8_t cmdType, uint8_t srcId) {
        return (static_cast<uint32_t>(cmdType) << 24) |
               (static_cast<uint32_t>(destId) << 8) |
               srcId;
    }

    static uint16_t floatToUint(float value, float min, float max, int bits) {
        float span = max - min;
        float normalized = (value - min) / span;
        if (normalized < 0) normalized = 0;
        if (normalized > 1) normalized = 1;
        return static_cast<uint16_t>(normalized * ((1 << bits) - 1));
    }

    static float uintToFloat(uint16_t value, float min, float max, int bits) {
        float span = max - min;
        float normalized = static_cast<float>(value) / ((1 << bits) - 1);
        return normalized * span + min;
    }
};

/**
 * @brief Damiao (DM) motor protocol
 */
class DmProtocol : public ICanProtocol {
public:
    explicit DmProtocol(uint8_t masterId = 0, uint8_t motorId = 1)
        : masterId_(masterId) {
        motorId_ = motorId;
    }

    void encodeCommand(const driver::MotorCommand& cmd,
                       hal::CanFrame& frame) override {
        // DM protocol similar to MIT
        frame.id = motorId_;
        frame.len = 8;
        frame.extendedId = false;
        frame.remoteRequest = false;

        uint16_t posInt = floatToUint(cmd.position, -12.5f, 12.5f, 16);
        uint16_t velInt = floatToUint(cmd.velocity, -45.0f, 45.0f, 12);
        uint16_t kpInt = floatToUint(cmd.kp, 0.0f, 500.0f, 12);
        uint16_t kdInt = floatToUint(cmd.kd, 0.0f, 5.0f, 12);
        uint16_t torqueInt = floatToUint(cmd.torque, -18.0f, 18.0f, 12);

        frame.data[0] = posInt >> 8;
        frame.data[1] = posInt & 0xFF;
        frame.data[2] = velInt >> 4;
        frame.data[3] = ((velInt & 0xF) << 4) | (kpInt >> 8);
        frame.data[4] = kpInt & 0xFF;
        frame.data[5] = kdInt >> 4;
        frame.data[6] = ((kdInt & 0xF) << 4) | (torqueInt >> 8);
        frame.data[7] = torqueInt & 0xFF;
    }

    bool decodeFeedback(const hal::CanFrame& frame,
                        driver::MotorState& state) override {
        if (frame.len < 8) return false;

        uint8_t id = frame.data[0];
        if (id != motorId_) return false;

        uint16_t posInt = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
        uint16_t velInt = (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
        uint16_t torqueInt = ((frame.data[4] & 0xF) << 8) | frame.data[5];

        state.position = uintToFloat(posInt, -12.5f, 12.5f, 16);
        state.velocity = uintToFloat(velInt, -45.0f, 45.0f, 12);
        state.torque = uintToFloat(torqueInt, -18.0f, 18.0f, 12);

        // Error and temperature
        state.temperature = static_cast<float>(frame.data[6]);
        state.errorCode = frame.data[7];

        return true;
    }

    uint32_t getCommandId(uint8_t motorId) override {
        return motorId;
    }

    uint32_t getFeedbackId(uint8_t motorId) override {
        return masterId_ + motorId;
    }

    void setMasterId(uint8_t masterId) { masterId_ = masterId; }
    uint8_t getMasterId() const { return masterId_; }

private:
    uint8_t masterId_;

    static uint16_t floatToUint(float value, float min, float max, int bits) {
        float span = max - min;
        float normalized = (value - min) / span;
        if (normalized < 0) normalized = 0;
        if (normalized > 1) normalized = 1;
        return static_cast<uint16_t>(normalized * ((1 << bits) - 1));
    }

    static float uintToFloat(uint16_t value, float min, float max, int bits) {
        float span = max - min;
        float normalized = static_cast<float>(value) / ((1 << bits) - 1);
        return normalized * span + min;
    }
};

}  // namespace omni::protocol
