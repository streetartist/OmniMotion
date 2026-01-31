/**
 * @file modbus_protocol.hpp
 * @brief Modbus RTU protocol for motor control
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace omni::protocol {

/**
 * @brief Modbus function codes
 */
enum class ModbusFunction : uint8_t {
    ReadCoils = 0x01,
    ReadDiscreteInputs = 0x02,
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters = 0x04,
    WriteSingleCoil = 0x05,
    WriteSingleRegister = 0x06,
    WriteMultipleCoils = 0x0F,
    WriteMultipleRegisters = 0x10
};

/**
 * @brief Standard motor register map
 */
struct ModbusRegisterMap {
    // Control registers
    uint16_t controlWord = 0x0000;
    uint16_t targetPosition = 0x0002;      // 32-bit (2 registers)
    uint16_t targetVelocity = 0x0004;      // 32-bit
    uint16_t targetTorque = 0x0006;        // 16-bit
    uint16_t targetCurrent = 0x0007;       // 16-bit

    // Status registers
    uint16_t statusWord = 0x0100;
    uint16_t actualPosition = 0x0102;      // 32-bit
    uint16_t actualVelocity = 0x0104;      // 32-bit
    uint16_t actualTorque = 0x0106;        // 16-bit
    uint16_t actualCurrent = 0x0107;       // 16-bit
    uint16_t dcBusVoltage = 0x0108;        // 16-bit
    uint16_t temperature = 0x0109;         // 16-bit
    uint16_t errorCode = 0x010A;           // 16-bit

    // Configuration registers
    uint16_t controlMode = 0x0200;
    uint16_t maxCurrent = 0x0201;
    uint16_t maxVelocity = 0x0202;
    uint16_t positionPGain = 0x0210;
    uint16_t positionIGain = 0x0211;
    uint16_t positionDGain = 0x0212;
    uint16_t velocityPGain = 0x0220;
    uint16_t velocityIGain = 0x0221;
    uint16_t currentPGain = 0x0230;
    uint16_t currentIGain = 0x0231;
};

/**
 * @brief Modbus RTU protocol handler
 */
class ModbusProtocol {
public:
    ModbusProtocol() : slaveId_(1) {}
    explicit ModbusProtocol(uint8_t slaveId) : slaveId_(slaveId) {}

    /**
     * @brief Set slave ID
     * @param id Slave address (1-247)
     */
    void setSlaveId(uint8_t id) { slaveId_ = id; }

    /**
     * @brief Get slave ID
     * @return Slave address
     */
    uint8_t getSlaveId() const { return slaveId_; }

    /**
     * @brief Set register map
     * @param map Register map definition
     */
    void setRegisterMap(const ModbusRegisterMap& map) { regMap_ = map; }

    /**
     * @brief Get register map
     * @return Current register map
     */
    const ModbusRegisterMap& getRegisterMap() const { return regMap_; }

    /**
     * @brief Build read holding registers request
     * @param startAddr Starting register address
     * @param count Number of registers to read
     * @param buffer Output buffer
     * @return Number of bytes written
     */
    size_t buildReadHoldingRegisters(uint16_t startAddr, uint16_t count,
                                      uint8_t* buffer) {
        buffer[0] = slaveId_;
        buffer[1] = static_cast<uint8_t>(ModbusFunction::ReadHoldingRegisters);
        buffer[2] = (startAddr >> 8) & 0xFF;
        buffer[3] = startAddr & 0xFF;
        buffer[4] = (count >> 8) & 0xFF;
        buffer[5] = count & 0xFF;

        uint16_t crc = calculateCrc(buffer, 6);
        buffer[6] = crc & 0xFF;
        buffer[7] = (crc >> 8) & 0xFF;

        return 8;
    }

    /**
     * @brief Build write single register request
     * @param addr Register address
     * @param value Register value
     * @param buffer Output buffer
     * @return Number of bytes written
     */
    size_t buildWriteSingleRegister(uint16_t addr, uint16_t value,
                                     uint8_t* buffer) {
        buffer[0] = slaveId_;
        buffer[1] = static_cast<uint8_t>(ModbusFunction::WriteSingleRegister);
        buffer[2] = (addr >> 8) & 0xFF;
        buffer[3] = addr & 0xFF;
        buffer[4] = (value >> 8) & 0xFF;
        buffer[5] = value & 0xFF;

        uint16_t crc = calculateCrc(buffer, 6);
        buffer[6] = crc & 0xFF;
        buffer[7] = (crc >> 8) & 0xFF;

        return 8;
    }

    /**
     * @brief Build write multiple registers request
     * @param startAddr Starting register address
     * @param values Register values
     * @param count Number of registers
     * @param buffer Output buffer
     * @return Number of bytes written
     */
    size_t buildWriteMultipleRegisters(uint16_t startAddr,
                                        const uint16_t* values, uint16_t count,
                                        uint8_t* buffer) {
        buffer[0] = slaveId_;
        buffer[1] = static_cast<uint8_t>(ModbusFunction::WriteMultipleRegisters);
        buffer[2] = (startAddr >> 8) & 0xFF;
        buffer[3] = startAddr & 0xFF;
        buffer[4] = (count >> 8) & 0xFF;
        buffer[5] = count & 0xFF;
        buffer[6] = count * 2;  // Byte count

        for (uint16_t i = 0; i < count; i++) {
            buffer[7 + i * 2] = (values[i] >> 8) & 0xFF;
            buffer[8 + i * 2] = values[i] & 0xFF;
        }

        size_t len = 7 + count * 2;
        uint16_t crc = calculateCrc(buffer, len);
        buffer[len] = crc & 0xFF;
        buffer[len + 1] = (crc >> 8) & 0xFF;

        return len + 2;
    }

    /**
     * @brief Parse read holding registers response
     * @param response Response buffer
     * @param len Response length
     * @param values Output values
     * @param maxCount Maximum values to read
     * @return Number of registers read, or -1 on error
     */
    int parseReadHoldingRegisters(const uint8_t* response, size_t len,
                                   uint16_t* values, size_t maxCount) {
        if (len < 5) return -1;
        if (response[0] != slaveId_) return -1;
        if (response[1] != static_cast<uint8_t>(ModbusFunction::ReadHoldingRegisters)) {
            return -1;
        }

        uint8_t byteCount = response[2];
        if (len < static_cast<size_t>(5 + byteCount)) return -1;

        // Verify CRC
        uint16_t receivedCrc = response[3 + byteCount] |
                               (response[4 + byteCount] << 8);
        uint16_t calculatedCrc = calculateCrc(response, 3 + byteCount);
        if (receivedCrc != calculatedCrc) return -1;

        // Extract values
        size_t count = byteCount / 2;
        if (count > maxCount) count = maxCount;

        for (size_t i = 0; i < count; i++) {
            values[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
        }

        return static_cast<int>(count);
    }

    /**
     * @brief Check if response indicates an error
     * @param response Response buffer
     * @return Error code, or 0 if no error
     */
    static uint8_t checkError(const uint8_t* response) {
        if (response[1] & 0x80) {
            return response[2];  // Exception code
        }
        return 0;
    }

    /**
     * @brief Calculate Modbus CRC-16
     * @param data Data buffer
     * @param len Data length
     * @return CRC-16 value
     */
    static uint16_t calculateCrc(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    // === Convenience methods for motor control ===

    /**
     * @brief Build position command
     * @param position Position in encoder counts
     * @param buffer Output buffer
     * @return Number of bytes
     */
    size_t buildPositionCommand(int32_t position, uint8_t* buffer) {
        uint16_t values[2] = {
            static_cast<uint16_t>((position >> 16) & 0xFFFF),
            static_cast<uint16_t>(position & 0xFFFF)
        };
        return buildWriteMultipleRegisters(regMap_.targetPosition, values, 2, buffer);
    }

    /**
     * @brief Build velocity command
     * @param velocity Velocity in internal units
     * @param buffer Output buffer
     * @return Number of bytes
     */
    size_t buildVelocityCommand(int32_t velocity, uint8_t* buffer) {
        uint16_t values[2] = {
            static_cast<uint16_t>((velocity >> 16) & 0xFFFF),
            static_cast<uint16_t>(velocity & 0xFFFF)
        };
        return buildWriteMultipleRegisters(regMap_.targetVelocity, values, 2, buffer);
    }

    /**
     * @brief Build torque/current command
     * @param torque Torque in internal units
     * @param buffer Output buffer
     * @return Number of bytes
     */
    size_t buildTorqueCommand(int16_t torque, uint8_t* buffer) {
        return buildWriteSingleRegister(regMap_.targetTorque,
                                        static_cast<uint16_t>(torque), buffer);
    }

    /**
     * @brief Build read status request
     * @param buffer Output buffer
     * @return Number of bytes
     */
    size_t buildReadStatus(uint8_t* buffer) {
        // Read status word through temperature (10 registers)
        return buildReadHoldingRegisters(regMap_.statusWord, 10, buffer);
    }

private:
    uint8_t slaveId_;
    ModbusRegisterMap regMap_;
};

}  // namespace omni::protocol
