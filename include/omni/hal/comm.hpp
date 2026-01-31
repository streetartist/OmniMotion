/**
 * @file comm.hpp
 * @brief Communication interfaces (CAN, SPI, UART)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "types.hpp"

namespace omni::hal {

/**
 * @brief Base communication interface
 *
 * Abstract base for all communication interfaces providing common
 * send/receive functionality.
 */
class IComm {
public:
    virtual ~IComm() = default;

    /**
     * @brief Send data
     * @param data Pointer to data buffer
     * @param len Number of bytes to send
     * @return true if send successful
     */
    virtual bool send(const uint8_t* data, size_t len) = 0;

    /**
     * @brief Receive data
     * @param buffer Buffer to store received data
     * @param maxLen Maximum bytes to receive
     * @return Number of bytes received
     */
    virtual size_t receive(uint8_t* buffer, size_t maxLen) = 0;

    /**
     * @brief Check if connection is active
     * @return true if connected
     */
    virtual bool isConnected() = 0;

    /**
     * @brief Set communication baud rate
     * @param baud Baud rate in bps
     */
    virtual void setBaudRate(uint32_t baud) = 0;

    /**
     * @brief Get current baud rate
     * @return Baud rate in bps
     */
    virtual uint32_t getBaudRate() const = 0;

    /**
     * @brief Get number of bytes available to read
     * @return Number of available bytes
     */
    virtual size_t available() = 0;

    /**
     * @brief Flush transmit buffer
     */
    virtual void flush() = 0;
};

/**
 * @brief CAN bus interface
 *
 * Interface for CAN communication with support for standard and
 * extended frames, filtering, and hardware timestamps.
 */
class ICan : public IComm {
public:
    /**
     * @brief Send a CAN frame
     * @param id CAN identifier
     * @param data Data bytes (max 8)
     * @param len Data length (0-8)
     * @return true if sent successfully
     */
    virtual bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len) = 0;

    /**
     * @brief Send a CAN frame structure
     * @param frame Frame to send
     * @return true if sent successfully
     */
    virtual bool sendFrame(const CanFrame& frame) = 0;

    /**
     * @brief Receive a CAN frame
     * @param frame Output frame structure
     * @return true if frame received
     */
    virtual bool receiveFrame(CanFrame& frame) = 0;

    /**
     * @brief Check if a frame is available
     * @return true if frame waiting
     */
    virtual bool frameAvailable() = 0;

    /**
     * @brief Set acceptance filter
     * @param id Filter ID
     * @param mask Filter mask
     */
    virtual void setFilter(uint32_t id, uint32_t mask) = 0;

    /**
     * @brief Add a filter bank
     * @param index Filter bank index
     * @param id Filter ID
     * @param mask Filter mask
     * @param extended true for extended ID filter
     */
    virtual void addFilter(uint8_t index, uint32_t id, uint32_t mask, bool extended = false) = 0;

    /**
     * @brief Clear all filters
     */
    virtual void clearFilters() = 0;

    /**
     * @brief Set CAN bit rate
     * @param bitrate Bit rate (125000, 250000, 500000, 1000000)
     */
    virtual void setBitrate(uint32_t bitrate) = 0;

    /**
     * @brief Get current bit rate
     * @return Bit rate in bps
     */
    virtual uint32_t getBitrate() const = 0;

    /**
     * @brief Get transmit error count
     * @return Error count
     */
    virtual uint8_t getTxErrorCount() const = 0;

    /**
     * @brief Get receive error count
     * @return Error count
     */
    virtual uint8_t getRxErrorCount() const = 0;

    /**
     * @brief Check if bus is in error state
     * @return true if in error state
     */
    virtual bool isBusOff() const = 0;

    /**
     * @brief Reset the CAN controller
     */
    virtual void reset() = 0;
};

/**
 * @brief SPI interface
 */
class ISpi : public IComm {
public:
    /**
     * @brief Transfer single byte
     * @param data Byte to send
     * @return Byte received
     */
    virtual uint8_t transfer(uint8_t data) = 0;

    /**
     * @brief Transfer multiple bytes
     * @param txBuf Transmit buffer (can be nullptr)
     * @param rxBuf Receive buffer (can be nullptr)
     * @param len Number of bytes
     */
    virtual void transfer(const uint8_t* txBuf, uint8_t* rxBuf, size_t len) = 0;

    /**
     * @brief Set clock speed
     * @param hz Clock frequency in Hz
     */
    virtual void setClockSpeed(uint32_t hz) = 0;

    /**
     * @brief Get current clock speed
     * @return Clock frequency in Hz
     */
    virtual uint32_t getClockSpeed() const = 0;

    /**
     * @brief Set SPI mode
     * @param mode SPI mode (0-3)
     */
    virtual void setMode(SpiMode mode) = 0;

    /**
     * @brief Set bit order
     * @param msbFirst true for MSB first
     */
    virtual void setBitOrder(bool msbFirst) = 0;

    /**
     * @brief Assert chip select
     */
    virtual void select() = 0;

    /**
     * @brief Deassert chip select
     */
    virtual void deselect() = 0;
};

/**
 * @brief UART interface
 */
class IUart : public IComm {
public:
    /**
     * @brief Configure UART parameters
     * @param dataBits Number of data bits (7, 8, 9)
     * @param parity Parity setting
     * @param stopBits Stop bits setting
     */
    virtual void configure(uint8_t dataBits, UartParity parity, UartStopBits stopBits) = 0;

    /**
     * @brief Enable hardware flow control
     * @param enable true to enable RTS/CTS
     */
    virtual void enableFlowControl(bool enable) = 0;

    /**
     * @brief Enable RS485 mode
     * @param enable true to enable
     * @param dePin GPIO for driver enable (optional)
     */
    virtual void enableRs485(bool enable, IGpio* dePin = nullptr) = 0;

    /**
     * @brief Set receive timeout
     * @param ms Timeout in milliseconds
     */
    virtual void setRxTimeout(uint32_t ms) = 0;

    /**
     * @brief Enable DMA for transmission
     * @param enable true to enable
     */
    virtual void enableTxDma(bool enable) = 0;

    /**
     * @brief Enable DMA for reception
     * @param enable true to enable
     */
    virtual void enableRxDma(bool enable) = 0;
};

}  // namespace omni::hal
