/**
 * @file arduino_comm.hpp
 * @brief Arduino Communication interfaces (SPI, UART) HAL implementation
 *
 * Supports Arduino AVR, ESP32, RP2040 and other compatible platforms.
 *
 * Note: Standard Arduino does not support CAN. For CAN communication,
 * use external modules like MCP2515 with appropriate libraries.
 */

#pragma once

#include <omni/hal/comm.hpp>
#include <Arduino.h>
#include <SPI.h>

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Arduino SPI interface implementation
 *
 * Wraps the Arduino SPI library for HAL-compatible communication.
 *
 * Usage:
 * @code
 * ArduinoSpi spi(10);  // CS pin 10
 * spi.begin();
 * spi.setClockSpeed(1000000);  // 1 MHz
 * spi.setMode(SpiMode::Mode0);
 *
 * spi.select();
 * uint8_t response = spi.transfer(0x55);
 * spi.deselect();
 * @endcode
 */
class ArduinoSpi : public hal::ISpi {
public:
    /**
     * @brief Constructor
     * @param csPin Chip select pin
     * @param spiClass SPI instance to use (default: SPI)
     */
    explicit ArduinoSpi(int8_t csPin, SPIClass& spiClass = SPI)
        : spi_(spiClass)
        , csPin_(csPin)
        , clockSpeed_(1000000)
        , mode_(hal::SpiMode::Mode0)
        , msbFirst_(true)
        , baudRate_(0)
    {
    }

    /**
     * @brief Initialize SPI
     */
    void begin() {
        if (csPin_ >= 0) {
            pinMode(csPin_, OUTPUT);
            digitalWrite(csPin_, HIGH);  // Deselect
        }
        spi_.begin();
        updateSettings();
    }

    /**
     * @brief Deinitialize SPI
     */
    void end() {
        spi_.end();
    }

    // IComm interface
    bool send(const uint8_t* data, size_t len) override {
        select();
        for (size_t i = 0; i < len; i++) {
            spi_.transfer(data[i]);
        }
        deselect();
        return true;
    }

    size_t receive(uint8_t* buffer, size_t maxLen) override {
        select();
        for (size_t i = 0; i < maxLen; i++) {
            buffer[i] = spi_.transfer(0x00);
        }
        deselect();
        return maxLen;
    }

    bool isConnected() override {
        return true;  // SPI is always "connected" if initialized
    }

    void setBaudRate(uint32_t baud) override {
        baudRate_ = baud;
        setClockSpeed(baud);
    }

    uint32_t getBaudRate() const override {
        return clockSpeed_;
    }

    size_t available() override {
        return 0;  // SPI doesn't buffer received data
    }

    void flush() override {
        // SPI is synchronous, no flush needed
    }

    // ISpi interface
    uint8_t transfer(uint8_t data) override {
        return spi_.transfer(data);
    }

    void transfer(const uint8_t* txBuf, uint8_t* rxBuf, size_t len) override {
        for (size_t i = 0; i < len; i++) {
            uint8_t tx = txBuf ? txBuf[i] : 0x00;
            uint8_t rx = spi_.transfer(tx);
            if (rxBuf) {
                rxBuf[i] = rx;
            }
        }
    }

    void setClockSpeed(uint32_t hz) override {
        clockSpeed_ = hz;
        updateSettings();
    }

    uint32_t getClockSpeed() const override {
        return clockSpeed_;
    }

    void setMode(hal::SpiMode mode) override {
        mode_ = mode;
        updateSettings();
    }

    void setBitOrder(bool msbFirst) override {
        msbFirst_ = msbFirst;
        updateSettings();
    }

    void select() override {
        spi_.beginTransaction(settings_);
        if (csPin_ >= 0) {
            digitalWrite(csPin_, LOW);
        }
    }

    void deselect() override {
        if (csPin_ >= 0) {
            digitalWrite(csPin_, HIGH);
        }
        spi_.endTransaction();
    }

    /**
     * @brief Get the chip select pin
     */
    int8_t getCsPin() const { return csPin_; }

    /**
     * @brief Set chip select pin
     */
    void setCsPin(int8_t pin) {
        csPin_ = pin;
        if (csPin_ >= 0) {
            pinMode(csPin_, OUTPUT);
            digitalWrite(csPin_, HIGH);
        }
    }

private:
    SPIClass& spi_;
    int8_t csPin_;
    uint32_t clockSpeed_;
    hal::SpiMode mode_;
    bool msbFirst_;
    uint32_t baudRate_;
    SPISettings settings_;

    void updateSettings() {
        uint8_t spiMode;
        switch (mode_) {
            case hal::SpiMode::Mode0: spiMode = SPI_MODE0; break;
            case hal::SpiMode::Mode1: spiMode = SPI_MODE1; break;
            case hal::SpiMode::Mode2: spiMode = SPI_MODE2; break;
            case hal::SpiMode::Mode3: spiMode = SPI_MODE3; break;
            default: spiMode = SPI_MODE0;
        }

        settings_ = SPISettings(clockSpeed_,
                               msbFirst_ ? MSBFIRST : LSBFIRST,
                               spiMode);
    }
};

/**
 * @brief Arduino UART interface implementation
 *
 * Wraps Arduino's HardwareSerial for HAL-compatible communication.
 *
 * Usage:
 * @code
 * ArduinoUart uart(Serial1);  // Use Serial1
 * uart.begin(115200);
 *
 * uart.send(data, len);
 * if (uart.available()) {
 *     size_t received = uart.receive(buffer, maxLen);
 * }
 * @endcode
 */
class ArduinoUart : public hal::IUart {
public:
    /**
     * @brief Constructor
     * @param serial HardwareSerial instance to use
     */
    explicit ArduinoUart(HardwareSerial& serial)
        : serial_(serial)
        , baudRate_(115200)
        , dataBits_(8)
        , parity_(hal::UartParity::None)
        , stopBits_(hal::UartStopBits::One)
        , flowControl_(false)
        , rxTimeout_(1000)
        , dePin_(nullptr)
    {
    }

    /**
     * @brief Initialize UART with specified baud rate
     * @param baud Baud rate
     */
    void begin(uint32_t baud = 115200) {
        baudRate_ = baud;
        applyConfiguration();
    }

    /**
     * @brief Deinitialize UART
     */
    void end() {
        serial_.end();
    }

    // IComm interface
    bool send(const uint8_t* data, size_t len) override {
        if (dePin_) {
            dePin_->write(true);  // Enable driver
        }

        size_t written = serial_.write(data, len);

        if (dePin_) {
            serial_.flush();  // Wait for transmission complete
            dePin_->write(false);  // Disable driver
        }

        return written == len;
    }

    size_t receive(uint8_t* buffer, size_t maxLen) override {
        size_t received = 0;
        uint32_t startTime = millis();

        while (received < maxLen) {
            if (serial_.available()) {
                buffer[received++] = serial_.read();
                startTime = millis();  // Reset timeout on data
            } else if (millis() - startTime > rxTimeout_) {
                break;  // Timeout
            }
        }

        return received;
    }

    bool isConnected() override {
        return true;  // Serial is always available
    }

    void setBaudRate(uint32_t baud) override {
        baudRate_ = baud;
        applyConfiguration();
    }

    uint32_t getBaudRate() const override {
        return baudRate_;
    }

    size_t available() override {
        return serial_.available();
    }

    void flush() override {
        serial_.flush();
    }

    // IUart interface
    void configure(uint8_t dataBits, hal::UartParity parity, hal::UartStopBits stopBits) override {
        dataBits_ = dataBits;
        parity_ = parity;
        stopBits_ = stopBits;
        applyConfiguration();
    }

    void enableFlowControl(bool enable) override {
        flowControl_ = enable;
        // Hardware flow control requires platform-specific implementation
#if defined(ARDUINO_ARCH_ESP32)
        // ESP32 supports hardware flow control
        // Would need to reconfigure UART with RTS/CTS pins
#endif
    }

    void enableRs485(bool enable, hal::IGpio* dePin = nullptr) override {
        if (enable) {
            dePin_ = dePin;
            if (dePin_) {
                dePin_->setMode(hal::PinMode::Output);
                dePin_->write(false);  // Receiver mode by default
            }
        } else {
            dePin_ = nullptr;
        }
    }

    void setRxTimeout(uint32_t ms) override {
        rxTimeout_ = ms;
#if defined(ARDUINO_ARCH_ESP32)
        serial_.setTimeout(ms);
#endif
    }

    void enableTxDma(bool enable) override {
        // Arduino doesn't expose DMA for UART
        (void)enable;
    }

    void enableRxDma(bool enable) override {
        // Arduino doesn't expose DMA for UART
        (void)enable;
    }

    /**
     * @brief Read a single byte (blocking with timeout)
     * @return Byte read, or -1 if timeout
     */
    int read() {
        return serial_.read();
    }

    /**
     * @brief Write a single byte
     */
    size_t write(uint8_t byte) {
        return serial_.write(byte);
    }

    /**
     * @brief Print string (for debugging)
     */
    template<typename T>
    void print(T value) {
        serial_.print(value);
    }

    /**
     * @brief Print string with newline
     */
    template<typename T>
    void println(T value) {
        serial_.println(value);
    }

    /**
     * @brief Get the underlying HardwareSerial reference
     */
    HardwareSerial& getSerial() { return serial_; }

private:
    HardwareSerial& serial_;
    uint32_t baudRate_;
    uint8_t dataBits_;
    hal::UartParity parity_;
    hal::UartStopBits stopBits_;
    bool flowControl_;
    uint32_t rxTimeout_;
    hal::IGpio* dePin_;

    void applyConfiguration() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
        // ESP32 and RP2040 support configuration
        uint32_t config = SERIAL_8N1;  // Default

        // Build config based on parameters
        switch (dataBits_) {
            case 5: config = SERIAL_5N1; break;
            case 6: config = SERIAL_6N1; break;
            case 7: config = SERIAL_7N1; break;
            case 8: config = SERIAL_8N1; break;
            default: config = SERIAL_8N1;
        }

        // Adjust for parity
        if (parity_ == hal::UartParity::Even) {
            switch (dataBits_) {
                case 5: config = SERIAL_5E1; break;
                case 6: config = SERIAL_6E1; break;
                case 7: config = SERIAL_7E1; break;
                case 8: config = SERIAL_8E1; break;
            }
        } else if (parity_ == hal::UartParity::Odd) {
            switch (dataBits_) {
                case 5: config = SERIAL_5O1; break;
                case 6: config = SERIAL_6O1; break;
                case 7: config = SERIAL_7O1; break;
                case 8: config = SERIAL_8O1; break;
            }
        }

        // Adjust for stop bits (two stop bits)
        if (stopBits_ == hal::UartStopBits::Two) {
            switch (dataBits_) {
                case 5: config = SERIAL_5N2; break;
                case 6: config = SERIAL_6N2; break;
                case 7: config = SERIAL_7N2; break;
                case 8: config = SERIAL_8N2; break;
            }
            // Would need to combine with parity for full support
        }

        serial_.begin(baudRate_, config);
#else
        // AVR: Limited configuration options
        serial_.begin(baudRate_);
#endif
    }
};

#if defined(ARDUINO_ARCH_ESP32)

/**
 * @brief ESP32 UART with extended features
 *
 * Provides access to ESP32-specific UART features like
 * multiple hardware serial ports and RS485 mode.
 */
class ArduinoEsp32Uart : public ArduinoUart {
public:
    /**
     * @brief Constructor with custom pins
     * @param serial HardwareSerial instance
     * @param rxPin RX pin number
     * @param txPin TX pin number
     */
    ArduinoEsp32Uart(HardwareSerial& serial, int8_t rxPin = -1, int8_t txPin = -1)
        : ArduinoUart(serial)
        , rxPin_(rxPin)
        , txPin_(txPin)
    {
    }

    /**
     * @brief Initialize with custom pins
     */
    void begin(uint32_t baud = 115200) {
        if (rxPin_ >= 0 && txPin_ >= 0) {
            getSerial().begin(baud, SERIAL_8N1, rxPin_, txPin_);
        } else {
            ArduinoUart::begin(baud);
        }
    }

private:
    int8_t rxPin_, txPin_;
};

#endif  // ESP32

/**
 * @brief Software Serial wrapper for additional UART ports
 *
 * For platforms with limited hardware UART, software serial can be used.
 * Requires including SoftwareSerial.h separately.
 */
#if defined(ARDUINO_ARCH_AVR)

// Forward declaration - user must include SoftwareSerial.h
class SoftwareSerial;

/**
 * @brief Software Serial UART implementation
 *
 * Note: Performance is limited compared to hardware UART.
 * Only use when hardware UART is not available.
 */
class ArduinoSoftwareUart : public hal::IUart {
public:
    ArduinoSoftwareUart(uint8_t rxPin, uint8_t txPin)
        : rxPin_(rxPin)
        , txPin_(txPin)
        , baudRate_(9600)
        , softSerial_(nullptr)
    {
    }

    void begin(uint32_t baud = 9600) {
        baudRate_ = baud;
        // User must create SoftwareSerial instance
        // softSerial_ = new SoftwareSerial(rxPin_, txPin_);
        // softSerial_->begin(baud);
    }

    bool send(const uint8_t* data, size_t len) override {
        if (!softSerial_) return false;
        // softSerial_->write(data, len);
        return true;
    }

    size_t receive(uint8_t* buffer, size_t maxLen) override {
        if (!softSerial_) return 0;
        size_t received = 0;
        // while (softSerial_->available() && received < maxLen) {
        //     buffer[received++] = softSerial_->read();
        // }
        return received;
    }

    bool isConnected() override { return softSerial_ != nullptr; }
    void setBaudRate(uint32_t baud) override { baudRate_ = baud; }
    uint32_t getBaudRate() const override { return baudRate_; }
    size_t available() override { return 0; }
    void flush() override {}

    void configure(uint8_t dataBits, hal::UartParity parity, hal::UartStopBits stopBits) override {
        (void)dataBits; (void)parity; (void)stopBits;
    }
    void enableFlowControl(bool enable) override { (void)enable; }
    void enableRs485(bool enable, hal::IGpio* dePin = nullptr) override { (void)enable; (void)dePin; }
    void setRxTimeout(uint32_t ms) override { (void)ms; }
    void enableTxDma(bool enable) override { (void)enable; }
    void enableRxDma(bool enable) override { (void)enable; }

private:
    uint8_t rxPin_, txPin_;
    uint32_t baudRate_;
    SoftwareSerial* softSerial_;
};

#endif  // AVR

}  // namespace arduino
}  // namespace platform
}  // namespace omni
