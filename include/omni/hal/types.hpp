/**
 * @file types.hpp
 * @brief Common types and enumerations for HAL layer
 */

#pragma once

#include <cstdint>

namespace omni::hal {

/**
 * @brief GPIO pin modes
 */
enum class PinMode {
    Input,
    Output,
    InputPullUp,
    InputPullDown,
    OutputOpenDrain,
    Alternate
};

/**
 * @brief ADC trigger sources
 */
enum class AdcTrigger {
    Software,
    Timer1,
    Timer2,
    Timer3,
    Timer4,
    ExternalRising,
    ExternalFalling
};

/**
 * @brief SPI operating modes
 */
enum class SpiMode {
    Mode0,  // CPOL=0, CPHA=0
    Mode1,  // CPOL=0, CPHA=1
    Mode2,  // CPOL=1, CPHA=0
    Mode3   // CPOL=1, CPHA=1
};

/**
 * @brief CAN frame structure
 */
struct CanFrame {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    bool extendedId;
    bool remoteRequest;
};

/**
 * @brief UART parity settings
 */
enum class UartParity {
    None,
    Even,
    Odd
};

/**
 * @brief UART stop bits
 */
enum class UartStopBits {
    One,
    OnePointFive,
    Two
};

/**
 * @brief Encoder counting mode
 */
enum class EncoderMode {
    X1,   // Count on single edge of channel A
    X2,   // Count on both edges of channel A
    X4    // Count on both edges of both channels
};

}  // namespace omni::hal
