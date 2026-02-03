/**
 * @file arduino_hal.hpp
 * @brief Arduino HAL main header file
 *
 * Includes all Arduino platform HAL implementations.
 */

#pragma once

// Include all Arduino HAL implementations
#include "arduino_gpio.hpp"
#include "arduino_pwm.hpp"
#include "arduino_adc.hpp"
#include "arduino_timer.hpp"
#include "arduino_encoder.hpp"
#include "arduino_comm.hpp"

/**
 * @namespace omni::platform::arduino
 * @brief Arduino platform HAL implementation
 *
 * This namespace contains all hardware abstraction layer implementations
 * for Arduino-compatible platforms including AVR, ESP32, and RP2040.
 *
 * ## Supported Platforms
 *
 * - **Arduino AVR** (Uno, Mega, Nano, etc.)
 * - **ESP32** (ESP32, ESP32-S2, ESP32-S3, ESP32-C3)
 * - **RP2040** (Raspberry Pi Pico, Pico W)
 * - **Arduino SAM** (Due)
 * - Other Arduino-compatible boards
 *
 * ## Usage
 *
 * ### 1. Include the main header
 *
 * ```cpp
 * #include "arduino_hal.hpp"
 *
 * using namespace omni::platform::arduino;
 * ```
 *
 * ### 2. Create HAL objects
 *
 * ```cpp
 * // GPIO
 * ArduinoGpio led(13);
 * led.setMode(hal::PinMode::Output);
 * led.write(true);
 *
 * // PWM
 * ArduinoPwm pwm(9);
 * pwm.setFrequency(20000);
 * pwm.setDuty(0.5f);
 * pwm.enable(true);
 *
 * // ADC
 * ArduinoAdc adc(A0);
 * float voltage = adc.readVoltage();
 *
 * // Encoder
 * ArduinoEncoder encoder(2, 3);  // Pins with interrupt support
 * encoder.setResolution(4096);
 * encoder.begin();
 *
 * // Timer
 * ArduinoTimer timer;
 * timer.setPeriod(1000);  // 1ms
 * timer.setCallback([]() { // handle timer });
 * timer.start();
 *
 * // SPI
 * ArduinoSpi spi(10);
 * spi.begin();
 * spi.setClockSpeed(1000000);
 *
 * // UART
 * ArduinoUart uart(Serial1);
 * uart.begin(115200);
 * ```
 *
 * ### 3. Create motor driver
 *
 * ```cpp
 * #include <omni/driver/bldc_driver.hpp>
 *
 * // Create three-phase PWM
 * ArduinoPwm3Phase pwm(9, 10, 11);
 * pwm.setFrequency(20000);
 *
 * // Create ADC for current sensing
 * uint8_t adcPins[] = {A0, A1, A2};
 * ArduinoAdcMultiChannel currentAdc(adcPins, 3);
 *
 * // Create encoder
 * ArduinoEncoder encoder(2, 3);
 * encoder.setResolution(4096);
 *
 * // Create BLDC driver
 * omni::driver::BldcDriver bldc(&pwm, &currentAdc, &encoder);
 * bldc.init();
 * ```
 *
 * ## Platform-Specific Features
 *
 * ### ESP32
 *
 * - Hardware three-phase PWM with MCPWM module
 * - Hardware encoder with PCNT module
 * - Hardware timers with esp_timer
 * - 12-bit ADC with configurable attenuation
 *
 * ```cpp
 * // ESP32 MCPWM three-phase PWM
 * ArduinoEsp32Pwm3Phase pwm(
 *     25, 26,  // Phase A high/low
 *     27, 14,  // Phase B high/low
 *     12, 13   // Phase C high/low
 * );
 * pwm.setDeadTime(500);  // 500ns hardware dead time
 *
 * // ESP32 PCNT encoder
 * ArduinoEsp32Encoder encoder(34, 35, PCNT_UNIT_0);
 * ```
 *
 * ### RP2040
 *
 * - 12-bit ADC
 * - Hardware timers with repeating_timer
 * - PIO-based encoder (simplified implementation)
 *
 * ### AVR
 *
 * - 10-bit ADC, 5V reference
 * - Limited PWM frequency options
 * - Software-based encoder with interrupts
 * - Software timer (requires polling)
 *
 * ## Limitations
 *
 * 1. **No DMA**: Arduino doesn't expose DMA, ADC DMA methods perform polling
 * 2. **No CAN**: Standard Arduino has no CAN, use external MCP2515 modules
 * 3. **Limited Timers**: AVR has limited timer resources
 * 4. **PWM Resolution**: AVR limited to 8-bit PWM
 * 5. **Dead Time**: Only ESP32 MCPWM supports hardware dead time
 *
 * ## Compiler Flags
 *
 * Platform is auto-detected via Arduino macros:
 * - `ARDUINO_ARCH_AVR` - AVR-based boards
 * - `ARDUINO_ARCH_ESP32` - ESP32 boards
 * - `ARDUINO_ARCH_RP2040` - RP2040 boards
 * - `ARDUINO_ARCH_SAM` - SAM-based boards (Due)
 */

namespace omni {
namespace platform {
namespace arduino {

/**
 * @brief Get platform name string
 */
inline const char* getPlatformName() {
#if defined(ARDUINO_ARCH_AVR)
    return "Arduino AVR";
#elif defined(ARDUINO_ARCH_ESP32)
    return "ESP32";
#elif defined(ARDUINO_ARCH_RP2040)
    return "RP2040";
#elif defined(ARDUINO_ARCH_SAM)
    return "Arduino SAM";
#else
    return "Arduino (Unknown)";
#endif
}

/**
 * @brief Check if platform supports hardware PWM dead time
 */
inline bool supportsHardwareDeadTime() {
#if defined(ARDUINO_ARCH_ESP32) && defined(OMNI_ESP32_MCPWM_AVAILABLE)
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if platform supports hardware quadrature encoder
 */
inline bool supportsHardwareEncoder() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
    return true;
#else
    return false;
#endif
}

/**
 * @brief Get ADC resolution for current platform
 */
inline uint8_t getAdcResolution() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
    return 12;
#elif defined(ARDUINO_ARCH_AVR)
    return 10;
#else
    return 10;
#endif
}

/**
 * @brief Get default reference voltage for current platform
 */
inline float getDefaultVref() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)
    return 3.3f;
#elif defined(ARDUINO_ARCH_AVR)
    return 5.0f;
#else
    return 3.3f;
#endif
}

/**
 * @brief Delay in microseconds
 */
inline void delayUs(uint32_t us) {
    delayMicroseconds(us);
}

/**
 * @brief Delay in milliseconds
 */
inline void delayMs(uint32_t ms) {
    delay(ms);
}

/**
 * @brief Get system time in milliseconds
 */
inline uint32_t getTickMs() {
    return millis();
}

/**
 * @brief Get system time in microseconds
 */
inline uint32_t getTickUs() {
    return micros();
}

/**
 * @brief Disable interrupts
 */
inline void disableInterrupts() {
    noInterrupts();
}

/**
 * @brief Enable interrupts
 */
inline void enableInterrupts() {
    interrupts();
}

/**
 * @brief Critical section guard
 *
 * Usage:
 * @code
 * {
 *     CriticalSection cs;
 *     // Interrupts disabled in this scope
 *     // ... critical code ...
 * }
 * // Interrupts restored
 * @endcode
 */
class CriticalSection {
public:
    CriticalSection() {
        noInterrupts();
    }

    ~CriticalSection() {
        interrupts();
    }

    // Non-copyable
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
};

}  // namespace arduino
}  // namespace platform
}  // namespace omni

/*
 * ============================================================================
 * Quick Reference
 * ============================================================================
 *
 * GPIO:
 *   ArduinoGpio gpio(13);
 *   gpio.setMode(PinMode::Output);
 *   gpio.write(true);
 *   gpio.toggle();
 *
 * GPIO with Interrupt:
 *   ArduinoGpioInterrupt btn(2);
 *   btn.setMode(PinMode::InputPullUp);
 *   btn.attachCallback(callback, FALLING);
 *
 * PWM:
 *   ArduinoPwm pwm(9);
 *   pwm.setFrequency(20000);
 *   pwm.setDuty(0.5f);
 *   pwm.enable(true);
 *
 * Three-phase PWM:
 *   ArduinoPwm3Phase pwm(9, 10, 11);
 *   pwm.setFrequency(20000);
 *   pwm.enable(true);
 *   pwm.setDuty(0.5f, 0.3f, 0.2f);
 *   pwm.setSvpwm(alpha, beta);
 *
 * ADC:
 *   ArduinoAdc adc(A0);
 *   uint16_t raw = adc.read();
 *   float voltage = adc.readVoltage();
 *
 * Multi-channel ADC:
 *   uint8_t pins[] = {A0, A1, A2};
 *   ArduinoAdcMultiChannel adc(pins, 3);
 *   uint16_t values[3];
 *   adc.readAllChannels(values, 3);
 *
 * Encoder:
 *   ArduinoEncoder encoder(2, 3);
 *   encoder.setResolution(4096);
 *   encoder.begin();
 *   encoder.update(dt);
 *   float angle = encoder.getAngle();
 *   float velocity = encoder.getVelocity();
 *
 * Timer:
 *   ArduinoTimer timer;
 *   timer.setPeriod(1000);
 *   timer.setCallback(callback);
 *   timer.start();
 *
 * Timestamp:
 *   ArduinoTimestamp ts;
 *   uint64_t start = ts.getMicros();
 *   // ... do work ...
 *   uint64_t elapsed = ts.elapsedMicros(start);
 *
 * SPI:
 *   ArduinoSpi spi(10);
 *   spi.begin();
 *   spi.setClockSpeed(1000000);
 *   spi.select();
 *   uint8_t rx = spi.transfer(0x55);
 *   spi.deselect();
 *
 * UART:
 *   ArduinoUart uart(Serial1);
 *   uart.begin(115200);
 *   uart.send(data, len);
 *   uart.receive(buffer, maxLen);
 *
 * Hall Sensors:
 *   ArduinoHallSensor hall(4, 5, 6);
 *   hall.begin();
 *   uint8_t state = hall.getState();
 *   float angle = hall.getElectricalAngle();
 */
