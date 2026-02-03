# Arduino HAL Implementation for OmniMotion

This directory contains the Hardware Abstraction Layer (HAL) implementation for Arduino-compatible platforms.

## Supported Platforms

| Platform | GPIO | PWM | ADC | Timer | Encoder | SPI | UART |
|----------|------|-----|-----|-------|---------|-----|------|
| Arduino AVR (Uno, Mega, Nano) | Full | Basic | 10-bit | Software | Software | Full | Full |
| ESP32 | Full | LEDC + MCPWM | 12-bit | Hardware | PCNT | Full | Full |
| RP2040 (Pico) | Full | Hardware | 12-bit | Hardware | PIO | Full | Full |
| Arduino SAM (Due) | Full | Hardware | 12-bit | Software | Software | Full | Full |

## Installation

### Arduino IDE

1. Copy the `platform/arduino/` folder to your Arduino libraries folder
2. Or include the files directly in your sketch folder

### PlatformIO

Add to your `platformio.ini`:

```ini
[env:your_board]
platform = espressif32  ; or other platform
board = esp32dev
framework = arduino
lib_deps =
    ; Add OmniMotion library
build_flags =
    -I include
```

## Quick Start

```cpp
#include "arduino_hal.hpp"

using namespace omni::platform::arduino;

// Create hardware objects
ArduinoGpio led(13);
ArduinoPwm pwm(9);
ArduinoAdc adc(A0);
ArduinoEncoder encoder(2, 3);
ArduinoTimer timer;

void setup() {
    Serial.begin(115200);

    // Configure GPIO
    led.setMode(hal::PinMode::Output);

    // Configure PWM
    pwm.setFrequency(20000);  // 20kHz
    pwm.setDuty(0.5f);
    pwm.enable(true);

    // Configure encoder
    encoder.setResolution(4096);
    encoder.begin();

    // Configure timer (1ms period)
    timer.setPeriod(1000);
    timer.setCallback([]() {
        // Timer interrupt handler
    });
    timer.start();
}

void loop() {
    // Read ADC
    float voltage = adc.readVoltage();

    // Update encoder velocity
    static uint32_t lastUpdate = 0;
    uint32_t now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    encoder.update(dt);

    // Toggle LED
    led.toggle();
    delay(500);
}
```

## API Reference

### GPIO

```cpp
// Basic GPIO
ArduinoGpio gpio(pin);
gpio.setMode(hal::PinMode::Output);  // Input, Output, InputPullUp, InputPullDown, OutputOpenDrain
gpio.write(true);
bool state = gpio.read();
gpio.toggle();

// GPIO with interrupt
ArduinoGpioInterrupt button(2);
button.setMode(hal::PinMode::InputPullUp);
button.attachCallback(myCallback, FALLING);  // RISING, FALLING, CHANGE
button.detachCallback();
```

### PWM

```cpp
// Single channel PWM
ArduinoPwm pwm(pin);
pwm.setFrequency(20000);  // Hz
pwm.setDuty(0.5f);        // 0.0 - 1.0
pwm.enable(true);

// Three-phase PWM (software synchronized)
ArduinoPwm3Phase pwm3(pinA, pinB, pinC);
pwm3.setFrequency(20000);
pwm3.setDuty(0.5f, 0.4f, 0.3f);
pwm3.setSvpwm(alpha, beta);  // Space vector modulation
pwm3.setPhaseEnable(true, true, true);

// ESP32 MCPWM (hardware three-phase with dead time)
#if defined(ARDUINO_ARCH_ESP32)
ArduinoEsp32Pwm3Phase pwm(
    25, 26,  // Phase A high/low
    27, 14,  // Phase B high/low
    12, 13   // Phase C high/low
);
pwm.setDeadTime(500);  // 500ns hardware dead time
#endif
```

### ADC

```cpp
// Single channel
ArduinoAdc adc(A0);
uint16_t raw = adc.read();
float voltage = adc.readVoltage();

// Multi-channel
uint8_t pins[] = {A0, A1, A2};
ArduinoAdcMultiChannel adcMulti(pins, 3);
uint16_t values[3];
adcMulti.readAllChannels(values, 3);

// ESP32 attenuation
#if defined(ARDUINO_ARCH_ESP32)
adc.setAttenuation(ADC_ATTEN_DB_11);  // Full 0-3.3V range
#endif
```

### Timer

```cpp
ArduinoTimer timer;
timer.setPeriod(1000);  // Microseconds
timer.setCallback([]() {
    // Called in interrupt context
});
timer.enableInterrupt(true);
timer.start();
timer.stop();
timer.setOneShot(true);  // One-time trigger

// Timestamp
ArduinoTimestamp ts;
uint64_t start = ts.getMicros();
// ... work ...
uint64_t elapsed = ts.elapsedMicros(start);

// Stopwatch
ArduinoStopwatch sw;
sw.start();
// ... work ...
sw.stop();
float ms = sw.getElapsedMs();
```

### Encoder

```cpp
// Software encoder (all platforms)
ArduinoSoftwareEncoder encoder(pinA, pinB, pinZ);  // pinZ optional
encoder.setResolution(4096);  // CPR after quadrature
encoder.begin();

// In loop
encoder.update(dt);  // Update velocity calculation
int32_t count = encoder.getCount();
float angle = encoder.getAngle();      // 0 to 2*PI
float velocity = encoder.getVelocity(); // rad/s

encoder.setMode(hal::EncoderMode::X4);  // X1, X2, X4
encoder.setInvert(true);
encoder.enableIndex(true);
if (encoder.indexDetected()) {
    encoder.clearIndex();
}

// ESP32 hardware encoder (PCNT)
#if defined(ARDUINO_ARCH_ESP32)
ArduinoEsp32Encoder encoder(pinA, pinB, PCNT_UNIT_0);
encoder.begin();
#endif

// Hall sensors
ArduinoHallSensor hall(pinH1, pinH2, pinH3);
hall.begin();
uint8_t state = hall.getState();
float elecAngle = hall.getElectricalAngle();
```

### SPI

```cpp
ArduinoSpi spi(csPin);
spi.begin();
spi.setClockSpeed(1000000);  // 1 MHz
spi.setMode(hal::SpiMode::Mode0);
spi.setBitOrder(true);  // MSB first

spi.select();
uint8_t rx = spi.transfer(0x55);
spi.deselect();

// Bulk transfer
uint8_t txBuf[10], rxBuf[10];
spi.select();
spi.transfer(txBuf, rxBuf, 10);
spi.deselect();
```

### UART

```cpp
ArduinoUart uart(Serial1);
uart.begin(115200);
uart.configure(8, hal::UartParity::None, hal::UartStopBits::One);

// Send/receive
uart.send(data, len);
size_t received = uart.receive(buffer, maxLen);

// RS-485 mode
ArduinoGpio dePin(8);
uart.enableRs485(true, &dePin);
```

## Platform-Specific Notes

### Arduino AVR

- **ADC**: Fixed 10-bit resolution, 5V reference
- **PWM**: 8-bit resolution, ~490Hz or ~980Hz frequency
- **Timer**: Software-based, requires `update()` calls
- **Encoder**: Interrupt-driven software counting
- **Pull-down**: Not supported (InputPullDown falls back to Input)
- **Open-drain**: Software emulated

### ESP32

- **ADC**: 12-bit resolution, configurable attenuation
- **PWM**: LEDC with configurable frequency/resolution
- **MCPWM**: Hardware three-phase with dead-time support
- **Timer**: Hardware esp_timer
- **Encoder**: PCNT hardware counting
- **All GPIO modes**: Fully supported

### RP2040

- **ADC**: 12-bit resolution, 3.3V reference
- **PWM**: Hardware with configurable frequency
- **Timer**: Hardware repeating_timer
- **Encoder**: PIO-capable (basic implementation)
- **All GPIO modes**: Fully supported

## Motor Control Example

```cpp
#include "arduino_hal.hpp"
#include <omni/driver/bldc_driver.hpp>

using namespace omni::platform::arduino;

// Hardware setup for ESP32
ArduinoEsp32Pwm3Phase pwm(25, 26, 27, 14, 12, 13);
uint8_t adcPins[] = {36, 39, 34};
ArduinoAdcMultiChannel currentAdc(adcPins, 3);
ArduinoEsp32Encoder encoder(32, 33, PCNT_UNIT_0);

omni::driver::BldcDriver* bldc;

void setup() {
    // Initialize hardware
    pwm.setFrequency(20000);
    pwm.setDeadTime(500);

    currentAdc.setResolution(12);

    encoder.setResolution(4096);
    encoder.begin();

    // Create driver
    bldc = new omni::driver::BldcDriver(&pwm, &currentAdc, &encoder);

    omni::driver::MotorParams params;
    params.polePairs = 7;
    params.maxCurrent = 10.0f;
    bldc->setParams(params);
    bldc->init();
    bldc->enable();
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;

    encoder.update(dt);
    bldc->update();

    // Control loop runs at ~10kHz
    delayMicroseconds(100);
}
```

## Limitations

1. **No DMA**: Arduino doesn't expose DMA. ADC DMA methods perform sequential reads.
2. **No CAN**: Standard Arduino has no CAN support. Use MCP2515 modules with dedicated libraries.
3. **Timer Resources**: AVR has limited timers. Using timer for PWM may conflict with timer HAL.
4. **PWM Frequency**: AVR boards have limited frequency options due to timer prescaler.
5. **Dead Time**: Only ESP32 MCPWM supports hardware dead-time insertion.

## Troubleshooting

### Encoder counts incorrectly
- Ensure pins support interrupts (`digitalPinToInterrupt()`)
- Use INPUT_PULLUP mode if encoder has open-collector outputs
- Check for electrical noise, add filtering capacitors

### PWM frequency not correct on AVR
- AVR uses fixed timer prescalers, frequency options are limited
- For precise frequency, use direct timer register manipulation

### ADC readings unstable
- Add decoupling capacitors on ADC input
- Use averaging: read multiple times and average
- On ESP32, reduce ADC attenuation for smaller signal ranges

### Timer callback not called
- On AVR, software timer requires `timer.update()` in loop
- Ensure callback is set before starting timer
- Check that another library isn't using the same timer

## License

Same as OmniMotion project license.
