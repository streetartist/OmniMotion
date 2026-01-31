# STM32 平台 HAL 实现

本目录包含 OmniMotion 的 STM32 平台硬件抽象层实现。

## 文件列表

| 文件 | 说明 |
|------|------|
| `stm32_gpio.hpp` | GPIO 接口实现 |
| `stm32_pwm.hpp` | 单通道 PWM 实现 |
| `stm32_pwm3phase.hpp` | 三相 PWM 实现 (FOC 用) |
| `stm32_adc.hpp` | ADC 实现 (含注入通道) |
| `stm32_encoder.hpp` | 编码器实现 (含 SPI 绝对编码器) |
| `stm32_can.hpp` | CAN 总线实现 (bxCAN / FDCAN) |
| `stm32_timer.hpp` | 定时器实现 (含 DWT 高精度计时) |
| `stm32_hal.hpp` | 主头文件 |

## 支持的 STM32 系列

- STM32F1 系列
- STM32F4 系列
- STM32F7 系列
- STM32H7 系列
- STM32G4 系列

## 使用方法

### 1. 使用 CubeMX 配置硬件

首先使用 STM32CubeMX 生成基础配置：

**BLDC FOC 控制推荐配置:**

| 外设 | 模式 | 参数 | 用途 |
|------|------|------|------|
| TIM1 | PWM 中心对齐 | 20kHz, 互补输出, 500ns死区 | 三相 PWM |
| TIM2 | 编码器模式 | TI1 和 TI2 | 编码器读取 |
| ADC1 | 注入模式 | 3通道, TIM1 TRGO 触发 | 电流采样 |
| TIM6 | 基本定时器 | 10kHz | 控制循环 |
| CAN1 | 正常模式 | 1Mbps | 通信 |

### 2. 包含头文件

```cpp
#include "platform/stm32/stm32_hal.hpp"
```

### 3. 编译选项

在编译时定义 STM32 系列宏：

```cmake
# CMakeLists.txt
add_definitions(-DSTM32F4)
```

或

```c
// 在代码中
#define STM32F4
```

### 4. 创建 HAL 对象

```cpp
using namespace omni::platform::stm32;

// 引用 CubeMX 生成的句柄
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;

// 创建 HAL 对象
Stm32Pwm3Phase pwm(&htim1);
Stm32Encoder encoder(&htim2);
Stm32Adc adcA(&hadc1, ADC_CHANNEL_0);
Stm32Adc adcB(&hadc1, ADC_CHANNEL_1);
Stm32Adc adcC(&hadc1, ADC_CHANNEL_2);
Stm32Can can(&hcan1);
```

### 5. 创建电机驱动

```cpp
#include <omni/driver/bldc_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 创建 BLDC 驱动
omni::driver::BldcDriver bldc(&pwm, &adcA, &adcB, &adcC, &encoder);

// 配置电机参数
omni::driver::MotorParams params;
params.polePairs = 7;
params.resistance = 0.5f;
params.inductance = 0.001f;
params.maxCurrent = 20.0f;
params.encoderCpr = 4096;
bldc.setParams(params);
bldc.init();

// 创建控制器
omni::app::MotorController motor(&bldc);
motor.setPositionPid(50.0f, 0.1f, 0.5f);
```

### 6. 实现中断回调

```cpp
// 在 stm32f4xx_it.c 中

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        // 10kHz 控制循环
        motor.update(0.0001f);
        bldc.update();
        encoder.update(0.0001f);
    }
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1) {
        // 电流采样完成，可在这里读取电流值
    }
}
```

## 示例代码

### BLDC FOC 控制

```cpp
#include "main.h"
#include "platform/stm32/stm32_hal.hpp"
#include <omni/driver/bldc_driver.hpp>
#include <omni/app/motor_controller.hpp>

using namespace omni::platform::stm32;
using namespace omni::driver;
using namespace omni::app;

// 全局对象
Stm32Pwm3Phase* pwm;
Stm32Encoder* encoder;
Stm32Adc* adcA;
Stm32Adc* adcB;
Stm32Adc* adcC;
BldcDriver* bldc;
MotorController* motor;

extern "C" void setup() {
    // 初始化 DWT
    Stm32DwtTimer::init();

    // 创建 HAL 对象
    pwm = new Stm32Pwm3Phase(&htim1);
    encoder = new Stm32Encoder(&htim2);
    adcA = new Stm32Adc(&hadc1, ADC_CHANNEL_0);
    adcB = new Stm32Adc(&hadc1, ADC_CHANNEL_1);
    adcC = new Stm32Adc(&hadc1, ADC_CHANNEL_2);

    // 配置硬件
    pwm->setFrequency(20000);
    pwm->setDeadTime(500);
    encoder->setResolution(4096);
    encoder->init();

    // 创建驱动
    bldc = new BldcDriver(pwm, adcA, adcB, adcC, encoder);

    MotorParams params;
    params.polePairs = 7;
    params.resistance = 0.5f;
    params.inductance = 0.001f;
    params.maxCurrent = 20.0f;
    params.encoderCpr = 4096;
    bldc->setParams(params);
    bldc->init();
    bldc->calibrateEncoder();

    // 创建控制器
    motor = new MotorController(bldc);
    motor->setPositionPid(50.0f, 0.1f, 0.5f);
    motor->setVelocityPid(0.1f, 0.001f, 0.0f);

    // 启动
    motor->enable();
    pwm->enable(true);

    // 启动控制定时器
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void loop() {
    // 执行 S 曲线运动
    motor->moveWithSCurve(3.14159f, 10.0f, 100.0f, 1000.0f);

    while (!motor->isSettled()) {
        HAL_Delay(10);
    }

    HAL_Delay(1000);

    // 返回原点
    motor->moveWithSCurve(0.0f, 10.0f, 100.0f, 1000.0f);

    while (!motor->isSettled()) {
        HAL_Delay(10);
    }

    HAL_Delay(1000);
}

// 控制循环中断
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        motor->update(0.0001f);  // 10kHz
        bldc->update();
        encoder->update(0.0001f);
    }
}
```

### 步进电机控制

```cpp
#include "platform/stm32/stm32_hal.hpp"
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

using namespace omni::platform::stm32;
using namespace omni::driver;
using namespace omni::app;

// 创建 GPIO
Stm32Gpio stepPin(GPIOA, GPIO_PIN_0);
Stm32Gpio dirPin(GPIOA, GPIO_PIN_1);
Stm32Gpio enPin(GPIOA, GPIO_PIN_2);

// 创建驱动
StepperDriver stepper(&stepPin, &dirPin, &enPin);

void setup() {
    stepPin.setMode(hal::PinMode::Output);
    dirPin.setMode(hal::PinMode::Output);
    enPin.setMode(hal::PinMode::Output);

    stepper.setStepsPerRev(200);
    stepper.setMicrostep(16);
    stepper.init();

    MotorController motor(&stepper);
    motor.enable();
    motor.moveWithSCurve(10.0f * 2 * M_PI, 5.0f, 50.0f, 500.0f);
}
```

### CAN 电机控制

```cpp
#include "platform/stm32/stm32_hal.hpp"
#include <omni/protocol/mit_protocol.hpp>

using namespace omni::platform::stm32;
using namespace omni::protocol;

Stm32Can can(&hcan1);

void setup() {
    can.setBitrate(1000000);
    can.setFilterAcceptAll();
    can.init();
}

void sendMitCommand(uint8_t motorId, float pos, float vel, float kp, float kd, float torque) {
    MitProtocol proto;
    proto.setPosRange(-12.5f, 12.5f);
    proto.setVelRange(-50.0f, 50.0f);
    proto.setTorqueRange(-18.0f, 18.0f);
    proto.setKpRange(0.0f, 500.0f);
    proto.setKdRange(0.0f, 5.0f);

    MitProtocol::Command cmd = {pos, vel, kp, kd, torque};
    hal::CanFrame frame;
    MitProtocol::encode(motorId, cmd, frame, proto);

    can.sendFrame(frame.id, frame.data, frame.len);
}
```

## 注意事项

1. **时钟配置**: 确保在 CubeMX 中正确配置了系统时钟和外设时钟

2. **中断优先级**: FOC 控制中断优先级应该较高

3. **DMA**: 推荐使用 DMA 进行 ADC 采样以减少 CPU 负载

4. **死区时间**: 根据实际功率器件调整死区时间

5. **编码器滤波**: 如有噪声问题，增加编码器输入滤波

## 常见问题

**Q: 编译报错找不到 HAL 头文件**

A: 确保包含了 STM32 HAL 库路径，并定义了正确的 STM32 系列宏

**Q: PWM 输出不正常**

A: 检查 CubeMX 配置，确保定时器配置为 PWM 模式，且引脚复用正确

**Q: 编码器计数不准确**

A: 检查编码器接线，确保 A/B 相正确连接，可能需要调整滤波参数
