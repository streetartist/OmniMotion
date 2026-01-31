# OmniMotion STM32 使用手册

**版本：1.0**
**日期：2026年1月**

---

# 目录

- [第一章 简介](#第一章-简介)
  - [1.1 OmniMotion 概述](#11-omnimotion-概述)
  - [1.2 支持的电机类型](#12-支持的电机类型)
  - [1.3 架构设计](#13-架构设计)
  - [1.4 开发环境](#14-开发环境)
- [第二章 快速开始](#第二章-快速开始)
  - [2.1 项目配置](#21-项目配置)
  - [2.2 第一个程序](#22-第一个程序)
  - [2.3 编译与烧录](#23-编译与烧录)
- [第三章 HAL 硬件抽象层](#第三章-hal-硬件抽象层)
  - [3.1 GPIO 适配](#31-gpio-适配)
  - [3.2 PWM 适配](#32-pwm-适配)
  - [3.3 编码器适配](#33-编码器适配)
  - [3.4 ADC 适配](#34-adc-适配)
  - [3.5 通信接口适配](#35-通信接口适配)
- [第四章 驱动层](#第四章-驱动层)
  - [4.1 步进电机驱动](#41-步进电机驱动)
  - [4.2 直流有刷电机驱动](#42-直流有刷电机驱动)
  - [4.3 无刷电机驱动](#43-无刷电机驱动)
  - [4.4 舵机驱动](#44-舵机驱动)
  - [4.5 直线电机驱动](#45-直线电机驱动)
  - [4.6 音圈电机驱动](#46-音圈电机驱动)
- [第五章 运动规划](#第五章-运动规划)
  - [5.1 梯形加减速](#51-梯形加减速)
  - [5.2 S曲线加减速](#52-s曲线加减速)
  - [5.3 在线规划器](#53-在线规划器)
  - [5.4 样条轨迹](#54-样条轨迹)
- [第六章 控制算法](#第六章-控制算法)
  - [6.1 PID 控制器](#61-pid-控制器)
  - [6.2 串级 PID](#62-串级-pid)
  - [6.3 前馈控制](#63-前馈控制)
  - [6.4 滤波器](#64-滤波器)
- [第七章 应用层](#第七章-应用层)
  - [7.1 电机控制器](#71-电机控制器)
  - [7.2 回零管理](#72-回零管理)
  - [7.3 多轴控制](#73-多轴控制)
  - [7.4 安全监控](#74-安全监控)
- [第八章 通信协议](#第八章-通信协议)
  - [8.1 CAN 协议](#81-can-协议)
  - [8.2 Modbus 协议](#82-modbus-协议)
- [第九章 实战案例](#第九章-实战案例)
  - [9.1 步进电机闭环控制](#91-步进电机闭环控制)
  - [9.2 双轴云台](#92-双轴云台)
  - [9.3 3D打印机控制](#93-3d打印机控制)
  - [9.4 机械臂关节](#94-机械臂关节)
- [附录](#附录)
  - [A. API 参考](#a-api-参考)
  - [B. 常见问题](#b-常见问题)
  - [C. 硬件参考设计](#c-硬件参考设计)

---

# 第一章 简介

## 1.1 OmniMotion 概述

OmniMotion 是一个跨平台的电机控制库，专为嵌入式系统设计。它提供了从底层硬件抽象到高级运动控制的完整解决方案。

**核心特性：**

- **多电机支持**：步进电机、直流有刷、无刷电机、舵机、直线电机、音圈电机
- **先进控制算法**：PID、串级PID、前馈、自适应控制、滑模控制
- **运动规划**：梯形加减速、S曲线、样条轨迹、在线规划
- **通信协议**：CAN、Modbus、MIT协议
- **应用功能**：回零、多轴同步、安全监控、示教再现

**设计理念：**

```
"但凡用电机，皆可用此库"
```

OmniMotion 采用分层架构，通过硬件抽象层(HAL)实现跨平台移植，用户只需实现少量接口即可在任意 MCU 上运行。

## 1.2 支持的电机类型

| 电机类型 | 驱动类 | 控制方式 | 典型应用 |
|---------|--------|---------|---------|
| 步进电机 | StepperDriver | STEP/DIR脉冲 | 3D打印机、CNC |
| 直流有刷 | DcMotorDriver | H桥PWM | 小车、机械臂 |
| 无刷电机 | BldcDriver | FOC矢量控制 | 无人机、电动车 |
| RC舵机 | ServoDriver | PWM脉宽 | 机器人关节 |
| 直线电机 | LinearSyncMotorDriver | FOC | 精密平台 |
| 音圈电机 | VcmDriver | 电流控制 | 光学对焦 |

## 1.3 架构设计

OmniMotion 采用五层架构：

```
┌─────────────────────────────────────────────────────────────┐
│                      应用层 (Application)                    │
│   MotorController │ HomingManager │ MultiAxisController     │
├─────────────────────────────────────────────────────────────┤
│                      运动规划层 (Motion)                     │
│   TrapezoidProfile │ SCurveProfile │ OnlinePlanner          │
├─────────────────────────────────────────────────────────────┤
│                      控制层 (Control)                        │
│   PIDController │ CascadePID │ Feedforward │ Filters        │
├─────────────────────────────────────────────────────────────┤
│                      驱动层 (Driver)                         │
│   StepperDriver │ DcMotorDriver │ BldcDriver │ ServoDriver  │
├─────────────────────────────────────────────────────────────┤
│                      硬件抽象层 (HAL)                        │
│   IGpio │ IPwm │ IEncoder │ IAdc │ IUart │ ISpi │ ICan     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    STM32 HAL / LL 库                         │
└─────────────────────────────────────────────────────────────┘
```

**数据流：**

```
目标位置 → 轨迹规划 → 位置环PID → 速度环PID → 电流环 → PWM输出
     ↑                                              ↓
     └──────────────── 编码器反馈 ←─────────────────┘
```

## 1.4 开发环境

### 支持的 STM32 系列

| 系列 | 推荐型号 | 特点 |
|------|---------|------|
| STM32F1 | F103C8T6 | 入门级，72MHz |
| STM32F4 | F407VET6 | 高性能，168MHz，DSP |
| STM32G4 | G431CBU6 | 电机控制专用，170MHz |
| STM32H7 | H743VIT6 | 旗舰级，480MHz |

### 开发工具

- **IDE**：STM32CubeIDE / CLion / VS Code
- **编译器**：ARM GCC 10.3+
- **构建系统**：CMake 3.22+
- **调试器**：ST-Link V2 / J-Link

### 软件依赖

```
OmniMotion
├── C++17 标准库
├── STM32 HAL 库 (由 CubeMX 生成)
└── CMSIS (ARM 核心支持)
```

---

# 第二章 快速开始

## 2.1 项目配置

### 步骤1：创建 STM32CubeMX 项目

1. 打开 STM32CubeMX，选择目标芯片（如 STM32G431CBU6）
2. 配置时钟树（建议使用最高主频）
3. 配置所需外设：
   - GPIO：电机使能、方向控制
   - TIM：PWM输出、编码器接口、控制定时器
   - SPI/I2C：磁编码器通信
   - UART：调试输出、TMC驱动通信
   - CAN/FDCAN：总线通信
4. 生成代码，选择 CMake 工程

### 步骤2：添加 OmniMotion 库

在 `CMakeLists.txt` 中添加：

```cmake
# OmniMotion 库路径
set(OMNIMOTION_PATH "/path/to/OmniMotion")

# 添加头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${OMNIMOTION_PATH}/include
    ${OMNIMOTION_PATH}/platform    # STM32 预写适配层
)

# 添加源文件
target_sources(${PROJECT_NAME} PRIVATE
    ${OMNIMOTION_PATH}/src/stepper_driver.cpp
    # 根据需要添加其他驱动
)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)

# 定义 STM32 系列宏 (根据实际芯片选择)
target_compile_definitions(${PROJECT_NAME} PRIVATE STM32G4)
```

## 2.2 第一个程序

OmniMotion 提供两种 HAL 适配方式：

1. **使用预写适配层**（推荐）：直接使用 `platform/stm32` 目录下的代码
2. **手动编写适配**：自己实现 HAL 接口

### 方式一：使用预写 STM32 适配层（推荐）

OmniMotion 在 `platform/stm32/` 目录下提供了完整的 STM32 HAL 适配实现，可直接使用。

**包含的适配类：**

| 文件 | 类 | 功能 |
|------|-----|------|
| stm32_gpio.hpp | Stm32Gpio | GPIO 控制 |
| stm32_gpio.hpp | Stm32GpioInterrupt | GPIO + 中断 |
| stm32_pwm.hpp | Stm32Pwm | 单通道 PWM |
| stm32_pwm.hpp | Stm32PwmComplementary | 互补 PWM |
| stm32_pwm3phase.hpp | Stm32Pwm3Phase | 三相 PWM (FOC) |
| stm32_encoder.hpp | Stm32Encoder | 正交编码器 |
| stm32_encoder.hpp | Stm32EncoderWithIndex | 带 Z 相编码器 |
| stm32_encoder.hpp | Stm32SpiEncoder | SPI 绝对式编码器 |
| stm32_adc.hpp | Stm32Adc | ADC 采样 |
| stm32_can.hpp | Stm32Can | CAN 通信 |
| stm32_timer.hpp | Stm32Timer | 定时器 |

**使用方法：**

```cpp
// main.cpp
#include "stm32/stm32_hal.hpp"  // 包含所有 STM32 适配
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

using namespace omni::platform::stm32;

// CubeMX 生成的句柄
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// 直接使用预写的类
Stm32Gpio stepPin(GPIOA, GPIO_PIN_0);
Stm32Gpio dirPin(GPIOA, GPIO_PIN_1);
Stm32Gpio enablePin(GPIOA, GPIO_PIN_2);
Stm32Encoder encoder(&htim2);

// 创建驱动
omni::driver::StepperDriver stepper(&stepPin, &dirPin, &encoder, &enablePin);
```

**支持的 STM32 系列：**

需要在编译时定义对应的宏：

| 系列 | 编译宏 |
|------|--------|
| STM32F1 | `-DSTM32F1` |
| STM32F4 | `-DSTM32F4` |
| STM32F7 | `-DSTM32F7` |
| STM32G4 | `-DSTM32G4` |
| STM32H7 | `-DSTM32H7` |

### 方式二：手动创建 HAL 适配文件

创建 `motor_hal.hpp`：

```cpp
#pragma once
#include <omni/hal/hal.hpp>
#include "stm32g4xx_hal.h"

// GPIO 适配
class Stm32Gpio : public omni::hal::IGpio {
public:
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void write(bool value) override {
        HAL_GPIO_WritePin(port_, pin_,
            value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    bool read() override {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    void setMode(omni::hal::PinMode) override {}
    omni::hal::PinMode getMode() const override {
        return omni::hal::PinMode::Output;
    }
    void enableInterrupt(bool, bool) override {}
    void disableInterrupt() override {}

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};
```

### 主程序

```cpp
// main.cpp
#include "motor_hal.hpp"
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 硬件对象
Stm32Gpio stepPin(GPIOA, GPIO_PIN_0);
Stm32Gpio dirPin(GPIOA, GPIO_PIN_1);
Stm32Gpio enablePin(GPIOA, GPIO_PIN_2);

// 驱动和控制器
omni::driver::StepperDriver* stepper;
omni::app::MotorController* motor;

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM17_Init();

    // 初始化驱动
    stepper = new omni::driver::StepperDriver(
        &stepPin, &dirPin, nullptr, &enablePin);
    stepper->setMicrostep(16);
    stepper->setStepsPerRev(200);
    stepper->init();

    // 初始化控制器
    motor = new omni::app::MotorController(stepper);
    motor->setConstraints({10.0f, 50.0f, 500.0f});
    motor->enable();

    // 启动控制定时器
    HAL_TIM_Base_Start_IT(&htim17);

    // 移动到目标位置
    motor->moveTo(6.28f);  // 1圈

    while (1) {
        HAL_Delay(100);
    }
}

// 1kHz 控制中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM17) {
        motor->update(0.001f);
        stepper->update();
    }
}
```

## 2.3 编译与烧录

```bash
# 配置
cmake --preset Debug

# 编译
cmake --build --preset Debug

# 烧录 (使用 ST-Link)
st-flash write build/Debug/project.bin 0x8000000
```

---

# 第三章 HAL 硬件抽象层

HAL 层是 OmniMotion 与硬件之间的桥梁。

> **提示**：OmniMotion 已在 `platform/stm32/` 目录下提供了完整的 STM32 HAL 适配实现，
> 可直接 `#include "stm32/stm32_hal.hpp"` 使用，无需手动编写以下代码。
> 本章内容供需要自定义或移植到其他平台的用户参考。

## 3.1 GPIO 适配

**预写实现**：`platform/stm32/stm32_gpio.hpp` 中的 `Stm32Gpio` 类

**手动实现参考**：

```cpp
class Stm32Gpio : public omni::hal::IGpio {
public:
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void write(bool value) override {
        HAL_GPIO_WritePin(port_, pin_,
            value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    bool read() override {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

// 使用示例
Stm32Gpio ledPin(GPIOC, GPIO_PIN_13);
ledPin.write(true);   // 点亮
ledPin.toggle();      // 翻转
```

## 3.2 PWM 适配

**预写实现**：`platform/stm32/stm32_pwm.hpp` 中的 `Stm32Pwm`、`Stm32PwmComplementary` 类

**手动实现参考**：

```cpp
class Stm32Pwm : public omni::hal::IPwm {
public:
    Stm32Pwm(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim), channel_(channel), enabled_(false) {}

    void setFrequency(uint32_t freq) override {
        if (freq == 0) return;
        uint32_t clock = HAL_RCC_GetPCLK1Freq() * 2;
        uint32_t psc = htim_->Init.Prescaler;
        uint32_t arr = clock / ((psc + 1) * freq) - 1;
        __HAL_TIM_SET_AUTORELOAD(htim_, arr);
    }

    void setDuty(float duty) override {
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
        uint32_t ccr = (uint32_t)(arr * duty);
        __HAL_TIM_SET_COMPARE(htim_, channel_, ccr);
    }

    void enable(bool en) override {
        if (en && !enabled_) {
            HAL_TIM_PWM_Start(htim_, channel_);
            enabled_ = true;
        } else if (!en && enabled_) {
            HAL_TIM_PWM_Stop(htim_, channel_);
            enabled_ = false;
        }
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    bool enabled_;
};

// 使用示例
Stm32Pwm motorPwm(&htim2, TIM_CHANNEL_1);
motorPwm.setFrequency(20000);  // 20kHz
motorPwm.setDuty(0.5f);        // 50%
motorPwm.enable(true);
```

## 3.3 编码器适配

**预写实现**：`platform/stm32/stm32_encoder.hpp` 中的：
- `Stm32Encoder` - 正交编码器
- `Stm32EncoderWithIndex` - 带 Z 相索引
- `Stm32SpiEncoder` - SPI 绝对式编码器 (AS5047, MA730 等)

**手动实现参考**：

### 增量式编码器（定时器模式）

```cpp
class Stm32QuadEncoder : public omni::hal::IEncoder {
public:
    Stm32QuadEncoder(TIM_HandleTypeDef* htim, uint32_t cpr)
        : htim_(htim), cpr_(cpr) {
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    }

    int32_t getCount() override {
        return (int16_t)__HAL_TIM_GET_COUNTER(htim_);
    }

    void resetCount() override {
        __HAL_TIM_SET_COUNTER(htim_, 0);
    }

    float getAngle() override {
        return (float)getCount() / cpr_ * 6.2832f;
    }

    float getVelocity() override { return velocity_; }

    void update(float dt) {
        int32_t count = getCount();
        velocity_ = (float)(count - lastCount_) / cpr_ * 6.2832f / dt;
        lastCount_ = count;
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t cpr_;
    int32_t lastCount_ = 0;
    float velocity_ = 0;
};
```

### 磁编码器（SPI）

```cpp
class MT6816Encoder : public omni::hal::IEncoder {
public:
    MT6816Encoder(SPI_HandleTypeDef* hspi,
                  GPIO_TypeDef* csPort, uint16_t csPin)
        : hspi_(hspi), csPort_(csPort), csPin_(csPin) {}

    float getAngle() override {
        uint16_t raw = readRaw();
        return (float)raw / 16384.0f * 6.2832f;
    }

    int32_t getCount() override {
        return (int32_t)(getAngle() / 6.2832f * 16384);
    }

private:
    uint16_t readRaw() {
        uint16_t tx = 0x8300, rx = 0;
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi_, (uint8_t*)&tx,
                                (uint8_t*)&rx, 1, 10);
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
        return (rx >> 2) & 0x3FFF;
    }

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
};
```

## 3.4 ADC 适配

**预写实现**：`platform/stm32/stm32_adc.hpp` 中的 `Stm32Adc` 类

**手动实现参考**：

```cpp
class Stm32Adc : public omni::hal::IAdc {
public:
    Stm32Adc(ADC_HandleTypeDef* hadc)
        : hadc_(hadc) {}

    float read() override {
        HAL_ADC_Start(hadc_);
        HAL_ADC_PollForConversion(hadc_, 10);
        uint32_t raw = HAL_ADC_GetValue(hadc_);
        return (float)raw / 4095.0f * 3.3f;  // 12位ADC
    }

    uint32_t readRaw() override {
        HAL_ADC_Start(hadc_);
        HAL_ADC_PollForConversion(hadc_, 10);
        return HAL_ADC_GetValue(hadc_);
    }

private:
    ADC_HandleTypeDef* hadc_;
};

// 使用示例：电流采样
Stm32Adc currentAdc(&hadc1);
float current = currentAdc.read() * 10.0f;  // 假设增益10
```

## 3.5 通信接口适配

**预写实现**：`platform/stm32/stm32_can.hpp` 中的 `Stm32Can` 类

**手动实现参考**：

### UART 适配

```cpp
class Stm32Uart : public omni::hal::IUart {
public:
    Stm32Uart(UART_HandleTypeDef* huart)
        : huart_(huart) {}

    bool send(const uint8_t* data, size_t len) override {
        return HAL_UART_Transmit(huart_, (uint8_t*)data,
                                 len, 100) == HAL_OK;
    }

    size_t receive(uint8_t* data, size_t len) override {
        if (HAL_UART_Receive(huart_, data, len, 10) == HAL_OK)
            return len;
        return 0;
    }

private:
    UART_HandleTypeDef* huart_;
};
```

---

# 第四章 驱动层

## 4.1 步进电机驱动

### 基本使用

```cpp
#include <omni/driver/stepper_driver.hpp>

// 创建驱动
StepperDriver stepper(&stepPin, &dirPin, &encoder, &enablePin);

// 配置
stepper.setStepsPerRev(200);   // 1.8°步距角
stepper.setMicrostep(16);      // 16细分
stepper.setDirectionInvert(false);

// 初始化
stepper.init();
stepper.enable();

// 速度控制
stepper.setVelocity(6.28f);    // 1转/秒

// 控制循环
while (true) {
    stepper.update();
    delay_us(100);
}
```

### TMC2209 配置

```cpp
// TMC2209 UART 驱动
class TMC2209 {
public:
    TMC2209(UART_HandleTypeDef* huart, uint8_t addr = 0);

    void init();
    void setCurrent(uint8_t run, uint8_t hold);
    void setMicrostep(uint16_t mstep);
    void setStealthChop(bool enable);
};

// 使用
TMC2209 tmc(&huart2, 0);
tmc.init();
tmc.setCurrent(16, 8);     // 运行/保持电流
tmc.setMicrostep(64);      // 64细分
tmc.setStealthChop(true);  // 静音模式
```

## 4.2 直流有刷电机驱动

```cpp
#include <omni/driver/dc_motor_driver.hpp>

// H桥模式（双PWM）
DcMotorDriver motor(&pwmA, &pwmB);

// PWM+方向模式
DcMotorDriver motor(&pwm, &dirPin);

// 带编码器
DcMotorDriver motor(&pwmA, &pwmB, nullptr, &encoder);

// 初始化
motor.init();
motor.enable();

// 开环控制
motor.setDuty(0.5f);   // 50%正转
motor.setDuty(-0.3f);  // 30%反转

// 闭环速度控制
motor.setControlMode(ControlMode::Velocity);
motor.setVelocityPid(0.5f, 0.1f, 0.0f);
motor.setVelocity(10.0f);  // rad/s
```

## 4.3 无刷电机驱动

```cpp
#include <omni/driver/bldc_driver.hpp>

// 三相PWM + 电流采样 + 编码器
BldcDriver bldc(&pwm3Phase, &adcA, &adcB, &adcC, &encoder);

// 配置
bldc.setPolePairs(7);          // 极对数
bldc.setCurrentPid(0.5f, 10.0f, 0);

// 初始化
bldc.init();
bldc.enable();

// FOC 电流控制
bldc.setControlMode(ControlMode::Current);
bldc.setCurrent(1.0f);  // 1A

// FOC 速度控制
bldc.setControlMode(ControlMode::Velocity);
bldc.setVelocity(100.0f);  // rad/s

// 高频控制循环 (10kHz+)
void TIM1_UP_IRQHandler() {
    bldc.update();
}
```

## 4.4 舵机驱动

```cpp
#include <omni/driver/servo_driver.hpp>

ServoDriver servo(&pwm);

// 配置脉宽范围
servo.setPulseRange(500, 2500);   // 500-2500us
servo.setAngleRange(-1.57f, 1.57f);  // ±90°

servo.init();
servo.enable();

// 角度控制
servo.setPosition(0);       // 中位
servo.setAngleDeg(45.0f);   // 45度
```

## 4.5 直线电机驱动

```cpp
#include <omni/driver/linear_driver.hpp>

LinearSyncMotorDriver linear(&pwm3Phase, &adcA, &adcB, &linearEncoder);

linear.setPolePitch(0.032f);     // 32mm极距
linear.setForceConstant(50.0f);  // 50N/A

linear.init();
linear.enable();

// 位置控制
linear.setLinearPosition(0.1f);  // 100mm
```

## 4.6 音圈电机驱动

```cpp
#include <omni/driver/vcm_driver.hpp>

VcmDriver vcm(&pwm, &positionAdc, &currentAdc);

vcm.setPositionPid(100.0f, 10.0f, 1.0f);
vcm.init();
vcm.enable();

// 位置控制
vcm.setPosition(0.001f);  // 1mm
```

---

# 第五章 运动规划

## 5.1 梯形加减速

```cpp
#include <omni/motion/trapezoidal_profile.hpp>

TrapezoidalProfile profile({
    .maxVelocity = 10.0f,      // rad/s
    .maxAcceleration = 50.0f   // rad/s²
});

// 规划轨迹
profile.plan(0, 100.0f, 0, 0);  // 从0到100

// 获取总时间
float duration = profile.getDuration();

// 采样轨迹点
for (float t = 0; t < duration; t += 0.001f) {
    TrajectoryPoint pt = profile.evaluate(t);
    // pt.position, pt.velocity, pt.acceleration
}
```

## 5.2 S曲线加减速

S曲线通过限制加加速度(Jerk)实现平滑运动，减少机械冲击。

```cpp
#include <omni/motion/scurve_profile.hpp>

SCurveProfile profile({
    .maxVelocity = 10.0f,       // rad/s
    .maxAcceleration = 50.0f,   // rad/s²
    .maxJerk = 500.0f           // rad/s³
});

profile.plan(0, 100.0f, 0, 0);

// S曲线7段：
// 1. 加加速 (jerk = +J)
// 2. 匀加速 (jerk = 0)
// 3. 减加速 (jerk = -J)
// 4. 匀速   (accel = 0)
// 5. 加减速 (jerk = -J)
// 6. 匀减速 (jerk = 0)
// 7. 减减速 (jerk = +J)
```

## 5.3 在线规划器

实时响应目标变化，适合交互式控制。

```cpp
#include <omni/motion/online_planner.hpp>

OnlinePlanner planner({10.0f, 50.0f, 500.0f});

// 控制循环中实时更新
void controlLoop(float dt) {
    planner.setTarget(newTarget);
    TrajectoryPoint pt = planner.update(dt);
    driver.setVelocity(pt.velocity);
}
```

## 5.4 样条轨迹

多点平滑插值。

```cpp
#include <omni/motion/spline_trajectory.hpp>

SplineTrajectory spline;
spline.addPoint(0, 0);
spline.addPoint(1.0f, 10.0f);
spline.addPoint(2.0f, 5.0f);
spline.addPoint(3.0f, 20.0f);
spline.build();

float pos = spline.evaluate(1.5f);
```

---

# 第六章 控制算法

## 6.1 PID 控制器

```cpp
#include <omni/control/pid_controller.hpp>

PidController pid({
    .kp = 10.0f,
    .ki = 1.0f,
    .kd = 0.5f,
    .outputLimit = 100.0f,
    .integralLimit = 50.0f
});

// 控制循环
float output = pid.update(setpoint, measurement, dt);

// 重置积分
pid.reset();
```

### PID 参数整定

**Ziegler-Nichols 方法：**

1. 设置 Ki=0, Kd=0
2. 逐渐增大 Kp 直到系统产生持续振荡
3. 记录临界增益 Ku 和振荡周期 Tu
4. 按下表计算参数：

| 控制器类型 | Kp | Ki | Kd |
|-----------|----|----|-----|
| P | 0.5*Ku | - | - |
| PI | 0.45*Ku | 1.2*Kp/Tu | - |
| PID | 0.6*Ku | 2*Kp/Tu | Kp*Tu/8 |

**手动整定技巧：**

```cpp
// 1. 先调 Kp，使系统快速响应但不过冲太多
pid.setGains(10.0f, 0, 0);

// 2. 加入 Kd 抑制过冲
pid.setGains(10.0f, 0, 0.5f);

// 3. 加入 Ki 消除稳态误差
pid.setGains(10.0f, 1.0f, 0.5f);
```

## 6.2 串级 PID

串级 PID 将多个控制环嵌套，内环响应快，外环精度高。

```cpp
#include <omni/control/cascade_pid.hpp>

// 位置-速度-电流三环控制
CascadePID cascade;

// 配置各环参数
cascade.setPositionPid(50.0f, 0, 5.0f);    // 外环：位置
cascade.setVelocityPid(10.0f, 1.0f, 0);    // 中环：速度
cascade.setCurrentPid(0.5f, 10.0f, 0);     // 内环：电流

// 设置各环限幅
cascade.setVelocityLimit(100.0f);  // rad/s
cascade.setCurrentLimit(10.0f);    // A

// 控制循环
void controlLoop(float dt) {
    float posRef = targetPosition;
    float posFb = encoder.getAngle();
    float velFb = encoder.getVelocity();
    float curFb = currentSensor.read();

    float output = cascade.compute(posRef, posFb, velFb, curFb, dt);
    driver.setVoltage(output);
}
```

**串级控制原理：**

```
位置环        速度环        电流环
  │            │            │
  ▼            ▼            ▼
┌────┐      ┌────┐      ┌────┐
│ Kp │─────▶│ Kp │─────▶│ Kp │─────▶ PWM
│ Ki │      │ Ki │      │ Ki │
│ Kd │      │ Kd │      │ Kd │
└────┘      └────┘      └────┘
  ▲            ▲            ▲
  │            │            │
位置反馈    速度反馈    电流反馈
```

## 6.3 前馈控制

前馈控制根据目标轨迹预测所需控制量，减少跟踪误差。

```cpp
#include <omni/control/feedforward.hpp>

// 速度前馈 + 加速度前馈
Feedforward ff;
ff.setVelocityGain(0.1f);      // Kv: 速度前馈系数
ff.setAccelerationGain(0.01f); // Ka: 加速度前馈系数

// 结合 PID 使用
void controlLoop(float dt) {
    TrajectoryPoint ref = planner.evaluate(t);

    // 前馈项
    float ffOutput = ff.compute(ref.velocity, ref.acceleration);

    // 反馈项
    float fbOutput = pid.compute(ref.position - encoder.getAngle());

    // 总输出 = 前馈 + 反馈
    driver.setVoltage(ffOutput + fbOutput);
}
```

**前馈参数计算：**

```cpp
// 对于直流电机：
// V = Ke * ω + R * I
// τ = Kt * I
// τ = J * α
//
// 因此：
// Kv = Ke (反电动势常数)
// Ka = R * J / Kt (加速度前馈)

float Ke = 0.1f;   // V/(rad/s)
float R = 1.0f;    // Ω
float J = 0.001f;  // kg·m²
float Kt = 0.1f;   // N·m/A

ff.setVelocityGain(Ke);
ff.setAccelerationGain(R * J / Kt);
```

## 6.4 滤波器

### 低通滤波器

```cpp
#include <omni/control/filters.hpp>

// 一阶低通滤波
LowPassFilter lpf(100.0f);  // 截止频率 100Hz

void loop(float dt) {
    float raw = adc.read();
    float filtered = lpf.update(raw, dt);
}

// 二阶低通滤波（Butterworth）
ButterworthFilter bw(100.0f, 1000.0f);  // 截止100Hz，采样1kHz

float filtered = bw.update(raw);
```

### 滑动平均滤波

```cpp
MovingAverageFilter maf(16);  // 16点滑动平均

float filtered = maf.update(raw);
```

### 陷波滤波器

用于消除特定频率干扰（如电机齿槽效应）。

```cpp
NotchFilter notch(50.0f, 0.1f, 1000.0f);  // 50Hz陷波，Q=0.1

float filtered = notch.update(raw);
```

### 微分滤波

避免直接微分带来的噪声放大。

```cpp
// 带滤波的微分器
FilteredDerivative deriv(50.0f);  // 截止频率50Hz

float velocity = deriv.update(position, dt);
```

---

# 第七章 应用层

## 7.1 电机控制器

MotorController 是最常用的高级接口，封装了驱动、规划、控制的完整流程。

```cpp
#include <omni/app/motor_controller.hpp>

// 创建控制器
MotorController motor(driver);

// 设置运动约束
motor.setConstraints({
    .maxVelocity = 10.0f,       // rad/s
    .maxAcceleration = 50.0f,   // rad/s²
    .maxJerk = 500.0f           // rad/s³
});

// 设置控制参数
motor.setPositionPid(50.0f, 0, 5.0f);
motor.setVelocityPid(10.0f, 1.0f, 0);

// 使能
motor.enable();

// 运动命令
motor.moveTo(100.0f);           // 绝对位置
motor.moveBy(10.0f);            // 相对位置
motor.setTargetVelocity(5.0f);  // 速度模式

// 状态查询
bool busy = motor.isBusy();
bool reached = motor.isTargetReached();
float pos = motor.getPosition();
float vel = motor.getVelocity();

// 控制循环（1kHz）
void TIM_IRQHandler() {
    motor.update(0.001f);
}
```

### 运动模式

```cpp
// 位置模式（默认）
motor.setMode(MotorController::Mode::Position);
motor.moveTo(target);

// 速度模式
motor.setMode(MotorController::Mode::Velocity);
motor.setTargetVelocity(10.0f);

// 力矩模式
motor.setMode(MotorController::Mode::Torque);
motor.setTargetTorque(0.5f);
```

### 轨迹类型选择

```cpp
// 梯形轨迹（默认）
motor.setProfileType(ProfileType::Trapezoidal);

// S曲线轨迹
motor.setProfileType(ProfileType::SCurve);

// 在线规划（实时响应）
motor.setProfileType(ProfileType::Online);
```

## 7.2 回零管理

HomingManager 提供多种回零策略，支持限位开关、编码器索引、堵转检测等。

```cpp
#include <omni/app/homing_manager.hpp>

HomingManager homing(motor, &limitSwitch);

// 配置回零参数
homing.setHomingVelocity(2.0f);      // 回零速度
homing.setHomingAcceleration(10.0f); // 回零加速度
homing.setBackoffDistance(0.1f);     // 回退距离
homing.setHomeOffset(0.05f);         // 零点偏移

// 选择回零方式
homing.setHomingMethod(HomingMethod::LimitSwitch);

// 启动回零
homing.start();

// 在控制循环中更新
void controlLoop(float dt) {
    homing.update(dt);

    if (homing.isComplete()) {
        // 回零完成
    }
    if (homing.hasFailed()) {
        // 回零失败
        HomingError err = homing.getError();
    }
}
```

### 回零方式

```cpp
enum class HomingMethod {
    LimitSwitch,      // 限位开关
    LimitAndIndex,    // 限位 + 编码器索引
    CurrentLimit,     // 堵转检测
    EncoderIndex,     // 仅编码器索引
    TouchProbe,       // 触碰探针
    Manual            // 手动设置
};

// 限位开关回零
homing.setHomingMethod(HomingMethod::LimitSwitch);
homing.setLimitSwitchPolarity(true);  // 高电平有效

// 堵转回零（无限位开关时）
homing.setHomingMethod(HomingMethod::CurrentLimit);
homing.setStallCurrentThreshold(2.0f);  // 堵转电流阈值
homing.setStallTime(0.1f);              // 堵转持续时间
```

### 回零流程

```
1. 快速寻找限位开关
   ──────────────────▶ [限位触发]

2. 回退一段距离
   ◀────────

3. 慢速再次接近
   ────▶ [限位触发]

4. 设置零点
   [HOME]
```

## 7.3 多轴控制

MultiAxisController 实现多轴协调运动，支持插补、同步启停。

```cpp
#include <omni/app/multi_axis_controller.hpp>

// 创建多轴控制器
MultiAxisController multiAxis;

// 添加轴
multiAxis.addAxis(&motorX, "X");
multiAxis.addAxis(&motorY, "Y");
multiAxis.addAxis(&motorZ, "Z");

// 设置全局约束
multiAxis.setGlobalConstraints({10.0f, 50.0f, 500.0f});

// 多轴同步移动
multiAxis.moveTo({100.0f, 50.0f, 25.0f});

// 直线插补
multiAxis.linearMove({100.0f, 100.0f, 0}, 10.0f);  // 目标点，速度

// 圆弧插补（XY平面）
multiAxis.arcMove({50.0f, 50.0f}, {100.0f, 0}, true);  // 圆心，终点，顺时针
```

### 同步控制

```cpp
// 所有轴同时到达目标
multiAxis.setSyncMode(SyncMode::TimeSync);

// 各轴独立运动
multiAxis.setSyncMode(SyncMode::Independent);

// 主从跟随
multiAxis.setMasterAxis("X");
multiAxis.setSlaveRatio("Y", 2.0f);  // Y轴速度是X轴2倍
```

### 轨迹队列

```cpp
// 添加多段轨迹
multiAxis.queueMove({10, 0, 0});
multiAxis.queueMove({10, 10, 0});
multiAxis.queueMove({0, 10, 0});
multiAxis.queueMove({0, 0, 0});

// 开始执行
multiAxis.startQueue();

// 查询进度
int remaining = multiAxis.getQueueSize();
```

## 7.4 安全监控

SafetyMonitor 提供全面的安全保护功能。

```cpp
#include <omni/app/safety_monitor.hpp>

SafetyMonitor safety;

// 添加监控项
safety.addMotor(&motor);
safety.setCurrentLimit(10.0f);        // 过流保护
safety.setTemperatureLimit(80.0f);    // 过温保护
safety.setPositionLimits(-100, 100);  // 软限位
safety.setVelocityLimit(50.0f);       // 超速保护

// 添加限位开关
safety.addLimitSwitch(&limitPos, LimitType::Positive);
safety.addLimitSwitch(&limitNeg, LimitType::Negative);

// 设置急停输入
safety.setEmergencyStop(&estopPin);

// 启用看门狗
safety.enableWatchdog(100);  // 100ms 超时

// 控制循环中检查
void controlLoop() {
    safety.update();

    if (safety.hasFault()) {
        SafetyFault fault = safety.getFault();
        handleFault(fault);
    }
}
```

### 故障类型

```cpp
enum class SafetyFault {
    None = 0,
    Overcurrent     = (1 << 0),
    Overtemperature = (1 << 1),
    PositionLimit   = (1 << 2),
    VelocityLimit   = (1 << 3),
    HardwareLimit   = (1 << 4),
    EmergencyStop   = (1 << 5),
    WatchdogTimeout = (1 << 6),
    EncoderError    = (1 << 7),
    CommunicationError = (1 << 8)
};

// 故障处理
void handleFault(SafetyFault fault) {
    motor.emergencyStop();

    if (fault & SafetyFault::Overcurrent) {
        // 过流处理
    }
    if (fault & SafetyFault::EmergencyStop) {
        // 等待急停解除
    }
}
```

---

# 第八章 通信协议

## 8.1 CAN 协议

OmniMotion 支持标准 CAN 和 CAN FD 通信。

### CAN 适配器

```cpp
#include <omni/hal/can_adapter.hpp>

class Stm32Can : public omni::hal::ICan {
public:
    Stm32Can(FDCAN_HandleTypeDef* hcan) : hcan_(hcan) {}

    bool send(uint32_t id, const uint8_t* data, uint8_t len) override {
        FDCAN_TxHeaderTypeDef header = {
            .Identifier = id,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = len << 16,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0
        };
        return HAL_FDCAN_AddMessageToTxFifoQ(hcan_, &header,
                                              (uint8_t*)data) == HAL_OK;
    }

    bool receive(uint32_t& id, uint8_t* data, uint8_t& len) override {
        FDCAN_RxHeaderTypeDef header;
        if (HAL_FDCAN_GetRxMessage(hcan_, FDCAN_RX_FIFO0,
                                    &header, data) == HAL_OK) {
            id = header.Identifier;
            len = header.DataLength >> 16;
            return true;
        }
        return false;
    }

private:
    FDCAN_HandleTypeDef* hcan_;
};
```

### CANopen 协议

```cpp
#include <omni/protocol/canopen.hpp>

CANopenNode node(&can, 0x01);  // 节点ID = 1

// 配置对象字典
node.setObject(0x6040, 0, controlWord);   // 控制字
node.setObject(0x6041, 0, statusWord);    // 状态字
node.setObject(0x607A, 0, targetPosition);// 目标位置
node.setObject(0x6064, 0, actualPosition);// 实际位置

// 启动节点
node.start();

// 处理 PDO
void CAN_RxCallback(uint32_t id, uint8_t* data) {
    node.processMessage(id, data);
}
```

### MIT 协议

MIT 协议是一种简洁的电机控制协议，常用于关节电机。

```cpp
#include <omni/protocol/mit_protocol.hpp>

MitProtocol mit(&can, 0x01);

// 发送控制命令
mit.sendCommand({
    .position = 1.57f,    // 目标位置 (rad)
    .velocity = 0,        // 目标速度 (rad/s)
    .kp = 50.0f,          // 位置刚度
    .kd = 2.0f,           // 阻尼
    .torque = 0           // 前馈力矩 (N·m)
});

// 接收反馈
MitFeedback fb = mit.getFeedback();
// fb.position, fb.velocity, fb.torque
```

## 8.2 Modbus 协议

### Modbus RTU

```cpp
#include <omni/protocol/modbus_rtu.hpp>

ModbusRTU modbus(&uart, 0x01);  // 从站地址 = 1

// 配置寄存器映射
modbus.mapHoldingRegister(0x0000, &controlWord);
modbus.mapHoldingRegister(0x0001, &targetPosition);
modbus.mapInputRegister(0x0000, &statusWord);
modbus.mapInputRegister(0x0001, &actualPosition);

// 处理接收数据
void UART_RxCallback(uint8_t* data, size_t len) {
    modbus.processRequest(data, len);
}
```

### 寄存器映射表

| 地址 | 类型 | 描述 | 读写 |
|------|------|------|------|
| 0x0000 | Holding | 控制字 | R/W |
| 0x0001 | Holding | 目标位置高16位 | R/W |
| 0x0002 | Holding | 目标位置低16位 | R/W |
| 0x0003 | Holding | 目标速度 | R/W |
| 0x0010 | Input | 状态字 | R |
| 0x0011 | Input | 实际位置高16位 | R |
| 0x0012 | Input | 实际位置低16位 | R |
| 0x0013 | Input | 实际速度 | R |
| 0x0014 | Input | 实际电流 | R |

---

# 第九章 实战案例

## 9.1 步进电机闭环控制

使用 TMC2209 驱动 + MT6816 磁编码器实现步进电机闭环控制。

### 硬件连接

```
STM32G431
├── PA0  ──▶ TMC2209 STEP
├── PA1  ──▶ TMC2209 DIR
├── PA2  ──▶ TMC2209 EN
├── PA3  ◀──▶ TMC2209 UART
├── PB3  ──▶ MT6816 CLK (SPI)
├── PB4  ◀── MT6816 MISO
└── PB5  ──▶ MT6816 CS
```

### 代码实现

```cpp
#include "motor_hal.hpp"
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

// HAL 对象
Stm32Gpio stepPin(GPIOA, GPIO_PIN_0);
Stm32Gpio dirPin(GPIOA, GPIO_PIN_1);
Stm32Gpio enablePin(GPIOA, GPIO_PIN_2);
MT6816Encoder encoder(&hspi1, GPIOB, GPIO_PIN_5);

// 驱动和控制器
StepperDriver stepper(&stepPin, &dirPin, &encoder, &enablePin);
MotorController motor(&stepper);

void setup() {
    // 配置步进驱动
    stepper.setStepsPerRev(200);
    stepper.setMicrostep(64);
    stepper.init();

    // 配置控制器
    motor.setConstraints({10.0f, 50.0f, 500.0f});
    motor.setPositionPid(100.0f, 0, 10.0f);
    motor.enable();
}

void loop() {
    motor.moveTo(6.28f);   // 转1圈
    while (motor.isBusy()) delay(10);

    motor.moveTo(0);       // 回到原点
    while (motor.isBusy()) delay(10);
}

// 1kHz 控制中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    encoder.update(0.001f);
    motor.update(0.001f);
    stepper.update();
}
```

## 9.2 双轴云台

使用两个步进电机实现俯仰(Pitch)和偏航(Yaw)控制。

### 系统架构

```
                    ┌─────────────┐
                    │   IMU       │
                    │  (MPU6050)  │
                    └──────┬──────┘
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
       ┌─────────────┐          ┌─────────────┐
       │ Pitch Motor │          │ Yaw Motor   │
       │ (步进+编码器)│          │ (步进+编码器)│
       └─────────────┘          └─────────────┘
```

### 代码实现

```cpp
#include <omni/app/motor_controller.hpp>

// 两轴电机
MotorController pitchMotor(&pitchStepper);
MotorController yawMotor(&yawStepper);

// IMU 数据
float imuPitch, imuYaw;

void setup() {
    // 配置两轴
    pitchMotor.setConstraints({5.0f, 30.0f, 300.0f});
    yawMotor.setConstraints({5.0f, 30.0f, 300.0f});

    pitchMotor.setPositionPid(80.0f, 0, 8.0f);
    yawMotor.setPositionPid(80.0f, 0, 8.0f);

    pitchMotor.enable();
    yawMotor.enable();
}

// 稳定控制
void stabilize(float targetPitch, float targetYaw) {
    // 读取 IMU
    readIMU(&imuPitch, &imuYaw);

    // 计算补偿
    float pitchError = targetPitch - imuPitch;
    float yawError = targetYaw - imuYaw;

    // 设置目标
    pitchMotor.moveTo(pitchMotor.getPosition() + pitchError);
    yawMotor.moveTo(yawMotor.getPosition() + yawError);
}
```

## 9.3 3D打印机控制

三轴 XYZ 运动控制 + 挤出机。

### 硬件配置

```cpp
// 三轴步进电机
StepperDriver xStepper(&xStep, &xDir, nullptr, &xEn);
StepperDriver yStepper(&yStep, &yDir, nullptr, &yEn);
StepperDriver zStepper(&zStep, &zDir, nullptr, &zEn);
StepperDriver eStepper(&eStep, &eDir, nullptr, &eEn);

// 控制器
MotorController xMotor(&xStepper);
MotorController yMotor(&yStepper);
MotorController zMotor(&zStepper);
MotorController eMotor(&eStepper);

// 多轴控制器
MultiAxisController printer;
```

### 初始化

```cpp
void setup() {
    // 配置步进参数
    xStepper.setStepsPerRev(200);
    xStepper.setMicrostep(16);
    // ... 其他轴类似

    // 添加轴到多轴控制器
    printer.addAxis(&xMotor, "X");
    printer.addAxis(&yMotor, "Y");
    printer.addAxis(&zMotor, "Z");

    // 设置运动参数 (mm/s, mm/s²)
    xMotor.setConstraints({100.0f, 1000.0f, 10000.0f});
    yMotor.setConstraints({100.0f, 1000.0f, 10000.0f});
    zMotor.setConstraints({10.0f, 100.0f, 1000.0f});

    printer.enableAll();
}
```

### G代码处理

```cpp
void processGCode(const char* line) {
    char cmd;
    int code;
    float x, y, z, e, f;

    sscanf(line, "%c%d", &cmd, &code);

    if (cmd == 'G') {
        switch (code) {
            case 0:  // 快速移动
            case 1:  // 直线插补
                parseXYZEF(line, &x, &y, &z, &e, &f);
                printer.linearMove({x, y, z}, f / 60.0f);
                break;
            case 28: // 回零
                printer.homeAll();
                break;
        }
    }
}
```

## 9.4 机械臂关节

使用无刷电机 FOC 控制实现高性能关节。

### 硬件配置

```cpp
// 无刷电机驱动
BldcDriver joint(&pwm3Phase, &adcA, &adcB, &adcC, &encoder);

// 配置
joint.setPolePairs(7);
joint.setCurrentPid(0.5f, 10.0f, 0);
joint.setVelocityPid(5.0f, 0.5f, 0);
joint.setPositionPid(50.0f, 0, 5.0f);
```

### 控制模式

```cpp
// 位置控制（关节角度）
joint.setControlMode(ControlMode::Position);
joint.setPosition(1.57f);  // 90度

// 阻抗控制（柔顺控制）
void impedanceControl(float targetPos, float kp, float kd) {
    float pos = joint.getPosition();
    float vel = joint.getVelocity();
    float torque = kp * (targetPos - pos) - kd * vel;
    joint.setTorque(torque);
}

// 力矩控制（力控）
joint.setControlMode(ControlMode::Torque);
joint.setTorque(0.5f);  // 0.5 N·m
```

### MIT 协议通信

```cpp
// 作为从站响应主控命令
MitProtocol mit(&can, 0x01);

void CAN_RxCallback(uint32_t id, uint8_t* data) {
    MitCommand cmd = mit.parseCommand(data);

    // 执行阻抗控制
    float torque = cmd.kp * (cmd.position - joint.getPosition())
                 + cmd.kd * (cmd.velocity - joint.getVelocity())
                 + cmd.torque;

    joint.setTorque(torque);

    // 发送反馈
    mit.sendFeedback({
        .position = joint.getPosition(),
        .velocity = joint.getVelocity(),
        .torque = joint.getTorque()
    });
}
```

---

# 附录

## A. API 参考

### 驱动层接口

| 类 | 方法 | 描述 |
|----|------|------|
| IMotorDriver | init() | 初始化驱动 |
| | enable() | 使能电机 |
| | disable() | 禁用电机 |
| | setPosition(float) | 设置目标位置 |
| | setVelocity(float) | 设置目标速度 |
| | setTorque(float) | 设置目标力矩 |
| | getPosition() | 获取当前位置 |
| | getVelocity() | 获取当前速度 |
| | update() | 更新控制循环 |

### HAL 接口

| 接口 | 方法 | 描述 |
|------|------|------|
| IGpio | write(bool) | 写入电平 |
| | read() | 读取电平 |
| | toggle() | 翻转电平 |
| IPwm | setDuty(float) | 设置占空比 |
| | setFrequency(uint32_t) | 设置频率 |
| | enable(bool) | 使能/禁用 |
| IEncoder | getAngle() | 获取角度 |
| | getVelocity() | 获取速度 |
| | getCount() | 获取计数 |
| IAdc | read() | 读取电压 |
| | readRaw() | 读取原始值 |

## B. 常见问题

### Q1: 电机不转动

**检查项：**
1. 使能信号是否正确（EN引脚电平）
2. 电源是否正常
3. PWM是否输出（示波器检查）
4. 驱动芯片是否过热保护

```cpp
// 调试代码
if (!motor.isEnabled()) {
    printf("Motor not enabled\n");
}
if (driver.hasFault()) {
    printf("Fault: 0x%08X\n", driver.getErrorCode());
}
```

### Q2: 电机振动/啸叫

**原因：**
- PID参数不当（Kp过大）
- 控制频率过低
- 编码器噪声

**解决：**
```cpp
// 降低 Kp，增加 Kd
pid.setGains(5.0f, 0.1f, 0.5f);

// 增加滤波
LowPassFilter filter(100.0f);
float filteredPos = filter.update(encoder.getAngle(), dt);
```

### Q3: 位置跟踪误差大

**原因：**
- 缺少前馈
- 积分项不足
- 机械间隙

**解决：**
```cpp
// 添加前馈
ff.setVelocityGain(0.1f);
ff.setAccelerationGain(0.01f);

// 增加积分
pid.setGains(10.0f, 2.0f, 0.5f);
```

### Q4: FOC 电机抖动

**原因：**
- 电角度偏移未校准
- 电流采样时机不对
- 极对数设置错误

**解决：**
```cpp
// 校准换相偏移
bldc.calibrateCommutation();

// 确认极对数
bldc.setPolePairs(7);  // 根据电机规格
```

## C. 硬件参考设计

### 步进电机驱动板

```
电源输入: 12-48V DC
驱动芯片: TMC2209
最大电流: 2A RMS
接口: STEP/DIR/EN/UART
编码器: MT6816 (SPI)
MCU: STM32G431CBU6
```

### 无刷电机驱动板

```
电源输入: 12-48V DC
驱动芯片: DRV8301 + MOSFET
最大电流: 30A
电流采样: 分流电阻 + 运放
编码器: AS5047P (SPI)
MCU: STM32G474RET6
通信: CAN FD
```

### 推荐引脚配置 (STM32G431)

| 功能 | 引脚 | 外设 |
|------|------|------|
| PWM_A | PA8 | TIM1_CH1 |
| PWM_B | PA9 | TIM1_CH2 |
| PWM_C | PA10 | TIM1_CH3 |
| ADC_IA | PA0 | ADC1_IN1 |
| ADC_IB | PA1 | ADC1_IN2 |
| ENC_A | PA6 | TIM3_CH1 |
| ENC_B | PA7 | TIM3_CH2 |
| CAN_TX | PA12 | FDCAN1_TX |
| CAN_RX | PA11 | FDCAN1_RX |

---

**OmniMotion STM32 使用手册**

*"但凡用电机，皆可用此库"*

版权所有 © 2026
