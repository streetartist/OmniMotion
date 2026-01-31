# OmniMotion 平台移植指南

本指南介绍如何将 OmniMotion 移植到不同的硬件平台。

## 目录

1. [移植概述](#移植概述)
2. [HAL 层接口](#hal-层接口)
3. [STM32 移植](#stm32-移植)
4. [ESP32 移植](#esp32-移植)
5. [Linux 移植](#linux-移植)
6. [测试与验证](#测试与验证)

---

## 移植概述

### 架构回顾

OmniMotion 采用分层架构，只需要实现 HAL (硬件抽象层) 即可完成移植。

```
┌─────────────────────────────────────┐
│  应用层 / 运动层 / 控制层 / 协议层    │ ← 平台无关
├─────────────────────────────────────┤
│            驱动层                    │ ← 平台无关
├─────────────────────────────────────┤
│         HAL 层 (接口)                │ ← 平台无关（接口定义）
├─────────────────────────────────────┤
│       平台实现 (您需要实现)           │ ← 平台相关
├─────────────────────────────────────┤
│           硬件                       │
└─────────────────────────────────────┘
```

### 需要实现的接口

| 接口 | 必要性 | 说明 |
|------|--------|------|
| `IGpio` | 必须 | GPIO 控制 |
| `IPwm` | 必须 | PWM 输出 |
| `IPwm3Phase` | FOC时需要 | 三相 PWM |
| `IAdc` | 闭环时需要 | ADC 采样 |
| `IEncoder` | 闭环时需要 | 编码器读取 |
| `ICan` | CAN电机需要 | CAN 通信 |
| `ISpi` | 部分传感器需要 | SPI 通信 |
| `IUart` | 部分协议需要 | UART 通信 |
| `ITimer` | 建议实现 | 定时器 |

---

## HAL 层接口

### IGpio 接口

```cpp
namespace omni::hal {

enum class PinMode {
    Input,
    Output,
    PushPull,
    OpenDrain,
    PullUp,
    PullDown
};

class IGpio {
public:
    virtual ~IGpio() = default;

    // 设置引脚模式
    virtual void setMode(PinMode mode) = 0;

    // 写入电平
    virtual void write(bool value) = 0;

    // 读取电平
    virtual bool read() = 0;

    // 翻转电平
    virtual void toggle() = 0;
};

}
```

### IPwm 接口

```cpp
namespace omni::hal {

class IPwm {
public:
    virtual ~IPwm() = default;

    // 设置频率 (Hz)
    virtual void setFrequency(uint32_t freq_hz) = 0;

    // 设置占空比 (0.0 ~ 1.0)
    virtual void setDuty(float duty) = 0;

    // 设置高电平时间 (纳秒)
    virtual void setDutyNs(uint32_t ns) = 0;

    // 使能/禁用
    virtual void enable(bool en) = 0;

    // 设置死区时间 (纳秒)
    virtual void setDeadTime(uint32_t ns) = 0;

    // 获取当前设置
    virtual uint32_t getFrequency() const = 0;
    virtual float getDuty() const = 0;
};

}
```

### IPwm3Phase 接口

```cpp
namespace omni::hal {

class IPwm3Phase {
public:
    virtual ~IPwm3Phase() = default;

    // 设置三相占空比
    virtual void setDuty(float a, float b, float c) = 0;

    // 使能/禁用
    virtual void enable(bool en) = 0;

    // 设置死区时间
    virtual void setDeadTime(uint32_t ns) = 0;

    // SVPWM 输出 (可选优化)
    virtual void setSvpwm(float alpha, float beta) = 0;

    // 设置频率
    virtual void setFrequency(uint32_t freq_hz) = 0;
};

}
```

### IAdc 接口

```cpp
namespace omni::hal {

enum class AdcTrigger {
    Software,
    Timer,
    External
};

class IAdc {
public:
    virtual ~IAdc() = default;

    // 读取原始值
    virtual uint16_t read() = 0;

    // 读取电压
    virtual float readVoltage() = 0;

    // DMA 传输
    virtual void startDma(uint16_t* buffer, size_t len) = 0;
    virtual bool isDmaComplete() = 0;

    // 触发设置
    virtual void setTrigger(AdcTrigger trig) = 0;

    // 分辨率设置
    virtual void setResolution(uint8_t bits) = 0;

    // 采样时间
    virtual void setSampleTime(uint32_t cycles) = 0;
};

}
```

### IEncoder 接口

```cpp
namespace omni::hal {

class IEncoder {
public:
    virtual ~IEncoder() = default;

    // 获取累计计数
    virtual int32_t getCount() = 0;

    // 重置计数
    virtual void resetCount() = 0;

    // 获取角度 (0 ~ 2π)
    virtual float getAngle() = 0;

    // 获取速度 (rad/s)
    virtual float getVelocity() = 0;

    // 设置分辨率 (每转脉冲数)
    virtual void setResolution(uint32_t cpr) = 0;

    // 获取分辨率
    virtual uint32_t getResolution() const = 0;

    // 设置方向
    virtual void setDirection(bool invert) = 0;
};

}
```

### ICan 接口

```cpp
namespace omni::hal {

struct CanFrame {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    bool extended;
    bool rtr;
};

enum class CanMode {
    Normal,
    Loopback,
    Silent,
    SilentLoopback
};

class ICan {
public:
    virtual ~ICan() = default;

    // 发送帧
    virtual bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len) = 0;

    // 接收帧
    virtual bool receiveFrame(CanFrame& frame) = 0;

    // 设置过滤器
    virtual void setFilter(uint32_t id, uint32_t mask) = 0;

    // 设置波特率
    virtual void setBitrate(uint32_t bitrate) = 0;

    // 设置模式
    virtual void setMode(CanMode mode) = 0;

    // 检查是否有帧
    virtual bool isFrameAvailable() = 0;
};

}
```

---

## STM32 移植

### 目录结构

```
platform/stm32/
├── stm32_gpio.hpp
├── stm32_gpio.cpp
├── stm32_pwm.hpp
├── stm32_pwm.cpp
├── stm32_pwm3phase.hpp
├── stm32_pwm3phase.cpp
├── stm32_adc.hpp
├── stm32_adc.cpp
├── stm32_encoder.hpp
├── stm32_encoder.cpp
├── stm32_can.hpp
├── stm32_can.cpp
└── stm32_hal.hpp
```

### GPIO 实现

```cpp
// stm32_gpio.hpp
#pragma once

#include <omni/hal/gpio.hpp>
#include "stm32f4xx_hal.h"

namespace omni::platform::stm32 {

class Stm32Gpio : public hal::IGpio {
public:
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void setMode(hal::PinMode mode) override {
        GPIO_InitTypeDef init = {};
        init.Pin = pin_;

        switch (mode) {
            case hal::PinMode::Input:
                init.Mode = GPIO_MODE_INPUT;
                init.Pull = GPIO_NOPULL;
                break;
            case hal::PinMode::Output:
            case hal::PinMode::PushPull:
                init.Mode = GPIO_MODE_OUTPUT_PP;
                init.Pull = GPIO_NOPULL;
                init.Speed = GPIO_SPEED_FREQ_HIGH;
                break;
            case hal::PinMode::OpenDrain:
                init.Mode = GPIO_MODE_OUTPUT_OD;
                init.Pull = GPIO_NOPULL;
                init.Speed = GPIO_SPEED_FREQ_HIGH;
                break;
            case hal::PinMode::PullUp:
                init.Mode = GPIO_MODE_INPUT;
                init.Pull = GPIO_PULLUP;
                break;
            case hal::PinMode::PullDown:
                init.Mode = GPIO_MODE_INPUT;
                init.Pull = GPIO_PULLDOWN;
                break;
        }

        HAL_GPIO_Init(port_, &init);
    }

    void write(bool value) override {
        HAL_GPIO_WritePin(port_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

}
```

### PWM 实现

```cpp
// stm32_pwm.hpp
#pragma once

#include <omni/hal/pwm.hpp>
#include "stm32f4xx_hal.h"

namespace omni::platform::stm32 {

class Stm32Pwm : public hal::IPwm {
public:
    Stm32Pwm(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim), channel_(channel), frequency_(0), duty_(0) {}

    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

        // 计算分频和周期
        uint32_t timerClock = HAL_RCC_GetPCLK1Freq() * 2;  // APB1
        uint32_t period = timerClock / freq_hz;

        // 选择合适的预分频
        uint32_t prescaler = 0;
        while (period > 65535) {
            prescaler++;
            period = timerClock / (freq_hz * (prescaler + 1));
        }

        htim_->Init.Prescaler = prescaler;
        htim_->Init.Period = period - 1;
        HAL_TIM_PWM_Init(htim_);

        // 重新设置占空比
        setDuty(duty_);
    }

    void setDuty(float duty) override {
        duty_ = duty;
        if (duty < 0) duty = 0;
        if (duty > 1) duty = 1;

        uint32_t compare = (uint32_t)((htim_->Init.Period + 1) * duty);
        __HAL_TIM_SET_COMPARE(htim_, channel_, compare);
    }

    void setDutyNs(uint32_t ns) override {
        if (frequency_ == 0) return;
        float duty = (float)ns * frequency_ / 1e9f;
        setDuty(duty);
    }

    void enable(bool en) override {
        if (en) {
            HAL_TIM_PWM_Start(htim_, channel_);
        } else {
            HAL_TIM_PWM_Stop(htim_, channel_);
        }
    }

    void setDeadTime(uint32_t ns) override {
        // 需要高级定时器 (TIM1, TIM8)
        // 略
    }

    uint32_t getFrequency() const override { return frequency_; }
    float getDuty() const override { return duty_; }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    uint32_t frequency_;
    float duty_;
};

}
```

### 三相 PWM 实现

```cpp
// stm32_pwm3phase.hpp
#pragma once

#include <omni/hal/pwm.hpp>
#include "stm32f4xx_hal.h"

namespace omni::platform::stm32 {

class Stm32Pwm3Phase : public hal::IPwm3Phase {
public:
    Stm32Pwm3Phase(TIM_HandleTypeDef* htim)
        : htim_(htim), enabled_(false) {}

    void setDuty(float a, float b, float c) override {
        // 限幅
        a = a < 0 ? 0 : (a > 1 ? 1 : a);
        b = b < 0 ? 0 : (b > 1 ? 1 : b);
        c = c < 0 ? 0 : (c > 1 ? 1 : c);

        uint32_t period = htim_->Init.Period + 1;
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, (uint32_t)(period * a));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, (uint32_t)(period * b));
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, (uint32_t)(period * c));
    }

    void enable(bool en) override {
        if (en && !enabled_) {
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_3);
            enabled_ = true;
        } else if (!en && enabled_) {
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_3);
            enabled_ = false;
        }
    }

    void setDeadTime(uint32_t ns) override {
        // TIM1/TIM8 死区设置
        uint32_t timerClock = HAL_RCC_GetPCLK2Freq() * 2;
        uint32_t deadTicks = ns * timerClock / 1000000000UL;

        TIM_BreakDeadTimeConfigTypeDef config = {};
        config.DeadTime = deadTicks > 255 ? 255 : deadTicks;
        HAL_TIMEx_ConfigBreakDeadTime(htim_, &config);
    }

    void setSvpwm(float alpha, float beta) override {
        // SVPWM 计算
        float va, vb, vc;
        svpwmCalculate(alpha, beta, va, vb, vc);
        setDuty(va, vb, vc);
    }

    void setFrequency(uint32_t freq_hz) override {
        uint32_t timerClock = HAL_RCC_GetPCLK2Freq() * 2;
        uint32_t period = timerClock / freq_hz;

        htim_->Init.Prescaler = 0;
        htim_->Init.Period = period - 1;
        HAL_TIM_PWM_Init(htim_);
    }

private:
    void svpwmCalculate(float alpha, float beta, float& ta, float& tb, float& tc) {
        // SVPWM 算法实现
        float va = alpha;
        float vb = -0.5f * alpha + 0.866f * beta;
        float vc = -0.5f * alpha - 0.866f * beta;

        float vmin = std::min({va, vb, vc});
        float vmax = std::max({va, vb, vc});
        float voffset = (vmin + vmax) / 2.0f;

        ta = (va - voffset + 0.5f);
        tb = (vb - voffset + 0.5f);
        tc = (vc - voffset + 0.5f);
    }

    TIM_HandleTypeDef* htim_;
    bool enabled_;
};

}
```

### 编码器实现

```cpp
// stm32_encoder.hpp
#pragma once

#include <omni/hal/encoder.hpp>
#include "stm32f4xx_hal.h"

namespace omni::platform::stm32 {

class Stm32Encoder : public hal::IEncoder {
public:
    Stm32Encoder(TIM_HandleTypeDef* htim)
        : htim_(htim), cpr_(4096), invert_(false), lastCount_(0), velocity_(0) {
        // 配置为编码器模式
        TIM_Encoder_InitTypeDef config = {};
        config.EncoderMode = TIM_ENCODERMODE_TI12;
        config.IC1Polarity = TIM_ICPOLARITY_RISING;
        config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        config.IC1Prescaler = TIM_ICPSC_DIV1;
        config.IC1Filter = 0x0F;
        config.IC2Polarity = TIM_ICPOLARITY_RISING;
        config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        config.IC2Prescaler = TIM_ICPSC_DIV1;
        config.IC2Filter = 0x0F;
        HAL_TIM_Encoder_Init(htim_, &config);
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    }

    int32_t getCount() override {
        int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(htim_);
        if (invert_) count = -count;
        return count;
    }

    void resetCount() override {
        __HAL_TIM_SET_COUNTER(htim_, 0);
        lastCount_ = 0;
    }

    float getAngle() override {
        int32_t count = getCount();
        float angle = (float)(count % (int32_t)cpr_) / cpr_ * 2.0f * 3.14159f;
        if (angle < 0) angle += 2.0f * 3.14159f;
        return angle;
    }

    float getVelocity() override {
        return velocity_;
    }

    void setResolution(uint32_t cpr) override {
        cpr_ = cpr;
    }

    uint32_t getResolution() const override {
        return cpr_;
    }

    void setDirection(bool invert) override {
        invert_ = invert;
    }

    // 需要定期调用以计算速度
    void updateVelocity(float dt) {
        int32_t count = getCount();
        int32_t delta = count - lastCount_;
        lastCount_ = count;

        velocity_ = (float)delta / cpr_ * 2.0f * 3.14159f / dt;
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t cpr_;
    bool invert_;
    int32_t lastCount_;
    float velocity_;
};

}
```

### 使用示例

```cpp
#include "stm32_hal.hpp"
#include <omni/driver/bldc_driver.hpp>
#include <omni/app/motor_controller.hpp>

using namespace omni::platform::stm32;

// 硬件句柄 (由 CubeMX 生成)
extern TIM_HandleTypeDef htim1;   // PWM
extern TIM_HandleTypeDef htim2;   // 编码器
extern ADC_HandleTypeDef hadc1;

int main() {
    HAL_Init();
    SystemClock_Config();

    // 创建 HAL 对象
    Stm32Pwm3Phase pwm(&htim1);
    Stm32Encoder encoder(&htim2);
    Stm32Adc adcA(&hadc1, ADC_CHANNEL_0);
    Stm32Adc adcB(&hadc1, ADC_CHANNEL_1);
    Stm32Adc adcC(&hadc1, ADC_CHANNEL_2);

    // 创建 BLDC 驱动
    omni::driver::BldcDriver bldc(&pwm, &adcA, &adcB, &adcC, &encoder);

    // 配置电机
    omni::driver::MotorParams params;
    params.polePairs = 7;
    params.maxCurrent = 20.0f;
    params.encoderCpr = 4096;
    bldc.setParams(params);
    bldc.init();

    // 创建控制器
    omni::app::MotorController motor(&bldc);
    motor.setPositionPid(50.0f, 0.1f, 0.5f);
    motor.enable();

    // 移动到目标
    motor.moveWithSCurve(3.14159f, 10.0f, 100.0f, 1000.0f);

    // 主循环
    while (1) {
        motor.update(0.001f);
        encoder.updateVelocity(0.001f);
        HAL_Delay(1);
    }
}
```

---

## ESP32 移植

### 目录结构

```
platform/esp32/
├── esp32_gpio.hpp
├── esp32_pwm.hpp
├── esp32_adc.hpp
├── esp32_encoder.hpp
└── esp32_hal.hpp
```

### GPIO 实现

```cpp
// esp32_gpio.hpp
#pragma once

#include <omni/hal/gpio.hpp>
#include "driver/gpio.h"

namespace omni::platform::esp32 {

class Esp32Gpio : public hal::IGpio {
public:
    Esp32Gpio(gpio_num_t pin) : pin_(pin) {}

    void setMode(hal::PinMode mode) override {
        gpio_config_t config = {};
        config.pin_bit_mask = (1ULL << pin_);

        switch (mode) {
            case hal::PinMode::Input:
                config.mode = GPIO_MODE_INPUT;
                config.pull_up_en = GPIO_PULLUP_DISABLE;
                config.pull_down_en = GPIO_PULLDOWN_DISABLE;
                break;
            case hal::PinMode::Output:
            case hal::PinMode::PushPull:
                config.mode = GPIO_MODE_OUTPUT;
                break;
            case hal::PinMode::OpenDrain:
                config.mode = GPIO_MODE_OUTPUT_OD;
                break;
            case hal::PinMode::PullUp:
                config.mode = GPIO_MODE_INPUT;
                config.pull_up_en = GPIO_PULLUP_ENABLE;
                break;
            case hal::PinMode::PullDown:
                config.mode = GPIO_MODE_INPUT;
                config.pull_down_en = GPIO_PULLDOWN_ENABLE;
                break;
        }

        gpio_config(&config);
    }

    void write(bool value) override {
        gpio_set_level(pin_, value ? 1 : 0);
    }

    bool read() override {
        return gpio_get_level(pin_) == 1;
    }

    void toggle() override {
        int level = gpio_get_level(pin_);
        gpio_set_level(pin_, level ? 0 : 1);
    }

private:
    gpio_num_t pin_;
};

}
```

### PWM 实现 (使用 LEDC)

```cpp
// esp32_pwm.hpp
#pragma once

#include <omni/hal/pwm.hpp>
#include "driver/ledc.h"

namespace omni::platform::esp32 {

class Esp32Pwm : public hal::IPwm {
public:
    Esp32Pwm(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer)
        : pin_(pin), channel_(channel), timer_(timer), frequency_(0), duty_(0) {

        // 初始化定时器
        ledc_timer_config_t timerConfig = {};
        timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
        timerConfig.timer_num = timer_;
        timerConfig.duty_resolution = LEDC_TIMER_12_BIT;
        timerConfig.freq_hz = 20000;
        timerConfig.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&timerConfig);

        // 初始化通道
        ledc_channel_config_t channelConfig = {};
        channelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
        channelConfig.channel = channel_;
        channelConfig.timer_sel = timer_;
        channelConfig.gpio_num = pin_;
        channelConfig.duty = 0;
        channelConfig.hpoint = 0;
        ledc_channel_config(&channelConfig);

        frequency_ = 20000;
    }

    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, timer_, freq_hz);
    }

    void setDuty(float duty) override {
        duty_ = duty;
        if (duty < 0) duty = 0;
        if (duty > 1) duty = 1;

        uint32_t dutyValue = (uint32_t)(duty * 4095);  // 12-bit
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_, dutyValue);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_);
    }

    void setDutyNs(uint32_t ns) override {
        if (frequency_ == 0) return;
        float duty = (float)ns * frequency_ / 1e9f;
        setDuty(duty);
    }

    void enable(bool en) override {
        if (en) {
            setDuty(duty_);
        } else {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel_, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel_);
        }
    }

    void setDeadTime(uint32_t ns) override {
        // ESP32 LEDC 不直接支持死区，需要使用 MCPWM
    }

    uint32_t getFrequency() const override { return frequency_; }
    float getDuty() const override { return duty_; }

private:
    gpio_num_t pin_;
    ledc_channel_t channel_;
    ledc_timer_t timer_;
    uint32_t frequency_;
    float duty_;
};

}
```

### 编码器实现 (使用 PCNT)

```cpp
// esp32_encoder.hpp
#pragma once

#include <omni/hal/encoder.hpp>
#include "driver/pcnt.h"

namespace omni::platform::esp32 {

class Esp32Encoder : public hal::IEncoder {
public:
    Esp32Encoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit)
        : pinA_(pinA), pinB_(pinB), unit_(unit), cpr_(4096), invert_(false) {

        // 配置 PCNT
        pcnt_config_t config = {};
        config.pulse_gpio_num = pinA_;
        config.ctrl_gpio_num = pinB_;
        config.lctrl_mode = PCNT_MODE_REVERSE;
        config.hctrl_mode = PCNT_MODE_KEEP;
        config.pos_mode = PCNT_COUNT_INC;
        config.neg_mode = PCNT_COUNT_DEC;
        config.counter_h_lim = 32767;
        config.counter_l_lim = -32768;
        config.unit = unit_;
        config.channel = PCNT_CHANNEL_0;
        pcnt_unit_config(&config);

        // 配置第二个通道
        config.pulse_gpio_num = pinB_;
        config.ctrl_gpio_num = pinA_;
        config.channel = PCNT_CHANNEL_1;
        pcnt_unit_config(&config);

        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        pcnt_counter_resume(unit_);
    }

    int32_t getCount() override {
        int16_t count;
        pcnt_get_counter_value(unit_, &count);
        return invert_ ? -count : count;
    }

    void resetCount() override {
        pcnt_counter_clear(unit_);
    }

    float getAngle() override {
        int32_t count = getCount();
        float angle = (float)(count % (int32_t)cpr_) / cpr_ * 2.0f * 3.14159f;
        if (angle < 0) angle += 2.0f * 3.14159f;
        return angle;
    }

    float getVelocity() override {
        // 需要外部计算
        return 0;
    }

    void setResolution(uint32_t cpr) override { cpr_ = cpr; }
    uint32_t getResolution() const override { return cpr_; }
    void setDirection(bool invert) override { invert_ = invert; }

private:
    gpio_num_t pinA_, pinB_;
    pcnt_unit_t unit_;
    uint32_t cpr_;
    bool invert_;
};

}
```

---

## Linux 移植

Linux 平台可以用于仿真测试或 Raspberry Pi 等嵌入式 Linux。

### 仿真 GPIO

```cpp
// linux_gpio.hpp
#pragma once

#include <omni/hal/gpio.hpp>

namespace omni::platform::linux_sim {

class SimGpio : public hal::IGpio {
public:
    SimGpio(const char* name) : name_(name), mode_(hal::PinMode::Input), value_(false) {}

    void setMode(hal::PinMode mode) override {
        mode_ = mode;
        printf("[GPIO %s] Mode set to %d\n", name_, (int)mode);
    }

    void write(bool value) override {
        value_ = value;
        printf("[GPIO %s] Write: %d\n", name_, value);
    }

    bool read() override {
        return value_;
    }

    void toggle() override {
        value_ = !value_;
        printf("[GPIO %s] Toggle: %d\n", name_, value_);
    }

private:
    const char* name_;
    hal::PinMode mode_;
    bool value_;
};

}
```

### Raspberry Pi GPIO (使用 pigpio)

```cpp
// rpi_gpio.hpp
#pragma once

#include <omni/hal/gpio.hpp>
#include <pigpio.h>

namespace omni::platform::rpi {

class RpiGpio : public hal::IGpio {
public:
    RpiGpio(int pin) : pin_(pin) {}

    void setMode(hal::PinMode mode) override {
        switch (mode) {
            case hal::PinMode::Input:
                gpioSetMode(pin_, PI_INPUT);
                gpioSetPullUpDown(pin_, PI_PUD_OFF);
                break;
            case hal::PinMode::Output:
            case hal::PinMode::PushPull:
                gpioSetMode(pin_, PI_OUTPUT);
                break;
            case hal::PinMode::PullUp:
                gpioSetMode(pin_, PI_INPUT);
                gpioSetPullUpDown(pin_, PI_PUD_UP);
                break;
            case hal::PinMode::PullDown:
                gpioSetMode(pin_, PI_INPUT);
                gpioSetPullUpDown(pin_, PI_PUD_DOWN);
                break;
        }
    }

    void write(bool value) override {
        gpioWrite(pin_, value ? 1 : 0);
    }

    bool read() override {
        return gpioRead(pin_) == 1;
    }

    void toggle() override {
        write(!read());
    }

private:
    int pin_;
};

}
```

---

## 测试与验证

### 单元测试

```cpp
#include <gtest/gtest.h>
#include "platform/linux/linux_hal.hpp"

TEST(GpioTest, BasicIO) {
    omni::platform::linux_sim::SimGpio gpio("TEST");

    gpio.setMode(omni::hal::PinMode::Output);
    gpio.write(true);
    EXPECT_TRUE(gpio.read());

    gpio.toggle();
    EXPECT_FALSE(gpio.read());
}
```

### 硬件测试

```cpp
// 测试 PWM 输出
void testPwm(hal::IPwm* pwm) {
    pwm->setFrequency(20000);  // 20kHz

    for (float duty = 0; duty <= 1.0f; duty += 0.1f) {
        pwm->setDuty(duty);
        printf("Duty: %.1f%%\n", duty * 100);
        delay_ms(100);
    }
}

// 测试编码器
void testEncoder(hal::IEncoder* encoder) {
    encoder->resetCount();

    while (true) {
        printf("Count: %d, Angle: %.2f rad\n",
               encoder->getCount(), encoder->getAngle());
        delay_ms(100);
    }
}
```

### 完整验证流程

```
1. 单元测试
   └── 测试 HAL 接口基本功能

2. 集成测试
   └── 测试驱动层与 HAL 配合

3. 硬件测试
   ├── GPIO: LED 闪烁
   ├── PWM: 示波器观察波形
   ├── ADC: 读取已知电压
   ├── 编码器: 手动转动观察计数
   └── CAN: 回环测试

4. 电机测试
   ├── 开环测试: 占空比控制
   ├── 闭环测试: 位置/速度控制
   └── 轨迹测试: S曲线运动
```

