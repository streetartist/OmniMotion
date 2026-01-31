/**
 * @file stm32_gpio.hpp
 * @brief STM32 GPIO HAL implementation
 */

#pragma once

#include <omni/hal/gpio.hpp>

// STM32 HAL 头文件 - 根据具体型号选择
#if defined(STM32F4)
    #include "stm32f4xx_hal.h"
#elif defined(STM32F1)
    #include "stm32f1xx_hal.h"
#elif defined(STM32F7)
    #include "stm32f7xx_hal.h"
#elif defined(STM32H7)
    #include "stm32h7xx_hal.h"
#elif defined(STM32G4)
    #include "stm32g4xx_hal.h"
#else
    // 默认使用 F4 系列，实际使用时需要定义正确的宏
    #include "stm32f4xx_hal.h"
#endif

namespace omni {
namespace platform {
namespace stm32 {

/**
 * @brief STM32 GPIO 实现类
 *
 * 使用示例:
 * @code
 * // 使用 CubeMX 生成的引脚定义
 * Stm32Gpio led(GPIOA, GPIO_PIN_5);
 * led.setMode(PinMode::Output);
 * led.write(true);  // LED 亮
 * led.toggle();     // LED 灭
 * @endcode
 */
class Stm32Gpio : public hal::IGpio {
public:
    /**
     * @brief 构造函数
     * @param port GPIO 端口 (GPIOA, GPIOB, ...)
     * @param pin  GPIO 引脚 (GPIO_PIN_0, GPIO_PIN_1, ...)
     */
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port)
        , pin_(pin)
        , mode_(hal::PinMode::Input)
    {
    }

    /**
     * @brief 设置引脚模式
     * @param mode 引脚模式
     */
    void setMode(hal::PinMode mode) override {
        mode_ = mode;

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

    /**
     * @brief 写入引脚电平
     * @param value true=高电平, false=低电平
     */
    void write(bool value) override {
        HAL_GPIO_WritePin(port_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /**
     * @brief 读取引脚电平
     * @return true=高电平, false=低电平
     */
    bool read() override {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

    /**
     * @brief 翻转引脚电平
     */
    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    /**
     * @brief 获取端口
     */
    GPIO_TypeDef* getPort() const { return port_; }

    /**
     * @brief 获取引脚
     */
    uint16_t getPin() const { return pin_; }

    /**
     * @brief 获取当前模式
     */
    hal::PinMode getMode() const { return mode_; }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
    hal::PinMode mode_;
};

/**
 * @brief GPIO 中断回调管理器
 *
 * 用于管理 GPIO 外部中断回调
 */
class GpioInterruptManager {
public:
    using Callback = void(*)(void* context);

    /**
     * @brief 获取单例实例
     */
    static GpioInterruptManager& instance() {
        static GpioInterruptManager inst;
        return inst;
    }

    /**
     * @brief 注册中断回调
     * @param pin 引脚号 (0-15)
     * @param callback 回调函数
     * @param context 用户上下文
     */
    void registerCallback(uint8_t pin, Callback callback, void* context) {
        if (pin < 16) {
            callbacks_[pin] = callback;
            contexts_[pin] = context;
        }
    }

    /**
     * @brief 取消注册中断回调
     * @param pin 引脚号
     */
    void unregisterCallback(uint8_t pin) {
        if (pin < 16) {
            callbacks_[pin] = nullptr;
            contexts_[pin] = nullptr;
        }
    }

    /**
     * @brief 处理中断 (由 HAL_GPIO_EXTI_Callback 调用)
     * @param pin 引脚号
     */
    void handleInterrupt(uint16_t pin) {
        for (uint8_t i = 0; i < 16; i++) {
            if (pin & (1 << i)) {
                if (callbacks_[i]) {
                    callbacks_[i](contexts_[i]);
                }
            }
        }
    }

private:
    GpioInterruptManager() {
        for (int i = 0; i < 16; i++) {
            callbacks_[i] = nullptr;
            contexts_[i] = nullptr;
        }
    }

    Callback callbacks_[16];
    void* contexts_[16];
};

/**
 * @brief 带中断功能的 GPIO
 */
class Stm32GpioInterrupt : public Stm32Gpio {
public:
    using Callback = void(*)(void* context);

    enum class Edge {
        Rising,
        Falling,
        Both
    };

    Stm32GpioInterrupt(GPIO_TypeDef* port, uint16_t pin)
        : Stm32Gpio(port, pin)
        , pinIndex_(getPinIndex(pin))
    {
    }

    /**
     * @brief 配置外部中断
     * @param edge 触发边沿
     * @param callback 回调函数
     * @param context 用户上下文
     */
    void enableInterrupt(Edge edge, Callback callback, void* context) {
        GPIO_InitTypeDef init = {};
        init.Pin = getPin();
        init.Pull = GPIO_NOPULL;

        switch (edge) {
            case Edge::Rising:
                init.Mode = GPIO_MODE_IT_RISING;
                break;
            case Edge::Falling:
                init.Mode = GPIO_MODE_IT_FALLING;
                break;
            case Edge::Both:
                init.Mode = GPIO_MODE_IT_RISING_FALLING;
                break;
        }

        HAL_GPIO_Init(getPort(), &init);

        // 注册回调
        GpioInterruptManager::instance().registerCallback(pinIndex_, callback, context);

        // 使能 NVIC 中断
        IRQn_Type irq = getIRQn();
        HAL_NVIC_SetPriority(irq, 5, 0);
        HAL_NVIC_EnableIRQ(irq);
    }

    /**
     * @brief 禁用中断
     */
    void disableInterrupt() {
        GpioInterruptManager::instance().unregisterCallback(pinIndex_);
    }

private:
    uint8_t pinIndex_;

    static uint8_t getPinIndex(uint16_t pin) {
        for (uint8_t i = 0; i < 16; i++) {
            if (pin == (1 << i)) return i;
        }
        return 0;
    }

    IRQn_Type getIRQn() const {
        switch (pinIndex_) {
            case 0: return EXTI0_IRQn;
            case 1: return EXTI1_IRQn;
            case 2: return EXTI2_IRQn;
            case 3: return EXTI3_IRQn;
            case 4: return EXTI4_IRQn;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9: return EXTI9_5_IRQn;
            default: return EXTI15_10_IRQn;
        }
    }
};

} // namespace stm32
} // namespace platform
} // namespace omni

// HAL 回调函数 - 需要在用户代码中实现
// extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     omni::platform::stm32::GpioInterruptManager::instance().handleInterrupt(GPIO_Pin);
// }
