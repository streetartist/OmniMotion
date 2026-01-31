/**
 * @file stm32_pwm.hpp
 * @brief STM32 PWM HAL implementation
 */

#pragma once

#include <omni/hal/pwm.hpp>

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
    #include "stm32f4xx_hal.h"
#endif

namespace omni {
namespace platform {
namespace stm32 {

/**
 * @brief STM32 PWM 实现类
 *
 * 使用示例:
 * @code
 * // htim1 需要先用 CubeMX 配置为 PWM 模式
 * extern TIM_HandleTypeDef htim1;
 *
 * Stm32Pwm pwm(&htim1, TIM_CHANNEL_1);
 * pwm.setFrequency(20000);  // 20kHz
 * pwm.setDuty(0.5f);        // 50% 占空比
 * pwm.enable(true);
 * @endcode
 */
class Stm32Pwm : public hal::IPwm {
public:
    /**
     * @brief 构造函数
     * @param htim 定时器句柄
     * @param channel 通道 (TIM_CHANNEL_1, TIM_CHANNEL_2, ...)
     */
    Stm32Pwm(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim)
        , channel_(channel)
        , frequency_(0)
        , duty_(0)
        , enabled_(false)
    {
    }

    /**
     * @brief 设置 PWM 频率
     * @param freq_hz 频率 (Hz)
     */
    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

        if (freq_hz == 0) return;

        // 获取定时器时钟
        uint32_t timerClock = getTimerClock();

        // 计算分频和周期
        uint32_t period = timerClock / freq_hz;

        // 选择合适的预分频器
        uint32_t prescaler = 0;
        while (period > 65535 && prescaler < 65535) {
            prescaler++;
            period = timerClock / (freq_hz * (prescaler + 1));
        }

        // 更新定时器配置
        __HAL_TIM_SET_PRESCALER(htim_, prescaler);
        __HAL_TIM_SET_AUTORELOAD(htim_, period - 1);

        // 生成更新事件以应用新配置
        htim_->Instance->EGR = TIM_EGR_UG;

        // 保存配置
        htim_->Init.Prescaler = prescaler;
        htim_->Init.Period = period - 1;

        // 重新设置占空比
        setDuty(duty_);
    }

    /**
     * @brief 设置占空比
     * @param duty 占空比 (0.0 ~ 1.0)
     */
    void setDuty(float duty) override {
        duty_ = duty;

        // 限幅
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;

        // 计算比较值
        uint32_t period = htim_->Init.Period + 1;
        uint32_t compare = static_cast<uint32_t>(period * duty);

        // 设置比较值
        __HAL_TIM_SET_COMPARE(htim_, channel_, compare);
    }

    /**
     * @brief 设置高电平时间 (纳秒)
     * @param ns 高电平持续时间
     */
    void setDutyNs(uint32_t ns) override {
        if (frequency_ == 0) return;

        // 将纳秒转换为占空比
        float periodNs = 1e9f / frequency_;
        float duty = static_cast<float>(ns) / periodNs;
        setDuty(duty);
    }

    /**
     * @brief 使能/禁用 PWM 输出
     * @param en true=使能, false=禁用
     */
    void enable(bool en) override {
        if (en && !enabled_) {
            HAL_TIM_PWM_Start(htim_, channel_);
            enabled_ = true;
        } else if (!en && enabled_) {
            HAL_TIM_PWM_Stop(htim_, channel_);
            enabled_ = false;
        }
    }

    /**
     * @brief 设置死区时间
     * @param ns 死区时间 (纳秒)
     * @note 仅高级定时器 (TIM1, TIM8) 支持
     */
    void setDeadTime(uint32_t ns) override {
        // 只有高级定时器支持死区
        if (htim_->Instance != TIM1
#ifdef TIM8
            && htim_->Instance != TIM8
#endif
        ) {
            return;
        }

        uint32_t timerClock = getTimerClock();
        uint32_t deadTicks = static_cast<uint32_t>((uint64_t)ns * timerClock / 1000000000ULL);

        // 死区时间寄存器配置
        // DTG[7:0] 的编码比较复杂，这里简化处理
        uint8_t dtg;
        if (deadTicks <= 127) {
            dtg = deadTicks;
        } else if (deadTicks <= 254) {
            dtg = 0x80 | ((deadTicks - 128) >> 1);
        } else if (deadTicks <= 504) {
            dtg = 0xC0 | ((deadTicks - 256) >> 3);
        } else if (deadTicks <= 1008) {
            dtg = 0xE0 | ((deadTicks - 512) >> 4);
        } else {
            dtg = 0xFF; // 最大值
        }

        TIM_BreakDeadTimeConfigTypeDef config = {};
        config.OffStateRunMode = TIM_OSSR_DISABLE;
        config.OffStateIDLEMode = TIM_OSSI_DISABLE;
        config.LockLevel = TIM_LOCKLEVEL_OFF;
        config.DeadTime = dtg;
        config.BreakState = TIM_BREAK_DISABLE;
        config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

        HAL_TIMEx_ConfigBreakDeadTime(htim_, &config);
    }

    /**
     * @brief 获取当前频率
     */
    uint32_t getFrequency() const override {
        return frequency_;
    }

    /**
     * @brief 获取当前占空比
     */
    float getDuty() const override {
        return duty_;
    }

    /**
     * @brief 获取定时器句柄
     */
    TIM_HandleTypeDef* getHandle() const {
        return htim_;
    }

    /**
     * @brief 获取通道
     */
    uint32_t getChannel() const {
        return channel_;
    }

    /**
     * @brief 是否使能
     */
    bool isEnabled() const {
        return enabled_;
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    uint32_t frequency_;
    float duty_;
    bool enabled_;

    /**
     * @brief 获取定时器时钟频率
     */
    uint32_t getTimerClock() const {
        uint32_t pclk;
        uint32_t timerClock;

        // 判断定时器在哪条总线上
        if (htim_->Instance == TIM1
#ifdef TIM8
            || htim_->Instance == TIM8
#endif
#ifdef TIM9
            || htim_->Instance == TIM9
#endif
#ifdef TIM10
            || htim_->Instance == TIM10
#endif
#ifdef TIM11
            || htim_->Instance == TIM11
#endif
        ) {
            // APB2 定时器
            pclk = HAL_RCC_GetPCLK2Freq();
            // 如果 APB2 预分频不为 1，定时器时钟 = PCLK2 * 2
            if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
                timerClock = pclk * 2;
            } else {
                timerClock = pclk;
            }
        } else {
            // APB1 定时器
            pclk = HAL_RCC_GetPCLK1Freq();
            // 如果 APB1 预分频不为 1，定时器时钟 = PCLK1 * 2
            if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
                timerClock = pclk * 2;
            } else {
                timerClock = pclk;
            }
        }

        return timerClock;
    }
};

/**
 * @brief 互补 PWM 输出 (带死区)
 *
 * 用于 H 桥驱动等需要互补输出的场合
 */
class Stm32PwmComplementary : public Stm32Pwm {
public:
    Stm32PwmComplementary(TIM_HandleTypeDef* htim, uint32_t channel)
        : Stm32Pwm(htim, channel)
        , nEnabled_(false)
    {
    }

    /**
     * @brief 使能主通道和互补通道
     */
    void enable(bool en) override {
        if (en && !isEnabled()) {
            HAL_TIM_PWM_Start(getHandle(), getChannel());
            HAL_TIMEx_PWMN_Start(getHandle(), getChannel());
            nEnabled_ = true;
        } else if (!en && isEnabled()) {
            HAL_TIM_PWM_Stop(getHandle(), getChannel());
            HAL_TIMEx_PWMN_Stop(getHandle(), getChannel());
            nEnabled_ = false;
        }
        Stm32Pwm::enable(en);
    }

    /**
     * @brief 单独控制互补通道
     */
    void enableComplementary(bool en) {
        if (en && !nEnabled_) {
            HAL_TIMEx_PWMN_Start(getHandle(), getChannel());
            nEnabled_ = true;
        } else if (!en && nEnabled_) {
            HAL_TIMEx_PWMN_Stop(getHandle(), getChannel());
            nEnabled_ = false;
        }
    }

private:
    bool nEnabled_;
};

} // namespace stm32
} // namespace platform
} // namespace omni
