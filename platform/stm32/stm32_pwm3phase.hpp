/**
 * @file stm32_pwm3phase.hpp
 * @brief STM32 三相 PWM HAL implementation (用于 FOC 控制)
 */

#pragma once

#include <omni/hal/pwm.hpp>
#include <cmath>
#include <algorithm>

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
 * @brief STM32 三相 PWM 实现类
 *
 * 使用高级定时器 (TIM1/TIM8) 输出三相互补 PWM，支持死区和 SVPWM
 *
 * 硬件连接:
 * - TIM1_CH1 / TIM1_CH1N : A 相上桥 / 下桥
 * - TIM1_CH2 / TIM1_CH2N : B 相上桥 / 下桥
 * - TIM1_CH3 / TIM1_CH3N : C 相上桥 / 下桥
 *
 * 使用示例:
 * @code
 * extern TIM_HandleTypeDef htim1;
 *
 * Stm32Pwm3Phase pwm(&htim1);
 * pwm.setFrequency(20000);   // 20kHz PWM
 * pwm.setDeadTime(500);      // 500ns 死区
 * pwm.enable(true);
 *
 * // 设置三相占空比
 * pwm.setDuty(0.5f, 0.3f, 0.2f);
 *
 * // 或使用 SVPWM
 * pwm.setSvpwm(alpha, beta);
 * @endcode
 */
class Stm32Pwm3Phase : public hal::IPwm3Phase {
public:
    /**
     * @brief 构造函数
     * @param htim 高级定时器句柄 (TIM1 或 TIM8)
     */
    explicit Stm32Pwm3Phase(TIM_HandleTypeDef* htim)
        : htim_(htim)
        , enabled_(false)
        , frequency_(20000)
        , deadTimeNs_(0)
    {
    }

    /**
     * @brief 设置三相占空比
     * @param a A 相占空比 (0.0 ~ 1.0)
     * @param b B 相占空比 (0.0 ~ 1.0)
     * @param c C 相占空比 (0.0 ~ 1.0)
     */
    void setDuty(float a, float b, float c) override {
        // 限幅
        a = clamp(a, 0.0f, 1.0f);
        b = clamp(b, 0.0f, 1.0f);
        c = clamp(c, 0.0f, 1.0f);

        // 计算比较值
        uint32_t period = htim_->Init.Period + 1;
        uint32_t compareA = static_cast<uint32_t>(period * a);
        uint32_t compareB = static_cast<uint32_t>(period * b);
        uint32_t compareC = static_cast<uint32_t>(period * c);

        // 设置三个通道的比较值
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, compareA);
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, compareB);
        __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, compareC);
    }

    /**
     * @brief 使能/禁用三相 PWM 输出
     * @param en true=使能, false=禁用
     */
    void enable(bool en) override {
        if (en && !enabled_) {
            // 启动三相 PWM 和互补输出
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_3);

            enabled_ = true;
        } else if (!en && enabled_) {
            // 停止三相 PWM
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_3);

            enabled_ = false;
        }
    }

    /**
     * @brief 设置死区时间
     * @param ns 死区时间 (纳秒)
     */
    void setDeadTime(uint32_t ns) override {
        deadTimeNs_ = ns;

        uint32_t timerClock = getTimerClock();
        uint32_t deadTicks = static_cast<uint32_t>((uint64_t)ns * timerClock / 1000000000ULL);

        // DTG[7:0] 编码
        uint8_t dtg;
        if (deadTicks <= 127) {
            dtg = static_cast<uint8_t>(deadTicks);
        } else if (deadTicks <= 254) {
            dtg = 0x80 | static_cast<uint8_t>((deadTicks - 128) >> 1);
        } else if (deadTicks <= 504) {
            dtg = 0xC0 | static_cast<uint8_t>((deadTicks - 256) >> 3);
        } else if (deadTicks <= 1008) {
            dtg = 0xE0 | static_cast<uint8_t>((deadTicks - 512) >> 4);
        } else {
            dtg = 0xFF;
        }

        TIM_BreakDeadTimeConfigTypeDef config = {};
        config.OffStateRunMode = TIM_OSSR_ENABLE;
        config.OffStateIDLEMode = TIM_OSSI_ENABLE;
        config.LockLevel = TIM_LOCKLEVEL_OFF;
        config.DeadTime = dtg;
        config.BreakState = TIM_BREAK_DISABLE;
        config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;

        HAL_TIMEx_ConfigBreakDeadTime(htim_, &config);
    }

    /**
     * @brief 空间矢量 PWM (SVPWM)
     * @param alpha α 轴分量 (-1.0 ~ 1.0)
     * @param beta  β 轴分量 (-1.0 ~ 1.0)
     */
    void setSvpwm(float alpha, float beta) override {
        float ta, tb, tc;
        calculateSvpwm(alpha, beta, ta, tb, tc);
        setDuty(ta, tb, tc);
    }

    /**
     * @brief 设置 PWM 频率
     * @param freq_hz 频率 (Hz)
     */
    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

        if (freq_hz == 0) return;

        uint32_t timerClock = getTimerClock();

        // 中心对齐模式下，实际 PWM 频率 = timerClock / (2 * period)
        // 所以 period = timerClock / (2 * freq_hz)
        uint32_t period = timerClock / (2 * freq_hz);

        uint32_t prescaler = 0;
        while (period > 65535 && prescaler < 65535) {
            prescaler++;
            period = timerClock / (2 * freq_hz * (prescaler + 1));
        }

        // 配置定时器
        htim_->Init.Prescaler = prescaler;
        htim_->Init.Period = period - 1;
        htim_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
        htim_->Init.RepetitionCounter = 0;

        HAL_TIM_PWM_Init(htim_);

        // 重新配置死区
        if (deadTimeNs_ > 0) {
            setDeadTime(deadTimeNs_);
        }
    }

    /**
     * @brief 获取当前频率
     */
    uint32_t getFrequency() const {
        return frequency_;
    }

    /**
     * @brief 是否使能
     */
    bool isEnabled() const {
        return enabled_;
    }

    /**
     * @brief 获取定时器句柄
     */
    TIM_HandleTypeDef* getHandle() const {
        return htim_;
    }

    /**
     * @brief 设置三相 PWM 为低电平 (安全状态)
     */
    void setLow() {
        setDuty(0.0f, 0.0f, 0.0f);
    }

    /**
     * @brief 设置三相 PWM 为 50% (中点)
     */
    void setMid() {
        setDuty(0.5f, 0.5f, 0.5f);
    }

    /**
     * @brief 使能 ADC 触发
     * @param channel 触发通道 (TIM_CHANNEL_4 常用于触发)
     */
    void enableAdcTrigger(uint32_t channel = TIM_CHANNEL_4) {
        TIM_OC_InitTypeDef config = {};
        config.OCMode = TIM_OCMODE_PWM2;
        config.Pulse = htim_->Init.Period / 2;  // 在周期中点触发
        config.OCPolarity = TIM_OCPOLARITY_HIGH;
        config.OCFastMode = TIM_OCFAST_DISABLE;

        HAL_TIM_OC_ConfigChannel(htim_, &config, channel);
        HAL_TIM_OC_Start(htim_, channel);

        // 配置 TRGO 输出用于 ADC 触发
        TIM_MasterConfigTypeDef masterConfig = {};
        masterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
        masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(htim_, &masterConfig);
    }

private:
    TIM_HandleTypeDef* htim_;
    bool enabled_;
    uint32_t frequency_;
    uint32_t deadTimeNs_;

    /**
     * @brief 限幅函数
     */
    static float clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * @brief 获取定时器时钟
     */
    uint32_t getTimerClock() const {
        uint32_t pclk;
        uint32_t timerClock;

        // TIM1 和 TIM8 在 APB2 上
        pclk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            timerClock = pclk * 2;
        } else {
            timerClock = pclk;
        }

        return timerClock;
    }

    /**
     * @brief SVPWM 计算
     *
     * 将 αβ 坐标转换为三相占空比
     *
     * @param alpha α 分量
     * @param beta  β 分量
     * @param ta    A 相占空比 (输出)
     * @param tb    B 相占空比 (输出)
     * @param tc    C 相占空比 (输出)
     */
    void calculateSvpwm(float alpha, float beta, float& ta, float& tb, float& tc) {
        // 逆 Clarke 变换: αβ -> abc
        float va = alpha;
        float vb = -0.5f * alpha + 0.866025f * beta;
        float vc = -0.5f * alpha - 0.866025f * beta;

        // 计算最大和最小值
        float vmax = va;
        if (vb > vmax) vmax = vb;
        if (vc > vmax) vmax = vc;

        float vmin = va;
        if (vb < vmin) vmin = vb;
        if (vc < vmin) vmin = vc;

        // 计算零序分量 (使波形中心对称)
        float voffset = (vmax + vmin) / 2.0f;

        // 添加偏移并归一化到 [0, 1]
        ta = clamp((va - voffset) * 0.5f + 0.5f, 0.0f, 1.0f);
        tb = clamp((vb - voffset) * 0.5f + 0.5f, 0.0f, 1.0f);
        tc = clamp((vc - voffset) * 0.5f + 0.5f, 0.0f, 1.0f);
    }
};

/**
 * @brief 带电流采样同步的三相 PWM
 *
 * 自动配置 ADC 触发，实现电流采样与 PWM 同步
 */
class Stm32Pwm3PhaseWithAdcSync : public Stm32Pwm3Phase {
public:
    Stm32Pwm3PhaseWithAdcSync(TIM_HandleTypeDef* htim, ADC_HandleTypeDef* hadc)
        : Stm32Pwm3Phase(htim)
        , hadc_(hadc)
    {
    }

    /**
     * @brief 初始化 ADC 触发
     */
    void initAdcTrigger() {
        enableAdcTrigger(TIM_CHANNEL_4);

        // 配置 ADC 触发源
        // 需要根据具体 ADC 和定时器配置
    }

    /**
     * @brief 获取 ADC 句柄
     */
    ADC_HandleTypeDef* getAdcHandle() const {
        return hadc_;
    }

private:
    ADC_HandleTypeDef* hadc_;
};

} // namespace stm32
} // namespace platform
} // namespace omni
