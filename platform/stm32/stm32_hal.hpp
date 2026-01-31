/**
 * @file stm32_hal.hpp
 * @brief STM32 HAL 主头文件
 *
 * 包含所有 STM32 平台的 HAL 实现
 */

#pragma once

// 包含所有 STM32 HAL 实现
#include "stm32_gpio.hpp"
#include "stm32_pwm.hpp"
#include "stm32_pwm3phase.hpp"
#include "stm32_adc.hpp"
#include "stm32_encoder.hpp"
#include "stm32_can.hpp"
#include "stm32_timer.hpp"

/**
 * @namespace omni::platform::stm32
 * @brief STM32 平台 HAL 实现
 *
 * 本命名空间包含所有 STM32 平台的硬件抽象层实现。
 *
 * ## 使用方法
 *
 * ### 1. 使用 CubeMX 配置硬件
 *
 * 首先使用 STM32CubeMX 生成基础配置代码：
 * - 配置时钟树
 * - 配置定时器 (PWM、编码器模式)
 * - 配置 ADC
 * - 配置 CAN
 * - 生成代码
 *
 * ### 2. 创建 HAL 对象
 *
 * ```cpp
 * #include "stm32_hal.hpp"
 *
 * using namespace omni::platform::stm32;
 *
 * // 假设 CubeMX 生成了以下句柄
 * extern TIM_HandleTypeDef htim1;   // PWM
 * extern TIM_HandleTypeDef htim2;   // 编码器
 * extern ADC_HandleTypeDef hadc1;   // ADC
 * extern CAN_HandleTypeDef hcan1;   // CAN
 *
 * // 创建 HAL 对象
 * Stm32Pwm3Phase pwm(&htim1);
 * Stm32Encoder encoder(&htim2);
 * Stm32Adc adcA(&hadc1, ADC_CHANNEL_0);
 * Stm32Adc adcB(&hadc1, ADC_CHANNEL_1);
 * Stm32Adc adcC(&hadc1, ADC_CHANNEL_2);
 * Stm32Can can(&hcan1);
 * ```
 *
 * ### 3. 创建电机驱动
 *
 * ```cpp
 * #include <omni/driver/bldc_driver.hpp>
 * #include <omni/app/motor_controller.hpp>
 *
 * // 创建 BLDC 驱动
 * omni::driver::BldcDriver bldc(&pwm, &adcA, &adcB, &adcC, &encoder);
 *
 * // 配置电机参数
 * omni::driver::MotorParams params;
 * params.polePairs = 7;
 * params.maxCurrent = 20.0f;
 * params.encoderCpr = 4096;
 * bldc.setParams(params);
 * bldc.init();
 *
 * // 创建高层控制器
 * omni::app::MotorController motor(&bldc);
 * motor.setPositionPid(50.0f, 0.1f, 0.5f);
 * motor.enable();
 * ```
 *
 * ### 4. 实现中断回调
 *
 * ```cpp
 * // 在 stm32f4xx_it.c 或类似文件中
 *
 * extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *     if (htim == &htim6) {
 *         // 控制循环定时器中断
 *         motor.update(0.0001f);  // 10kHz
 *         bldc.update();
 *     }
 * }
 *
 * extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
 *     if (hadc == &hadc1) {
 *         // 电流采样完成
 *     }
 * }
 *
 * extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
 *     if (hcan == &hcan1) {
 *         // CAN 接收中断
 *     }
 * }
 * ```
 *
 * ## 支持的 STM32 系列
 *
 * - STM32F1 系列
 * - STM32F4 系列
 * - STM32F7 系列
 * - STM32H7 系列 (使用 FDCAN)
 * - STM32G4 系列 (使用 FDCAN)
 *
 * ## 编译选项
 *
 * 需要在编译时定义 STM32 系列宏：
 * - `-DSTM32F4` 用于 F4 系列
 * - `-DSTM32F1` 用于 F1 系列
 * - `-DSTM32H7` 用于 H7 系列
 * - 等等
 *
 * ## 推荐配置
 *
 * ### BLDC FOC 控制推荐配置
 *
 * | 外设 | 配置 | 说明 |
 * |------|------|------|
 * | TIM1 | PWM 中心对齐 | 三相 PWM 输出 |
 * | TIM2 | 编码器模式 | 读取编码器 |
 * | ADC1 | 注入模式 | 电流采样 |
 * | TIM6 | 基本定时器 | 控制循环触发 |
 *
 * ### PWM 频率
 * - BLDC FOC: 10-20 kHz
 * - 步进电机: 可变
 *
 * ### 控制频率
 * - 电流环: 10-20 kHz
 * - 速度环: 1-5 kHz
 * - 位置环: 0.1-1 kHz
 */

namespace omni {
namespace platform {
namespace stm32 {

/**
 * @brief 初始化 DWT 计时器
 *
 * 应该在 main() 开始时调用
 */
inline void initDwt() {
    Stm32DwtTimer::init();
}

/**
 * @brief 延时微秒
 */
inline void delayUs(uint32_t us) {
    Stm32DwtTimer::delayUs(us);
}

/**
 * @brief 延时毫秒 (使用 HAL)
 */
inline void delayMs(uint32_t ms) {
    HAL_Delay(ms);
}

/**
 * @brief 获取系统时间 (毫秒)
 */
inline uint32_t getTickMs() {
    return HAL_GetTick();
}

/**
 * @brief 获取系统时间 (微秒)
 */
inline uint32_t getTickUs() {
    return Stm32DwtTimer::getCycles() / (SystemCoreClock / 1000000);
}

} // namespace stm32
} // namespace platform
} // namespace omni

/*
 * ============================================================================
 * 快速参考
 * ============================================================================
 *
 * GPIO:
 *   Stm32Gpio gpio(GPIOA, GPIO_PIN_5);
 *   gpio.setMode(PinMode::Output);
 *   gpio.write(true);
 *   gpio.toggle();
 *
 * PWM:
 *   Stm32Pwm pwm(&htim1, TIM_CHANNEL_1);
 *   pwm.setFrequency(20000);
 *   pwm.setDuty(0.5f);
 *   pwm.enable(true);
 *
 * 三相 PWM:
 *   Stm32Pwm3Phase pwm(&htim1);
 *   pwm.setFrequency(20000);
 *   pwm.setDeadTime(500);
 *   pwm.enable(true);
 *   pwm.setDuty(0.5f, 0.3f, 0.2f);
 *   pwm.setSvpwm(alpha, beta);
 *
 * ADC:
 *   Stm32Adc adc(&hadc1, ADC_CHANNEL_0);
 *   uint16_t raw = adc.read();
 *   float voltage = adc.readVoltage();
 *
 * 编码器:
 *   Stm32Encoder encoder(&htim2);
 *   encoder.setResolution(4096);
 *   encoder.init();
 *   float angle = encoder.getAngle();
 *   encoder.update(dt);  // 更新速度
 *   float velocity = encoder.getVelocity();
 *
 * CAN:
 *   Stm32Can can(&hcan1);
 *   can.setBitrate(1000000);
 *   can.init();
 *   can.sendFrame(0x100, data, 8);
 *   if (can.receiveFrame(frame)) { ... }
 *
 * 定时器:
 *   Stm32Timer timer(&htim6);
 *   timer.setFrequency(10000);
 *   timer.setCallback(callback, context);
 *   timer.start();
 *
 * DWT 计时:
 *   Stm32DwtTimer::init();
 *   Stm32Stopwatch sw;
 *   sw.start();
 *   // ... 代码 ...
 *   sw.stop();
 *   float us = sw.getElapsedUs();
 */
