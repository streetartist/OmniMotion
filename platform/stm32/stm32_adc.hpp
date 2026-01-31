/**
 * @file stm32_adc.hpp
 * @brief STM32 ADC HAL implementation
 */

#pragma once

#include <omni/hal/adc.hpp>

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
 * @brief STM32 ADC 实现类
 *
 * 支持单次转换、连续转换和 DMA 模式
 *
 * 使用示例:
 * @code
 * extern ADC_HandleTypeDef hadc1;
 *
 * Stm32Adc adc(&hadc1, ADC_CHANNEL_0);
 * adc.setResolution(12);
 *
 * uint16_t raw = adc.read();
 * float voltage = adc.readVoltage();
 * @endcode
 */
class Stm32Adc : public hal::IAdc {
public:
    /**
     * @brief 构造函数
     * @param hadc ADC 句柄
     * @param channel ADC 通道 (ADC_CHANNEL_0, ADC_CHANNEL_1, ...)
     * @param vref 参考电压 (默认 3.3V)
     */
    Stm32Adc(ADC_HandleTypeDef* hadc, uint32_t channel, float vref = 3.3f)
        : hadc_(hadc)
        , channel_(channel)
        , vref_(vref)
        , resolution_(12)
        , dmaComplete_(false)
    {
    }

    /**
     * @brief 读取原始 ADC 值
     * @return 原始采样值
     */
    uint16_t read() override {
        // 配置通道
        configureChannel();

        // 启动转换
        HAL_ADC_Start(hadc_);

        // 等待转换完成
        if (HAL_ADC_PollForConversion(hadc_, 10) == HAL_OK) {
            return static_cast<uint16_t>(HAL_ADC_GetValue(hadc_));
        }

        return 0;
    }

    /**
     * @brief 读取电压值
     * @return 电压 (V)
     */
    float readVoltage() override {
        uint16_t raw = read();
        uint32_t maxValue = (1 << resolution_) - 1;
        return (static_cast<float>(raw) / maxValue) * vref_;
    }

    /**
     * @brief 启动 DMA 传输
     * @param buffer 数据缓冲区
     * @param len 缓冲区长度
     */
    void startDma(uint16_t* buffer, size_t len) override {
        dmaComplete_ = false;
        HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(buffer), len);
    }

    /**
     * @brief 检查 DMA 是否完成
     * @return true=完成
     */
    bool isDmaComplete() override {
        return dmaComplete_;
    }

    /**
     * @brief 设置 DMA 完成标志 (由中断回调设置)
     */
    void setDmaComplete(bool complete) {
        dmaComplete_ = complete;
    }

    /**
     * @brief 设置触发源
     * @param trig 触发源
     */
    void setTrigger(hal::AdcTrigger trig) override {
        // 触发源配置需要重新初始化 ADC
        // 这里只是保存设置，实际配置需要用户通过 CubeMX 完成
        trigger_ = trig;
    }

    /**
     * @brief 设置分辨率
     * @param bits 分辨率位数 (6, 8, 10, 12)
     */
    void setResolution(uint8_t bits) override {
        resolution_ = bits;

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        uint32_t res;
        switch (bits) {
            case 6:
                res = ADC_RESOLUTION_6B;
                break;
            case 8:
                res = ADC_RESOLUTION_8B;
                break;
            case 10:
                res = ADC_RESOLUTION_10B;
                break;
            case 12:
            default:
                res = ADC_RESOLUTION_12B;
                break;
        }

        hadc_->Init.Resolution = res;
        HAL_ADC_Init(hadc_);
#endif
    }

    /**
     * @brief 设置采样时间
     * @param cycles 采样周期数
     */
    void setSampleTime(uint32_t cycles) override {
        sampleTime_ = cycles;
        // 采样时间在 configureChannel() 中应用
    }

    /**
     * @brief 获取 ADC 句柄
     */
    ADC_HandleTypeDef* getHandle() const {
        return hadc_;
    }

    /**
     * @brief 获取通道
     */
    uint32_t getChannel() const {
        return channel_;
    }

    /**
     * @brief 设置参考电压
     */
    void setVref(float vref) {
        vref_ = vref;
    }

    /**
     * @brief 获取参考电压
     */
    float getVref() const {
        return vref_;
    }

    /**
     * @brief 校准 ADC
     */
    void calibrate() {
#if defined(STM32F1)
        HAL_ADCEx_Calibration_Start(hadc_);
#elif defined(STM32G4) || defined(STM32H7)
        HAL_ADCEx_Calibration_Start(hadc_, ADC_SINGLE_ENDED);
#endif
    }

private:
    ADC_HandleTypeDef* hadc_;
    uint32_t channel_;
    float vref_;
    uint8_t resolution_;
    bool dmaComplete_;
    hal::AdcTrigger trigger_ = hal::AdcTrigger::Software;
    uint32_t sampleTime_ = ADC_SAMPLETIME_15CYCLES;

    /**
     * @brief 配置 ADC 通道
     */
    void configureChannel() {
        ADC_ChannelConfTypeDef config = {};
        config.Channel = channel_;
        config.Rank = 1;
        config.SamplingTime = getSampleTimeValue();

#if defined(STM32G4) || defined(STM32H7)
        config.SingleDiff = ADC_SINGLE_ENDED;
        config.OffsetNumber = ADC_OFFSET_NONE;
#endif

        HAL_ADC_ConfigChannel(hadc_, &config);
    }

    /**
     * @brief 获取采样时间值
     */
    uint32_t getSampleTimeValue() const {
#if defined(STM32F4) || defined(STM32F7)
        if (sampleTime_ <= 3) return ADC_SAMPLETIME_3CYCLES;
        if (sampleTime_ <= 15) return ADC_SAMPLETIME_15CYCLES;
        if (sampleTime_ <= 28) return ADC_SAMPLETIME_28CYCLES;
        if (sampleTime_ <= 56) return ADC_SAMPLETIME_56CYCLES;
        if (sampleTime_ <= 84) return ADC_SAMPLETIME_84CYCLES;
        if (sampleTime_ <= 112) return ADC_SAMPLETIME_112CYCLES;
        if (sampleTime_ <= 144) return ADC_SAMPLETIME_144CYCLES;
        return ADC_SAMPLETIME_480CYCLES;
#else
        return ADC_SAMPLETIME_15CYCLES;
#endif
    }
};

/**
 * @brief 多通道 ADC (用于电流采样等)
 *
 * 支持同时采样多个通道
 */
class Stm32AdcMultiChannel {
public:
    static constexpr size_t MAX_CHANNELS = 8;

    /**
     * @brief 构造函数
     * @param hadc ADC 句柄
     */
    explicit Stm32AdcMultiChannel(ADC_HandleTypeDef* hadc)
        : hadc_(hadc)
        , numChannels_(0)
        , dmaComplete_(false)
    {
    }

    /**
     * @brief 添加通道
     * @param channel ADC 通道
     * @return 通道索引
     */
    size_t addChannel(uint32_t channel) {
        if (numChannels_ >= MAX_CHANNELS) return numChannels_;

        channels_[numChannels_] = channel;
        return numChannels_++;
    }

    /**
     * @brief 配置所有通道
     */
    void configure() {
        for (size_t i = 0; i < numChannels_; i++) {
            ADC_ChannelConfTypeDef config = {};
            config.Channel = channels_[i];
            config.Rank = i + 1;
            config.SamplingTime = ADC_SAMPLETIME_15CYCLES;

#if defined(STM32G4) || defined(STM32H7)
            config.SingleDiff = ADC_SINGLE_ENDED;
            config.OffsetNumber = ADC_OFFSET_NONE;
#endif

            HAL_ADC_ConfigChannel(hadc_, &config);
        }

        // 配置扫描模式
        hadc_->Init.ScanConvMode = ENABLE;
        hadc_->Init.NbrOfConversion = numChannels_;
        HAL_ADC_Init(hadc_);
    }

    /**
     * @brief 启动 DMA 转换
     * @param buffer 缓冲区 (大小 = numChannels_)
     */
    void startDma(uint16_t* buffer) {
        dmaComplete_ = false;
        dmaBuffer_ = buffer;
        HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(buffer), numChannels_);
    }

    /**
     * @brief 启动连续 DMA 转换
     * @param buffer 缓冲区
     * @param numSamples 采样次数 (总数据量 = numChannels_ * numSamples)
     */
    void startContinuousDma(uint16_t* buffer, size_t numSamples) {
        dmaComplete_ = false;
        dmaBuffer_ = buffer;
        HAL_ADC_Start_DMA(hadc_, reinterpret_cast<uint32_t*>(buffer),
                          numChannels_ * numSamples);
    }

    /**
     * @brief 停止 DMA
     */
    void stopDma() {
        HAL_ADC_Stop_DMA(hadc_);
    }

    /**
     * @brief 检查 DMA 是否完成
     */
    bool isDmaComplete() const {
        return dmaComplete_;
    }

    /**
     * @brief 设置 DMA 完成标志
     */
    void setDmaComplete(bool complete) {
        dmaComplete_ = complete;
    }

    /**
     * @brief 获取指定通道的值
     * @param index 通道索引
     */
    uint16_t getValue(size_t index) const {
        if (index < numChannels_ && dmaBuffer_) {
            return dmaBuffer_[index];
        }
        return 0;
    }

    /**
     * @brief 获取通道数量
     */
    size_t getNumChannels() const {
        return numChannels_;
    }

    /**
     * @brief 获取 ADC 句柄
     */
    ADC_HandleTypeDef* getHandle() const {
        return hadc_;
    }

private:
    ADC_HandleTypeDef* hadc_;
    uint32_t channels_[MAX_CHANNELS];
    size_t numChannels_;
    uint16_t* dmaBuffer_ = nullptr;
    volatile bool dmaComplete_;
};

/**
 * @brief 注入通道 ADC (用于 FOC 电流采样)
 *
 * 注入通道可以在 PWM 触发时进行采样，实现与 PWM 同步的电流采样
 */
class Stm32AdcInjected {
public:
    static constexpr size_t MAX_INJECTED_CHANNELS = 4;

    explicit Stm32AdcInjected(ADC_HandleTypeDef* hadc)
        : hadc_(hadc)
        , numChannels_(0)
    {
    }

    /**
     * @brief 添加注入通道
     */
    size_t addChannel(uint32_t channel) {
        if (numChannels_ >= MAX_INJECTED_CHANNELS) return numChannels_;

        channels_[numChannels_] = channel;
        return numChannels_++;
    }

    /**
     * @brief 配置注入通道
     * @param trigger 触发源 (如 ADC_EXTERNALTRIGINJECCONV_T1_TRGO)
     */
    void configure(uint32_t trigger) {
        for (size_t i = 0; i < numChannels_; i++) {
            ADC_InjectionConfTypeDef config = {};
            config.InjectedChannel = channels_[i];
            config.InjectedRank = i + 1;
            config.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
            config.InjectedOffset = 0;
            config.InjectedNbrOfConversion = numChannels_;
            config.InjectedDiscontinuousConvMode = DISABLE;
            config.AutoInjectedConv = DISABLE;
            config.ExternalTrigInjecConv = trigger;
            config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

            HAL_ADCEx_InjectedConfigChannel(hadc_, &config);
        }
    }

    /**
     * @brief 启动注入转换
     */
    void start() {
        HAL_ADCEx_InjectedStart(hadc_);
    }

    /**
     * @brief 启动中断模式
     */
    void startIT() {
        HAL_ADCEx_InjectedStart_IT(hadc_);
    }

    /**
     * @brief 停止注入转换
     */
    void stop() {
        HAL_ADCEx_InjectedStop(hadc_);
    }

    /**
     * @brief 获取注入通道值
     * @param rank 注入排名 (1-4)
     */
    uint16_t getValue(uint32_t rank) const {
        return static_cast<uint16_t>(HAL_ADCEx_InjectedGetValue(hadc_, rank));
    }

    /**
     * @brief 获取所有通道值
     */
    void getValues(uint16_t* values) const {
        for (size_t i = 0; i < numChannels_; i++) {
            values[i] = static_cast<uint16_t>(
                HAL_ADCEx_InjectedGetValue(hadc_, i + 1));
        }
    }

private:
    ADC_HandleTypeDef* hadc_;
    uint32_t channels_[MAX_INJECTED_CHANNELS];
    size_t numChannels_;
};

} // namespace stm32
} // namespace platform
} // namespace omni

// ADC DMA 完成回调示例
// extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//     // 通知相应的 ADC 对象 DMA 完成
// }

// 注入转换完成回调示例
// extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
//     // 处理电流采样
// }
