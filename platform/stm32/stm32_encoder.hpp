/**
 * @file stm32_encoder.hpp
 * @brief STM32 编码器 HAL implementation
 */

#pragma once

#include <omni/hal/encoder.hpp>
#include <cmath>

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
 * @brief STM32 正交编码器实现类
 *
 * 使用定时器的编码器模式读取正交编码器
 *
 * 硬件连接:
 * - TIMx_CH1: 编码器 A 相
 * - TIMx_CH2: 编码器 B 相
 *
 * 使用示例:
 * @code
 * extern TIM_HandleTypeDef htim2;
 *
 * Stm32Encoder encoder(&htim2);
 * encoder.setResolution(4096);  // 4096 线编码器
 *
 * // 在控制循环中
 * encoder.update(0.001f);  // 更新速度计算
 * float angle = encoder.getAngle();
 * float velocity = encoder.getVelocity();
 * @endcode
 */
class Stm32Encoder : public hal::IEncoder {
public:
    /**
     * @brief 构造函数
     * @param htim 定时器句柄 (需配置为编码器模式)
     */
    explicit Stm32Encoder(TIM_HandleTypeDef* htim)
        : htim_(htim)
        , cpr_(4096)
        , invert_(false)
        , lastCount_(0)
        , totalCount_(0)
        , velocity_(0)
        , overflow_(0)
        , initialized_(false)
    {
    }

    /**
     * @brief 初始化编码器
     *
     * 配置定时器为编码器模式并启动
     */
    void init() {
        if (initialized_) return;

        // 配置编码器模式
        TIM_Encoder_InitTypeDef encoderConfig = {};
        encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC1Filter = 0x0F;  // 滤波
        encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        encoderConfig.IC2Filter = 0x0F;

        HAL_TIM_Encoder_Init(htim_, &encoderConfig);

        // 启动编码器
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);

        // 设置计数器初值为中间值，便于检测溢出
        __HAL_TIM_SET_COUNTER(htim_, 32768);
        lastCount_ = 32768;

        initialized_ = true;
    }

    /**
     * @brief 获取累计计数值
     * @return 编码器计数 (考虑溢出)
     */
    int32_t getCount() override {
        if (!initialized_) init();

        uint16_t currentCount = __HAL_TIM_GET_COUNTER(htim_);

        // 计算相对变化 (处理16位溢出)
        int16_t delta = static_cast<int16_t>(currentCount - lastCount_);

        // 更新总计数
        totalCount_ += delta;
        lastCount_ = currentCount;

        return invert_ ? -totalCount_ : totalCount_;
    }

    /**
     * @brief 重置计数器
     */
    void resetCount() override {
        __HAL_TIM_SET_COUNTER(htim_, 32768);
        lastCount_ = 32768;
        totalCount_ = 0;
        overflow_ = 0;
    }

    /**
     * @brief 获取角度 (0 ~ 2π)
     * @return 角度 (弧度)
     */
    float getAngle() override {
        int32_t count = getCount();

        // 计算在一圈内的位置
        int32_t countInRev = count % static_cast<int32_t>(cpr_);
        if (countInRev < 0) countInRev += cpr_;

        float angle = static_cast<float>(countInRev) / cpr_ * 2.0f * M_PI;
        return angle;
    }

    /**
     * @brief 获取绝对角度 (可以超过 2π)
     * @return 绝对角度 (弧度)
     */
    float getAbsoluteAngle() {
        int32_t count = getCount();
        return static_cast<float>(count) / cpr_ * 2.0f * M_PI;
    }

    /**
     * @brief 获取速度
     * @return 速度 (rad/s)
     */
    float getVelocity() override {
        return velocity_;
    }

    /**
     * @brief 更新速度计算 (需要定期调用)
     * @param dt 时间间隔 (秒)
     */
    void update(float dt) {
        static int32_t prevCount = 0;
        int32_t currentCount = getCount();

        if (dt > 0) {
            int32_t delta = currentCount - prevCount;
            float deltaAngle = static_cast<float>(delta) / cpr_ * 2.0f * M_PI;
            velocity_ = deltaAngle / dt;
        }

        prevCount = currentCount;
    }

    /**
     * @brief 设置编码器分辨率
     * @param cpr 每转脉冲数 (Count Per Revolution)
     */
    void setResolution(uint32_t cpr) override {
        cpr_ = cpr;
    }

    /**
     * @brief 获取分辨率
     */
    uint32_t getResolution() const override {
        return cpr_;
    }

    /**
     * @brief 设置方向
     * @param invert true=反向
     */
    void setDirection(bool invert) override {
        invert_ = invert;
    }

    /**
     * @brief 获取定时器句柄
     */
    TIM_HandleTypeDef* getHandle() const {
        return htim_;
    }

    /**
     * @brief 获取原始计数值 (不考虑溢出)
     */
    uint16_t getRawCount() const {
        return __HAL_TIM_GET_COUNTER(htim_);
    }

    /**
     * @brief 获取圈数
     */
    int32_t getRevolutions() const {
        return totalCount_ / static_cast<int32_t>(cpr_);
    }

    /**
     * @brief 设置计数值
     * @param count 计数值
     */
    void setCount(int32_t count) {
        totalCount_ = invert_ ? -count : count;
        lastCount_ = __HAL_TIM_GET_COUNTER(htim_);
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t cpr_;
    bool invert_;
    uint16_t lastCount_;
    int32_t totalCount_;
    float velocity_;
    int32_t overflow_;
    bool initialized_;
};

/**
 * @brief 带索引脉冲的编码器
 *
 * 支持 Z 相信号，用于绝对定位
 */
class Stm32EncoderWithIndex : public Stm32Encoder {
public:
    using IndexCallback = void(*)(void* context);

    /**
     * @brief 构造函数
     * @param htim 编码器定时器
     * @param indexPort Z 相端口
     * @param indexPin Z 相引脚
     */
    Stm32EncoderWithIndex(TIM_HandleTypeDef* htim,
                          GPIO_TypeDef* indexPort, uint16_t indexPin)
        : Stm32Encoder(htim)
        , indexPort_(indexPort)
        , indexPin_(indexPin)
        , indexCallback_(nullptr)
        , indexContext_(nullptr)
        , indexDetected_(false)
        , indexCount_(0)
    {
    }

    /**
     * @brief 配置索引中断
     */
    void configureIndexInterrupt(IndexCallback callback, void* context) {
        indexCallback_ = callback;
        indexContext_ = context;

        // 配置 GPIO 为中断模式
        GPIO_InitTypeDef init = {};
        init.Pin = indexPin_;
        init.Mode = GPIO_MODE_IT_RISING;
        init.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(indexPort_, &init);

        // 使能中断 (需要用户在 NVIC 中配置)
    }

    /**
     * @brief 索引中断处理 (由外部中断调用)
     */
    void handleIndexInterrupt() {
        indexDetected_ = true;
        indexCount_ = getCount();

        if (indexCallback_) {
            indexCallback_(indexContext_);
        }
    }

    /**
     * @brief 等待索引信号
     * @param timeout 超时时间 (ms)
     * @return true=检测到索引
     */
    bool waitForIndex(uint32_t timeout) {
        indexDetected_ = false;
        uint32_t start = HAL_GetTick();

        while (!indexDetected_) {
            if ((HAL_GetTick() - start) > timeout) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief 获取索引位置的计数值
     */
    int32_t getIndexCount() const {
        return indexCount_;
    }

    /**
     * @brief 是否检测到索引
     */
    bool isIndexDetected() const {
        return indexDetected_;
    }

    /**
     * @brief 清除索引标志
     */
    void clearIndexFlag() {
        indexDetected_ = false;
    }

    /**
     * @brief 在索引位置重置计数器
     */
    void resetAtIndex() {
        if (indexDetected_) {
            // 设置当前位置为0
            setCount(0);
            indexDetected_ = false;
        }
    }

private:
    GPIO_TypeDef* indexPort_;
    uint16_t indexPin_;
    IndexCallback indexCallback_;
    void* indexContext_;
    volatile bool indexDetected_;
    int32_t indexCount_;
};

/**
 * @brief SPI 绝对式编码器 (如 AS5047, MA730 等)
 */
class Stm32SpiEncoder : public hal::IEncoder {
public:
    Stm32SpiEncoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin)
        : hspi_(hspi)
        , csPort_(csPort)
        , csPin_(csPin)
        , cpr_(16384)  // 14位分辨率
        , invert_(false)
        , offset_(0)
        , lastAngle_(0)
        , totalAngle_(0)
        , velocity_(0)
    {
        // 初始化 CS 引脚为高电平
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
    }

    /**
     * @brief 读取原始角度值
     */
    uint16_t readRawAngle() {
        uint8_t txData[2] = {0xFF, 0xFF};  // 读取命令
        uint8_t rxData[2] = {0, 0};

        // CS 拉低
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);

        // SPI 传输
        HAL_SPI_TransmitReceive(hspi_, txData, rxData, 2, 10);

        // CS 拉高
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);

        // 解析数据 (根据具体编码器协议)
        uint16_t rawAngle = ((rxData[0] & 0x3F) << 8) | rxData[1];

        return rawAngle;
    }

    int32_t getCount() override {
        uint16_t rawAngle = readRawAngle();
        return static_cast<int32_t>(rawAngle) - static_cast<int32_t>(offset_);
    }

    void resetCount() override {
        offset_ = readRawAngle();
        totalAngle_ = 0;
    }

    float getAngle() override {
        uint16_t rawAngle = readRawAngle();
        float angle = static_cast<float>(rawAngle) / cpr_ * 2.0f * M_PI;

        // 处理跨越零点
        float delta = angle - lastAngle_;
        if (delta > M_PI) {
            delta -= 2.0f * M_PI;
        } else if (delta < -M_PI) {
            delta += 2.0f * M_PI;
        }
        totalAngle_ += delta;
        lastAngle_ = angle;

        return angle;
    }

    float getVelocity() override {
        return velocity_;
    }

    void update(float dt) {
        float currentAngle = getAngle();
        static float prevAngle = 0;

        if (dt > 0) {
            float delta = currentAngle - prevAngle;
            if (delta > M_PI) delta -= 2.0f * M_PI;
            if (delta < -M_PI) delta += 2.0f * M_PI;
            velocity_ = delta / dt;
        }

        prevAngle = currentAngle;
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

    /**
     * @brief 获取绝对角度 (多圈)
     */
    float getAbsoluteAngle() const {
        return totalAngle_;
    }

    /**
     * @brief 设置偏移 (用于编码器校准)
     */
    void setOffset(uint16_t offset) {
        offset_ = offset;
    }

private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
    uint32_t cpr_;
    bool invert_;
    uint16_t offset_;
    float lastAngle_;
    float totalAngle_;
    float velocity_;
};

} // namespace stm32
} // namespace platform
} // namespace omni
