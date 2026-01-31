/**
 * @file stm32_can.hpp
 * @brief STM32 CAN 总线 HAL implementation
 */

#pragma once

#include <omni/hal/comm.hpp>
#include <cstring>

#if defined(STM32F4)
    #include "stm32f4xx_hal.h"
#elif defined(STM32F1)
    #include "stm32f1xx_hal.h"
#elif defined(STM32F7)
    #include "stm32f7xx_hal.h"
#elif defined(STM32H7)
    #include "stm32h7xx_hal.h"
    #define USE_FDCAN
#elif defined(STM32G4)
    #include "stm32g4xx_hal.h"
    #define USE_FDCAN
#else
    #include "stm32f4xx_hal.h"
#endif

namespace omni {
namespace platform {
namespace stm32 {

#ifndef USE_FDCAN
/**
 * @brief STM32 CAN 实现类 (用于 F1/F4/F7 系列的 bxCAN)
 *
 * 使用示例:
 * @code
 * extern CAN_HandleTypeDef hcan1;
 *
 * Stm32Can can(&hcan1);
 * can.setBitrate(1000000);  // 1Mbps
 * can.init();
 *
 * // 发送
 * uint8_t data[] = {0x01, 0x02, 0x03};
 * can.sendFrame(0x100, data, 3);
 *
 * // 接收
 * hal::CanFrame frame;
 * if (can.receiveFrame(frame)) {
 *     // 处理接收到的帧
 * }
 * @endcode
 */
class Stm32Can : public hal::ICan {
public:
    /**
     * @brief 构造函数
     * @param hcan CAN 句柄
     */
    explicit Stm32Can(CAN_HandleTypeDef* hcan)
        : hcan_(hcan)
        , initialized_(false)
    {
    }

    /**
     * @brief 初始化 CAN
     */
    bool init() {
        if (initialized_) return true;

        // 启动 CAN
        if (HAL_CAN_Start(hcan_) != HAL_OK) {
            return false;
        }

        // 激活接收中断
        if (HAL_CAN_ActivateNotification(hcan_,
                CAN_IT_RX_FIFO0_MSG_PENDING |
                CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
            return false;
        }

        initialized_ = true;
        return true;
    }

    /**
     * @brief 发送 CAN 帧
     * @param id 帧 ID
     * @param data 数据
     * @param len 数据长度 (0-8)
     * @return true=发送成功
     */
    bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len) override {
        if (!initialized_) return false;

        CAN_TxHeaderTypeDef header = {};
        uint32_t mailbox;

        // 配置帧头
        if (id > 0x7FF) {
            header.ExtId = id;
            header.IDE = CAN_ID_EXT;
        } else {
            header.StdId = id;
            header.IDE = CAN_ID_STD;
        }
        header.RTR = CAN_RTR_DATA;
        header.DLC = len;
        header.TransmitGlobalTime = DISABLE;

        // 发送
        if (HAL_CAN_AddTxMessage(hcan_, &header, const_cast<uint8_t*>(data), &mailbox) != HAL_OK) {
            return false;
        }

        return true;
    }

    /**
     * @brief 接收 CAN 帧
     * @param frame 接收到的帧
     * @return true=接收成功
     */
    bool receiveFrame(hal::CanFrame& frame) override {
        if (!initialized_) return false;

        CAN_RxHeaderTypeDef header;
        uint8_t data[8];

        // 检查 FIFO0
        if (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0) > 0) {
            if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &header, data) == HAL_OK) {
                frame.id = (header.IDE == CAN_ID_EXT) ? header.ExtId : header.StdId;
                frame.len = header.DLC;
                frame.extended = (header.IDE == CAN_ID_EXT);
                frame.rtr = (header.RTR == CAN_RTR_REMOTE);
                std::memcpy(frame.data, data, header.DLC);
                return true;
            }
        }

        // 检查 FIFO1
        if (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO1) > 0) {
            if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO1, &header, data) == HAL_OK) {
                frame.id = (header.IDE == CAN_ID_EXT) ? header.ExtId : header.StdId;
                frame.len = header.DLC;
                frame.extended = (header.IDE == CAN_ID_EXT);
                frame.rtr = (header.RTR == CAN_RTR_REMOTE);
                std::memcpy(frame.data, data, header.DLC);
                return true;
            }
        }

        return false;
    }

    /**
     * @brief 设置接收过滤器
     * @param id 过滤 ID
     * @param mask 掩码
     */
    void setFilter(uint32_t id, uint32_t mask) override {
        CAN_FilterTypeDef filter = {};

        filter.FilterBank = 0;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = (id >> 13) & 0xFFFF;
        filter.FilterIdLow = ((id << 3) & 0xFFF8) | CAN_ID_STD;
        filter.FilterMaskIdHigh = (mask >> 13) & 0xFFFF;
        filter.FilterMaskIdLow = ((mask << 3) & 0xFFF8) | CAN_ID_STD;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation = ENABLE;
        filter.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(hcan_, &filter);
    }

    /**
     * @brief 配置过滤器接受所有帧
     */
    void setFilterAcceptAll() {
        CAN_FilterTypeDef filter = {};

        filter.FilterBank = 0;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = 0;
        filter.FilterIdLow = 0;
        filter.FilterMaskIdHigh = 0;
        filter.FilterMaskIdLow = 0;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterActivation = ENABLE;
        filter.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(hcan_, &filter);
    }

    /**
     * @brief 设置波特率
     * @param bitrate 波特率 (bps)
     */
    void setBitrate(uint32_t bitrate) override {
        // 波特率需要在 CubeMX 中配置或重新初始化 CAN
        // 这里提供常见配置参考

        // 假设 APB1 时钟为 45MHz (F4/F7) 或 36MHz (F1)
        // 波特率 = APB1_CLK / (Prescaler * (1 + BS1 + BS2))

        // 需要停止 CAN 才能重新配置
        HAL_CAN_Stop(hcan_);

        switch (bitrate) {
            case 1000000:  // 1Mbps
                hcan_->Init.Prescaler = 3;
                hcan_->Init.TimeSeg1 = CAN_BS1_12TQ;
                hcan_->Init.TimeSeg2 = CAN_BS2_2TQ;
                break;

            case 500000:   // 500kbps
                hcan_->Init.Prescaler = 6;
                hcan_->Init.TimeSeg1 = CAN_BS1_12TQ;
                hcan_->Init.TimeSeg2 = CAN_BS2_2TQ;
                break;

            case 250000:   // 250kbps
                hcan_->Init.Prescaler = 12;
                hcan_->Init.TimeSeg1 = CAN_BS1_12TQ;
                hcan_->Init.TimeSeg2 = CAN_BS2_2TQ;
                break;

            case 125000:   // 125kbps
                hcan_->Init.Prescaler = 24;
                hcan_->Init.TimeSeg1 = CAN_BS1_12TQ;
                hcan_->Init.TimeSeg2 = CAN_BS2_2TQ;
                break;
        }

        hcan_->Init.SyncJumpWidth = CAN_SJW_1TQ;

        HAL_CAN_Init(hcan_);
        setFilterAcceptAll();
        HAL_CAN_Start(hcan_);
    }

    /**
     * @brief 设置工作模式
     * @param mode CAN 模式
     */
    void setMode(hal::CanMode mode) override {
        HAL_CAN_Stop(hcan_);

        switch (mode) {
            case hal::CanMode::Normal:
                hcan_->Init.Mode = CAN_MODE_NORMAL;
                break;
            case hal::CanMode::Loopback:
                hcan_->Init.Mode = CAN_MODE_LOOPBACK;
                break;
            case hal::CanMode::Silent:
                hcan_->Init.Mode = CAN_MODE_SILENT;
                break;
            case hal::CanMode::SilentLoopback:
                hcan_->Init.Mode = CAN_MODE_SILENT_LOOPBACK;
                break;
        }

        HAL_CAN_Init(hcan_);
        HAL_CAN_Start(hcan_);
    }

    /**
     * @brief 检查是否有帧可读
     */
    bool isFrameAvailable() override {
        return (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0) > 0) ||
               (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO1) > 0);
    }

    /**
     * @brief 发送数据 (IComm 接口)
     */
    bool send(const uint8_t* data, size_t len) override {
        // 对于 CAN，这个接口不太适用
        return false;
    }

    /**
     * @brief 接收数据 (IComm 接口)
     */
    size_t receive(uint8_t* buffer, size_t maxLen) override {
        hal::CanFrame frame;
        if (receiveFrame(frame)) {
            size_t copyLen = (frame.len < maxLen) ? frame.len : maxLen;
            std::memcpy(buffer, frame.data, copyLen);
            return copyLen;
        }
        return 0;
    }

    /**
     * @brief 检查是否连接 (对于 CAN 总线始终返回 true)
     */
    bool isConnected() override {
        return initialized_;
    }

    /**
     * @brief 设置波特率 (IComm 接口)
     */
    void setBaudRate(uint32_t baud) override {
        setBitrate(baud);
    }

    /**
     * @brief 获取 CAN 句柄
     */
    CAN_HandleTypeDef* getHandle() const {
        return hcan_;
    }

    /**
     * @brief 获取发送邮箱空闲数量
     */
    uint32_t getFreeTxMailboxes() const {
        return HAL_CAN_GetTxMailboxesFreeLevel(hcan_);
    }

    /**
     * @brief 获取错误状态
     */
    uint32_t getError() const {
        return HAL_CAN_GetError(hcan_);
    }

private:
    CAN_HandleTypeDef* hcan_;
    bool initialized_;
};

#else  // USE_FDCAN (for H7, G4)

/**
 * @brief STM32 FDCAN 实现类 (用于 H7/G4 系列)
 */
class Stm32Can : public hal::ICan {
public:
    explicit Stm32Can(FDCAN_HandleTypeDef* hfdcan)
        : hfdcan_(hfdcan)
        , initialized_(false)
    {
    }

    bool init() {
        if (initialized_) return true;

        // 配置过滤器
        FDCAN_FilterTypeDef filter = {};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0;
        filter.FilterID2 = 0;

        if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter) != HAL_OK) {
            return false;
        }

        // 配置全局过滤器
        HAL_FDCAN_ConfigGlobalFilter(hfdcan_,
            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

        // 启动 FDCAN
        if (HAL_FDCAN_Start(hfdcan_) != HAL_OK) {
            return false;
        }

        // 激活接收中断
        HAL_FDCAN_ActivateNotification(hfdcan_,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

        initialized_ = true;
        return true;
    }

    bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len) override {
        if (!initialized_) return false;

        FDCAN_TxHeaderTypeDef header = {};

        if (id > 0x7FF) {
            header.Identifier = id;
            header.IdType = FDCAN_EXTENDED_ID;
        } else {
            header.Identifier = id;
            header.IdType = FDCAN_STANDARD_ID;
        }

        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = dlcToFdcanDlc(len);
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0;

        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header,
                const_cast<uint8_t*>(data)) != HAL_OK) {
            return false;
        }

        return true;
    }

    bool receiveFrame(hal::CanFrame& frame) override {
        if (!initialized_) return false;

        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];

        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
                frame.id = header.Identifier;
                frame.len = fdcanDlcToLen(header.DataLength);
                frame.extended = (header.IdType == FDCAN_EXTENDED_ID);
                frame.rtr = (header.RxFrameType == FDCAN_REMOTE_FRAME);
                std::memcpy(frame.data, data, frame.len);
                return true;
            }
        }

        return false;
    }

    void setFilter(uint32_t id, uint32_t mask) override {
        FDCAN_FilterTypeDef filter = {};
        filter.IdType = (id > 0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        filter.FilterIndex = 0;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = id;
        filter.FilterID2 = mask;

        HAL_FDCAN_ConfigFilter(hfdcan_, &filter);
    }

    void setBitrate(uint32_t bitrate) override {
        // FDCAN 波特率配置需要重新初始化
        // 具体参数根据时钟配置计算
    }

    void setMode(hal::CanMode mode) override {
        // FDCAN 模式配置
    }

    bool isFrameAvailable() override {
        return HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0;
    }

    bool send(const uint8_t* data, size_t len) override { return false; }
    size_t receive(uint8_t* buffer, size_t maxLen) override { return 0; }
    bool isConnected() override { return initialized_; }
    void setBaudRate(uint32_t baud) override { setBitrate(baud); }

    FDCAN_HandleTypeDef* getHandle() const { return hfdcan_; }

private:
    FDCAN_HandleTypeDef* hfdcan_;
    bool initialized_;

    static uint32_t dlcToFdcanDlc(uint8_t len) {
        switch (len) {
            case 0: return FDCAN_DLC_BYTES_0;
            case 1: return FDCAN_DLC_BYTES_1;
            case 2: return FDCAN_DLC_BYTES_2;
            case 3: return FDCAN_DLC_BYTES_3;
            case 4: return FDCAN_DLC_BYTES_4;
            case 5: return FDCAN_DLC_BYTES_5;
            case 6: return FDCAN_DLC_BYTES_6;
            case 7: return FDCAN_DLC_BYTES_7;
            default: return FDCAN_DLC_BYTES_8;
        }
    }

    static uint8_t fdcanDlcToLen(uint32_t dlc) {
        switch (dlc) {
            case FDCAN_DLC_BYTES_0: return 0;
            case FDCAN_DLC_BYTES_1: return 1;
            case FDCAN_DLC_BYTES_2: return 2;
            case FDCAN_DLC_BYTES_3: return 3;
            case FDCAN_DLC_BYTES_4: return 4;
            case FDCAN_DLC_BYTES_5: return 5;
            case FDCAN_DLC_BYTES_6: return 6;
            case FDCAN_DLC_BYTES_7: return 7;
            default: return 8;
        }
    }
};

#endif  // USE_FDCAN

/**
 * @brief CAN 接收缓冲区
 *
 * 用于中断接收模式下缓存接收到的帧
 */
template<size_t Size>
class CanRxBuffer {
public:
    CanRxBuffer() : head_(0), tail_(0), count_(0) {}

    /**
     * @brief 压入帧
     */
    bool push(const hal::CanFrame& frame) {
        if (count_ >= Size) return false;

        buffer_[head_] = frame;
        head_ = (head_ + 1) % Size;
        count_++;
        return true;
    }

    /**
     * @brief 弹出帧
     */
    bool pop(hal::CanFrame& frame) {
        if (count_ == 0) return false;

        frame = buffer_[tail_];
        tail_ = (tail_ + 1) % Size;
        count_--;
        return true;
    }

    /**
     * @brief 是否为空
     */
    bool isEmpty() const { return count_ == 0; }

    /**
     * @brief 是否已满
     */
    bool isFull() const { return count_ >= Size; }

    /**
     * @brief 获取帧数量
     */
    size_t size() const { return count_; }

    /**
     * @brief 清空
     */
    void clear() {
        head_ = tail_ = count_ = 0;
    }

private:
    hal::CanFrame buffer_[Size];
    volatile size_t head_;
    volatile size_t tail_;
    volatile size_t count_;
};

} // namespace stm32
} // namespace platform
} // namespace omni
