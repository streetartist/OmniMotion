/**
 * @file stm32_timer.hpp
 * @brief STM32 定时器 HAL implementation
 */

#pragma once

#include <omni/hal/timer.hpp>

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
 * @brief STM32 定时器实现类
 *
 * 用于产生周期性中断，驱动控制循环
 *
 * 使用示例:
 * @code
 * extern TIM_HandleTypeDef htim6;
 *
 * Stm32Timer timer(&htim6);
 * timer.setFrequency(10000);  // 10kHz
 * timer.setCallback(controlLoop, nullptr);
 * timer.start();
 * @endcode
 */
class Stm32Timer : public hal::ITimer {
public:
    using Callback = void(*)(void* context);

    /**
     * @brief 构造函数
     * @param htim 定时器句柄
     */
    explicit Stm32Timer(TIM_HandleTypeDef* htim)
        : htim_(htim)
        , callback_(nullptr)
        , context_(nullptr)
        , running_(false)
        , frequency_(0)
    {
    }

    /**
     * @brief 设置定时器频率
     * @param freq_hz 频率 (Hz)
     */
    void setFrequency(uint32_t freq_hz) override {
        frequency_ = freq_hz;

        if (freq_hz == 0) return;

        uint32_t timerClock = getTimerClock();

        // 计算预分频和周期
        // 频率 = timerClock / ((PSC + 1) * (ARR + 1))
        uint32_t period = timerClock / freq_hz;

        uint32_t prescaler = 0;
        while (period > 65535 && prescaler < 65535) {
            prescaler++;
            period = timerClock / (freq_hz * (prescaler + 1));
        }

        // 更新定时器配置
        __HAL_TIM_SET_PRESCALER(htim_, prescaler);
        __HAL_TIM_SET_AUTORELOAD(htim_, period - 1);

        // 生成更新事件
        htim_->Instance->EGR = TIM_EGR_UG;
    }

    /**
     * @brief 设置定时器周期
     * @param period_us 周期 (微秒)
     */
    void setPeriod(uint32_t period_us) override {
        if (period_us > 0) {
            setFrequency(1000000 / period_us);
        }
    }

    /**
     * @brief 启动定时器
     */
    void start() override {
        if (running_) return;

        HAL_TIM_Base_Start_IT(htim_);
        running_ = true;
    }

    /**
     * @brief 停止定时器
     */
    void stop() override {
        if (!running_) return;

        HAL_TIM_Base_Stop_IT(htim_);
        running_ = false;
    }

    /**
     * @brief 是否正在运行
     */
    bool isRunning() const override {
        return running_;
    }

    /**
     * @brief 设置回调函数
     * @param callback 回调函数
     * @param context 用户上下文
     */
    void setCallback(Callback callback, void* context) {
        callback_ = callback;
        context_ = context;
    }

    /**
     * @brief 中断处理 (需要在 HAL_TIM_PeriodElapsedCallback 中调用)
     */
    void handleInterrupt() {
        if (callback_) {
            callback_(context_);
        }
    }

    /**
     * @brief 获取定时器句柄
     */
    TIM_HandleTypeDef* getHandle() const {
        return htim_;
    }

    /**
     * @brief 获取当前计数值
     */
    uint32_t getCount() const {
        return __HAL_TIM_GET_COUNTER(htim_);
    }

    /**
     * @brief 获取频率
     */
    uint32_t getFrequency() const {
        return frequency_;
    }

private:
    TIM_HandleTypeDef* htim_;
    Callback callback_;
    void* context_;
    bool running_;
    uint32_t frequency_;

    /**
     * @brief 获取定时器时钟
     */
    uint32_t getTimerClock() const {
        uint32_t pclk;
        uint32_t timerClock;

        // 判断定时器在哪条总线上
        // TIM1, TIM8, TIM9, TIM10, TIM11 在 APB2
        // 其他在 APB1
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
            pclk = HAL_RCC_GetPCLK2Freq();
            if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
                timerClock = pclk * 2;
            } else {
                timerClock = pclk;
            }
        } else {
            pclk = HAL_RCC_GetPCLK1Freq();
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
 * @brief 高精度计时器 (使用 DWT)
 *
 * 用于微秒/纳秒级计时
 */
class Stm32DwtTimer {
public:
    /**
     * @brief 初始化 DWT
     */
    static void init() {
        // 使能 DWT
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(STM32F7) || defined(STM32H7)
        // F7/H7 需要解锁 DWT
        DWT->LAR = 0xC5ACCE55;
#endif

        // 使能计数器
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    /**
     * @brief 获取当前周期数
     */
    static uint32_t getCycles() {
        return DWT->CYCCNT;
    }

    /**
     * @brief 获取系统核心时钟频率
     */
    static uint32_t getCoreClock() {
        return SystemCoreClock;
    }

    /**
     * @brief 周期数转微秒
     */
    static float cyclesToUs(uint32_t cycles) {
        return static_cast<float>(cycles) * 1e6f / SystemCoreClock;
    }

    /**
     * @brief 周期数转纳秒
     */
    static float cyclesToNs(uint32_t cycles) {
        return static_cast<float>(cycles) * 1e9f / SystemCoreClock;
    }

    /**
     * @brief 延时微秒
     */
    static void delayUs(uint32_t us) {
        uint32_t startCycles = DWT->CYCCNT;
        uint32_t delayCycles = us * (SystemCoreClock / 1000000);

        while ((DWT->CYCCNT - startCycles) < delayCycles) {
            // 等待
        }
    }

    /**
     * @brief 延时纳秒
     */
    static void delayNs(uint32_t ns) {
        uint32_t startCycles = DWT->CYCCNT;
        uint32_t delayCycles = ns * (SystemCoreClock / 1000000000);

        while ((DWT->CYCCNT - startCycles) < delayCycles) {
            // 等待
        }
    }
};

/**
 * @brief 秒表类
 *
 * 用于测量代码执行时间
 */
class Stm32Stopwatch {
public:
    Stm32Stopwatch() : startCycles_(0), running_(false) {}

    /**
     * @brief 开始计时
     */
    void start() {
        startCycles_ = DWT->CYCCNT;
        running_ = true;
    }

    /**
     * @brief 停止计时
     */
    void stop() {
        if (running_) {
            endCycles_ = DWT->CYCCNT;
            running_ = false;
        }
    }

    /**
     * @brief 获取经过的周期数
     */
    uint32_t getElapsedCycles() const {
        if (running_) {
            return DWT->CYCCNT - startCycles_;
        }
        return endCycles_ - startCycles_;
    }

    /**
     * @brief 获取经过的微秒数
     */
    float getElapsedUs() const {
        return Stm32DwtTimer::cyclesToUs(getElapsedCycles());
    }

    /**
     * @brief 获取经过的纳秒数
     */
    float getElapsedNs() const {
        return Stm32DwtTimer::cyclesToNs(getElapsedCycles());
    }

    /**
     * @brief 重置
     */
    void reset() {
        startCycles_ = 0;
        endCycles_ = 0;
        running_ = false;
    }

private:
    uint32_t startCycles_;
    uint32_t endCycles_;
    bool running_;
};

/**
 * @brief 周期性任务调度器
 *
 * 用于按固定频率执行多个任务
 */
class Stm32TaskScheduler {
public:
    struct Task {
        void (*callback)(void* context);
        void* context;
        uint32_t periodTicks;
        uint32_t lastRunTick;
        bool enabled;
    };

    static constexpr size_t MAX_TASKS = 8;

    Stm32TaskScheduler() : numTasks_(0) {}

    /**
     * @brief 添加任务
     * @param callback 回调函数
     * @param context 上下文
     * @param periodMs 周期 (毫秒)
     * @return 任务索引
     */
    int addTask(void (*callback)(void*), void* context, uint32_t periodMs) {
        if (numTasks_ >= MAX_TASKS) return -1;

        tasks_[numTasks_].callback = callback;
        tasks_[numTasks_].context = context;
        tasks_[numTasks_].periodTicks = periodMs;
        tasks_[numTasks_].lastRunTick = HAL_GetTick();
        tasks_[numTasks_].enabled = true;

        return numTasks_++;
    }

    /**
     * @brief 使能/禁用任务
     */
    void enableTask(int index, bool enable) {
        if (index >= 0 && index < static_cast<int>(numTasks_)) {
            tasks_[index].enabled = enable;
        }
    }

    /**
     * @brief 运行调度器 (在主循环中调用)
     */
    void run() {
        uint32_t currentTick = HAL_GetTick();

        for (size_t i = 0; i < numTasks_; i++) {
            if (!tasks_[i].enabled) continue;

            if ((currentTick - tasks_[i].lastRunTick) >= tasks_[i].periodTicks) {
                tasks_[i].lastRunTick = currentTick;
                if (tasks_[i].callback) {
                    tasks_[i].callback(tasks_[i].context);
                }
            }
        }
    }

private:
    Task tasks_[MAX_TASKS];
    size_t numTasks_;
};

/**
 * @brief 速率限制器
 *
 * 确保代码不会执行得太快
 */
class Stm32RateLimiter {
public:
    /**
     * @brief 构造函数
     * @param rateHz 目标频率 (Hz)
     */
    explicit Stm32RateLimiter(float rateHz)
        : periodUs_(1e6f / rateHz)
        , lastTick_(0)
    {
    }

    /**
     * @brief 等待直到可以执行
     * @return 实际经过的时间 (秒)
     */
    float wait() {
        uint32_t currentCycles = DWT->CYCCNT;
        uint32_t elapsed = currentCycles - lastTick_;
        float elapsedUs = Stm32DwtTimer::cyclesToUs(elapsed);

        if (elapsedUs < periodUs_) {
            uint32_t waitUs = static_cast<uint32_t>(periodUs_ - elapsedUs);
            Stm32DwtTimer::delayUs(waitUs);
            elapsedUs = periodUs_;
        }

        lastTick_ = DWT->CYCCNT;
        return elapsedUs * 1e-6f;
    }

    /**
     * @brief 检查是否可以执行 (非阻塞)
     * @return true=可以执行
     */
    bool ready() {
        uint32_t currentCycles = DWT->CYCCNT;
        uint32_t elapsed = currentCycles - lastTick_;
        float elapsedUs = Stm32DwtTimer::cyclesToUs(elapsed);

        if (elapsedUs >= periodUs_) {
            lastTick_ = currentCycles;
            return true;
        }
        return false;
    }

    /**
     * @brief 设置频率
     */
    void setRate(float rateHz) {
        periodUs_ = 1e6f / rateHz;
    }

private:
    float periodUs_;
    uint32_t lastTick_;
};

} // namespace stm32
} // namespace platform
} // namespace omni

// 定时器中断回调示例
// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim6) {
//         timer6.handleInterrupt();
//     }
// }
