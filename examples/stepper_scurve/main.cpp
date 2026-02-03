/**
 * @file main.cpp
 * @brief STM32 步进电机 S曲线运动控制示例
 *
 * 功能:
 * - STEP/DIR 接口步进驱动 (TMC2209, DRV8825, A4988等)
 * - S曲线平滑加减速
 * - 串口命令交互
 * - 轨迹预览
 *
 * 硬件配置 (CubeMX):
 * - GPIO: STEP(PA0), DIR(PA1), EN(PA2)
 * - TIM6: 控制循环定时器 (10kHz)
 * - USART2: 串口调试 (115200)
 *
 * 串口命令:
 * - g<rad>  : 移动到绝对位置 (弧度)
 * - r<rev>  : 相对移动 (圈数)
 * - v<vel>  : 设置速度 (rad/s)
 * - s       : 急停
 * - e       : 使能/禁用切换
 * - h       : 归零
 * - d       : 轨迹预览
 * - ?       : 状态查询
 */

#include "main.h"
#include "platform/stm32/stm32_hal.hpp"
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>
#include <omni/motion/scurve_profile.hpp>
#include <cstdio>
#include <cstring>
#include <cmath>

using namespace omni::platform::stm32;
using namespace omni::driver;
using namespace omni::app;
using namespace omni::motion;

// ============== CubeMX 生成的句柄 ==============
extern UART_HandleTypeDef huart2;

// ============== 引脚配置 ==============
#define STEP_PORT   GPIOA
#define STEP_PIN    GPIO_PIN_0
#define DIR_PORT    GPIOA
#define DIR_PIN     GPIO_PIN_1
#define EN_PORT     GPIOA
#define EN_PIN      GPIO_PIN_2

// ============== 电机配置 ==============
#define STEPS_PER_REV   200         // 每圈全步数 (1.8°电机)
#define MICROSTEP       16          // 细分数
#define GEAR_RATIO      1.0f        // 减速比

// ============== 运动约束 ==============
#define MAX_VEL         (10.0f * 2 * M_PI)   // 10 rev/s
#define MAX_ACCEL       (50.0f * 2 * M_PI)   // 50 rev/s²
#define MAX_JERK        (500.0f * 2 * M_PI)  // 500 rev/s³

// ============== 控制频率 ==============
#define CONTROL_FREQ_HZ 10000   // 10kHz

// ============== 全局对象 ==============
Stm32Gpio* stepPin = nullptr;
Stm32Gpio* dirPin = nullptr;
Stm32Gpio* enablePin = nullptr;
StepperDriver* stepper = nullptr;
MotorController* controller = nullptr;

// 串口缓冲
char rxBuffer[64];
uint8_t rxIndex = 0;

// 控制循环计时
volatile uint32_t lastControlTime = 0;

// ============== 函数声明 ==============
void processCommand(const char* cmd);
void printStatus();
void showTrajectoryPreview();
void printHelp();

// ============== 初始化 ==============
extern "C" void setup() {
    // 初始化 DWT
    Stm32DwtTimer::init();

    printf("\r\n=== OmniMotion STM32 Stepper S-Curve ===\r\n");

    // 创建 GPIO
    stepPin = new Stm32Gpio(STEP_PORT, STEP_PIN);
    dirPin = new Stm32Gpio(DIR_PORT, DIR_PIN);
    enablePin = new Stm32Gpio(EN_PORT, EN_PIN);

    stepPin->setMode(omni::hal::PinMode::Output);
    dirPin->setMode(omni::hal::PinMode::Output);
    enablePin->setMode(omni::hal::PinMode::Output);

    // 创建步进驱动
    stepper = new StepperDriver(stepPin, dirPin, enablePin);
    stepper->init();

    // 配置步进参数
    stepper->setStepsPerRev(STEPS_PER_REV);
    stepper->setMicrostep(MICROSTEP);
    stepper->setDirectionInvert(false);
    stepper->setMaxStepRate(STEPS_PER_REV * MICROSTEP * 20);  // 20 rev/s max

    // 设置电机参数
    MotorParams params;
    params.encoderCpr = STEPS_PER_REV * MICROSTEP;
    stepper->setParams(params);

    // 创建高层控制器
    controller = new MotorController(stepper);

    // 设置 S曲线约束
    MotionConstraints constraints;
    constraints.maxVelocity = MAX_VEL;
    constraints.maxAcceleration = MAX_ACCEL;
    constraints.maxJerk = MAX_JERK;
    controller->setConstraints(constraints);

    // 使能电机
    controller->enable();
    controller->home();

    printf("每圈步数: %lu\r\n", (unsigned long)(STEPS_PER_REV * MICROSTEP));
    printf("最大速度: %.1f rev/s\r\n", MAX_VEL / (2 * M_PI));
    printf("最大加速度: %.1f rev/s^2\r\n", MAX_ACCEL / (2 * M_PI));

    printHelp();
}

// ============== 控制循环 (由TIM6中断调用) ==============
extern "C" void controlLoop(float dt) {
    controller->update(dt);
}

// ============== 主循环 ==============
extern "C" void loop() {
    // 处理串口输入
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
        if (ch == '\r' || ch == '\n') {
            rxBuffer[rxIndex] = '\0';
            if (rxIndex > 0) {
                processCommand(rxBuffer);
            }
            rxIndex = 0;
        } else if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = ch;
        }
    }

    // 定期打印运动状态
    static uint32_t lastPrint = 0;
    uint32_t now = HAL_GetTick();
    if (controller->isMoving() && (now - lastPrint >= 100)) {
        lastPrint = now;
        printf("pos=%.3f rev  vel=%.2f rev/s  steps=%ld\r\n",
               controller->getPosition() / (2 * M_PI),
               controller->getVelocity() / (2 * M_PI),
               stepper->getStepPosition());
    }
}

// ============== 命令处理 ==============
void processCommand(const char* cmd) {
    if (strlen(cmd) == 0) return;

    char c = cmd[0];
    float value = 0;
    if (strlen(cmd) > 1) {
        value = atof(cmd + 1);
    }

    switch (c) {
        case 'g': case 'G':  // 绝对位置 (弧度)
            controller->moveTo(value);
            printf("移动到: %.3f rad (%.3f rev)\r\n", value, value / (2 * M_PI));
            break;

        case 'r': case 'R':  // 相对移动 (圈数)
            controller->moveBy(value * 2.0f * M_PI);
            printf("移动 %.2f 圈\r\n", value);
            break;

        case 'v': case 'V':  // 速度模式
            controller->setVelocity(value);
            printf("速度: %.2f rad/s (%.2f rev/s)\r\n", value, value / (2 * M_PI));
            break;

        case 's': case 'S':  // 急停
            controller->emergencyStop();
            printf("急停\r\n");
            break;

        case 'e': case 'E':  // 使能切换
            if (controller->isEnabled()) {
                controller->disable();
                printf("已禁用\r\n");
            } else {
                controller->enable();
                printf("已使能\r\n");
            }
            break;

        case 'h': case 'H':  // 归零
            controller->home();
            stepper->resetStepPosition();
            printf("已归零\r\n");
            break;

        case 'd': case 'D':  // 轨迹预览
            showTrajectoryPreview();
            break;

        case '?':  // 状态
            printStatus();
            break;

        default:
            printHelp();
            break;
    }
}

// ============== 状态打印 ==============
void printStatus() {
    printf("\r\n--- 状态 ---\r\n");
    printf("位置: %.4f rev (%.3f rad)\r\n",
           controller->getPosition() / (2 * M_PI),
           controller->getPosition());
    printf("速度: %.2f rev/s\r\n", controller->getVelocity() / (2 * M_PI));
    printf("步数: %ld\r\n", stepper->getStepPosition());
    printf("使能: %s\r\n", controller->isEnabled() ? "是" : "否");
    printf("运动中: %s\r\n", controller->isMoving() ? "是" : "否");
}

// ============== 轨迹预览 ==============
void showTrajectoryPreview() {
    printf("\r\n--- 轨迹预览 (1圈移动) ---\r\n");

    MotionConstraints c;
    c.maxVelocity = MAX_VEL;
    c.maxAcceleration = MAX_ACCEL;
    c.maxJerk = MAX_JERK;

    SCurveProfile profile(c);
    profile.plan(0, 2.0f * M_PI, 0, 0);  // 1圈

    float duration = profile.getDuration();
    printf("持续时间: %.1f ms\r\n", duration * 1000);
    printf("峰值速度: %.2f rev/s\r\n", profile.getPeakVelocity() / (2 * M_PI));

    // 打印各段时间
    float times[7];
    profile.getSegmentTimes(times);
    printf("各段时间 (ms):\r\n");
    const char* names[] = {"加加速", "匀加速", "减加速", "匀速", "加减速", "匀减速", "减减速"};
    for (int i = 0; i < 7; i++) {
        printf("  %s: %.2f\r\n", names[i], times[i] * 1000);
    }

    // 采样轨迹
    printf("\r\n时间(ms), 位置(rev), 速度(rev/s), 加速度(rev/s2)\r\n");
    for (int i = 0; i <= 20; i++) {
        float t = duration * i / 20.0f;
        TrajectoryPoint pt = profile.evaluate(t);
        printf("%.1f, %.4f, %.3f, %.2f\r\n",
               t * 1000,
               pt.position / (2 * M_PI),
               pt.velocity / (2 * M_PI),
               pt.acceleration / (2 * M_PI));
    }
}

void printHelp() {
    printf("\r\n--- 命令 ---\r\n");
    printf("g<rad>  - 移动到位置 (弧度)\r\n");
    printf("r<rev>  - 移动圈数\r\n");
    printf("v<vel>  - 设置速度 (rad/s)\r\n");
    printf("s       - 急停\r\n");
    printf("e       - 使能/禁用\r\n");
    printf("h       - 归零\r\n");
    printf("d       - 轨迹预览\r\n");
    printf("?       - 状态\r\n\r\n");
}

// ============== 定时器中断回调 ==============
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        controlLoop(1.0f / CONTROL_FREQ_HZ);
    }
}

// ============== printf 重定向 ==============
extern "C" int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
