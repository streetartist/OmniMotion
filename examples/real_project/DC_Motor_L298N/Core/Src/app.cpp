/**
 * @file app.cpp
 * @brief OmniMotion L298N 直流电机闭环控制
 *
 * 使用 OmniMotion 的 PID 控制器和 S曲线轨迹规划
 * 硬件通过 CubeMX 生成的 HAL 直接操作
 *
 * 功能:
 * - 速度闭环 (v命令)
 * - 位置闭环 (p命令)
 * - S曲线轨迹规划 + 支持重规划 (t命令)
 * - 开环测试 (o命令)
 * - 轨迹预览 (d命令)
 */

#include "main.h"
#include "app.h"

#include <omni/control/pid_controller.hpp>
#include <omni/motion/scurve_profile.hpp>

#include <cstdio>
#include <cstring>
#include <cmath>

using namespace omni::control;
using namespace omni::motion;

// ============== CubeMX 外部句柄 (C语言链接) ==============
extern "C" {
    extern TIM_HandleTypeDef htim2;   // 编码器
    extern TIM_HandleTypeDef htim3;   // PWM
    extern TIM_HandleTypeDef htim4;   // 控制循环
    extern UART_HandleTypeDef huart2; // 串口
}

// ============== 编码器参数 ==============
static const uint32_t ENCODER_CPR = 1336;  // 334线 x 4
static const float RAD_PER_COUNT = 2.0f * (float)M_PI / ENCODER_CPR;

// ============== 编码器状态 ==============
static volatile int32_t encoderTotal = 0;    // 累计脉冲 (处理溢出)
static uint16_t encoderLastRaw = 0;
static float encoderAngle = 0;     // 绝对角度 (rad)
static float encoderVelocity = 0;  // 速度 (rad/s)
static float velocityFilter = 0.1f;

static void encoderUpdate(float dt) {
    uint16_t raw = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = (int16_t)(raw - encoderLastRaw);
    encoderLastRaw = raw;
    encoderTotal += diff;

    encoderAngle = encoderTotal * RAD_PER_COUNT;

    if (dt > 0) {
        float rawVel = diff * RAD_PER_COUNT / dt;
        encoderVelocity = velocityFilter * rawVel + (1.0f - velocityFilter) * encoderVelocity;
    }
}

static void encoderReset() {
    __HAL_TIM_SET_COUNTER(&htim2, 32768);
    encoderLastRaw = 32768;
    encoderTotal = 0;
    encoderAngle = 0;
    encoderVelocity = 0;
}

// ============== L298N 驱动 ==============
static void motorSetDuty(float duty) {
    // 限幅
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;

    if (duty > 0.01f) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(duty * period));
    } else if (duty < -0.01f) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(-duty * period));
    } else {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    }
}

static void motorStop() {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

// ============== PID 控制器 ==============
static PidController velocityPid;
static PidController positionPid;

// ============== S曲线轨迹 ==============
static MotionConstraints constraints = {200.0f, 100.0f, 0, 200.0f, 0};
static SCurveProfile trajectory(constraints);
static float trajectoryStartTime = 0;
static bool inTrajectory = false;

// ============== 控制模式 ==============
enum ControlMode {
    MODE_IDLE,
    MODE_OPENLOOP,
    MODE_VELOCITY,
    MODE_POSITION,
    MODE_TRAJECTORY
};

static ControlMode controlMode = MODE_IDLE;
static float targetVelocity = 0;
static float targetPosition = 0;
static float openLoopPwm = 0;

// ============== 串口缓冲 ==============
static char rxBuf[64];
static uint8_t rxIdx = 0;

// ============== 函数声明 ==============
static void processCommand(const char* cmd);
static void planTrajectory(float endPos);
static void printHelp();

// ============== 时间获取 ==============
static float getTimeS() {
    return HAL_GetTick() / 1000.0f;
}

// ============== printf 重定向 ==============
extern "C" int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// ============== 初始化 ==============
extern "C" void app_setup(void) {
    // 启动编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 32768);
    encoderLastRaw = 32768;

    // 启动 PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    printf("\r\n=== OmniMotion STM32 L298N ===\r\n");
    printf("编码器CPR: %lu\r\n", (unsigned long)ENCODER_CPR);

    // 配置速度 PID
    velocityPid.setGains(0.01f, 0.02f, 0.0001f);
    velocityPid.setOutputLimit(1.0f);
    velocityPid.setIntegralLimit(0.5f);
    velocityPid.setDeadband(0.5f);

    // 配置位置 PID
    positionPid.setGains(3.0f, 0.0f, 0.05f);
    positionPid.setOutputLimit(10.0f);
    positionPid.setDeadband(0.05f);

    // 最后启动控制循环定时器
    HAL_TIM_Base_Start_IT(&htim4);

    printHelp();
}

// ============== 控制循环 (TIM4中断, 1kHz) ==============
extern "C" void app_control_loop(void) {
    const float dt = 0.001f;

    encoderUpdate(dt);

    float output = 0;

    switch (controlMode) {
        case MODE_OPENLOOP:
            output = openLoopPwm;
            break;

        case MODE_VELOCITY:
            output = velocityPid.update(targetVelocity, encoderVelocity, dt);
            break;

        case MODE_POSITION: {
            float velCmd = positionPid.update(targetPosition, encoderAngle, dt);
            output = velocityPid.update(velCmd, encoderVelocity, dt);
            break;
        }

        case MODE_TRAJECTORY: {
            float elapsed = getTimeS() - trajectoryStartTime;

            if (elapsed >= trajectory.getDuration()) {
                // 轨迹完成 → 位置保持
                targetPosition = trajectory.getEndPosition();
                controlMode = MODE_POSITION;
                positionPid.reset(encoderAngle);
                break;
            }

            TrajectoryPoint pt = trajectory.evaluate(elapsed);

            // 位置PID + 速度前馈
            float velCmd = positionPid.update(pt.position, encoderAngle, dt);
            velCmd += pt.velocity;
            output = velocityPid.update(velCmd, encoderVelocity, dt);
            break;
        }

        case MODE_IDLE:
        default:
            output = 0;
            break;
    }

    motorSetDuty(output);
}

// ============== 主循环 ==============
extern "C" void app_loop(void) {
    // 处理串口输入
    uint8_t ch;
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
        if (ch == '\r' || ch == '\n') {
            rxBuf[rxIdx] = '\0';
            if (rxIdx > 0) {
                processCommand(rxBuf);
            }
            rxIdx = 0;
        } else if (rxIdx < sizeof(rxBuf) - 1) {
            rxBuf[rxIdx++] = ch;
        }
    }

    // 运动中定期打印状态
    static uint32_t lastPrint = 0;
    uint32_t now = HAL_GetTick();
    if (controlMode != MODE_IDLE && (now - lastPrint >= 100)) {
        lastPrint = now;

        printf("p=%.2f v=%.2f out=%.3f cnt=%ld\r\n",
               (double)encoderAngle, (double)encoderVelocity,
               (double)velocityPid.getOutput(), (long)encoderTotal);
    }
}

// ============== 轨迹规划 (支持重规划) ==============
static void planTrajectory(float endPos) {
    float startPos, startVel;

    if (controlMode == MODE_TRAJECTORY) {
        float elapsed = getTimeS() - trajectoryStartTime;
        if (elapsed < trajectory.getDuration()) {
            TrajectoryPoint pt = trajectory.evaluate(elapsed);
            startPos = pt.position;
            startVel = pt.velocity;

            float direction = endPos - startPos;
            if ((direction * startVel < 0) && (fabsf(startVel) > 1.0f)) {
                startPos = encoderAngle;
                startVel = encoderVelocity;
                printf(">> 重规划 (反向, 使用实测值)\r\n");
            } else {
                printf(">> 重规划 (平滑衔接)\r\n");
            }
        } else {
            startPos = encoderAngle;
            startVel = encoderVelocity;
        }
    } else {
        startPos = encoderAngle;
        startVel = encoderVelocity;
    }

    if (trajectory.plan(startPos, endPos, startVel, 0)) {
        trajectoryStartTime = getTimeS();
        controlMode = MODE_TRAJECTORY;
        positionPid.reset(startPos);
        velocityPid.reset(startVel);

        printf("轨迹: %.2f (v=%.2f) -> %.2f, 时间: %.2fs\r\n",
               startPos, startVel, endPos, trajectory.getDuration());
    } else {
        printf("轨迹规划失败!\r\n");
    }
}

// ============== 命令处理 ==============
static void processCommand(const char* cmd) {
    if (strlen(cmd) == 0) return;

    char c = cmd[0];
    float value = 0;
    if (strlen(cmd) > 1) {
        value = (float)atof(cmd + 1);
    }

    switch (c) {
        case 'v': case 'V':
            controlMode = MODE_VELOCITY;
            targetVelocity = value;
            velocityPid.reset(encoderVelocity);
            printf("速度: %.2f rad/s\r\n", value);
            break;

        case 'p': case 'P':
            controlMode = MODE_POSITION;
            targetPosition = value;
            positionPid.reset(encoderAngle);
            velocityPid.reset(encoderVelocity);
            printf("位置: %.2f rad\r\n", value);
            break;

        case 't': case 'T':
            planTrajectory(value);
            break;

        case 'o': case 'O':
            controlMode = MODE_OPENLOOP;
            openLoopPwm = value;
            printf("开环PWM: %.2f\r\n", value);
            break;

        case 's': case 'S':
            controlMode = MODE_IDLE;
            motorStop();
            velocityPid.reset();
            positionPid.reset();
            printf("停止\r\n");
            break;

        case 'z': case 'Z':
            controlMode = MODE_IDLE;
            motorStop();
            encoderReset();
            velocityPid.reset();
            positionPid.reset();
            inTrajectory = false;
            printf("已清零\r\n");
            break;

        case 'd': case 'D': {
            float startPos = encoderAngle;
            SCurveProfile preview(constraints);
            if (preview.plan(startPos, value, 0, 0)) {
                printf("\r\n=== 轨迹预览 ===\r\n");
                printf("持续时间: %.3fs\r\n", preview.getDuration());

                float times[7];
                preview.getSegmentTimes(times);
                const char* names[] = {"J+", "Ac", "J-", "Cv", "J-", "Dc", "J+"};
                printf("段时间(ms): ");
                for (int i = 0; i < 7; i++) {
                    printf("%s=%.1f ", names[i], (double)(times[i] * 1000));
                }

                printf("\r\n\r\nt(s)\tpos\tvel\tacc\r\n");
                for (int i = 0; i <= 10; i++) {
                    float t = preview.getDuration() * i / 10.0f;
                    TrajectoryPoint pt = preview.evaluate(t);
                    printf("%.2f\t%.2f\t%.2f\t%.2f\r\n",
                           (double)t, (double)pt.position,
                           (double)pt.velocity, (double)pt.acceleration);
                }
            } else {
                printf("预览失败\r\n");
            }
            break;
        }

        case '?':
            printf("\r\n--- 状态 ---\r\n");
            printf("位置: %.3f rad\r\n", (double)encoderAngle);
            printf("速度: %.2f rad/s\r\n", (double)encoderVelocity);
            printf("计数: %ld\r\n", (long)encoderTotal);
            printf("模式: ");
            switch (controlMode) {
                case MODE_IDLE:       printf("空闲\r\n"); break;
                case MODE_OPENLOOP:   printf("开环\r\n"); break;
                case MODE_VELOCITY:   printf("速度\r\n"); break;
                case MODE_POSITION:   printf("位置\r\n"); break;
                case MODE_TRAJECTORY: printf("轨迹\r\n"); break;
            }
            break;

        default:
            printHelp();
            break;
    }
}

static void printHelp() {
    printf("\r\n命令:\r\n");
    printf("  v<速度>  - 速度模式 (rad/s)\r\n");
    printf("  p<位置>  - 位置模式 (rad)\r\n");
    printf("  t<位置>  - S曲线轨迹 (支持重规划)\r\n");
    printf("  o<PWM>   - 开环测试 (-1~1)\r\n");
    printf("  d<位置>  - 预览轨迹\r\n");
    printf("  s        - 停止\r\n");
    printf("  z        - 清零\r\n");
    printf("  ?        - 状态\r\n\r\n");
}

// ============== TIM4 中断回调 ==============
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        app_control_loop();
    }
}
