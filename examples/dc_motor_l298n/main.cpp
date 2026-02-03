/**
 * @file main.cpp
 * @brief STM32 L298N 直流电机闭环控制示例
 *
 * 功能与 Arduino 版本对齐:
 * - L298N PWM 驱动封装
 * - 编码器闭环控制
 * - S曲线轨迹规划 (支持重规划)
 * - 多种控制模式 (开环/速度/位置/轨迹)
 * - 串口命令交互
 *
 * 硬件配置 (CubeMX):
 * - TIM2: 编码器模式 (PA0/PA1)
 * - TIM3_CH1: PWM (PA6)
 * - GPIO: IN1(PB0), IN2(PB1)
 * - USART2: 串口调试
 * - TIM6: 控制循环定时器 (1kHz)
 */

#include "main.h"
#include "platform/stm32/stm32_hal.hpp"
#include <omni/driver/dc_motor_driver.hpp>
#include <omni/app/motor_controller.hpp>
#include <omni/motion/scurve_profile.hpp>
#include <omni/control/pid_controller.hpp>
#include <cstdio>
#include <cstring>
#include <cmath>

using namespace omni::platform::stm32;
using namespace omni::driver;
using namespace omni::app;
using namespace omni::motion;
using namespace omni::control;

// ============== CubeMX 生成的句柄 ==============
extern TIM_HandleTypeDef htim2;   // 编码器
extern TIM_HandleTypeDef htim3;   // PWM
extern UART_HandleTypeDef huart2; // 串口

// ============== 引脚配置 ==============
#define IN1_PORT GPIOB
#define IN1_PIN  GPIO_PIN_0
#define IN2_PORT GPIOB
#define IN2_PIN  GPIO_PIN_1

// ============== 编码器参数 ==============
const uint32_t ENCODER_CPR = 1336;  // 334线 x 4 (X4解码)

// ============== L298N PWM 封装 ==============
/**
 * @brief L298N 电机驱动 PWM 封装
 *
 * 实现 IPwm 接口，用于与 DcMotorDriver 配合
 * 支持正反转和刹车控制
 */
class L298NPwm : public omni::hal::IPwm {
public:
    L298NPwm(TIM_HandleTypeDef* htim, uint32_t channel,
             GPIO_TypeDef* in1Port, uint16_t in1Pin,
             GPIO_TypeDef* in2Port, uint16_t in2Pin)
        : htim_(htim), channel_(channel)
        , in1Port_(in1Port), in1Pin_(in1Pin)
        , in2Port_(in2Port), in2Pin_(in2Pin)
        , duty_(0), enabled_(false)
    {
        // 初始化方向引脚
        HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
    }

    void setFrequency(uint32_t freq) override {
        // PWM频率由CubeMX配置，这里不做修改
        (void)freq;
    }

    uint32_t getFrequency() const override {
        return 1000;  // 默认1kHz
    }

    void setDuty(float duty) override {
        duty_ = fmaxf(-1.0f, fminf(1.0f, duty));
        if (!enabled_) return;

        uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_) + 1;

        if (duty_ > 0.01f) {
            // 正转
            HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(htim_, channel_, (uint32_t)(duty_ * period));
        } else if (duty_ < -0.01f) {
            // 反转
            HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(htim_, channel_, (uint32_t)(-duty_ * period));
        } else {
            // 停止 (滑行)
            HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
        }
    }

    void setDutyNs(uint32_t ns) override {
        (void)ns;  // 不支持
    }

    float getDuty() const override {
        return duty_;
    }

    void enable(bool en) override {
        enabled_ = en;
        if (en) {
            HAL_TIM_PWM_Start(htim_, channel_);
        } else {
            HAL_TIM_PWM_Stop(htim_, channel_);
            HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
        }
    }

    bool isEnabled() const override {
        return enabled_;
    }

    void setDeadTime(uint32_t ns) override {
        (void)ns;  // L298N不需要死区
    }

    uint32_t getDeadTime() const override {
        return 0;
    }

    /**
     * @brief 刹车模式 (短接两端)
     */
    void brake() {
        HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(htim_, channel_, __HAL_TIM_GET_AUTORELOAD(htim_) + 1);
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    GPIO_TypeDef* in1Port_;
    uint16_t in1Pin_;
    GPIO_TypeDef* in2Port_;
    uint16_t in2Pin_;
    float duty_;
    bool enabled_;
};

// ============== 全局对象 ==============
L298NPwm* l298n = nullptr;
Stm32Encoder* encoder = nullptr;
DcMotorDriver* motor = nullptr;
MotorController* ctrl = nullptr;

// 轨迹规划
MotionConstraints constraints = {200.0f, 100.0f, 0, 200.0f, 0};
SCurveProfile* trajectory = nullptr;
float trajectoryStartTime = 0;
bool inTrajectory = false;

// 控制模式
enum Mode { MODE_CTRL, MODE_OPENLOOP };
Mode mode = MODE_CTRL;
float openLoopPwm = 0;

// 控制循环
volatile uint32_t lastControlTime = 0;
const uint32_t CONTROL_PERIOD_US = 1000;

// 串口缓冲
char rxBuffer[64];
uint8_t rxIndex = 0;

// ============== 函数声明 ==============
void processCommand(const char* cmd);
void planTrajectory(float endPos);
void printStatus();
void printHelp();

// ============== 初始化 ==============
extern "C" void setup() {
    // 初始化 DWT 计时
    Stm32DwtTimer::init();

    // 创建 HAL 对象
    l298n = new L298NPwm(&htim3, TIM_CHANNEL_1,
                         IN1_PORT, IN1_PIN,
                         IN2_PORT, IN2_PIN);

    encoder = new Stm32Encoder(&htim2);
    encoder->setResolution(ENCODER_CPR);
    encoder->init();

    // 创建电机驱动
    motor = new DcMotorDriver(l298n, (omni::hal::IPwm*)nullptr);
    motor->init();

    MotorParams params;
    params.encoderCpr = ENCODER_CPR;
    motor->setParams(params);
    motor->setVelocityPid(0.01f, 0.02f, 0.0001f);

    // 创建高层控制器
    ctrl = new MotorController(motor);
    ctrl->setConstraints(constraints);
    ctrl->setPositionPid(3.0f, 0.0f, 0.05f);
    ctrl->setVelocityPid(0.01f, 0.02f, 0.0001f);

    // 轨迹生成器
    trajectory = new SCurveProfile(constraints);

    // 使能
    l298n->enable(true);
    ctrl->enable();
    ctrl->home();

    printf("\r\n=== OmniMotion STM32 L298N Demo ===\r\n");
    printHelp();
}

// ============== 控制循环 (由TIM6中断调用) ==============
extern "C" void controlLoop(float dt) {
    encoder->update(dt);

    if (mode == MODE_OPENLOOP) {
        l298n->setDuty(openLoopPwm);
    } else {
        // 检查轨迹是否完成
        if (inTrajectory) {
            float now = Stm32DwtTimer::getCycles() / (float)SystemCoreClock;
            float elapsed = now - trajectoryStartTime;
            if (elapsed >= trajectory->getDuration()) {
                inTrajectory = false;
                printf("轨迹完成!\r\n");
            }
        }
        ctrl->update(dt);
    }
}

// ============== 轨迹规划 (支持重规划) ==============
void planTrajectory(float endPos) {
    float startPos, startVel;
    float now = Stm32DwtTimer::getCycles() / (float)SystemCoreClock;

    // 如果正在运行轨迹，从当前轨迹点开始
    if (inTrajectory) {
        float elapsed = now - trajectoryStartTime;
        if (elapsed < trajectory->getDuration()) {
            TrajectoryPoint pt = trajectory->evaluate(elapsed);
            startPos = pt.position;
            startVel = pt.velocity;

            // 检查速度方向
            float direction = endPos - startPos;
            if ((direction * startVel < 0) && (fabsf(startVel) > 1.0f)) {
                startPos = encoder->getAngle();
                startVel = encoder->getVelocity();
                printf(">> 重规划 (反向, 使用实测值)\r\n");
            } else {
                printf(">> 重规划 (平滑衔接)\r\n");
            }
        } else {
            startPos = encoder->getAngle();
            startVel = encoder->getVelocity();
        }
    } else {
        startPos = encoder->getAngle();
        startVel = encoder->getVelocity();
    }

    if (trajectory->plan(startPos, endPos, startVel, 0)) {
        trajectoryStartTime = now;
        inTrajectory = true;
        ctrl->moveTo(endPos);

        printf("轨迹: %.2f (v=%.2f) -> %.2f, 时间: %.2fs\r\n",
               startPos, startVel, endPos, trajectory->getDuration());
    } else {
        printf("轨迹规划失败!\r\n");
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
        case 'v': case 'V':
            mode = MODE_CTRL;
            inTrajectory = false;
            ctrl->setVelocity(value);
            printf("速度: %.2f rad/s\r\n", value);
            break;

        case 'p': case 'P':
            mode = MODE_CTRL;
            inTrajectory = false;
            ctrl->moveTo(value);
            printf("位置: %.2f rad\r\n", value);
            break;

        case 't': case 'T':
            mode = MODE_CTRL;
            planTrajectory(value);
            break;

        case 'o': case 'O':
            mode = MODE_OPENLOOP;
            inTrajectory = false;
            openLoopPwm = value;
            printf("开环PWM: %.2f\r\n", value);
            break;

        case 's': case 'S':
            mode = MODE_CTRL;
            inTrajectory = false;
            ctrl->emergencyStop();
            printf("停止\r\n");
            break;

        case 'z': case 'Z':
            encoder->resetCount();
            ctrl->home();
            inTrajectory = false;
            printf("已清零\r\n");
            break;

        case 'd': case 'D': {
            float startPos = encoder->getAngle();
            SCurveProfile preview(constraints);
            if (preview.plan(startPos, value, 0, 0)) {
                printf("\r\n=== 轨迹预览 ===\r\n");
                printf("持续时间: %.3fs\r\n", preview.getDuration());

                float times[7];
                preview.getSegmentTimes(times);
                const char* names[] = {"J+", "Ac", "J-", "Cv", "J-", "Dc", "J+"};
                printf("段时间(ms): ");
                for (int i = 0; i < 7; i++) {
                    printf("%s=%.1f ", names[i], times[i] * 1000);
                }
                printf("\r\n\r\nt(s)\tpos\tvel\tacc\r\n");
                for (int i = 0; i <= 10; i++) {
                    float t = preview.getDuration() * i / 10.0f;
                    TrajectoryPoint pt = preview.evaluate(t);
                    printf("%.2f\t%.2f\t%.2f\t%.2f\r\n",
                           t, pt.position, pt.velocity, pt.acceleration);
                }
            } else {
                printf("预览失败\r\n");
            }
            break;
        }

        case '?':
            printf("\r\n--- 状态 ---\r\n");
            printf("位置: %.3f rad\r\n", encoder->getAngle());
            printf("速度: %.2f rad/s\r\n", encoder->getVelocity());
            printf("计数: %ld\r\n", encoder->getCount());
            printf("模式: ");
            if (mode == MODE_OPENLOOP) printf("开环\r\n");
            else if (inTrajectory) printf("轨迹\r\n");
            else if (ctrl->isMoving()) printf("移动中\r\n");
            else printf("空闲\r\n");
            break;

        default:
            printHelp();
            break;
    }
}

// ============== 状态打印 ==============
void printStatus() {
    static uint32_t lastPrint = 0;
    uint32_t now = HAL_GetTick();
    if (now - lastPrint < 100) return;
    lastPrint = now;

    if (mode == MODE_OPENLOOP || !ctrl->isMoving()) return;

    printf("p=%.2f v=%.2f", encoder->getAngle(), encoder->getVelocity());

    if (inTrajectory) {
        float elapsed = (Stm32DwtTimer::getCycles() / (float)SystemCoreClock) - trajectoryStartTime;
        if (elapsed < trajectory->getDuration()) {
            TrajectoryPoint pt = trajectory->evaluate(elapsed);
            printf(" tgt_p=%.2f tgt_v=%.2f", pt.position, pt.velocity);
        }
    }

    printf(" cnt=%ld\r\n", encoder->getCount());
}

void printHelp() {
    printf("\r\n命令:\r\n");
    printf("  v<速度>  - 速度模式 (rad/s)\r\n");
    printf("  p<位置>  - 位置模式 (rad)\r\n");
    printf("  t<位置>  - S曲线轨迹 (支持重规划)\r\n");
    printf("  o<PWM>   - 开环测试 (-1~1)\r\n");
    printf("  d<位置>  - 预览轨迹\r\n");
    printf("  s        - 停止\r\n");
    printf("  z        - 清零\r\n");
    printf("  ?        - 状态\r\n");
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

    printStatus();
}

// ============== 定时器中断回调 ==============
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        controlLoop(0.001f);  // 1kHz
    }
}

// ============== printf 重定向 ==============
extern "C" int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
