# OmniMotion 电机控制库

> 但凡用电机，皆可用此库

## 目录

1. [简介](#简介)
2. [快速开始](#快速开始)
3. [架构概述](#架构概述)
4. [HAL 硬件抽象层](#hal-硬件抽象层)
5. [驱动层](#驱动层)
6. [运动规划](#运动规划)
7. [控制器](#控制器)
8. [应用示例](#应用示例)
9. [API 参考](#api-参考)

---

## 简介

OmniMotion 是一个跨平台的电机控制库，支持：

- **多种电机类型**：步进电机、直流有刷电机、无刷电机(BLDC/PMSM)
- **多种控制模式**：位置、速度、力矩、电流、电压、占空比
- **运动规划**：梯形加减速、S曲线加减速
- **闭环控制**：PID 位置环、速度环、电流环
- **跨平台**：STM32、ESP32、Arduino、Linux 等

### 设计理念

```
┌─────────────────────────────────────────────────────┐
│                   应用层 (App)                       │
│         MotorController, GimbalController           │
├─────────────────────────────────────────────────────┤
│                  运动规划 (Motion)                   │
│        TrapezoidProfile, SCurveProfile              │
├─────────────────────────────────────────────────────┤
│                   驱动层 (Driver)                    │
│    StepperDriver, BLDCDriver, DCMotorDriver         │
├─────────────────────────────────────────────────────┤
│                 HAL 硬件抽象层                       │
│       IGpio, IPwm, IEncoder, ISpi, IUart...         │
└─────────────────────────────────────────────────────┘
```

---

## 快速开始

### 安装

将 OmniMotion 添加到项目：

```cmake
# CMakeLists.txt
set(OMNIMOTION_PATH "/path/to/OmniMotion")
target_include_directories(${PROJECT_NAME} PRIVATE
    ${OMNIMOTION_PATH}/include
    ${OMNIMOTION_PATH}/platform/stm32  # 或其他平台
)
```

### 最小示例

```cpp
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 1. 实现 HAL 接口 (见 HAL 章节)
MyGpio stepPin, dirPin, enablePin;
MyEncoder encoder;

// 2. 创建驱动
omni::driver::StepperDriver driver(&stepPin, &dirPin, &encoder, &enablePin);
driver.setMicrostep(16);
driver.setStepsPerRev(200);
driver.init();

// 3. 创建控制器
omni::app::MotorController motor(&driver);
motor.setConstraints({10.0f, 50.0f, 500.0f});  // vel, accel, jerk
motor.enable();

// 4. 运动
motor.moveTo(3.14f);  // 移动到 π 弧度

// 5. 控制循环 (1kHz)
while (!motor.isSettled()) {
    encoder.update(0.001f);
    motor.update(0.001f);
    driver.update();
}
```

---

## 架构概述

### 层次结构

| 层 | 职责 | 主要类 |
|---|---|---|
| **App** | 高级应用逻辑 | `MotorController`, `GimbalController` |
| **Motion** | 轨迹规划 | `TrapezoidProfile`, `SCurveProfile`, `OnlinePlanner` |
| **Control** | 闭环控制 | `PIDController` |
| **Driver** | 电机驱动 | `StepperDriver`, `BLDCDriver`, `DCMotorDriver` |
| **HAL** | 硬件抽象 | `IGpio`, `IPwm`, `IEncoder`, `IAdc`, `ISpi`, `IUart` |

### 数据流

```
目标位置 → 轨迹规划 → 位置PID → 速度PID → 驱动输出 → 电机
                ↑                              ↓
                └────────── 编码器反馈 ←────────┘
```

### 控制模式

```cpp
enum class ControlMode {
    Position,   // 位置控制 (闭环)
    Velocity,   // 速度控制 (闭环)
    Torque,     // 力矩控制
    Current,    // 电流控制
    Voltage,    // 电压控制 (开环)
    Duty        // 占空比控制 (开环)
};
```

---

## HAL 硬件抽象层

HAL 层定义了硬件接口，用户需要根据自己的平台实现这些接口。

### IGpio - 通用IO

```cpp
class IGpio {
public:
    virtual void setMode(PinMode mode) = 0;
    virtual void write(bool value) = 0;
    virtual bool read() = 0;
    virtual void toggle() = 0;
};

// 示例：STM32 实现
class Stm32Gpio : public omni::hal::IGpio {
public:
    Stm32Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    void write(bool value) override {
        HAL_GPIO_WritePin(port_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    bool read() override {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

    void toggle() override {
        HAL_GPIO_TogglePin(port_, pin_);
    }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};
```

### IPwm - PWM 输出

```cpp
class IPwm {
public:
    virtual void setFrequency(uint32_t freq_hz) = 0;
    virtual void setDuty(float duty) = 0;  // 0.0 ~ 1.0
    virtual void enable(bool en) = 0;
};

// 示例：STM32 实现
class Stm32Pwm : public omni::hal::IPwm {
public:
    Stm32Pwm(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim), channel_(channel) {}

    void setFrequency(uint32_t freq_hz) override {
        uint32_t period = SystemCoreClock / freq_hz;
        __HAL_TIM_SET_AUTORELOAD(htim_, period - 1);
    }

    void setDuty(float duty) override {
        uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_);
        __HAL_TIM_SET_COMPARE(htim_, channel_, (uint32_t)(period * duty));
    }

    void enable(bool en) override {
        if (en) HAL_TIM_PWM_Start(htim_, channel_);
        else HAL_TIM_PWM_Stop(htim_, channel_);
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
};
```

### IEncoder - 编码器

```cpp
class IEncoder {
public:
    virtual int32_t getCount() = 0;
    virtual void resetCount() = 0;
    virtual float getAngle() = 0;      // 弧度
    virtual float getVelocity() = 0;   // rad/s
};

// 示例：增量式编码器 (STM32 定时器)
class Stm32QuadEncoder : public omni::hal::IEncoder {
public:
    Stm32QuadEncoder(TIM_HandleTypeDef* htim, uint32_t cpr)
        : htim_(htim), cpr_(cpr) {
        HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    }

    int32_t getCount() override {
        return (int16_t)__HAL_TIM_GET_COUNTER(htim_);
    }

    float getAngle() override {
        return (float)getCount() / cpr_ * 2.0f * 3.14159f;
    }

private:
    TIM_HandleTypeDef* htim_;
    uint32_t cpr_;
};

// 示例：磁编码器 (SPI)
class MT6816Encoder : public omni::hal::IEncoder {
public:
    MT6816Encoder(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);
    float getAngle() override;
    void update(float dt);  // 需定期调用以计算速度
};
```

---

## 驱动层

### IMotorDriver 接口

所有电机驱动都实现此接口：

```cpp
class IMotorDriver {
public:
    // 生命周期
    virtual bool init() = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void emergencyStop() = 0;

    // 控制模式
    virtual void setControlMode(ControlMode mode) = 0;

    // 设置目标
    virtual void setPosition(float pos) = 0;   // rad
    virtual void setVelocity(float vel) = 0;   // rad/s
    virtual void setTorque(float torque) = 0;  // Nm
    virtual void setCurrent(float current) = 0; // A
    virtual void setVoltage(float voltage) = 0; // V
    virtual void setDuty(float duty) = 0;      // 0~1

    // 获取状态
    virtual float getPosition() const = 0;
    virtual float getVelocity() const = 0;
    virtual float getTorque() const = 0;
    virtual float getCurrent() const = 0;

    // 更新 (控制循环中调用)
    virtual void update() = 0;
};
```

### StepperDriver - 步进电机

```cpp
#include <omni/driver/stepper_driver.hpp>

// 创建驱动
StepperDriver driver(&stepPin, &dirPin, &encoder, &enablePin);

// 配置
driver.setStepsPerRev(200);    // 电机步数/转
driver.setMicrostep(16);       // 微步细分
driver.setDirectionInvert(false);

// 初始化
driver.init();
driver.enable();

// 速度控制
driver.setVelocity(3.14f);  // 0.5 转/秒

// 控制循环
while (true) {
    driver.update();  // 生成步进脉冲
    delay_us(100);
}
```

**TMC2209 高级功能：**

```cpp
// 需要额外的 TMC2209 UART 驱动
TMC2209 tmc(&huart, 0);
tmc.init();
tmc.setCurrent(16, 8);     // 运行/保持电流 (0-31)
tmc.setMicrostep(64);      // 微步
tmc.setStealthChop(true);  // 静音模式
```

### DCMotorDriver - 直流有刷电机

```cpp
#include <omni/driver/dc_motor_driver.hpp>

// H桥驱动 (双PWM模式)
DCMotorDriver driver(&pwmA, &pwmB, &encoder);

// 或 PWM+方向模式
DCMotorDriver driver(&pwm, &dirPin, &encoder);

driver.init();
driver.enable();

// 占空比控制 (开环)
driver.setDuty(0.5f);   // 50% 正转
driver.setDuty(-0.3f);  // 30% 反转

// 速度控制 (闭环)
driver.setControlMode(ControlMode::Velocity);
driver.setVelocity(10.0f);  // rad/s
```

### BLDCDriver - 无刷电机

```cpp
#include <omni/driver/bldc_driver.hpp>

// 三相PWM + 编码器
BLDCDriver driver(&pwmU, &pwmV, &pwmW, &encoder);

// 配置
driver.setPolePairs(7);        // 极对数
driver.setPhaseResistance(0.5f);
driver.setPhaseInductance(0.001f);

driver.init();
driver.enable();

// FOC 控制
driver.setControlMode(ControlMode::Velocity);
driver.setVelocity(100.0f);  // rad/s

// 控制循环 (10kHz+)
while (true) {
    driver.update();
    delay_us(100);
}
```

---

## 运动规划

### 运动约束

```cpp
struct MotionConstraints {
    float maxVelocity;      // 最大速度 (rad/s)
    float maxAcceleration;  // 最大加速度 (rad/s²)
    float maxJerk;          // 最大加加速度 (rad/s³)
};
```

### TrapezoidProfile - 梯形加减速

简单高效，适合大多数场景。

```cpp
#include <omni/motion/trapezoid_profile.hpp>

TrapezoidProfile profile;
profile.setConstraints({10.0f, 50.0f, 0});

// 规划: 从0到100弧度
profile.plan(0, 100.0f, 0, 0);

// 采样轨迹
for (float t = 0; t < profile.getDuration(); t += 0.001f) {
    TrajectoryPoint pt = profile.evaluate(t);
    // pt.position, pt.velocity, pt.acceleration
}
```

### SCurveProfile - S曲线加减速

平滑运动，减少机械冲击。

```cpp
#include <omni/motion/scurve_profile.hpp>

SCurveProfile profile;
profile.setConstraints({10.0f, 50.0f, 500.0f});

profile.plan(0, 100.0f, 0, 0);

// S曲线7段: 加加速→匀加速→减加速→匀速→加减速→匀减速→减减速
```

### OnlinePlanner - 在线规划器

实时响应目标变化。

```cpp
#include <omni/motion/online_planner.hpp>

OnlinePlanner planner;
planner.setConstraints({10.0f, 50.0f, 500.0f});

// 控制循环中
void controlLoop(float dt) {
    planner.setTarget(targetPosition);
    TrajectoryPoint pt = planner.update(dt);
    driver.setVelocity(pt.velocity);
}
```

---

## 控制器

### MotorController - 电机控制器

整合驱动、规划、PID 的高级控制器。

```cpp
#include <omni/app/motor_controller.hpp>

MotorController motor(&driver);

// 配置运动约束
motor.setConstraints({10.0f, 50.0f, 500.0f});

// 配置 PID
motor.setPositionPid(10.0f, 0.1f, 0.5f);
motor.setVelocityPid(0.5f, 0.01f, 0.0f);

// 启用
motor.enable();

// 位置运动
motor.moveTo(3.14f);      // 移动到目标位置
motor.moveBy(1.0f);       // 相对移动

// 速度运动
motor.setVelocity(5.0f);  // 持续速度

// 状态查询
motor.getPosition();
motor.getVelocity();
motor.isSettled();        // 是否到达目标
```

---

## 应用示例

### 示例1：单轴定位系统

```cpp
// 直线模组、旋转台等
MotorController axis(&stepperDriver);
axis.setConstraints({100.0f, 500.0f, 2000.0f});
axis.enable();

// 移动到位置
axis.moveTo(targetPos);
while (!axis.isSettled()) {
    axis.update(0.001f);
    delay_ms(1);
}
```

### 示例2：双轴云台

```cpp
// Pan-Tilt 云台
MotorController pan(&panDriver);
MotorController tilt(&tiltDriver);

pan.setConstraints({5.0f, 20.0f, 100.0f});
tilt.setConstraints({5.0f, 20.0f, 100.0f});

pan.enable();
tilt.enable();

// 移动到角度
void lookAt(float panAngle, float tiltAngle) {
    pan.moveTo(panAngle);
    tilt.moveTo(tiltAngle);
}

// 控制循环
void controlLoop() {
    pan.update(0.001f);
    tilt.update(0.001f);
}
```

### 示例3：轮式机器人

```cpp
// 差速驱动
MotorController leftWheel(&leftDriver);
MotorController rightWheel(&rightDriver);

// 速度控制模式
void setSpeed(float linear, float angular) {
    float wheelBase = 0.3f;  // 轮距
    float leftVel = linear - angular * wheelBase / 2;
    float rightVel = linear + angular * wheelBase / 2;

    leftWheel.setVelocity(leftVel);
    rightWheel.setVelocity(rightVel);
}
```

### 示例4：3D打印机

```cpp
// XYZ 三轴
MotorController xAxis(&xDriver);
MotorController yAxis(&yDriver);
MotorController zAxis(&zDriver);
MotorController extruder(&eDriver);

// G代码移动
void G1(float x, float y, float z, float e, float feedrate) {
    xAxis.moveTo(x);
    yAxis.moveTo(y);
    zAxis.moveTo(z);
    extruder.moveTo(e);
}

// 回零
void G28() {
    // 移动直到触发限位开关
    xAxis.setVelocity(-10.0f);
    while (!xEndstop.read()) {
        xAxis.update(0.001f);
    }
    xAxis.stop();
    xAxis.setHome();
}
```

### 示例5：机械臂

```cpp
// 6轴机械臂
MotorController joints[6];

// 关节空间运动
void moveJoints(float angles[6]) {
    for (int i = 0; i < 6; i++) {
        joints[i].moveTo(angles[i]);
    }
}

// 等待所有轴到位
bool allSettled() {
    for (int i = 0; i < 6; i++) {
        if (!joints[i].isSettled()) return false;
    }
    return true;
}
```

### 示例6：伺服电机

```cpp
// 舵机/伺服
MotorController servo(&bldcDriver);
servo.setConstraints({50.0f, 200.0f, 1000.0f});

// 角度控制
void setAngle(float degrees) {
    servo.moveTo(degrees * 3.14159f / 180.0f);
}
```

---

## API 参考

### MotorController

| 方法 | 说明 |
|------|------|
| `enable()` | 使能电机 |
| `disable()` | 禁用电机 |
| `moveTo(pos)` | 移动到绝对位置 (rad) |
| `moveBy(delta)` | 相对移动 |
| `setVelocity(vel)` | 设置目标速度 (rad/s) |
| `stop()` | 停止运动 |
| `getPosition()` | 获取当前位置 |
| `getVelocity()` | 获取当前速度 |
| `isSettled()` | 是否到达目标 |
| `update(dt)` | 更新控制器 |

### StepperDriver

| 方法 | 说明 |
|------|------|
| `init()` | 初始化驱动 |
| `enable()` | 使能电机 |
| `disable()` | 禁用电机 |
| `setMicrostep(n)` | 设置微步 (1/2/4/8/16/32/64) |
| `setStepsPerRev(n)` | 设置步数/转 |
| `setVelocity(vel)` | 设置速度 (rad/s) |
| `update()` | 生成步进脉冲 |

### MotionConstraints

| 字段 | 类型 | 说明 |
|------|------|------|
| `maxVelocity` | float | 最大速度 (rad/s) |
| `maxAcceleration` | float | 最大加速度 (rad/s²) |
| `maxJerk` | float | 最大加加速度 (rad/s³) |

### TrajectoryPoint

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | float | 位置 (rad) |
| `velocity` | float | 速度 (rad/s) |
| `acceleration` | float | 加速度 (rad/s²) |

---

## 平台支持

### STM32

```cpp
#include <omni/platform/stm32/stm32_hal.hpp>
// 使用 HAL 库实现
```

### ESP32

```cpp
#include <omni/platform/esp32/esp32_hal.hpp>
// 使用 ESP-IDF 实现
```

### Arduino

```cpp
#include <omni/platform/arduino/arduino_hal.hpp>
// 使用 Arduino API 实现
```

---

## 常见问题

### Q: 电机不转？

1. 检查 `enable()` 是否调用
2. 检查 `update()` 是否在循环中调用
3. 检查硬件连接和电源

### Q: 运动不平滑？

1. 提高控制循环频率 (建议 1kHz+)
2. 调整 PID 参数
3. 使用 S曲线代替梯形规划

### Q: 位置不准确？

1. 检查编码器分辨率设置
2. 检查微步设置是否与驱动一致
3. 校准编码器零点

### Q: 如何选择控制频率？

| 电机类型 | 建议频率 |
|---------|---------|
| 步进电机 | 1-10 kHz |
| 直流电机 | 1-5 kHz |
| 无刷电机 | 10-40 kHz |

---

## 版本历史

- **v1.0** - 初始版本，支持步进电机
- **v1.1** - 添加 S曲线规划
- **v1.2** - 添加直流电机、无刷电机支持

---

*OmniMotion - 但凡用电机，皆可用此库*
