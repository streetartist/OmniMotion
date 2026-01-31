# OmniMotion 电机驱动指南

本指南详细介绍 OmniMotion 支持的各种电机类型及其使用方法。

## 目录

1. [电机类型概述](#电机类型概述)
2. [BLDC/PMSM 电机](#bldcpmsm-电机)
3. [步进电机](#步进电机)
4. [有刷直流电机](#有刷直流电机)
5. [舵机](#舵机)
6. [音圈电机](#音圈电机)
7. [直线电机](#直线电机)
8. [自定义电机驱动](#自定义电机驱动)
9. [协议电机驱动](#协议电机驱动)

---

## 电机类型概述

OmniMotion 支持以下电机类型：

| 电机类型 | 驱动类 | 控制方式 | 典型应用 |
|----------|--------|----------|----------|
| BLDC/PMSM | `BldcDriver` | FOC/六步换相 | 无人机、电动车、机器人 |
| 步进电机 | `StepperDriver` | 脉冲/方向 | 3D打印机、CNC、定位台 |
| 有刷DC | `DcMotorDriver` | PWM | 小型机器人、玩具 |
| 舵机 | `ServoDriver` | PWM脉宽 | 机器人关节、云台 |
| 音圈电机 | `VcmDriver` | 电流/电压 | 光学对焦、硬盘磁头 |
| 直线电机 | `LinearMotorDriver` | FOC/脉冲 | 半导体设备、精密定位 |

### 选择合适的电机

```
需要高精度定位？
├── 是 ─→ 需要高速？
│         ├── 是 ─→ BLDC/PMSM (伺服)
│         └── 否 ─→ 步进电机
└── 否 ─→ 需要高扭矩？
          ├── 是 ─→ BLDC/PMSM
          └── 否 ─→ 有刷DC / 舵机
```

---

## BLDC/PMSM 电机

### 简介

无刷直流电机 (BLDC) 和永磁同步电机 (PMSM) 是现代高性能应用的首选。OmniMotion 提供完整的 FOC (磁场定向控制) 支持。

### 硬件连接

```
BLDC电机典型连接:
┌─────────────────────────────────────────────┐
│                MCU                          │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐     │
│  │ PWM_A   │  │ PWM_B   │  │ PWM_C   │     │
│  │ PWM_A_N │  │ PWM_B_N │  │ PWM_C_N │     │
│  └────┬────┘  └────┬────┘  └────┬────┘     │
│       │            │            │           │
│  ┌────┴────────────┴────────────┴────┐     │
│  │         三相PWM输出                 │     │
│  └────────────────┬──────────────────┘     │
│                   │                         │
│  ┌────────────────┴──────────────────┐     │
│  │         半桥驱动器 (如 DRV8301)    │     │
│  │    ┌─────┐  ┌─────┐  ┌─────┐     │     │
│  │    │ A相 │  │ B相 │  │ C相 │     │     │
│  │    └──┬──┘  └──┬──┘  └──┬──┘     │     │
│  └───────┼────────┼────────┼────────┘     │
│          │        │        │               │
│  ┌───────┴────────┴────────┴────────┐     │
│  │           BLDC 电机               │     │
│  │     ┌───────────────────┐        │     │
│  │     │    ╱╲              │        │     │
│  │     │   ╱  ╲   编码器    │        │     │
│  │     │  ╱ ○  ╲            │        │     │
│  │     └───────────────────┘        │     │
│  └───────────────────────────────────┘     │
│                                             │
│  电流采样: ADC_A, ADC_B, ADC_C (或两相)     │
│  编码器: TIM_ENC (正交编码器)               │
└─────────────────────────────────────────────┘
```

### 基本使用

```cpp
#include <omni/driver/bldc_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 1. 创建硬件接口 (平台相关实现)
auto pwm = platform::createPwm3Phase(TIM1);
auto encoder = platform::createEncoder(TIM2);
auto adcA = platform::createAdc(ADC1, 0);
auto adcB = platform::createAdc(ADC1, 1);
auto adcC = platform::createAdc(ADC1, 2);

// 2. 创建BLDC驱动
omni::driver::BldcDriver bldc(pwm, adcA, adcB, adcC, encoder);

// 3. 配置电机参数
omni::driver::MotorParams params;
params.polePairs = 7;              // 7对极
params.resistance = 0.5f;          // 相电阻 0.5Ω
params.inductance = 0.001f;        // 相电感 1mH
params.fluxLinkage = 0.01f;        // 磁链 0.01Wb
params.maxCurrent = 20.0f;         // 最大电流 20A
params.encoderCpr = 4096;          // 编码器4096线
bldc.setParams(params);

// 4. 初始化并校准
bldc.init();
bldc.calibrateEncoder();           // 编码器零点校准
bldc.calibrateCurrentSensor();     // 电流传感器校准

// 5. 配置FOC控制器
bldc.setCurrentPid(0.5f, 0.01f, 0.0f);    // 电流环PID
bldc.setVelocityPid(0.1f, 0.001f, 0.0f);  // 速度环PID
bldc.setPositionPid(50.0f, 0.0f, 0.5f);   // 位置环PID

// 6. 使能并控制
bldc.enable();
bldc.setControlMode(omni::driver::ControlMode::Position);
bldc.setPosition(3.14159f);  // 转到π位置

// 7. 控制循环
while (true) {
    bldc.update();
    delay_us(50);  // 20kHz控制频率
}
```

### FOC 模式选择

```cpp
// SVPWM - 空间矢量调制 (推荐，效率最高)
bldc.setFocMode(omni::driver::FocMode::SVPWM);

// SPWM - 正弦PWM
bldc.setFocMode(omni::driver::FocMode::SPWM);

// 六步换相 - 简单但有扭矩脉动
bldc.setFocMode(omni::driver::FocMode::SixStep);
```

### 电流控制

```cpp
// 直接电流控制 (扭矩模式)
bldc.setControlMode(omni::driver::ControlMode::Current);
bldc.setIdRef(0.0f);      // d轴电流 = 0 (最大扭矩/电流比)
bldc.setIqRef(5.0f);      // q轴电流 = 5A

// 读取实际电流
float id = bldc.getIdActual();
float iq = bldc.getIqActual();
```

### 弱磁控制

```cpp
// 高速运行时使能弱磁
omni::protocol::FocController* foc = bldc.getFocController();
foc->enableFieldWeakening(true);   // 使能弱磁
foc->enableMtpa(true);             // 最大扭矩/电流比
```

---

## 步进电机

### 简介

步进电机具有开环精确定位的优点，广泛用于 3D 打印机、CNC 等场合。

### 硬件连接

```
步进电机典型连接 (脉冲/方向模式):
┌───────────────────────────────────────┐
│              MCU                      │
│  ┌────────┐  ┌────────┐  ┌────────┐  │
│  │ STEP   │  │ DIR    │  │ ENABLE │  │
│  └───┬────┘  └───┬────┘  └───┬────┘  │
└──────┼───────────┼───────────┼────────┘
       │           │           │
┌──────┴───────────┴───────────┴────────┐
│         步进驱动器 (如 TMC2209)        │
│    ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐│
│    │ A+  │  │ A-  │  │ B+  │  │ B-  ││
│    └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘│
└───────┼────────┼────────┼────────┼───┘
        │        │        │        │
     ┌──┴────────┴────────┴────────┴──┐
     │        两相步进电机             │
     │    ┌─────────────────────┐     │
     │    │   线圈A    线圈B     │     │
     │    │   ████     ████     │     │
     │    └─────────────────────┘     │
     └────────────────────────────────┘
```

### 基本使用

```cpp
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 1. 创建GPIO接口
auto stepPin = platform::createGpio(PA0);
auto dirPin = platform::createGpio(PA1);
auto enablePin = platform::createGpio(PA2);

// 2. 创建步进驱动
omni::driver::StepperDriver stepper(stepPin, dirPin, enablePin);

// 3. 配置参数
stepper.setStepsPerRev(200);       // 1.8度步进角，200步/转
stepper.setMicrostep(16);          // 16细分
stepper.setMaxStepRate(100000);    // 最大步进频率100kHz
stepper.init();

// 4. 使用高层控制器
omni::app::MotorController motor(&stepper);

// 设置运动约束
omni::motion::MotionConstraints constraints;
constraints.maxVelocity = 10.0f;       // 10 rad/s
constraints.maxAcceleration = 100.0f;  // 100 rad/s²
constraints.maxJerk = 1000.0f;         // 1000 rad/s³
motor.setConstraints(constraints);

// 5. 执行运动
motor.enable();
motor.moveWithSCurve(
    10.0f * 2 * M_PI,    // 目标: 10圈
    5.0f * 2 * M_PI,     // 最大速度
    20.0f * 2 * M_PI,    // 最大加速度
    200.0f * 2 * M_PI    // 最大加加速度
);

// 6. 控制循环
while (!motor.isSettled()) {
    motor.update(0.001f);
    stepper.update();
    delay_ms(1);
}
```

### 细分设置

```cpp
// 设置细分模式
stepper.setMicrostep(1);   // 全步
stepper.setMicrostep(2);   // 半步
stepper.setMicrostep(4);   // 1/4步
stepper.setMicrostep(8);   // 1/8步
stepper.setMicrostep(16);  // 1/16步
stepper.setMicrostep(32);  // 1/32步

// 或使用枚举
stepper.setStepMode(omni::driver::StepMode::Full);
stepper.setStepMode(omni::driver::StepMode::Half);
stepper.setStepMode(omni::driver::StepMode::Sixteenth);
```

### TMC 驱动器高级功能

```cpp
// TMC 系列驱动器特有功能

// StealthChop - 静音模式
stepper.setStealthChop(true);

// SpreadCycle - 高速模式
stepper.setSpreadCycle(true);

// StallGuard - 堵转检测
stepper.setStallGuard(50);  // 阈值 0-255

// 检测堵转
if (stepper.isStalled()) {
    motor.emergencyStop();
    printf("电机堵转!\n");
}

// 电流设置
stepper.setCurrentLimit(1.5f);  // 1.5A
```

### 闭环步进

```cpp
// 带编码器的步进电机 (闭环)
auto encoder = platform::createEncoder(TIM2);
omni::driver::StepperDriver stepper(stepPin, dirPin, enablePin, encoder);

stepper.enableClosedLoop(true);
stepper.setPositionPid(10.0f, 0.1f, 0.0f);

// 现在可以进行位置闭环控制
stepper.setControlMode(omni::driver::ControlMode::Position);
```

---

## 有刷直流电机

### 简介

有刷直流电机结构简单，控制容易，适合低成本应用。

### 硬件连接

```
H桥驱动模式:
┌─────────────────────────────────┐
│           MCU                   │
│  ┌────────┐    ┌────────┐      │
│  │ PWM_A  │    │ PWM_B  │      │
│  └───┬────┘    └───┬────┘      │
└──────┼─────────────┼────────────┘
       │             │
┌──────┴─────────────┴────────────┐
│      H桥驱动器 (如 L298N)       │
│  ┌─────┐              ┌─────┐  │
│  │ OUT1├──────────────┤OUT2 │  │
│  └─────┘      DC      └─────┘  │
│              电机               │
└─────────────────────────────────┘

PWM+方向模式:
┌─────────────────────────────────┐
│           MCU                   │
│  ┌────────┐    ┌────────┐      │
│  │  PWM   │    │  DIR   │      │
│  └───┬────┘    └───┬────┘      │
└──────┼─────────────┼────────────┘
       │             │
┌──────┴─────────────┴────────────┐
│      电机驱动器 (如 BTS7960)    │
│  ┌─────┐              ┌─────┐  │
│  │ OUT+├──────────────┤OUT- │  │
│  └─────┘      DC      └─────┘  │
│              电机               │
└─────────────────────────────────┘
```

### 基本使用

```cpp
#include <omni/driver/dc_motor_driver.hpp>

// H桥模式
auto pwmA = platform::createPwm(TIM1, CH1);
auto pwmB = platform::createPwm(TIM1, CH2);
omni::driver::DcMotorDriver dcMotor(pwmA, pwmB);

// 或 PWM+方向模式
auto pwm = platform::createPwm(TIM1, CH1);
auto dir = platform::createGpio(PA0);
omni::driver::DcMotorDriver dcMotor(pwm, dir);

// 初始化
dcMotor.init();
dcMotor.enable();

// 设置占空比 (-1.0 ~ 1.0)
dcMotor.setDuty(0.5f);   // 正转 50%
dcMotor.setDuty(-0.3f);  // 反转 30%

// 制动模式
dcMotor.setBrakeMode(omni::driver::BrakeMode::Coast);  // 滑行
dcMotor.setBrakeMode(omni::driver::BrakeMode::Brake);  // 制动
dcMotor.setBrakeMode(omni::driver::BrakeMode::Hold);   // 锁定
```

### 速度闭环控制

```cpp
// 带编码器的速度闭环
auto encoder = platform::createEncoder(TIM2);
omni::driver::DcMotorDriver dcMotor(pwm, dir, encoder);

// 配置PID
dcMotor.setVelocityPid(0.1f, 0.01f, 0.0f);

// 速度控制
dcMotor.setControlMode(omni::driver::ControlMode::Velocity);
dcMotor.setVelocity(10.0f);  // 10 rad/s

// 控制循环
while (true) {
    dcMotor.update();
    delay_ms(1);
}
```

---

## 舵机

### 简介

舵机内置控制器，通过 PWM 脉宽控制角度，使用简单。

### 硬件连接

```
舵机连接:
┌─────────────────────────────────┐
│           MCU                   │
│  ┌────────┐                    │
│  │  PWM   │ (50Hz, 1-2ms脉宽)  │
│  └───┬────┘                    │
└──────┼──────────────────────────┘
       │
┌──────┴──────────────────────────┐
│           舵机                  │
│  ┌───────────────────────────┐ │
│  │ 信号(橙) VCC(红) GND(棕)  │ │
│  └───────────────────────────┘ │
│         PWM → 角度             │
│         1ms → 0°               │
│        1.5ms → 90°             │
│         2ms → 180°             │
└─────────────────────────────────┘
```

### 基本使用

```cpp
#include <omni/driver/servo_driver.hpp>

// 创建舵机驱动
auto pwm = platform::createPwm(TIM1, CH1);
omni::driver::ServoDriver servo(pwm);

// 配置参数
servo.setPulseRange(500, 2500);    // 脉宽范围 (微秒)
servo.setAngleRange(0.0f, 180.0f); // 角度范围 (度)
servo.init();

// 控制角度
servo.setAngle(0.0f);    // 转到0度
servo.setAngle(90.0f);   // 转到90度
servo.setAngle(180.0f);  // 转到180度

// 设置速度限制 (度/秒)
servo.setSpeed(60.0f);   // 60度/秒
```

### 使用高层控制器

```cpp
// 使用MotorController进行平滑运动
omni::app::MotorController motor(&servo);

// 设置运动约束 (角度单位)
omni::motion::MotionConstraints constraints;
constraints.maxVelocity = 90.0f;       // 90度/秒
constraints.maxAcceleration = 180.0f;  // 180度/秒²
motor.setConstraints(constraints);

// 平滑移动到目标角度
motor.enable();
motor.moveWithTrapezoid(90.0f, 60.0f, 120.0f);

while (!motor.isSettled()) {
    motor.update(0.001f);
    servo.update();
    delay_ms(1);
}
```

### 连续旋转舵机

```cpp
// 连续旋转舵机作为速度控制
servo.setContinuousMode(true);

// 设置速度 (-1.0 ~ 1.0)
servo.setVelocity(0.5f);   // 正转 50% 速度
servo.setVelocity(-0.3f);  // 反转 30% 速度
servo.setVelocity(0.0f);   // 停止
```

---

## 音圈电机

### 简介

音圈电机 (VCM) 是一种直线电机，具有响应快、精度高的特点，常用于光学对焦、硬盘磁头等场合。

### 硬件连接

```
音圈电机连接:
┌─────────────────────────────────┐
│           MCU                   │
│  ┌────────┐    ┌────────┐      │
│  │  PWM   │    │  ADC   │      │
│  └───┬────┘    └───┬────┘      │
└──────┼─────────────┼────────────┘
       │             │
┌──────┴─────────────┴────────────┐
│      电流驱动器                  │
│  ┌────────┐    ┌────────┐      │
│  │ 电流输出│    │位置反馈│      │
│  └───┬────┘    └───┬────┘      │
└──────┼─────────────┼────────────┘
       │             │
┌──────┴─────────────┴────────────┐
│      音圈电机                    │
│  ┌───────────────────────────┐ │
│  │ 线圈──────磁铁             │ │
│  │   ↑          │            │ │
│  │   └──位置传感器            │ │
│  └───────────────────────────┘ │
└─────────────────────────────────┘
```

### 基本使用

```cpp
#include <omni/driver/vcm_driver.hpp>

// 创建VCM驱动
auto pwm = platform::createPwm(TIM1, CH1);
auto posAdc = platform::createAdc(ADC1, 0);
omni::driver::VcmDriver vcm(pwm, posAdc);

// 配置参数
vcm.setForceConstant(10.0f);       // 力常数 10 N/A
vcm.setPositionRange(0.0f, 10.0f); // 行程 0-10mm
vcm.init();

// 力控制
vcm.setControlMode(omni::driver::ControlMode::Torque);
vcm.setForce(0.5f);  // 输出 0.5N 推力

// 位置控制
vcm.setControlMode(omni::driver::ControlMode::Position);
vcm.setPositionPid(100.0f, 10.0f, 1.0f);
vcm.setPosition(5.0f);  // 移动到 5mm

// 控制循环
while (true) {
    vcm.update();
    printf("位置: %.3f mm\n", vcm.getLinearPosition());
    delay_us(100);  // 10kHz 控制频率
}
```

### 高带宽控制

```cpp
// 对于高带宽应用（如光学防抖）
vcm.setCurrentBandwidth(2000.0f);  // 2kHz 电流环带宽
vcm.setPositionBandwidth(200.0f);  // 200Hz 位置环带宽

// 使能前馈
vcm.enableFeedforward(true);
vcm.setFeedforwardGain(0.8f);
```

---

## 直线电机

### 简介

直线电机直接产生直线运动，无需丝杠等传动机构，精度高、速度快。

### 基本使用

```cpp
#include <omni/driver/linear_driver.hpp>

// 创建直线电机驱动 (基于FOC)
auto pwm = platform::createPwm3Phase(TIM1);
auto encoder = platform::createLinearEncoder(TIM2);  // 光栅尺
auto adcA = platform::createAdc(ADC1, 0);
auto adcB = platform::createAdc(ADC1, 1);
auto adcC = platform::createAdc(ADC1, 2);

omni::driver::LinearMotorDriver linear(pwm, adcA, adcB, adcC, encoder);

// 配置参数
omni::driver::MotorParams params;
params.polePairs = 10;             // 极对数
params.resistance = 2.0f;          // 相电阻
params.inductance = 0.005f;        // 相电感
params.maxCurrent = 10.0f;
linear.setParams(params);

// 设置线性参数
linear.setPolePitch(0.01f);        // 极距 10mm
linear.setEncoderResolution(0.001f); // 光栅分辨率 1um

// 初始化
linear.init();
linear.enable();

// 位置控制 (单位: m)
linear.setLinearPosition(0.1f);    // 移动到 100mm

// 速度控制 (单位: m/s)
linear.setLinearVelocity(0.5f);    // 0.5 m/s
```

### 使用高层控制器

```cpp
omni::app::MotorController motor(&linear);

// 设置约束 (线性单位)
omni::motion::MotionConstraints constraints;
constraints.maxVelocity = 1.0f;        // 1 m/s
constraints.maxAcceleration = 10.0f;   // 10 m/s²
constraints.maxJerk = 100.0f;          // 100 m/s³
motor.setConstraints(constraints);

// 精密定位
motor.enable();
motor.moveWithSCurve(0.1f, 0.5f, 5.0f, 50.0f);  // 移动到100mm
```

---

## 自定义电机驱动

### 使用回调函数

如果您的电机类型不在支持列表中，可以使用 `CustomMotorDriver` 自定义驱动。

```cpp
#include <omni/driver/custom_driver.hpp>

// 创建自定义驱动
omni::driver::CustomMotorDriver custom;

// 设置初始化回调
custom.setInitCallback([]() {
    // 初始化您的硬件
    myMotor_init();
    return true;
});

// 设置更新回调
custom.setUpdateCallback([]() {
    // 更新电机状态
    myMotor_update();
});

// 设置控制回调
custom.setControlCallback([](omni::driver::ControlMode mode, float value) {
    switch (mode) {
        case omni::driver::ControlMode::Position:
            myMotor_setPosition(value);
            break;
        case omni::driver::ControlMode::Velocity:
            myMotor_setVelocity(value);
            break;
        case omni::driver::ControlMode::Torque:
            myMotor_setTorque(value);
            break;
    }
});

// 设置状态回调
custom.setStateCallback([]() {
    omni::driver::MotorState state;
    state.position = myMotor_getPosition();
    state.velocity = myMotor_getVelocity();
    state.torque = myMotor_getTorque();
    state.enabled = myMotor_isEnabled();
    return state;
});

// 使用
custom.init();
custom.enable();
custom.setPosition(10.0f);
```

### 继承基类

对于更复杂的自定义驱动，建议继承 `IMotorDriver`：

```cpp
class MySpecialMotor : public omni::driver::IMotorDriver {
public:
    bool init() override {
        // 初始化代码
        return true;
    }

    void deinit() override {
        // 清理代码
    }

    void enable() override {
        enabled_ = true;
        // 使能硬件
    }

    void disable() override {
        enabled_ = false;
        // 禁用硬件
    }

    void emergencyStop() override {
        disable();
        // 紧急停止逻辑
    }

    void setControlMode(ControlMode mode) override {
        mode_ = mode;
    }

    ControlMode getControlMode() const override {
        return mode_;
    }

    void setPosition(float pos) override {
        targetPos_ = pos;
    }

    void setVelocity(float vel) override {
        targetVel_ = vel;
    }

    void setTorque(float torque) override {
        targetTorque_ = torque;
    }

    // ... 实现其他方法

    void update() override {
        // 控制循环
        switch (mode_) {
            case ControlMode::Position:
                // 位置控制算法
                break;
            case ControlMode::Velocity:
                // 速度控制算法
                break;
            case ControlMode::Torque:
                // 扭矩控制算法
                break;
        }
    }

    MotorType getType() const override {
        return MotorType::Custom;
    }

    const char* getName() const override {
        return "MySpecialMotor";
    }

private:
    bool enabled_ = false;
    ControlMode mode_ = ControlMode::Off;
    float targetPos_ = 0;
    float targetVel_ = 0;
    float targetTorque_ = 0;
};
```

---

## 协议电机驱动

### MIT 协议

MIT 协议广泛用于四足机器人电机（如 T-Motor、达妙等）。

```cpp
#include <omni/protocol/mit_protocol.hpp>
#include <omni/driver/can_motor_driver.hpp>

// 创建CAN接口
auto can = platform::createCan(CAN1);
can->setBitrate(1000000);  // 1Mbps

// 创建MIT协议驱动
omni::driver::CanMotorDriver motor(can, 1);  // 电机ID = 1

// 配置MIT协议
auto proto = new omni::protocol::MitProtocol();
proto->setPosRange(-12.5f, 12.5f);
proto->setVelRange(-50.0f, 50.0f);
proto->setTorqueRange(-18.0f, 18.0f);
proto->setKpRange(0.0f, 500.0f);
proto->setKdRange(0.0f, 5.0f);
motor.setProtocol(proto);

// 使能电机
motor.init();
motor.enable();

// MIT混合控制
omni::protocol::MitProtocol::Command cmd;
cmd.position = 1.57f;    // 目标位置
cmd.velocity = 0.0f;     // 目标速度
cmd.kp = 50.0f;          // 位置刚度
cmd.kd = 2.0f;           // 阻尼
cmd.torque = 0.0f;       // 前馈扭矩
motor.sendMitCommand(cmd);
```

### DJI 协议

```cpp
#include <omni/protocol/can_protocol.hpp>

auto can = platform::createCan(CAN1);
can->setBitrate(1000000);

// DJI M3508 电机
omni::driver::CanMotorDriver motor(can, 1);
motor.setProtocol(new omni::protocol::DjiCanProtocol());
motor.init();

// 电流控制
motor.setControlMode(omni::driver::ControlMode::Current);
motor.setCurrent(5.0f);  // 5A
```

### ODrive 协议

```cpp
#include <omni/protocol/can_protocol.hpp>

auto can = platform::createCan(CAN1);
omni::driver::CanMotorDriver motor(can, 0);
motor.setProtocol(new omni::protocol::OdriveCanProtocol());
motor.init();

// 位置控制
motor.setControlMode(omni::driver::ControlMode::Position);
motor.setPosition(10.0f);
```

### Modbus RTU

```cpp
#include <omni/protocol/modbus_protocol.hpp>

auto serial = platform::createUart(USART1);
serial->setBaudRate(115200);

omni::driver::ModbusMotorDriver motor(serial, 1);  // 从站地址 1

// 配置寄存器映射
omni::protocol::ModbusRegisterMap map;
map.controlWord = 0x0000;
map.statusWord = 0x0001;
map.targetPosition = 0x0002;
map.actualPosition = 0x0004;
motor.setRegisterMap(map);

motor.init();
motor.enable();
motor.setPosition(1000.0f);  // 发送位置命令
```

---

## 常见问题

### Q: 如何选择控制频率？

**A:** 建议控制频率：
- BLDC/PMSM FOC: 10-20 kHz
- 步进电机: 1-10 kHz
- 有刷DC: 1-5 kHz
- 舵机: 50-100 Hz
- 音圈电机: 5-20 kHz

### Q: 如何调试电机 PID？

**A:** 推荐调试顺序：
1. 先调电流环 (最内环)
2. 再调速度环
3. 最后调位置环

每个环的调试方法：
1. 先加 P，观察响应
2. 加 I 消除稳态误差
3. 加 D 改善动态性能

### Q: 编码器校准失败怎么办？

**A:** 常见原因：
1. 编码器信号质量差 → 检查接线，加滤波
2. 电机负载太大 → 减小负载或增大校准电流
3. 编码器方向错误 → 设置 `encoderInvert`

### Q: 电机振动怎么解决？

**A:**
1. 检查 PID 参数是否过大
2. 使用滤波器滤除高频噪声
3. 使用陷波滤波器消除共振
4. 对于步进电机，增加细分或使用静音模式

