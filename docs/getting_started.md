# OmniMotion 快速入门指南

本指南将帮助您快速了解 OmniMotion 库的基本使用方法。

## 目录

1. [安装配置](#安装配置)
2. [基本概念](#基本概念)
3. [第一个程序](#第一个程序)
4. [常用操作](#常用操作)
5. [下一步](#下一步)

---

## 安装配置

### 方式一：使用 CMake (推荐)

```bash
# 1. 克隆仓库
git clone https://github.com/your-repo/OmniMotion.git

# 2. 在您的 CMakeLists.txt 中添加
add_subdirectory(OmniMotion)
target_link_libraries(your_project PRIVATE omnimotion)
```

### 方式二：头文件包含

OmniMotion 主要是一个头文件库，您可以直接将 `include` 目录添加到您的项目中：

```cpp
// 在您的代码中包含主头文件
#include <omni/omnimotion.hpp>
```

### 编译选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `OMNI_BUILD_EXAMPLES` | ON | 编译示例程序 |
| `OMNI_BUILD_TESTS` | ON | 编译单元测试 |
| `OMNI_BUILD_DOCS` | OFF | 生成文档 |
| `OMNI_BUILD_SHARED` | OFF | 编译为动态库 |

---

## 基本概念

### 分层架构

OmniMotion 采用分层架构设计：

```
应用层 (app)      ← 用户主要使用的接口
    ↓
运动规划层 (motion) ← 轨迹生成
    ↓
控制算法层 (control) ← PID等控制器
    ↓
协议层 (protocol)   ← 通信协议
    ↓
驱动层 (driver)    ← 电机类型抽象
    ↓
硬件抽象层 (hal)    ← 硬件接口
```

### 核心类介绍

| 类 | 命名空间 | 说明 |
|----|----------|------|
| `MotorController` | `omni::app` | 高层电机控制器，提供简单易用的API |
| `IMotorDriver` | `omni::driver` | 电机驱动接口基类 |
| `BldcDriver` | `omni::driver` | BLDC/PMSM电机驱动 |
| `StepperDriver` | `omni::driver` | 步进电机驱动 |
| `PidController` | `omni::control` | PID控制器 |
| `SCurveProfile` | `omni::motion` | S曲线轨迹生成器 |

### 命名空间结构

```cpp
namespace omni {
    namespace hal { }      // 硬件抽象层
    namespace driver { }   // 电机驱动
    namespace protocol { } // 通信协议
    namespace control { }  // 控制算法
    namespace motion { }   // 运动规划
    namespace app { }      // 应用层
    namespace utils { }    // 工具类
}
```

---

## 第一个程序

### 示例：控制步进电机

```cpp
#include <omni/omnimotion.hpp>
#include <iostream>

// 假设您已经实现了平台相关的GPIO类
class MyGpio : public omni::hal::IGpio {
    // ... 实现接口方法
};

int main() {
    // 1. 创建硬件接口
    MyGpio stepPin, dirPin, enablePin;

    // 2. 创建步进电机驱动
    omni::driver::StepperDriver stepper(&stepPin, &dirPin, &enablePin);
    stepper.setMicrostep(16);      // 16细分
    stepper.setStepsPerRev(200);   // 每转200步 (1.8度)
    stepper.init();

    // 3. 创建高层控制器
    omni::app::MotorController motor(&stepper);

    // 4. 配置运动参数
    omni::motion::MotionConstraints constraints;
    constraints.maxVelocity = 10.0f;        // 最大速度: 10 rad/s
    constraints.maxAcceleration = 100.0f;    // 最大加速度: 100 rad/s²
    constraints.maxJerk = 1000.0f;           // 最大加加速度: 1000 rad/s³
    motor.setConstraints(constraints);

    // 5. 使能电机
    motor.enable();
    std::cout << "电机已使能" << std::endl;

    // 6. 执行运动 - 转动5圈
    float targetPos = 5.0f * 2.0f * 3.14159f;  // 5圈
    motor.moveTo(targetPos);
    std::cout << "目标位置: " << targetPos << " rad" << std::endl;

    // 7. 主循环
    const float dt = 0.001f;  // 1ms控制周期
    while (!motor.isSettled()) {
        motor.update(dt);
        stepper.update();

        // 打印当前位置
        std::cout << "位置: " << motor.getPosition() << " rad\r";

        // 等待1ms (平台相关)
        delay_ms(1);
    }

    std::cout << "\n运动完成!" << std::endl;

    // 8. 禁用电机
    motor.disable();

    return 0;
}
```

### 示例：BLDC电机速度控制

```cpp
#include <omni/omnimotion.hpp>

int main() {
    // 创建硬件接口 (平台相关实现)
    auto pwm = createPwm3Phase();
    auto encoder = createEncoder();
    auto adcA = createAdc();
    auto adcB = createAdc();
    auto adcC = createAdc();

    // 创建BLDC驱动
    omni::driver::BldcDriver bldc(pwm, adcA, adcB, adcC, encoder);

    // 配置电机参数
    omni::driver::MotorParams params;
    params.polePairs = 7;           // 7对极
    params.resistance = 0.5f;       // 相电阻 0.5Ω
    params.inductance = 0.001f;     // 相电感 1mH
    params.maxCurrent = 20.0f;      // 最大电流 20A
    params.encoderCpr = 4096;       // 编码器分辨率
    bldc.setParams(params);

    // 校准编码器
    bldc.calibrateEncoder();

    // 创建控制器
    omni::app::MotorController motor(&bldc);
    motor.setVelocityPid(0.5f, 0.01f, 0.0f);
    motor.enable();

    // 速度控制
    float targetVelocity = 100.0f;  // 100 rad/s
    motor.setVelocity(targetVelocity);

    // 运行5秒
    for (int i = 0; i < 5000; i++) {
        motor.update(0.001f);
        bldc.update();
        delay_ms(1);
    }

    motor.disable();
    return 0;
}
```

---

## 常用操作

### 位置控制

```cpp
// 移动到绝对位置
motor.moveTo(100.0f);

// 相对移动
motor.moveBy(10.0f);

// 使用S曲线规划
motor.moveWithSCurve(100.0f, 10.0f, 100.0f, 1000.0f);
//                    目标位置  最大速度  最大加速度  最大加加速度

// 使用梯形曲线规划
motor.moveWithTrapezoid(100.0f, 10.0f, 100.0f);
```

### 速度控制

```cpp
// 设置目标速度
motor.setVelocity(50.0f);

// 带加速度限制的速度设置
motor.setVelocity(50.0f, 100.0f);

// 斜坡加速到目标速度
motor.rampVelocity(100.0f, 2.0f);  // 2秒内加速到100
```

### 扭矩/力控制

```cpp
// 设置扭矩 (旋转电机)
motor.setTorque(5.0f);  // 5 Nm

// 设置力 (直线电机)
motor.setForce(10.0f);  // 10 N
```

### 状态查询

```cpp
float pos = motor.getPosition();    // 获取位置
float vel = motor.getVelocity();    // 获取速度
float torque = motor.getTorque();   // 获取扭矩

bool moving = motor.isMoving();     // 是否在运动
bool settled = motor.isSettled();   // 是否到位
bool error = motor.hasError();      // 是否有错误
```

### 回零操作

```cpp
// 使用限位传感器回零
motor.home(omni::app::HomingMode::SensorBased);

// 使用电流检测回零 (碰到机械限位)
motor.home(omni::app::HomingMode::CurrentBased);

// 设置原点偏移
motor.setHomeOffset(0.5f);
```

### 软件限位

```cpp
// 设置软件限位
motor.setSoftLimits(-100.0f, 100.0f);

// 使能软件限位
motor.enableSoftLimits(true);
```

### 紧急停止

```cpp
// 紧急停止
motor.emergencyStop();
```

---

## 下一步

- 阅读 [API 参考手册](api_reference.md) 了解完整的API
- 阅读 [电机驱动指南](motor_drivers.md) 了解各种电机的使用方法
- 阅读 [控制算法详解](control_algorithms.md) 了解控制算法的原理和调参
- 阅读 [运动规划教程](motion_planning.md) 了解轨迹规划的使用
- 阅读 [平台移植指南](porting_guide.md) 了解如何在您的硬件上使用
- 查看 [示例代码](../examples/) 获取更多使用示例
