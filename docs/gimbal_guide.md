# OmniMotion 双轴云台实现指南

## 概述

使用 OmniMotion 库实现双轴云台（Pan-Tilt），支持指定坐标运动。

## 架构

```
GimbalController
├── MotorController (Pan轴)
│   └── StepperDriver
├── MotorController (Tilt轴)
│   └── StepperDriver
└── 坐标转换 (可选)
```

## 基本用法

### 1. 初始化两个电机

```cpp
#include <omni/driver/stepper_driver.hpp>
#include <omni/app/motor_controller.hpp>

// 硬件接口
StepperDriver panDriver(&panStep, &panDir, &panEncoder, &panEnable);
StepperDriver tiltDriver(&tiltStep, &tiltDir, &tiltEncoder, &tiltEnable);

// 运动控制器
MotorController panMotor(&panDriver);
MotorController tiltMotor(&tiltDriver);

// 配置约束
MotionConstraints constraints;
constraints.maxVelocity = 5.0f;       // rad/s
constraints.maxAcceleration = 20.0f;  // rad/s²
constraints.maxJerk = 100.0f;         // rad/s³

panMotor.setConstraints(constraints);
tiltMotor.setConstraints(constraints);
```

### 2. 移动到指定角度

```cpp
// 目标角度 (弧度)
float panAngle = 1.57f;   // 90°
float tiltAngle = 0.52f;  // 30°

// 启动运动
panMotor.enable();
tiltMotor.enable();
panMotor.moveTo(panAngle);
tiltMotor.moveTo(tiltAngle);
```

### 3. 控制循环

```cpp
// 1kHz 定时器中断
void controlLoop(float dt) {
    // 更新编码器
    panEncoder.update(dt);
    tiltEncoder.update(dt);

    // 更新控制器
    panMotor.update(dt);
    tiltMotor.update(dt);

    // 更新驱动
    panDriver.update();
    tiltDriver.update();
}
```

### 4. 检查是否到达

```cpp
bool isSettled() {
    return panMotor.isSettled() && tiltMotor.isSettled();
}
```

## 进阶：封装云台控制器

```cpp
class GimbalController {
public:
    GimbalController(MotorController* pan, MotorController* tilt)
        : pan_(pan), tilt_(tilt) {}

    void init() {
        pan_->enable();
        tilt_->enable();
    }

    // 移动到角度 (弧度)
    void moveTo(float panAngle, float tiltAngle) {
        pan_->moveTo(panAngle);
        tilt_->moveTo(tiltAngle);
    }

    // 移动到角度 (度)
    void moveToDeg(float panDeg, float tiltDeg) {
        moveTo(panDeg * DEG_TO_RAD, tiltDeg * DEG_TO_RAD);
    }

    // 获取当前位置
    void getPosition(float& pan, float& tilt) {
        pan = pan_->getPosition();
        tilt = tilt_->getPosition();
    }

    // 是否到达目标
    bool isSettled() {
        return pan_->isSettled() && tilt_->isSettled();
    }

    // 更新 (在控制循环中调用)
    void update(float dt) {
        pan_->update(dt);
        tilt_->update(dt);
    }

private:
    static constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
    MotorController* pan_;
    MotorController* tilt_;
};
```

## 进阶：笛卡尔坐标指向

如果需要指向空间中某点 (x, y, z)：

```cpp
void lookAt(float x, float y, float z) {
    // 计算方位角 (Pan)
    float panAngle = std::atan2(x, z);

    // 计算俯仰角 (Tilt)
    float dist = std::sqrt(x*x + z*z);
    float tiltAngle = std::atan2(y, dist);

    moveTo(panAngle, tiltAngle);
}
```

## 同步运动

默认情况下两轴独立运动。如需同步到达，可调整约束使运动时间一致：

```cpp
void moveToSync(float panTarget, float tiltTarget) {
    float panDist = std::fabs(panTarget - pan_->getPosition());
    float tiltDist = std::fabs(tiltTarget - tilt_->getPosition());

    // 计算各轴所需时间，取较长者
    float panTime = estimateTime(panDist, panConstraints_);
    float tiltTime = estimateTime(tiltDist, tiltConstraints_);
    float maxTime = std::max(panTime, tiltTime);

    // 调整较快轴的速度
    if (panTime < maxTime) {
        adjustVelocity(pan_, panDist, maxTime);
    }
    if (tiltTime < maxTime) {
        adjustVelocity(tilt_, tiltDist, maxTime);
    }

    pan_->moveTo(panTarget);
    tilt_->moveTo(tiltTarget);
}
```

## 限位保护

```cpp
void moveTo(float panAngle, float tiltAngle) {
    // 限制范围
    panAngle = std::clamp(panAngle, PAN_MIN, PAN_MAX);
    tiltAngle = std::clamp(tiltAngle, TILT_MIN, TILT_MAX);

    pan_->moveTo(panAngle);
    tilt_->moveTo(tiltAngle);
}

static constexpr float PAN_MIN = -3.14f;   // -180°
static constexpr float PAN_MAX = 3.14f;    // +180°
static constexpr float TILT_MIN = -1.57f;  // -90°
static constexpr float TILT_MAX = 1.57f;   // +90°
```
