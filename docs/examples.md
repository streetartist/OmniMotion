# OmniMotion 示例代码说明

本文档详细介绍 OmniMotion 提供的示例程序。

## 目录

1. [示例概览](#示例概览)
2. [BLDC FOC 控制](#bldc-foc-控制)
3. [步进电机 S 曲线运动](#步进电机-s-曲线运动)
4. [多轴同步运动](#多轴同步运动)
5. [轨迹跟踪](#轨迹跟踪)
6. [示教回放](#示教回放)
7. [运行示例](#运行示例)

---

## 示例概览

| 示例 | 目录 | 功能 | 所需硬件 |
|------|------|------|----------|
| BLDC FOC | `examples/bldc_foc/` | BLDC 电机 FOC 控制 | BLDC电机、驱动板、编码器 |
| 步进 S 曲线 | `examples/stepper_scurve/` | 步进电机平滑运动 | 步进电机、驱动器 |
| 多轴同步 | `examples/multi_axis_sync/` | 多轴同步运动 | 多个电机 |
| 轨迹跟踪 | `examples/trajectory_tracking/` | 轨迹跟踪控制 | 任意电机 |
| 示教回放 | `examples/teach_playback/` | 示教和回放 | 任意电机 |

---

## BLDC FOC 控制

### 目录结构

```
examples/bldc_foc/
├── main.cpp
├── CMakeLists.txt
└── README.md
```

### 功能说明

该示例演示如何使用 OmniMotion 进行 BLDC 电机的 FOC (磁场定向控制)：

- 三相 PWM 输出
- 三相电流采样
- 编码器读取
- 电流环、速度环、位置环级联控制
- S 曲线运动规划

### 代码结构

```cpp
// 1. 创建硬件接口
SimPwm3Phase pwm;
SimAdc adcA, adcB, adcC;
SimEncoder encoder;

// 2. 创建 BLDC 驱动
omni::driver::BldcDriver bldc(&pwm, &adcA, &adcB, &adcC, &encoder);

// 3. 配置电机参数
omni::driver::MotorParams params;
params.polePairs = 7;
params.resistance = 0.5f;
params.inductance = 0.001f;
params.maxCurrent = 20.0f;
params.encoderCpr = 4096;
bldc.setParams(params);

// 4. 初始化
bldc.init();
bldc.calibrateEncoder();

// 5. 配置控制器
bldc.setCurrentPid(0.5f, 0.01f, 0.0f);
bldc.setVelocityPid(0.1f, 0.001f, 0.0f);
bldc.setPositionPid(50.0f, 0.0f, 0.5f);

// 6. 创建高层控制器
omni::app::MotorController motor(&bldc);

// 7. 执行运动
motor.enable();
motor.moveWithSCurve(M_PI, 10.0f, 100.0f, 1000.0f);

// 8. 控制循环
while (!motor.isSettled()) {
    motor.update(dt);
    bldc.update();
}
```

### 关键技术点

**FOC 控制原理:**

```
                 ┌─────────────────────────────────────┐
                 │           位置控制器                 │
目标位置 ───────→│  ┌─────┐                            │
                 │  │位置PID│───→ 速度目标              │
                 │  └─────┘                            │
                 │     ↑                               │
                 │   位置反馈                           │
                 └─────────────────────────────────────┘
                                 │
                                 ▼
                 ┌─────────────────────────────────────┐
                 │           速度控制器                 │
                 │  ┌─────┐                            │
                 │  │速度PID│───→ Iq 目标              │
                 │  └─────┘                            │
                 │     ↑                               │
                 │   速度反馈                           │
                 └─────────────────────────────────────┘
                                 │
                                 ▼
                 ┌─────────────────────────────────────┐
                 │           电流控制器                 │
Id目标=0 ──────→│  ┌───────┐   ┌───────┐              │
                 │  │Id PID │   │Iq PID │              │
                 │  └───────┘   └───────┘              │
                 │     ↑           ↑                   │
                 │   Id反馈      Iq反馈                 │
                 └─────────────────────────────────────┘
                                 │
                                 ▼
                 ┌─────────────────────────────────────┐
                 │        逆 Park / Clarke 变换        │
                 │                                     │
                 │    (Vd, Vq) ───→ (Vα, Vβ)          │
                 └─────────────────────────────────────┘
                                 │
                                 ▼
                 ┌─────────────────────────────────────┐
                 │              SVPWM                  │
                 │                                     │
                 │    (Vα, Vβ) ───→ (Ta, Tb, Tc)      │
                 └─────────────────────────────────────┘
```

---

## 步进电机 S 曲线运动

### 目录结构

```
examples/stepper_scurve/
├── main.cpp
├── CMakeLists.txt
└── README.md
```

### 功能说明

该示例演示步进电机的 S 曲线运动控制：

- 脉冲/方向控制
- S 曲线轨迹规划
- 平滑加减速
- 运动状态监控

### 代码结构

```cpp
// 1. 创建 GPIO
SimGpio stepPin("STEP");
SimGpio dirPin("DIR");
SimGpio enablePin("EN");

// 2. 创建步进驱动
omni::driver::StepperDriver stepper(&stepPin, &dirPin, &enablePin);
stepper.setStepsPerRev(200);
stepper.setMicrostep(16);
stepper.setMaxStepRate(100000);
stepper.init();

// 3. 创建控制器
omni::app::MotorController motor(&stepper);

// 4. 设置运动约束
omni::motion::MotionConstraints constraints;
constraints.maxVelocity = 10.0f;
constraints.maxAcceleration = 100.0f;
constraints.maxJerk = 1000.0f;
motor.setConstraints(constraints);

// 5. 执行 S 曲线运动
motor.enable();
motor.moveWithSCurve(10.0f * 2 * M_PI, 5.0f * 2 * M_PI,
                     20.0f * 2 * M_PI, 200.0f * 2 * M_PI);

// 6. 控制循环
while (!motor.isSettled()) {
    motor.update(dt);
    stepper.update();
    printf("位置: %.2f rad, 速度: %.2f rad/s\n",
           motor.getPosition(), motor.getVelocity());
}
```

### S 曲线可视化

```
速度
  │
  │          ╱───────────╲
  │         ╱             ╲
  │        ╱               ╲
  │       ╱                 ╲
  │──────╱                   ╲──────
  └────────────────────────────────── 时间

加速度
  │
  │      ╱─────╲
  │     ╱       ╲
  │────╱         ╲────
  │                    ╲       ╱
  │                     ╲─────╱
  └────────────────────────────────── 时间
```

---

## 多轴同步运动

### 目录结构

```
examples/multi_axis_sync/
├── main.cpp
├── CMakeLists.txt
└── README.md
```

### 功能说明

该示例演示多个电机的同步运动：

- 三轴 (XYZ) 同步控制
- 直线插补
- 圆弧插补
- 同时到达目标点

### 代码结构

```cpp
// 1. 创建三个电机驱动
SimGpio stepX("X_STEP"), dirX("X_DIR"), enX("X_EN");
SimGpio stepY("Y_STEP"), dirY("Y_DIR"), enY("Y_EN");
SimGpio stepZ("Z_STEP"), dirZ("Z_DIR"), enZ("Z_EN");

omni::driver::StepperDriver driverX(&stepX, &dirX, &enX);
omni::driver::StepperDriver driverY(&stepY, &dirY, &enY);
omni::driver::StepperDriver driverZ(&stepZ, &dirZ, &enZ);

// 2. 创建多轴控制器
omni::app::MultiAxisController robot(3);
robot.addAxis(0, &driverX);
robot.addAxis(1, &driverY);
robot.addAxis(2, &driverZ);

// 3. 同步移动到目标点
std::vector<float> target = {100.0f, 50.0f, 25.0f};
robot.moveTo(target);

// 4. 等待完成
while (!robot.allSettled()) {
    robot.update(dt);
}

// 5. 直线插补
robot.linearMove({200.0f, 100.0f, 50.0f}, 10.0f);  // 进给速度 10

// 6. 圆弧插补
robot.arcMove({300.0f, 100.0f, 50.0f},   // 终点
              {250.0f, 100.0f, 50.0f},   // 圆心
              omni::app::ArcPlane::XY,   // XY 平面
              true);                      // 顺时针
```

### 同步原理

```
时间同步前:
  轴X: ─────────────────────────────→
  轴Y: ──────────────→
  轴Z: ────────→

时间同步后 (所有轴同时到达):
  轴X: ─────────────────────────────→
  轴Y: ─────────────────────────────→ (降低速度)
  轴Z: ─────────────────────────────→ (降低速度)
                                     ↑
                                同时到达
```

---

## 轨迹跟踪

### 目录结构

```
examples/trajectory_tracking/
├── main.cpp
├── trajectory.csv
├── CMakeLists.txt
└── README.md
```

### 功能说明

该示例演示轨迹跟踪控制：

- 从文件加载轨迹
- 实时轨迹跟踪
- 速度倍率调节
- 暂停/恢复功能

### 代码结构

```cpp
// 1. 创建电机和控制器
SimGpio step("STEP"), dir("DIR"), en("EN");
omni::driver::StepperDriver driver(&step, &dir, &en);
omni::app::MotorController motor(&driver);

// 2. 创建轨迹跟踪器
omni::app::TrajectoryTracker tracker(&motor);

// 3. 加载轨迹
tracker.loadFromFile("trajectory.csv");

// 4. 设置速度倍率
tracker.setSpeedOverride(1.0f);  // 100% 速度

// 5. 开始跟踪
motor.enable();
tracker.start();

// 6. 控制循环
while (!tracker.isFinished()) {
    tracker.update(dt);
    motor.update(dt);
    driver.update();

    printf("进度: %.1f%%, 位置: %.2f\n",
           tracker.getProgress() * 100, motor.getPosition());

    // 模拟用户输入
    if (pauseRequested) {
        tracker.pause();
    }
    if (resumeRequested) {
        tracker.resume();
    }
}
```

### 轨迹文件格式

```csv
# trajectory.csv
# time,position,velocity,acceleration
0.0,0.0,0.0,0.0
0.1,0.5,10.0,100.0
0.2,2.0,20.0,100.0
0.3,4.5,30.0,100.0
...
```

---

## 示教回放

### 目录结构

```
examples/teach_playback/
├── main.cpp
├── CMakeLists.txt
└── README.md
```

### 功能说明

该示例演示示教和回放功能：

- 手动移动电机记录点
- 保存轨迹到文件
- 加载并回放轨迹
- 循环回放

### 代码结构

```cpp
// 1. 创建多轴控制器
omni::app::MultiAxisController robot(3);
robot.addAxis(0, &motor0);
robot.addAxis(1, &motor1);
robot.addAxis(2, &motor2);

// 2. 创建示教器
omni::app::TeachRecorder teacher(&robot);

// 3. 示教模式
teacher.startTeaching();
printf("进入示教模式，请手动移动机器人...\n");

// 4. 记录关键点
while (teaching) {
    if (recordButtonPressed) {
        teacher.recordPoint();
        printf("记录点 %zu: [%.2f, %.2f, %.2f]\n",
               teacher.getPointCount(),
               robot.getAxis(0)->getPosition(),
               robot.getAxis(1)->getPosition(),
               robot.getAxis(2)->getPosition());
    }

    if (finishButtonPressed) {
        break;
    }
}

teacher.stopTeaching();

// 5. 保存轨迹
teacher.saveToFile("taught_trajectory.json");

// 6. 回放
printf("开始回放...\n");
teacher.playback(0.5f);  // 50% 速度

while (teacher.isPlaying()) {
    teacher.update(dt);
    robot.update(dt);
}

// 7. 循环回放
teacher.playbackLoop(3);  // 重复3次
```

### 示教文件格式

```json
{
  "version": "1.0",
  "axes": 3,
  "points": [
    {
      "time": 0.0,
      "positions": [0.0, 0.0, 0.0]
    },
    {
      "time": 1.0,
      "positions": [100.0, 50.0, 25.0]
    },
    {
      "time": 2.5,
      "positions": [200.0, 100.0, 50.0]
    }
  ]
}
```

---

## 运行示例

### 编译

```bash
# 在项目根目录
mkdir build && cd build
cmake .. -DOMNI_BUILD_EXAMPLES=ON
cmake --build .
```

### 运行

```bash
# BLDC FOC 示例
./examples/bldc_foc/bldc_foc_example

# 步进电机示例
./examples/stepper_scurve/stepper_scurve_example

# 多轴同步示例
./examples/multi_axis_sync/multi_axis_sync_example

# 轨迹跟踪示例
./examples/trajectory_tracking/trajectory_tracking_example

# 示教回放示例
./examples/teach_playback/teach_playback_example
```

### 预期输出

**BLDC FOC 示例:**
```
OmniMotion BLDC FOC 示例
========================
初始化 BLDC 驱动...
配置电机参数...
校准编码器...
使能电机
执行 S 曲线运动到 π rad
t=0.000: pos=0.00, vel=0.00, acc=0.00
t=0.100: pos=0.05, vel=1.00, acc=10.00
...
运动完成！最终位置: 3.14 rad
```

**步进电机示例:**
```
OmniMotion 步进电机 S 曲线示例
==============================
初始化步进驱动...
配置: 200步/转, 16细分
开始运动: 目标 10 圈
t=0.000: pos=0.00 rad, vel=0.00 rad/s
t=0.100: pos=0.31 rad, vel=3.14 rad/s
...
运动完成！总步数: 32000
```

**多轴同步示例:**
```
OmniMotion 多轴同步示例
======================
初始化 3 轴...
同步移动到 [100, 50, 25]
同步时间: 2.500 s
t=0.000: X=0.00, Y=0.00, Z=0.00
t=0.500: X=20.00, Y=10.00, Z=5.00
...
所有轴到达目标位置
```

### 在实际硬件上运行

要在实际硬件上运行示例，需要：

1. 实现对应平台的 HAL 层
2. 修改示例中的硬件接口创建代码
3. 调整控制参数以匹配实际电机

```cpp
// 示例修改：从仿真改为 STM32
// 原代码:
SimGpio step("STEP"), dir("DIR"), en("EN");

// 改为:
#include "platform/stm32/stm32_hal.hpp"
Stm32Gpio step(GPIOA, GPIO_PIN_0);
Stm32Gpio dir(GPIOA, GPIO_PIN_1);
Stm32Gpio en(GPIOA, GPIO_PIN_2);
```

---

## 扩展示例

基于这些示例，您可以进一步开发：

- **CNC 控制**: 基于多轴同步 + G 代码解析
- **机器人关节**: 基于 BLDC FOC + 轨迹规划
- **3D 打印机**: 基于步进电机 + 多轴同步
- **自动化设备**: 基于示教回放 + 安全监控

