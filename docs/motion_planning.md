# OmniMotion 运动规划教程

本教程详细介绍 OmniMotion 的运动规划功能，包括各种轨迹曲线的原理和使用方法。

## 目录

1. [运动规划基础](#运动规划基础)
2. [梯形速度曲线](#梯形速度曲线)
3. [S型速度曲线](#s型速度曲线)
4. [多项式轨迹](#多项式轨迹)
5. [样条曲线](#样条曲线)
6. [在线轨迹规划](#在线轨迹规划)
7. [多轴同步运动](#多轴同步运动)
8. [轨迹跟踪](#轨迹跟踪)

---

## 运动规划基础

### 为什么需要运动规划？

直接给电机发送目标位置会导致：
- 加速度无穷大 → 机械冲击
- 电流瞬间很大 → 过流保护
- 定位精度下降

运动规划通过限制速度、加速度、加加速度，实现平稳运动。

### 运动约束

```cpp
struct MotionConstraints {
    float maxVelocity;      // 最大速度 (rad/s 或 m/s)
    float maxAcceleration;  // 最大加速度 (rad/s² 或 m/s²)
    float maxDeceleration;  // 最大减速度 (可选)
    float maxJerk;          // 最大加加速度 (rad/s³ 或 m/s³)
};
```

### 轨迹点

```cpp
struct TrajectoryPoint {
    float time;          // 时间 (s)
    float position;      // 位置
    float velocity;      // 速度
    float acceleration;  // 加速度
    float jerk;          // 加加速度
};
```

### 曲线类型对比

| 曲线类型 | 加速度 | 加加速度 | 平滑度 | 计算量 | 适用场景 |
|----------|--------|----------|--------|--------|----------|
| 梯形 | 阶跃 | 无穷大 | 低 | 低 | 一般定位 |
| S曲线 | 连续 | 有限 | 高 | 中 | 精密设备 |
| 多项式 | 连续 | 连续 | 很高 | 中 | 机器人 |
| 样条 | 连续 | 连续 | 最高 | 高 | 复杂路径 |

---

## 梯形速度曲线

### 原理介绍

梯形曲线是最简单的运动规划，速度曲线呈梯形。

```
速度
  │
  │     ┌───────────┐
  │    ╱             ╲
  │   ╱               ╲
  │  ╱                 ╲
  └─────────────────────── 时间
    加速    匀速    减速
```

**三个阶段:**
1. 加速段：从 0 加速到最大速度
2. 匀速段：以最大速度运动
3. 减速段：从最大速度减速到 0

### 基本使用

```cpp
#include <omni/motion/trapezoidal_profile.hpp>

using namespace omni::motion;

// 创建运动约束
MotionConstraints constraints;
constraints.maxVelocity = 10.0f;       // 10 rad/s
constraints.maxAcceleration = 100.0f;  // 100 rad/s²

// 创建梯形曲线生成器
TrapezoidalProfile profile(constraints);

// 规划轨迹
float startPos = 0.0f;
float endPos = 100.0f;
profile.plan(startPos, endPos);

// 检查规划是否成功
if (!profile.isValid()) {
    printf("规划失败!\n");
    return;
}

// 获取总时间
float duration = profile.getDuration();
printf("运动时间: %.3f s\n", duration);
```

### 执行轨迹

```cpp
// 方法一：按时间评估
float t = 0;
float dt = 0.001f;

while (t <= profile.getDuration()) {
    TrajectoryPoint point = profile.evaluate(t);

    printf("t=%.3f: pos=%.2f, vel=%.2f, acc=%.2f\n",
           t, point.position, point.velocity, point.acceleration);

    // 发送到电机控制器
    motor.setPosition(point.position);
    motor.setVelocityFeedforward(point.velocity);

    t += dt;
    delay_ms(1);
}
```

### 短距离运动

当距离不足以达到最大速度时，自动变为三角形：

```cpp
// 短距离示例
profile.plan(0.0f, 1.0f);  // 只移动 1 rad

float peakVel = profile.getPeakVelocity();
printf("峰值速度: %.2f (低于最大速度 %.2f)\n", peakVel, constraints.maxVelocity);
```

### 获取各段时间

```cpp
float accelTime = profile.getAccelTime();
float constTime = profile.getConstTime();
float decelTime = profile.getDecelTime();

printf("加速: %.3f s, 匀速: %.3f s, 减速: %.3f s\n",
       accelTime, constTime, decelTime);
```

---

## S型速度曲线

### 原理介绍

S曲线（七段式）通过限制加加速度使加速度连续变化，更加平滑。

```
加速度
  │
  │      ╱─────╲
  │     ╱       ╲
  │────╱         ╲────
  │                    ╲       ╱
  │                     ╲─────╱
  └────────────────────────────── 时间
    T1  T2  T3  T4  T5  T6  T7
```

**七个阶段:**
1. T1: 加加速（加速度增加）
2. T2: 匀加速（加速度恒定）
3. T3: 减加速（加速度减小到0）
4. T4: 匀速（加速度为0）
5. T5: 加减速（加速度增加，负向）
6. T6: 匀减速（加速度恒定，负向）
7. T7: 减减速（加速度减小到0）

### 基本使用

```cpp
#include <omni/motion/scurve_profile.hpp>

using namespace omni::motion;

// 创建约束
MotionConstraints constraints;
constraints.maxVelocity = 10.0f;       // 最大速度
constraints.maxAcceleration = 100.0f;  // 最大加速度
constraints.maxJerk = 1000.0f;         // 最大加加速度

// 创建S曲线生成器
SCurveProfile profile(constraints);

// 规划轨迹
profile.plan(0.0f, 100.0f);

if (profile.isValid()) {
    printf("规划成功，时间: %.3f s\n", profile.getDuration());
}
```

### 获取各段时间

```cpp
float times[7];
profile.getSegmentTimes(times);

printf("七段时间:\n");
printf("T1 (加加速): %.4f s\n", times[0]);
printf("T2 (匀加速): %.4f s\n", times[1]);
printf("T3 (减加速): %.4f s\n", times[2]);
printf("T4 (匀  速): %.4f s\n", times[3]);
printf("T5 (加减速): %.4f s\n", times[4]);
printf("T6 (匀减速): %.4f s\n", times[5]);
printf("T7 (减减速): %.4f s\n", times[6]);
```

### 带初始速度/终止速度

```cpp
// 从运动中开始/结束
float startVel = 5.0f;  // 初始速度 5 rad/s
float endVel = 2.0f;    // 终止速度 2 rad/s

profile.plan(0.0f, 100.0f, startVel, endVel);
```

### 与电机控制器配合

```cpp
omni::app::MotorController motor(&driver);

// 使用内置的 S 曲线运动
motor.moveWithSCurve(
    100.0f,    // 目标位置
    10.0f,     // 最大速度
    100.0f,    // 最大加速度
    1000.0f    // 最大加加速度
);

// 控制循环
while (!motor.isSettled()) {
    motor.update(0.001f);
    delay_ms(1);
}
```

---

## 多项式轨迹

### 原理介绍

多项式轨迹通过指定边界条件（位置、速度、加速度等）来确定轨迹。

**三次多项式:** 可指定起止位置和速度
```
p(t) = a₀ + a₁t + a₂t² + a₃t³
```

**五次多项式:** 可指定起止位置、速度和加速度
```
p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

**七次多项式:** 可指定起止位置、速度、加速度和加加速度

### 三次多项式

```cpp
#include <omni/motion/polynomial_trajectory.hpp>

using namespace omni::motion;

// 三次多项式：指定起止位置和速度
auto traj = PolynomialTrajectory::cubic(
    2.0f,      // 持续时间 2s
    0.0f,      // 起始位置 0
    100.0f,    // 终止位置 100
    0.0f,      // 起始速度 0
    0.0f       // 终止速度 0
);

// 评估轨迹
TrajectoryPoint point = traj.evaluate(1.0f);  // t=1s 时的状态
```

### 五次多项式

```cpp
// 五次多项式：额外指定起止加速度
auto traj = PolynomialTrajectory::quintic(
    2.0f,      // 持续时间
    0.0f,      // 起始位置
    100.0f,    // 终止位置
    0.0f,      // 起始速度
    0.0f,      // 终止速度
    0.0f,      // 起始加速度
    0.0f       // 终止加速度
);
```

### 七次多项式

```cpp
// 七次多项式：额外指定起止加加速度
auto traj = PolynomialTrajectory::septic(
    2.0f,      // 持续时间
    0.0f,      // 起始位置
    100.0f,    // 终止位置
    0.0f,      // 起始速度
    0.0f,      // 终止速度
    0.0f,      // 起始加速度
    0.0f,      // 终止加速度
    0.0f,      // 起始加加速度
    0.0f       // 终止加加速度
);
```

### 应用：机器人关节运动

```cpp
// 从当前状态平滑过渡到目标状态
float currentPos = motor.getPosition();
float currentVel = motor.getVelocity();
float targetPos = 1.57f;  // π/2
float targetVel = 0.0f;

auto traj = PolynomialTrajectory::quintic(
    1.0f,           // 1秒完成
    currentPos, targetPos,
    currentVel, targetVel,
    0.0f, 0.0f      // 加速度归零
);

// 执行
for (float t = 0; t <= 1.0f; t += 0.001f) {
    auto point = traj.evaluate(t);
    motor.setPosition(point.position);
    delay_ms(1);
}
```

---

## 样条曲线

### 原理介绍

样条曲线用于通过多个路径点的平滑轨迹。

### 支持的样条类型

| 类型 | 特点 | 适用场景 |
|------|------|----------|
| 三次样条 | 位置、速度连续 | 一般路径 |
| B样条 | 局部可调 | 交互设计 |
| Bezier | 控制点直观 | 图形动画 |
| Catmull-Rom | 过所有控制点 | 动画 |

### 三次样条

```cpp
#include <omni/motion/spline_trajectory.hpp>

using namespace omni::motion;

// 创建三次样条
SplineTrajectory spline(SplineTrajectory::Type::CubicSpline);

// 添加路径点
spline.addWaypoint(0.0f);     // 位置 0
spline.addWaypoint(50.0f);    // 位置 50
spline.addWaypoint(80.0f);    // 位置 80
spline.addWaypoint(100.0f);   // 位置 100

// 设置速度限制（自动计算时间）
spline.setVelocityLimit(10.0f);

// 生成轨迹
spline.generate();

// 使用
float duration = spline.getDuration();
for (float t = 0; t <= duration; t += 0.01f) {
    auto point = spline.evaluate(t);
    motor.setPosition(point.position);
}
```

### 指定时间的路径点

```cpp
SplineTrajectory spline(SplineTrajectory::Type::CubicSpline);

// 添加带时间的路径点
spline.addWaypoint(0.0f, 0.0f);      // t=0s, pos=0
spline.addWaypoint(50.0f, 1.0f);     // t=1s, pos=50
spline.addWaypoint(80.0f, 1.5f);     // t=1.5s, pos=80
spline.addWaypoint(100.0f, 2.0f);    // t=2s, pos=100

spline.generate();
```

### 添加完整轨迹点

```cpp
SplineTrajectory spline(SplineTrajectory::Type::CubicSpline);

// 添加完整轨迹点（包含速度信息）
TrajectoryPoint p1 = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};      // 静止开始
TrajectoryPoint p2 = {1.0f, 50.0f, 10.0f, 0.0f, 0.0f};    // 匀速通过
TrajectoryPoint p3 = {2.0f, 100.0f, 0.0f, 0.0f, 0.0f};    // 静止结束

spline.addWaypoint(p1);
spline.addWaypoint(p2);
spline.addWaypoint(p3);

spline.generate();
```

---

## 在线轨迹规划

### 原理介绍

在线规划允许在运动过程中修改目标，而不需要停止。

### 基本使用

```cpp
#include <omni/motion/online_planner.hpp>

using namespace omni::motion;

// 创建在线规划器
MotionConstraints constraints;
constraints.maxVelocity = 10.0f;
constraints.maxAcceleration = 100.0f;
constraints.maxJerk = 1000.0f;

OnlineTrajectoryPlanner planner(constraints);

// 设置初始目标
planner.setTarget(100.0f);

// 控制循环
while (true) {
    // 更新规划器
    TrajectoryPoint point = planner.update(0.001f);

    // 发送到电机
    motor.setPosition(point.position);

    // 检查用户输入，可能修改目标
    if (newTargetReceived) {
        planner.setTarget(newTarget);  // 平滑切换到新目标
    }

    // 检查是否到达
    if (planner.isSettled()) {
        printf("到达目标位置\n");
    }

    delay_ms(1);
}
```

### 动态修改目标

```cpp
// 初始目标
planner.setTarget(100.0f);

// 运动过程中修改目标
// 规划器会自动计算从当前状态到新目标的最优路径
planner.setTarget(50.0f);   // 改为更近的位置
planner.setTarget(200.0f);  // 改为更远的位置
planner.setTarget(-50.0f);  // 反向运动
```

### 紧急停止

```cpp
// 使用最大减速度紧急停止
planner.emergencyStop();

// 停止后重置
planner.reset(currentPosition, 0.0f);
```

### 带目标速度

```cpp
// 设置目标位置和目标速度
planner.setTarget(100.0f, 5.0f);  // 到达 100 时速度为 5 rad/s

// 持续速度运动
planner.setTarget(FLT_MAX, 10.0f);  // 以 10 rad/s 持续运动
```

---

## 多轴同步运动

### 原理介绍

多轴同步确保所有轴在同一时刻到达各自的目标位置。

```
轴1: ────────────────────────┐
轴2: ──────────────┐         │
轴3: ─────────┐    │         │
              │    │         │
              │    │         │ ← 同时到达
              ▼    ▼         ▼
```

### 基本使用

```cpp
#include <omni/motion/multi_axis_trajectory.hpp>

using namespace omni::motion;

// 创建3轴同步轨迹
MultiAxisTrajectory multiTraj(3);

// 为每轴创建轨迹
MotionConstraints constraints;
constraints.maxVelocity = 10.0f;
constraints.maxAcceleration = 100.0f;
constraints.maxJerk = 1000.0f;

SCurveProfile* traj0 = new SCurveProfile(constraints);
SCurveProfile* traj1 = new SCurveProfile(constraints);
SCurveProfile* traj2 = new SCurveProfile(constraints);

traj0->plan(0.0f, 100.0f);  // 轴0: 0 → 100
traj1->plan(0.0f, 50.0f);   // 轴1: 0 → 50
traj2->plan(0.0f, 200.0f);  // 轴2: 0 → 200

multiTraj.setAxisTrajectory(0, traj0);
multiTraj.setAxisTrajectory(1, traj1);
multiTraj.setAxisTrajectory(2, traj2);

// 同步：调整各轴速度使总时间一致
multiTraj.synchronize();

float duration = multiTraj.getDuration();
printf("同步后总时间: %.3f s\n", duration);
```

### 执行多轴轨迹

```cpp
std::vector<TrajectoryPoint> points(3);

for (float t = 0; t <= multiTraj.getDuration(); t += 0.001f) {
    multiTraj.evaluate(t, points);

    motor0.setPosition(points[0].position);
    motor1.setPosition(points[1].position);
    motor2.setPosition(points[2].position);

    delay_ms(1);
}
```

### 使用 MultiAxisController

```cpp
#include <omni/app/multi_axis_controller.hpp>

using namespace omni::app;

// 创建多轴控制器
MultiAxisController robot(3);
robot.addAxis(0, &motor0);
robot.addAxis(1, &motor1);
robot.addAxis(2, &motor2);

// 同步移动到目标位置
std::vector<float> target = {100.0f, 50.0f, 200.0f};
robot.moveTo(target);

// 等待完成
while (!robot.allSettled()) {
    robot.update(0.001f);
    delay_ms(1);
}
```

### 直线插补

```cpp
// 直线插补：多轴沿直线路径运动
std::vector<float> endPos = {100.0f, 100.0f, 50.0f};
float feedrate = 10.0f;  // 合成速度 10 单位/秒

robot.linearMove(endPos, feedrate);
```

### 圆弧插补

```cpp
// XY平面圆弧
std::vector<float> endPos = {100.0f, 100.0f, 0.0f};
std::vector<float> center = {50.0f, 50.0f, 0.0f};

robot.arcMove(endPos, center, ArcPlane::XY, true);  // 顺时针
```

---

## 轨迹跟踪

### 基本使用

```cpp
#include <omni/app/trajectory_tracker.hpp>

using namespace omni::app;

TrajectoryTracker tracker(&motor);

// 加载轨迹
SCurveProfile profile(constraints);
profile.plan(0.0f, 100.0f);
tracker.loadTrajectory(&profile);

// 开始跟踪
tracker.start();

// 控制循环
while (!tracker.isFinished()) {
    tracker.update(0.001f);

    // 显示进度
    printf("进度: %.1f%%\n", tracker.getProgress() * 100);

    delay_ms(1);
}
```

### 暂停/恢复

```cpp
// 暂停
tracker.pause();

// 恢复
tracker.resume();

// 停止
tracker.stop();

// 重置到起点
tracker.reset();
```

### 速度调节

```cpp
// 实时调整速度倍率
tracker.setSpeedOverride(0.5f);   // 50% 速度
tracker.setSpeedOverride(1.0f);   // 正常速度
tracker.setSpeedOverride(2.0f);   // 200% 速度
```

### 从文件加载轨迹

```cpp
// CSV 格式
// time,position,velocity,acceleration
// 0.0,0.0,0.0,0.0
// 0.1,1.0,10.0,100.0
// ...

tracker.loadFromFile("trajectory.csv");
```

### 加载离散点

```cpp
std::vector<TrajectoryPoint> points;
points.push_back({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
points.push_back({0.5f, 25.0f, 100.0f, 0.0f, 0.0f});
points.push_back({1.0f, 100.0f, 0.0f, 0.0f, 0.0f});

tracker.loadTrajectory(points);
```

---

## 实用技巧

### 选择合适的曲线类型

```
简单定位任务 → 梯形曲线
精密设备 → S曲线
机器人运动 → 多项式/样条
实时交互 → 在线规划
```

### 参数选择建议

```
最大速度：根据机械极限和任务需求
最大加速度：根据电机扭矩和负载惯量
最大加加速度：通常为加速度的 5-20 倍

公式估算：
  Vmax = 额定转速 × 0.8
  Amax = 额定扭矩 / 负载惯量 × 0.5
  Jmax = Amax × 10
```

### 调试方法

```cpp
// 打印轨迹数据
for (float t = 0; t <= profile.getDuration(); t += 0.01f) {
    auto p = profile.evaluate(t);
    printf("%.3f, %.3f, %.3f, %.3f\n",
           t, p.position, p.velocity, p.acceleration);
}

// 可以将数据导入 Excel 或 Python 绘图
```

### 常见问题

**Q: 规划失败怎么办？**
A: 检查约束是否合理，距离是否太短

**Q: 运动时振动？**
A: 降低加加速度，增加轨迹平滑度

**Q: 跟踪误差大？**
A: 添加前馈控制，提高控制器带宽

