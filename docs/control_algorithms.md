# OmniMotion 控制算法详解

本文档详细介绍 OmniMotion 中的控制算法原理和使用方法。

## 目录

1. [PID 控制器](#pid-控制器)
2. [级联 PID 控制](#级联-pid-控制)
3. [前馈控制](#前馈控制)
4. [滑模控制](#滑模控制)
5. [扰动观测器](#扰动观测器)
6. [数字滤波器](#数字滤波器)
7. [自适应控制](#自适应控制)
8. [模型预测控制](#模型预测控制)
9. [参数调节指南](#参数调节指南)

---

## PID 控制器

### 原理介绍

PID (比例-积分-微分) 控制器是最常用的反馈控制器。

**基本公式:**

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

其中：
- `e(t)` = 目标值 - 实际值 (误差)
- `Kp` = 比例增益
- `Ki` = 积分增益
- `Kd` = 微分增益

**各项作用:**

| 参数 | 作用 | 增大效果 | 过大后果 |
|------|------|----------|----------|
| Kp | 减小稳态误差 | 响应更快 | 振荡 |
| Ki | 消除稳态误差 | 精度更高 | 超调、积分饱和 |
| Kd | 抑制振荡 | 阻尼增加 | 噪声放大 |

### 基本使用

```cpp
#include <omni/control/pid_controller.hpp>

using namespace omni::control;

// 方法一：使用参数结构体
PidController::Params params;
params.kp = 10.0f;
params.ki = 1.0f;
params.kd = 0.5f;
params.outputLimit = 100.0f;      // 输出限幅
params.integralLimit = 50.0f;     // 积分限幅
params.deadband = 0.01f;          // 死区
params.enableAntiWindup = true;   // 抗积分饱和

PidController pid(params);

// 方法二：直接设置增益
PidController pid2;
pid2.setGains(10.0f, 1.0f, 0.5f);
pid2.setOutputLimit(100.0f);
```

### 控制循环

```cpp
float setpoint = 100.0f;  // 目标值
float dt = 0.001f;        // 控制周期 (1ms)

while (true) {
    float measurement = readSensor();  // 读取实际值
    float output = pid.update(setpoint, measurement, dt);
    writeActuator(output);  // 输出到执行器

    delay_ms(1);
}
```

### 高级功能

```cpp
// 设置目标值斜坡限制
pid.setSetpointRamp(10.0f);  // 目标值最大变化率 10/s

// 获取内部状态（用于调试）
float error = pid.getError();
float integral = pid.getIntegral();
float derivative = pid.getDerivative();
float output = pid.getOutput();

// 重置控制器
pid.reset();
```

### 抗积分饱和

积分饱和是 PID 控制的常见问题。当执行器饱和时，积分项会持续累积，导致超调。

```cpp
// OmniMotion 默认使能抗积分饱和
params.enableAntiWindup = true;

// 工作原理：
// 1. 当输出饱和时，停止积分累积
// 2. 使用积分限幅限制积分项范围
params.integralLimit = 50.0f;
```

### PID 调参建议

**Ziegler-Nichols 方法:**

1. 设置 Ki = Kd = 0
2. 增加 Kp 直到系统产生等幅振荡
3. 记录此时的 Kp (记为 Ku) 和振荡周期 Tu
4. 根据下表计算参数：

| 控制器类型 | Kp | Ki | Kd |
|-----------|----|----|-----|
| P | 0.5Ku | - | - |
| PI | 0.45Ku | 1.2Kp/Tu | - |
| PID | 0.6Ku | 2Kp/Tu | KpTu/8 |

---

## 级联 PID 控制

### 原理介绍

级联控制使用多个串联的 PID 控制器，外环的输出作为内环的目标值。

```
位置目标 → [位置PID] → 速度目标 → [速度PID] → 电流目标 → [电流PID] → 电压
     ↑                      ↑                      ↑
   位置反馈              速度反馈               电流反馈
```

**优点:**
- 内环响应快，抑制内部扰动
- 各环独立调参，简化调试
- 限制中间变量，保护系统

### 基本使用

```cpp
#include <omni/control/cascade_pid.hpp>

using namespace omni::control;

CascadePidController cascade;

// 配置三环 PID
PidController::Params posParams, velParams, curParams;

// 位置环：低频响应
posParams.kp = 50.0f;
posParams.ki = 0.0f;
posParams.kd = 0.5f;
posParams.outputLimit = 100.0f;  // 限制速度指令

// 速度环：中频响应
velParams.kp = 0.5f;
velParams.ki = 0.01f;
velParams.kd = 0.0f;
velParams.outputLimit = 20.0f;   // 限制电流指令

// 电流环：高频响应
curParams.kp = 0.2f;
curParams.ki = 0.1f;
curParams.kd = 0.0f;
curParams.outputLimit = 1.0f;    // 限制电压

cascade.setPositionPid(posParams);
cascade.setVelocityPid(velParams);
cascade.setCurrentPid(curParams);
```

### 控制模式

```cpp
// 位置模式：三环都工作
cascade.setMode(CascadeMode::Position);
float output = cascade.update(posRef, posFb, velFb, curFb, dt);

// 速度模式：跳过位置环
cascade.setMode(CascadeMode::Velocity);
float output = cascade.update(velRef, 0, velFb, curFb, dt);

// 电流模式：只有电流环
cascade.setMode(CascadeMode::Current);
float output = cascade.update(curRef, 0, 0, curFb, dt);
```

### 单独调用各环

```cpp
// 可以单独调用某一环
float velCmd = cascade.updatePositionLoop(posRef, posFb, dt);
float curCmd = cascade.updateVelocityLoop(velRef, velFb, dt);
float voltage = cascade.updateCurrentLoop(curRef, curFb, dt);
```

### 调参建议

**从内到外调参:**

1. **首先调电流环** (最内环)
   - 给定阶跃电流指令
   - 目标：快速跟踪，无超调
   - 典型带宽：1-5 kHz

2. **然后调速度环**
   - 给定阶跃速度指令
   - 目标：快速响应，小超调 (<10%)
   - 典型带宽：100-500 Hz

3. **最后调位置环** (最外环)
   - 给定阶跃位置指令
   - 目标：平稳到位，无超调或小超调
   - 典型带宽：10-50 Hz

---

## 前馈控制

### 原理介绍

前馈控制基于系统模型预测需要的控制量，与反馈控制配合使用。

```
         前馈
目标 ─────────────────────┐
  │                       │
  └──→ [反馈PID] ─────────┼──→ 输出
            ↑             │
          反馈 ←──────────┘
```

**前馈公式:**

```
u_ff = Kv·v + Ka·a + Kj·j + Ks·sign(v) + Kf·v + Kg·sin(θ)
```

其中：
- `Kv`: 速度前馈（补偿粘滞摩擦和电机反电动势）
- `Ka`: 加速度前馈（补偿惯量）
- `Kj`: 加加速度前馈（用于 S 曲线）
- `Ks`: 静摩擦补偿
- `Kf`: 动摩擦补偿
- `Kg`: 重力补偿

### 基本使用

```cpp
#include <omni/control/feedforward.hpp>

using namespace omni::control;

FeedforwardController ff;

// 配置前馈参数
FeedforwardController::Params params;
params.kv = 0.1f;      // 速度前馈
params.ka = 0.01f;     // 加速度前馈
params.kj = 0.001f;    // 加加速度前馈
params.ks = 0.5f;      // 静摩擦补偿
params.kf = 0.02f;     // 动摩擦补偿
params.kg = 0.0f;      // 重力补偿
ff.setParams(params);
```

### 与 PID 配合

```cpp
PidController pid;
FeedforwardController ff;

// 轨迹规划器提供目标和导数
float targetPos, targetVel, targetAcc, targetJerk;
trajectory.evaluate(t, targetPos, targetVel, targetAcc, targetJerk);

// 计算前馈
float ffOutput = ff.calculate(targetVel, targetAcc, targetJerk);

// 计算反馈
float fbOutput = pid.update(targetPos, actualPos, dt);

// 合并输出
float totalOutput = ffOutput + fbOutput;
```

### 重力补偿

对于垂直运动的轴，需要补偿重力：

```cpp
// 设置重力补偿
ff.setGravityAngle(theta);  // 轴与水平面的夹角

// 重力补偿会自动计算：
// gravity_compensation = Kg * sin(theta)
```

### 参数辨识

前馈参数可以通过实验辨识：

```cpp
// 方法1：恒速运动
// 在不同速度下测量所需扭矩，拟合得到 Kv 和静摩擦

// 方法2：加速度测试
// 在不同加速度下测量所需扭矩，拟合得到 Ka

// 理论计算：
// Ka ≈ J (转动惯量)
// Kv ≈ Ke (电机反电动势常数)
```

---

## 滑模控制

### 原理介绍

滑模控制 (SMC) 是一种鲁棒控制方法，能够处理模型不确定性和外部扰动。

**基本思想:**
1. 设计一个滑模面 s = 0
2. 设计控制律使状态向滑模面运动
3. 到达滑模面后，状态沿滑模面滑动到原点

**滑模面:**

```
s = e' + λ·e
```

其中 e 为跟踪误差，λ > 0。

**控制律:**

```
u = u_eq + u_sw
u_eq = 等效控制（维持在滑模面上）
u_sw = -η·sat(s/φ)  切换控制（趋近滑模面）
```

### 基本使用

```cpp
#include <omni/control/sliding_mode.hpp>

using namespace omni::control;

SlidingModeController::Params params;
params.lambda = 10.0f;   // 滑模面斜率
params.eta = 5.0f;       // 趋近律增益
params.phi = 0.1f;       // 边界层厚度（消除抖振）

SlidingModeController smc(params);
```

### 控制循环

```cpp
float posError = targetPos - actualPos;
float velError = targetVel - actualVel;

float output = smc.update(posError, velError, dt);
```

### 消除抖振

标准滑模控制会产生高频抖振。使用边界层可以消除抖振：

```cpp
// 边界层厚度
params.phi = 0.1f;  // 较大值：平滑但精度降低
params.phi = 0.01f; // 较小值：精度高但可能抖振
```

### 自适应滑模

```cpp
// 使能自适应功能
smc.enableAdaptive(true);

// 自适应滑模会在线估计扰动上界，
// 自动调整切换增益
```

---

## 扰动观测器

### 原理介绍

扰动观测器 (DOB) 估计系统受到的总扰动，并进行补偿。

```
                    扰动 d
                      ↓
输入 u → [系统 P] → 输出 y
                      ↓
              [扰动观测器]
                      ↓
                   估计 d̂
                      ↓
                 补偿 u - d̂
```

**工作原理:**

对于系统 `J·ω' = τ - d`（惯量·加速度 = 扭矩 - 扰动）

通过低通滤波器估计扰动：
```
d̂ = Q(s)·(τ - J·s·ω)
```

其中 Q(s) 为低通滤波器。

### 基本使用

```cpp
#include <omni/control/disturbance_observer.hpp>

using namespace omni::control;

// 参数：带宽 和 惯量
float bandwidth = 100.0f;  // Hz
float inertia = 0.001f;    // kg·m²

DisturbanceObserver dob(bandwidth, inertia);
```

### 控制循环

```cpp
// 输入：电流（或扭矩）和速度
float current = 5.0f;      // A
float velocity = 10.0f;    // rad/s

// 估计扰动
float disturbance = dob.estimate(current, velocity, dt);

// 获取补偿量
float compensation = dob.getCompensation();

// 在控制输出中加入补偿
float output = pid.update(setpoint, measurement, dt) + compensation;
```

### 参数选择

```cpp
// 带宽选择：
// - 太低：扰动抑制能力弱
// - 太高：噪声放大
// 建议：系统截止频率的 1-5 倍

dob.setBandwidth(100.0f);  // 100 Hz

// 惯量需要准确
// 可以通过加速度测试辨识
dob.setInertia(0.001f);
```

---

## 数字滤波器

### 低通滤波器

一阶低通滤波器，用于滤除高频噪声。

```cpp
#include <omni/control/filters.hpp>

using namespace omni::control;

// 创建滤波器
// 参数：截止频率 和 采样频率
LowPassFilter lpf(100.0f, 1000.0f);  // 100Hz 截止, 1kHz 采样

// 滤波
float filtered = lpf.filter(rawValue);

// 修改参数
lpf.setCutoff(50.0f);  // 改为 50Hz

// 重置
lpf.reset(0.0f);
```

### Biquad 滤波器

二阶 IIR 滤波器，支持多种类型。

```cpp
// 低通
BiquadFilter lpf(BiquadFilter::Type::LowPass, 100.0f, 1000.0f, 0.707f);

// 高通
BiquadFilter hpf(BiquadFilter::Type::HighPass, 10.0f, 1000.0f, 0.707f);

// 带通
BiquadFilter bpf(BiquadFilter::Type::BandPass, 50.0f, 1000.0f, 2.0f);

// 陷波
BiquadFilter notch(BiquadFilter::Type::Notch, 60.0f, 1000.0f, 10.0f);

// 峰值
BiquadFilter peak(BiquadFilter::Type::Peak, 100.0f, 1000.0f, 5.0f);

// 使用
float filtered = bpf.filter(input);
```

### 陷波滤波器

专门用于消除特定频率（如电网干扰、机械共振）。

```cpp
// 60Hz 陷波滤波器
NotchFilter notch(60.0f, 5.0f, 1000.0f);
// 参数：中心频率, 带宽, 采样频率

float filtered = notch.filter(input);

// 修改参数
notch.setParams(50.0f, 3.0f);  // 改为 50Hz
```

### 移动平均滤波器

```cpp
// 10 点移动平均
MovingAverageFilter<float, 10> ma;

float filtered = ma.filter(input);
float average = ma.getAverage();

// 重置
ma.reset();
```

### 滤波器级联

```cpp
// 多个滤波器级联
LowPassFilter lpf(100.0f, 1000.0f);
NotchFilter notch(60.0f, 5.0f, 1000.0f);

float filtered = notch.filter(lpf.filter(input));
```

---

## 自适应控制

### 原理介绍

自适应控制在运行过程中在线估计系统参数，调整控制器以适应变化。

### 模型参考自适应控制 (MRAC)

```cpp
#include <omni/control/adaptive_controller.hpp>

using namespace omni::control;

AdaptiveController adaptive;

// 设置参考模型
// 期望系统表现为二阶系统
float wn = 10.0f;   // 自然频率
float zeta = 0.7f;  // 阻尼比
adaptive.setReferenceModel(wn, zeta);

// 设置自适应增益
adaptive.setAdaptationGain(0.1f);
```

### 控制循环

```cpp
float output = adaptive.update(reference, measurement, dt);

// 获取估计的参数
auto params = adaptive.getEstimatedParams();
printf("估计参数: %f, %f\n", params[0], params[1]);
```

---

## 模型预测控制

### 原理介绍

模型预测控制 (MPC) 通过预测未来状态来优化控制序列。

**核心思想:**
1. 使用系统模型预测未来 N 步状态
2. 优化控制序列使预测轨迹跟踪参考
3. 只执行第一个控制量，然后重复

### 基本使用

```cpp
#include <omni/control/mpc_controller.hpp>

using namespace omni::control;

MpcController::Config config;
config.predictionHorizon = 10;  // 预测步长
config.controlHorizon = 5;      // 控制步长
config.dt = 0.001f;             // 采样时间

// 权重矩阵
config.Q = Matrix<float>::identity(2) * 100.0f;  // 状态权重
config.R = Matrix<float>::identity(1) * 1.0f;    // 控制权重

MpcController mpc(config);
```

### 设置系统模型

```cpp
// 离散状态空间模型
// x[k+1] = A·x[k] + B·u[k]
// y[k] = C·x[k]

StateSpaceModel model;
model.A = {{1.0f, 0.001f}, {0.0f, 1.0f}};
model.B = {{0.0f}, {0.001f}};
model.C = {{1.0f, 0.0f}};

mpc.setModel(model);
```

### 设置约束

```cpp
// 状态约束
mpc.setStateConstraints(
    {-100.0f, -50.0f},   // 最小值
    {100.0f, 50.0f}      // 最大值
);

// 控制约束
mpc.setControlConstraints(
    {-10.0f},   // 最小控制量
    {10.0f}     // 最大控制量
);
```

### 控制循环

```cpp
Vector<float> state = {position, velocity};
Vector<float> reference = {targetPosition, 0.0f};

float output = mpc.update(state, reference);
```

---

## 参数调节指南

### PID 快速调参法

```
1. 只开 P，从小到大增加，直到系统开始振荡
2. P 降低 30-50%
3. 加入 I，从小到大，直到稳态误差可接受
4. 如果振荡，加入 D 来阻尼

常见问题：
- 响应慢 → 增大 P
- 振荡 → 减小 P 或增大 D
- 稳态误差 → 增大 I
- 超调大 → 减小 I 或增大 D
```

### 级联控制调参顺序

```
1. 断开外环，先调内环
2. 从最内环开始（通常是电流环）
3. 确保内环稳定后，再调外环
4. 外环带宽应低于内环（约 5-10 倍）
```

### 带宽选择原则

```
电流环 > 速度环 > 位置环

典型配置：
- 电流环: 1-5 kHz
- 速度环: 100-500 Hz
- 位置环: 10-50 Hz

规则：外环带宽 ≤ 内环带宽 / 5
```

### 控制器对比

| 控制器 | 优点 | 缺点 | 适用场景 |
|--------|------|------|----------|
| PID | 简单, 无需模型 | 抗扰差, 难调优 | 大多数情况 |
| 前馈+PID | 响应快 | 需要模型 | 轨迹跟踪 |
| 滑模 | 鲁棒性强 | 抖振 | 不确定系统 |
| DOB | 扰动抑制好 | 需要惯量 | 精密系统 |
| MPC | 处理约束, 最优 | 计算量大 | 高性能系统 |

### 常见问题诊断

**振荡:**
- 增益过高
- 采样频率低
- 时延大

**超调:**
- I 增益过高
- D 增益过低
- 目标变化太快

**稳态误差:**
- 无 I 控制
- I 增益太小
- 积分饱和

**响应慢:**
- P 增益太小
- 带宽太低
- 输出饱和

