# OmniMotion API 参考手册

本文档提供 OmniMotion 库的完整 API 参考。

## 目录

1. [硬件抽象层 (HAL)](#硬件抽象层-hal)
2. [驱动层 (Driver)](#驱动层-driver)
3. [协议层 (Protocol)](#协议层-protocol)
4. [控制层 (Control)](#控制层-control)
5. [运动规划层 (Motion)](#运动规划层-motion)
6. [应用层 (App)](#应用层-app)
7. [工具类 (Utils)](#工具类-utils)

---

## 硬件抽象层 (HAL)

命名空间: `omni::hal`

### IGpio

GPIO 接口基类。

```cpp
class IGpio {
public:
    virtual void setMode(PinMode mode) = 0;
    virtual void write(bool value) = 0;
    virtual bool read() = 0;
    virtual void toggle() = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setMode` | `PinMode mode` | void | 设置引脚模式 (Input/Output/PushPull/OpenDrain) |
| `write` | `bool value` | void | 写入引脚电平 |
| `read` | - | bool | 读取引脚电平 |
| `toggle` | - | void | 翻转引脚电平 |

### IPwm

PWM 接口基类。

```cpp
class IPwm {
public:
    virtual void setFrequency(uint32_t freq_hz) = 0;
    virtual void setDuty(float duty) = 0;
    virtual void setDutyNs(uint32_t ns) = 0;
    virtual void enable(bool en) = 0;
    virtual void setDeadTime(uint32_t ns) = 0;
    virtual uint32_t getFrequency() const = 0;
    virtual float getDuty() const = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setFrequency` | `uint32_t freq_hz` | void | 设置 PWM 频率 (Hz) |
| `setDuty` | `float duty` | void | 设置占空比 (0.0 ~ 1.0) |
| `setDutyNs` | `uint32_t ns` | void | 设置高电平时间 (纳秒) |
| `enable` | `bool en` | void | 使能/禁用 PWM |
| `setDeadTime` | `uint32_t ns` | void | 设置死区时间 (纳秒) |
| `getFrequency` | - | uint32_t | 获取当前频率 |
| `getDuty` | - | float | 获取当前占空比 |

### IPwm3Phase

三相 PWM 接口，用于 FOC 控制。

```cpp
class IPwm3Phase {
public:
    virtual void setDuty(float a, float b, float c) = 0;
    virtual void enable(bool en) = 0;
    virtual void setDeadTime(uint32_t ns) = 0;
    virtual void setSvpwm(float alpha, float beta) = 0;
    virtual void setFrequency(uint32_t freq_hz) = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setDuty` | `float a, b, c` | void | 设置三相占空比 |
| `enable` | `bool en` | void | 使能/禁用三相输出 |
| `setDeadTime` | `uint32_t ns` | void | 设置死区时间 |
| `setSvpwm` | `float alpha, beta` | void | 空间矢量调制 |
| `setFrequency` | `uint32_t freq_hz` | void | 设置 PWM 频率 |

### IAdc

ADC 接口基类。

```cpp
class IAdc {
public:
    virtual uint16_t read() = 0;
    virtual float readVoltage() = 0;
    virtual void startDma(uint16_t* buffer, size_t len) = 0;
    virtual bool isDmaComplete() = 0;
    virtual void setTrigger(AdcTrigger trig) = 0;
    virtual void setResolution(uint8_t bits) = 0;
    virtual void setSampleTime(uint32_t cycles) = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `read` | - | uint16_t | 读取原始 ADC 值 |
| `readVoltage` | - | float | 读取电压值 (V) |
| `startDma` | `buffer, len` | void | 启动 DMA 传输 |
| `isDmaComplete` | - | bool | DMA 是否完成 |
| `setTrigger` | `AdcTrigger trig` | void | 设置触发源 |
| `setResolution` | `uint8_t bits` | void | 设置分辨率 |
| `setSampleTime` | `uint32_t cycles` | void | 设置采样时间 |

### IEncoder

编码器接口基类。

```cpp
class IEncoder {
public:
    virtual int32_t getCount() = 0;
    virtual void resetCount() = 0;
    virtual float getAngle() = 0;
    virtual float getVelocity() = 0;
    virtual void setResolution(uint32_t cpr) = 0;
    virtual uint32_t getResolution() const = 0;
    virtual void setDirection(bool invert) = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `getCount` | - | int32_t | 获取累计计数 |
| `resetCount` | - | void | 重置计数器 |
| `getAngle` | - | float | 获取角度 (0 ~ 2π) |
| `getVelocity` | - | float | 获取速度 (rad/s) |
| `setResolution` | `uint32_t cpr` | void | 设置每转脉冲数 |
| `getResolution` | - | uint32_t | 获取分辨率 |
| `setDirection` | `bool invert` | void | 设置方向反转 |

### ICan

CAN 总线接口。

```cpp
class ICan : public IComm {
public:
    virtual bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len) = 0;
    virtual bool receiveFrame(CanFrame& frame) = 0;
    virtual void setFilter(uint32_t id, uint32_t mask) = 0;
    virtual void setBitrate(uint32_t bitrate) = 0;
    virtual void setMode(CanMode mode) = 0;
    virtual bool isFrameAvailable() = 0;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `sendFrame` | `id, data, len` | bool | 发送 CAN 帧 |
| `receiveFrame` | `CanFrame& frame` | bool | 接收 CAN 帧 |
| `setFilter` | `id, mask` | void | 设置接收过滤器 |
| `setBitrate` | `uint32_t bitrate` | void | 设置波特率 |
| `setMode` | `CanMode mode` | void | 设置工作模式 |
| `isFrameAvailable` | - | bool | 是否有帧可读 |

---

## 驱动层 (Driver)

命名空间: `omni::driver`

### MotorState

电机状态结构体。

```cpp
struct MotorState {
    float position = 0.0f;      // 位置 (rad 或 m)
    float velocity = 0.0f;      // 速度 (rad/s 或 m/s)
    float acceleration = 0.0f;  // 加速度
    float torque = 0.0f;        // 扭矩/力 (Nm 或 N)
    float current = 0.0f;       // 电流 (A)
    float temperature = 0.0f;   // 温度 (°C)
    float voltage = 0.0f;       // 电压 (V)
    uint32_t errorCode = 0;     // 错误码
    bool enabled = false;       // 使能状态
    bool fault = false;         // 故障标志
};
```

### MotorParams

电机参数结构体。

```cpp
struct MotorParams {
    // 机械参数
    float polePairs = 1.0f;     // 极对数
    float resistance = 0.0f;    // 相电阻 (Ω)
    float inductance = 0.0f;    // 相电感 (H)
    float fluxLinkage = 0.0f;   // 磁链 (Wb)
    float inertia = 0.0f;       // 转动惯量 (kg·m²)
    float friction = 0.0f;      // 摩擦系数

    // 限制参数
    float maxCurrent = 10.0f;   // 最大电流 (A)
    float maxVelocity = 100.0f; // 最大速度 (rad/s)
    float maxTorque = 10.0f;    // 最大扭矩 (Nm)
    float maxTemperature = 80.0f; // 最大温度 (°C)

    // 编码器参数
    uint32_t encoderCpr = 4096; // 编码器分辨率
    float encoderOffset = 0.0f; // 编码器偏移
    bool encoderInvert = false; // 编码器反向
};
```

### IMotorDriver

电机驱动接口基类。

```cpp
class IMotorDriver {
public:
    // 基本控制
    virtual bool init() = 0;
    virtual void deinit() = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void emergencyStop() = 0;

    // 控制模式
    virtual void setControlMode(ControlMode mode) = 0;
    virtual ControlMode getControlMode() const = 0;

    // 目标设定
    virtual void setPosition(float pos) = 0;
    virtual void setVelocity(float vel) = 0;
    virtual void setTorque(float torque) = 0;
    virtual void setCurrent(float current) = 0;
    virtual void setVoltage(float voltage) = 0;
    virtual void setDuty(float duty) = 0;

    // 状态读取
    virtual MotorState getState() const = 0;
    virtual float getPosition() const = 0;
    virtual float getVelocity() const = 0;
    virtual float getTorque() const = 0;
    virtual float getCurrent() const = 0;

    // 参数配置
    virtual void setParams(const MotorParams& params) = 0;
    virtual MotorParams getParams() const = 0;

    // 实时更新
    virtual void update() = 0;

    // 类型标识
    virtual MotorType getType() const = 0;
    virtual const char* getName() const = 0;
};
```

### BldcDriver

BLDC/PMSM 电机驱动。

```cpp
class BldcDriver : public IMotorDriver {
public:
    BldcDriver(hal::IPwm3Phase* pwm,
               hal::IAdc* adcA, hal::IAdc* adcB, hal::IAdc* adcC,
               hal::IEncoder* encoder);

    // FOC 特有接口
    void setFocMode(FocMode mode);
    void setCurrentPid(float kp, float ki, float kd);
    void setVelocityPid(float kp, float ki, float kd);
    void setPositionPid(float kp, float ki, float kd);
    void setIdRef(float id);
    void setIqRef(float iq);
    float getIdActual() const;
    float getIqActual() const;
    void calibrateEncoder();
    void calibrateCurrentSensor();
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setFocMode` | `FocMode mode` | void | 设置 FOC 模式 (SVPWM/SPWM/SixStep) |
| `setCurrentPid` | `kp, ki, kd` | void | 设置电流环 PID |
| `setVelocityPid` | `kp, ki, kd` | void | 设置速度环 PID |
| `setPositionPid` | `kp, ki, kd` | void | 设置位置环 PID |
| `setIdRef` | `float id` | void | 设置 d 轴电流参考 |
| `setIqRef` | `float iq` | void | 设置 q 轴电流参考 |
| `getIdActual` | - | float | 获取实际 d 轴电流 |
| `getIqActual` | - | float | 获取实际 q 轴电流 |
| `calibrateEncoder` | - | void | 编码器校准 |
| `calibrateCurrentSensor` | - | void | 电流传感器校准 |

### StepperDriver

步进电机驱动。

```cpp
class StepperDriver : public IMotorDriver {
public:
    StepperDriver(hal::IGpio* stepPin, hal::IGpio* dirPin,
                  hal::IGpio* enablePin = nullptr);

    void setMicrostep(uint16_t microstep);
    void setStepMode(StepMode mode);
    void step(int32_t steps);
    void setStepsPerRev(uint32_t steps);
    void setMaxStepRate(uint32_t stepsPerSec);
    void setCurrentLimit(float current);
    void setStealthChop(bool enable);
    void setSpreadCycle(bool enable);
    void setStallGuard(uint8_t threshold);
    bool isStalled() const;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setMicrostep` | `uint16_t microstep` | void | 设置细分 (1/2/4/8/16/32) |
| `setStepMode` | `StepMode mode` | void | 设置步进模式 |
| `step` | `int32_t steps` | void | 步进指定步数 |
| `setStepsPerRev` | `uint32_t steps` | void | 设置每转步数 |
| `setMaxStepRate` | `uint32_t rate` | void | 设置最大步进频率 |
| `setCurrentLimit` | `float current` | void | 设置电流限制 |
| `setStealthChop` | `bool enable` | void | 使能静音模式 |
| `setSpreadCycle` | `bool enable` | void | 使能 SpreadCycle |
| `setStallGuard` | `uint8_t threshold` | void | 设置堵转检测阈值 |
| `isStalled` | - | bool | 是否堵转 |

### DcMotorDriver

有刷直流电机驱动。

```cpp
class DcMotorDriver : public IMotorDriver {
public:
    // H桥模式
    DcMotorDriver(hal::IPwm* pwmA, hal::IPwm* pwmB);
    // PWM+方向模式
    DcMotorDriver(hal::IPwm* pwm, hal::IGpio* dir);

    void setBrakeMode(BrakeMode mode);
    BrakeMode getBrakeMode() const;
    void setDeadband(float deadband);
};
```

### ServoDriver

舵机驱动。

```cpp
class ServoDriver : public IMotorDriver {
public:
    ServoDriver(hal::IPwm* pwm);

    void setAngle(float angle);
    float getAngle() const;
    void setPulseRange(uint16_t minUs, uint16_t maxUs);
    void setAngleRange(float minDeg, float maxDeg);
    void setSpeed(float degPerSec);
};
```

---

## 协议层 (Protocol)

命名空间: `omni::protocol`

### FocController

FOC 控制器。

```cpp
class FocController {
public:
    struct Config {
        float currentBandwidth = 1000.0f;
        float velocityBandwidth = 100.0f;
        float positionBandwidth = 10.0f;
        float pwmFrequency = 20000.0f;
        FocMode mode = FocMode::SVPWM;
    };

    FocController(const Config& config = {});

    void update(float dt);
    void setPositionRef(float pos);
    void setVelocityRef(float vel);
    void setCurrentRef(float id, float iq);
    void getPwmDuty(float& ta, float& tb, float& tc);
    void setCurrentFeedback(float ia, float ib, float ic);
    void setPositionFeedback(float pos);
    void setVelocityFeedback(float vel);
    void enableFieldWeakening(bool enable);
    void enableMtpa(bool enable);
    void reset();
};
```

### MitProtocol

MIT 协议编解码。

```cpp
class MitProtocol {
public:
    struct Command {
        float position;
        float velocity;
        float kp;
        float kd;
        float torque;
    };

    struct Feedback {
        uint8_t motorId;
        float position;
        float velocity;
        float torque;
    };

    MitProtocol();

    void setPosRange(float min, float max);
    void setVelRange(float min, float max);
    void setTorqueRange(float min, float max);
    void setKpRange(float min, float max);
    void setKdRange(float min, float max);

    static void encode(uint8_t motorId, const Command& cmd,
                       hal::CanFrame& frame, const MitProtocol& proto);
    static bool decode(const hal::CanFrame& frame, Feedback& fb,
                       const MitProtocol& proto);

    static void encodeEnable(uint8_t motorId, hal::CanFrame& frame);
    static void encodeDisable(uint8_t motorId, hal::CanFrame& frame);
    static void encodeSetZero(uint8_t motorId, hal::CanFrame& frame);
};
```

### ICanProtocol

CAN 协议接口。

```cpp
class ICanProtocol {
public:
    virtual void encodeCommand(const driver::MotorCommand& cmd,
                               hal::CanFrame& frame) = 0;
    virtual bool decodeFeedback(const hal::CanFrame& frame,
                                driver::MotorState& state) = 0;
    virtual uint32_t getCommandId(uint8_t motorId) const = 0;
    virtual uint32_t getFeedbackId(uint8_t motorId) const = 0;
};
```

**预定义协议实现:**
- `DjiCanProtocol` - DJI 电机协议
- `OdriveCanProtocol` - ODrive 协议
- `CybergearProtocol` - 小米 CyberGear 协议
- `DmProtocol` - 达妙电机协议

---

## 控制层 (Control)

命名空间: `omni::control`

### PidController

PID 控制器。

```cpp
class PidController {
public:
    struct Params {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float outputLimit = FLT_MAX;
        float integralLimit = FLT_MAX;
        float deadband = 0.0f;
        float setpointRamp = 0.0f;
        bool enableAntiWindup = true;
    };

    PidController(const Params& params = {});

    float update(float setpoint, float measurement, float dt);
    void reset();
    void setGains(float kp, float ki, float kd);
    void setOutputLimit(float limit);
    void setIntegralLimit(float limit);
    void setDeadband(float deadband);
    void setSetpointRamp(float ramp);
    void enableAntiWindup(bool enable);

    float getError() const;
    float getIntegral() const;
    float getDerivative() const;
    float getOutput() const;
    Params getParams() const;
};
```

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `update` | `setpoint, measurement, dt` | float | 更新控制器，返回输出 |
| `reset` | - | void | 重置控制器状态 |
| `setGains` | `kp, ki, kd` | void | 设置 PID 增益 |
| `setOutputLimit` | `float limit` | void | 设置输出限幅 |
| `setIntegralLimit` | `float limit` | void | 设置积分限幅 |
| `setDeadband` | `float deadband` | void | 设置死区 |

### CascadePidController

级联 PID 控制器 (位置-速度-电流三环)。

```cpp
class CascadePidController {
public:
    CascadePidController();

    void setPositionPid(const PidController::Params& params);
    void setVelocityPid(const PidController::Params& params);
    void setCurrentPid(const PidController::Params& params);

    float update(float posRef, float posFb, float velFb, float curFb, float dt);
    float updatePositionLoop(float posRef, float posFb, float dt);
    float updateVelocityLoop(float velRef, float velFb, float dt);
    float updateCurrentLoop(float curRef, float curFb, float dt);

    void setMode(CascadeMode mode);
    CascadeMode getMode() const;
    void reset();
};
```

### FeedforwardController

前馈控制器。

```cpp
class FeedforwardController {
public:
    struct Params {
        float kv = 0.0f;    // 速度前馈
        float ka = 0.0f;    // 加速度前馈
        float kj = 0.0f;    // 加加速度前馈
        float ks = 0.0f;    // 静摩擦补偿
        float kf = 0.0f;    // 动摩擦补偿
        float kg = 0.0f;    // 重力补偿
    };

    FeedforwardController(const Params& params = {});

    float calculate(float velocity, float acceleration, float jerk = 0.0f);
    void setParams(const Params& params);
    Params getParams() const;
    void setGravityAngle(float angle);
};
```

### LowPassFilter

低通滤波器。

```cpp
class LowPassFilter {
public:
    LowPassFilter(float cutoffFreq = 100.0f, float sampleFreq = 1000.0f);

    float filter(float input);
    void reset(float value = 0.0f);
    void setCutoff(float freq);
    void setSampleFreq(float freq);
    float getOutput() const;
    float getCutoff() const;
};
```

### NotchFilter

陷波滤波器。

```cpp
class NotchFilter {
public:
    NotchFilter(float centerFreq, float bandwidth, float sampleFreq);

    float filter(float input);
    void reset();
    void setParams(float centerFreq, float bandwidth);
};
```

### DisturbanceObserver

扰动观测器。

```cpp
class DisturbanceObserver {
public:
    DisturbanceObserver(float bandwidth, float inertia);

    float estimate(float current, float velocity, float dt);
    float getCompensation() const;
    float getEstimatedDisturbance() const;
    void setBandwidth(float bandwidth);
    void setInertia(float inertia);
    void reset();
};
```

---

## 运动规划层 (Motion)

命名空间: `omni::motion`

### TrajectoryPoint

轨迹点结构体。

```cpp
struct TrajectoryPoint {
    float time = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
    float jerk = 0.0f;
};
```

### MotionConstraints

运动约束结构体。

```cpp
struct MotionConstraints {
    float maxVelocity = 10.0f;
    float maxAcceleration = 100.0f;
    float maxDeceleration = 0.0f;  // 0 = 与加速度相同
    float maxJerk = 1000.0f;
};
```

### ITrajectoryGenerator

轨迹生成器接口。

```cpp
class ITrajectoryGenerator {
public:
    virtual bool plan(float startPos, float endPos,
                      float startVel = 0, float endVel = 0) = 0;
    virtual TrajectoryPoint evaluate(float time) const = 0;
    virtual float getDuration() const = 0;
    virtual bool isValid() const = 0;
};
```

### TrapezoidalProfile

梯形速度曲线。

```cpp
class TrapezoidalProfile : public ITrajectoryGenerator {
public:
    TrapezoidalProfile(const MotionConstraints& constraints);

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override;
    TrajectoryPoint evaluate(float time) const override;
    float getDuration() const override;
    bool isValid() const override;

    float getAccelTime() const;
    float getConstTime() const;
    float getDecelTime() const;
    float getPeakVelocity() const;
};
```

### SCurveProfile

S 型速度曲线 (7段式)。

```cpp
class SCurveProfile : public ITrajectoryGenerator {
public:
    SCurveProfile(const MotionConstraints& constraints);

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override;
    TrajectoryPoint evaluate(float time) const override;
    float getDuration() const override;
    bool isValid() const override;

    void getSegmentTimes(float times[7]) const;
    float getPeakVelocity() const;
    float getPeakAcceleration() const;
    float getStartPosition() const;
    float getEndPosition() const;
};
```

### PolynomialTrajectory

多项式轨迹。

```cpp
class PolynomialTrajectory : public ITrajectoryGenerator {
public:
    static PolynomialTrajectory cubic(
        float duration,
        float startPos, float endPos,
        float startVel = 0, float endVel = 0);

    static PolynomialTrajectory quintic(
        float duration,
        float startPos, float endPos,
        float startVel = 0, float endVel = 0,
        float startAccel = 0, float endAccel = 0);

    static PolynomialTrajectory septic(
        float duration,
        float startPos, float endPos,
        float startVel = 0, float endVel = 0,
        float startAccel = 0, float endAccel = 0,
        float startJerk = 0, float endJerk = 0);

    TrajectoryPoint evaluate(float time) const override;
};
```

### OnlineTrajectoryPlanner

在线轨迹规划器。

```cpp
class OnlineTrajectoryPlanner {
public:
    OnlineTrajectoryPlanner(const MotionConstraints& constraints);

    void setTarget(float targetPos);
    void setTarget(float targetPos, float targetVel);
    TrajectoryPoint update(float dt);
    float getCurrentPosition() const;
    float getCurrentVelocity() const;
    bool isSettled() const;
    void emergencyStop();
    void reset(float position = 0.0f, float velocity = 0.0f);
};
```

### MultiAxisTrajectory

多轴轨迹同步。

```cpp
class MultiAxisTrajectory {
public:
    MultiAxisTrajectory(size_t numAxes);

    void setAxisTrajectory(size_t axis, ITrajectoryGenerator* traj);
    bool synchronize();
    void evaluate(float time, std::vector<TrajectoryPoint>& points);
    float getDuration() const;
    size_t getNumAxes() const;
};
```

---

## 应用层 (App)

命名空间: `omni::app`

### MotorController

电机控制器高层封装。

```cpp
class MotorController {
public:
    MotorController(driver::IMotorDriver* driver);

    // 基本操作
    void enable();
    void disable();
    void emergencyStop();
    bool isEnabled() const;

    // 位置控制
    void moveTo(float position);
    void moveBy(float distance);
    void moveTo(float position, float velocity);
    void moveTo(float position, float velocity, float accel);

    // 带曲线的运动
    void moveWithProfile(float position, motion::ITrajectoryGenerator* profile);
    void moveWithSCurve(float position, float maxVel, float maxAccel, float maxJerk);
    void moveWithTrapezoid(float position, float maxVel, float maxAccel);

    // 速度控制
    void setVelocity(float velocity);
    void setVelocity(float velocity, float accel);
    void rampVelocity(float targetVel, float rampTime);

    // 扭矩/力控制
    void setTorque(float torque);
    void setForce(float force);

    // 阻抗控制
    void setImpedance(float stiffness, float damping);
    void setImpedance(float stiffness, float damping, float inertia);

    // 状态查询
    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;
    bool isMoving() const;
    bool isSettled() const;
    bool hasError() const;
    uint32_t getErrorCode() const;

    // 回零
    void home(HomingMode mode = HomingMode::SensorBased);
    void setHomeOffset(float offset);
    bool isHomed() const;

    // 限位
    void setSoftLimits(float minPos, float maxPos);
    void enableSoftLimits(bool enable);

    // 控制参数
    void setPositionPid(float kp, float ki, float kd);
    void setVelocityPid(float kp, float ki, float kd);
    void setFeedforward(float kv, float ka);
    void setConstraints(const motion::MotionConstraints& constraints);

    // 实时更新
    void update(float dt);
};
```

### MultiAxisController

多轴运动控制器。

```cpp
class MultiAxisController {
public:
    MultiAxisController(size_t numAxes);

    void addAxis(size_t index, MotorController* axis);
    void addAxis(size_t index, driver::IMotorDriver* driver);
    MotorController* getAxis(size_t index);

    // 同步运动
    void moveTo(const std::vector<float>& positions);
    void moveTo(const std::vector<float>& positions, float velocity);

    // 直线插补
    void linearMove(const std::vector<float>& endPos);
    void linearMove(const std::vector<float>& endPos, float feedrate);

    // 圆弧插补
    void arcMove(const std::vector<float>& endPos,
                 const std::vector<float>& center,
                 ArcPlane plane, bool clockwise);

    // 同步控制
    void enableAll();
    void disableAll();
    void emergencyStopAll();
    void waitForCompletion();
    bool allSettled() const;

    // 更新
    void update(float dt);
};
```

### TrajectoryTracker

轨迹跟踪器。

```cpp
class TrajectoryTracker {
public:
    TrajectoryTracker(MotorController* controller);

    void loadTrajectory(motion::ITrajectoryGenerator* trajectory);
    void loadTrajectory(const std::vector<motion::TrajectoryPoint>& points);
    bool loadFromFile(const char* filename);

    void start();
    void pause();
    void resume();
    void stop();
    void reset();

    void setSpeedOverride(float factor);
    float getProgress() const;
    float getElapsedTime() const;
    float getRemainingTime() const;
    bool isRunning() const;
    bool isFinished() const;

    void update(float dt);
};
```

### TeachRecorder

示教记录器。

```cpp
class TeachRecorder {
public:
    TeachRecorder(MotorController* controller);
    TeachRecorder(MultiAxisController* controller);

    void startTeaching();
    void stopTeaching();
    bool isTeaching() const;

    void recordPoint();
    void recordTrajectory(float interval);
    void stopRecordTrajectory();

    void deletePoint(size_t index);
    void insertPoint(size_t index, const std::vector<float>& position);
    void clear();

    bool saveToFile(const char* filename);
    bool loadFromFile(const char* filename);

    void playback(float speedFactor = 1.0f);
    void playbackLoop(int count = -1);
    void stopPlayback();

    size_t getPointCount() const;
    std::vector<float> getPoint(size_t index) const;

    void update(float dt);
};
```

### HomingManager

回零管理器。

```cpp
class HomingManager {
public:
    enum class Mode {
        SensorBased,
        CurrentBased,
        IndexPulse,
        Hardstop,
        AbsoluteEncoder,
        Manual
    };

    struct Config {
        Mode mode = Mode::SensorBased;
        float searchVelocity = 10.0f;
        float approachVelocity = 1.0f;
        float offset = 0.0f;
        float currentThreshold = 5.0f;
        bool reverseDirection = false;
    };

    HomingManager(MotorController* controller, const Config& config);

    void start();
    void abort();
    bool isRunning() const;
    bool isCompleted() const;
    HomingError getError() const;
    void update(float dt);
};
```

### SafetyMonitor

安全监控器。

```cpp
class SafetyMonitor {
public:
    SafetyMonitor(driver::IMotorDriver* driver);

    void setPositionLimits(float min, float max);
    void setVelocityLimit(float max);
    void setCurrentLimit(float max);
    void setTemperatureLimit(float max);

    void enablePositionMonitor(bool en);
    void enableVelocityMonitor(bool en);
    void enableCurrentMonitor(bool en);
    void enableTemperatureMonitor(bool en);
    void enableFollowingErrorMonitor(bool en, float threshold);

    void update();
    bool hasFault() const;
    SafetyFault getFault() const;
    void clearFault();

    using FaultCallback = std::function<void(SafetyFault)>;
    void setFaultCallback(FaultCallback callback);
};
```

---

## 工具类 (Utils)

命名空间: `omni::utils`

### MathUtils

数学工具函数。

```cpp
namespace math {
    // 限幅
    template<typename T>
    T clamp(T value, T min, T max);

    // 角度归一化
    float normalizeAngle(float angle);

    // 线性插值
    float lerp(float a, float b, float t);

    // 符号函数
    template<typename T>
    int sign(T value);

    // 死区处理
    float deadband(float value, float band);

    // 角度转换
    float degToRad(float deg);
    float radToDeg(float rad);

    // 平方
    template<typename T>
    T sq(T value);
}
```

### RingBuffer

环形缓冲区。

```cpp
template<typename T, size_t Size>
class RingBuffer {
public:
    bool push(const T& item);
    bool pop(T& item);
    bool peek(T& item) const;
    void clear();
    size_t size() const;
    size_t capacity() const;
    bool isEmpty() const;
    bool isFull() const;
};
```

### Timer

计时器。

```cpp
class Timer {
public:
    void start();
    void stop();
    void reset();
    float getElapsedMs() const;
    float getElapsedUs() const;
    float getElapsedSec() const;
    bool isRunning() const;
};

class RateTimer {
public:
    RateTimer(float rateHz);
    bool ready();
    void reset();
    float getActualRate() const;
};
```

### Logger

日志工具。

```cpp
class Logger {
public:
    enum class Level { Debug, Info, Warning, Error };

    static void setLevel(Level level);
    static void setOutput(std::ostream& os);

    static void debug(const char* fmt, ...);
    static void info(const char* fmt, ...);
    static void warning(const char* fmt, ...);
    static void error(const char* fmt, ...);
};

// 宏定义
#define OMNI_LOG_DEBUG(fmt, ...)
#define OMNI_LOG_INFO(fmt, ...)
#define OMNI_LOG_WARN(fmt, ...)
#define OMNI_LOG_ERROR(fmt, ...)
```

---

## 枚举类型汇总

### ControlMode

```cpp
enum class ControlMode {
    Off,
    Position,
    Velocity,
    Torque,
    Current,
    Voltage,
    Duty
};
```

### MotorType

```cpp
enum class MotorType {
    Unknown,
    BLDC,
    PMSM,
    Stepper,
    BrushedDC,
    Servo,
    VoiceCoil,
    LinearSynchronous,
    LinearStepper,
    Custom
};
```

### FocMode

```cpp
enum class FocMode {
    SVPWM,
    SPWM,
    SixStep
};
```

### BrakeMode

```cpp
enum class BrakeMode {
    Coast,
    Brake,
    Hold
};
```

### StepMode

```cpp
enum class StepMode {
    Full,
    Half,
    Quarter,
    Eighth,
    Sixteenth,
    ThirtySecond
};
```

### HomingMode

```cpp
enum class HomingMode {
    SensorBased,
    CurrentBased,
    IndexPulse,
    Hardstop,
    Manual
};
```

