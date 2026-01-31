# OmniMotion - 通用电机控制库

<p align="center">
  <b>但凡用电机，皆可用此库</b>
</p>

OmniMotion 是一个全面的 C++ 电机控制库，支持市面上几乎所有主流电机类型、通信协议和控制算法。无论您是开发机器人、CNC机床、3D打印机还是自动化设备，OmniMotion 都能为您提供统一、高效的电机控制解决方案。

## 特性

### 全类型覆盖
- **旋转电机**: BLDC、PMSM、步进、有刷DC、伺服、空心杯等
- **直线电机**: 直线同步电机、音圈电机、直线步进等
- **特殊电机**: 舵机、轮毂电机、力矩电机等

### 全协议兼容
- **现场总线**: CAN、RS485、EtherCAT
- **应用协议**: MIT协议、Modbus RTU、DJI协议、ODrive协议
- **控制算法**: FOC、六步换相、SVPWM

### 全算法集成
- **闭环控制**: PID、级联PID、滑模控制、MPC、自适应控制
- **运动规划**: S曲线、梯形曲线、多项式、样条插值
- **高级功能**: 扰动观测、振动抑制、阻抗控制

### 高度可扩展
- 清晰的接口抽象
- 自定义驱动支持
- 平台无关的HAL层

## 快速开始

### 环境要求

- C++17 兼容编译器 (GCC 7+, Clang 8+, MSVC 2019+)
- CMake 3.14+
- (可选) Google Test 用于单元测试

### 编译安装

```bash
# 克隆仓库
git clone https://github.com/your-repo/OmniMotion.git
cd OmniMotion

# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 编译
cmake --build .

# 安装 (可选)
cmake --install .
```

### 基本使用

```cpp
#include <omni/omnimotion.hpp>

int main() {
    // 创建硬件接口 (需要根据平台实现)
    auto pwm = createPwm3Phase(TIM1);
    auto encoder = createEncoder(TIM2);
    auto adcA = createAdc(ADC1, CH0);
    auto adcB = createAdc(ADC1, CH1);
    auto adcC = createAdc(ADC1, CH2);

    // 创建BLDC驱动
    omni::driver::BldcDriver motor(pwm, adcA, adcB, adcC, encoder);

    // 配置电机参数
    omni::driver::MotorParams params;
    params.polePairs = 7;
    params.maxCurrent = 20.0f;
    motor.setParams(params);
    motor.init();

    // 创建高层控制器
    omni::app::MotorController ctrl(&motor);
    ctrl.setPositionPid(50.0f, 0.1f, 0.5f);
    ctrl.enable();

    // S曲线运动到目标位置
    ctrl.moveWithSCurve(3.14159f, 10.0f, 100.0f, 1000.0f);

    // 主控制循环
    while (true) {
        ctrl.update(0.001f);  // 1ms控制周期
        delay_ms(1);
    }
}
```

## 架构概览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Application Layer (应用层)                        │
│     MotorController │ MultiAxisController │ TeachRecorder │ ...        │
├─────────────────────────────────────────────────────────────────────────┤
│                         Motion Layer (运动规划层)                         │
│     S-Curve │ Trapezoidal │ Polynomial │ Spline │ OnlinePlanner        │
├─────────────────────────────────────────────────────────────────────────┤
│                         Control Layer (控制算法层)                        │
│     PID │ CascadePID │ Feedforward │ SlidingMode │ MPC │ Adaptive      │
├─────────────────────────────────────────────────────────────────────────┤
│                         Protocol Layer (协议层)                          │
│     FOC │ MIT │ CAN │ Modbus │ DJI │ ODrive │ CyberGear                │
├─────────────────────────────────────────────────────────────────────────┤
│                          Driver Layer (驱动层)                           │
│     BLDC │ Stepper │ DC │ Servo │ VCM │ Linear │ Custom                │
├─────────────────────────────────────────────────────────────────────────┤
│                           HAL Layer (硬件抽象层)                          │
│     GPIO │ PWM │ ADC │ Encoder │ CAN │ SPI │ UART │ Timer              │
└─────────────────────────────────────────────────────────────────────────┘
```

## 文档

- [快速入门指南](docs/getting_started.md)
- [API 参考手册](docs/api_reference.md)
- [电机驱动指南](docs/motor_drivers.md)
- [控制算法详解](docs/control_algorithms.md)
- [运动规划教程](docs/motion_planning.md)
- [平台移植指南](docs/porting_guide.md)
- [示例代码说明](docs/examples.md)

## 示例

| 示例 | 说明 |
|------|------|
| [bldc_foc](examples/bldc_foc/) | BLDC电机FOC控制 |
| [stepper_scurve](examples/stepper_scurve/) | 步进电机S曲线运动 |
| [multi_axis_sync](examples/multi_axis_sync/) | 多轴同步运动 |
| [trajectory_tracking](examples/trajectory_tracking/) | 轨迹跟踪 |
| [teach_playback](examples/teach_playback/) | 示教回放 |

## 支持的平台

- STM32 (HAL/LL)
- ESP32
- Raspberry Pi
- Linux (通用)
- Windows (仿真)

## 贡献

欢迎贡献代码！请阅读 [CONTRIBUTING.md](CONTRIBUTING.md) 了解如何参与。

## 许可证

本项目采用 GPL-3.0 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 致谢

感谢所有为本项目做出贡献的开发者。
