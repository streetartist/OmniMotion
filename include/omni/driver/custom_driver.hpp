/**
 * @file custom_driver.hpp
 * @brief Custom motor driver with callback-based implementation
 */

#pragma once

#include "motor_driver.hpp"
#include <functional>

namespace omni::driver {

/**
 * @brief Custom motor driver
 *
 * Allows implementing motor control through callback functions,
 * making it easy to integrate unsupported motor types.
 */
class CustomMotorDriver : public IMotorDriver {
public:
    // Callback types
    using InitCallback = std::function<bool()>;
    using DeinitCallback = std::function<void()>;
    using EnableCallback = std::function<void(bool)>;
    using UpdateCallback = std::function<void()>;
    using ControlCallback = std::function<void(ControlMode, float)>;
    using StateCallback = std::function<MotorState()>;
    using PositionCallback = std::function<float()>;
    using VelocityCallback = std::function<float()>;
    using SetPositionCallback = std::function<void(float)>;
    using SetVelocityCallback = std::function<void(float)>;
    using SetTorqueCallback = std::function<void(float)>;

    CustomMotorDriver();
    ~CustomMotorDriver() override = default;

    // IMotorDriver interface
    bool init() override;
    void deinit() override;
    void enable() override;
    void disable() override;
    bool isEnabled() const override;
    void emergencyStop() override;

    void setControlMode(ControlMode mode) override;
    ControlMode getControlMode() const override;

    void setPosition(float pos) override;
    void setVelocity(float vel) override;
    void setTorque(float torque) override;
    void setCurrent(float current) override;
    void setVoltage(float voltage) override;
    void setDuty(float duty) override;

    MotorState getState() const override;
    float getPosition() const override;
    float getVelocity() const override;
    float getTorque() const override;
    float getCurrent() const override;
    float getTemperature() const override;

    void setParams(const MotorParams& params) override;
    MotorParams getParams() const override;

    void update() override;

    MotorType getType() const override { return motorType_; }
    const char* getName() const override { return name_.c_str(); }

    uint32_t getErrorCode() const override;
    void clearErrors() override;
    bool hasFault() const override;

    // ===== Callback Registration =====

    /**
     * @brief Set initialization callback
     * @param cb Callback returning true on success
     */
    void setInitCallback(InitCallback cb) { initCb_ = cb; }

    /**
     * @brief Set deinitialization callback
     * @param cb Callback for cleanup
     */
    void setDeinitCallback(DeinitCallback cb) { deinitCb_ = cb; }

    /**
     * @brief Set enable/disable callback
     * @param cb Callback receiving enable state
     */
    void setEnableCallback(EnableCallback cb) { enableCb_ = cb; }

    /**
     * @brief Set update callback (called every control cycle)
     * @param cb Update callback
     */
    void setUpdateCallback(UpdateCallback cb) { updateCb_ = cb; }

    /**
     * @brief Set control callback
     * @param cb Callback receiving control mode and value
     */
    void setControlCallback(ControlCallback cb) { controlCb_ = cb; }

    /**
     * @brief Set state callback
     * @param cb Callback returning current motor state
     */
    void setStateCallback(StateCallback cb) { stateCb_ = cb; }

    /**
     * @brief Set position read callback
     * @param cb Callback returning position
     */
    void setPositionCallback(PositionCallback cb) { positionCb_ = cb; }

    /**
     * @brief Set velocity read callback
     * @param cb Callback returning velocity
     */
    void setVelocityCallback(VelocityCallback cb) { velocityCb_ = cb; }

    /**
     * @brief Set position write callback
     * @param cb Callback to set position target
     */
    void setSetPositionCallback(SetPositionCallback cb) { setPositionCb_ = cb; }

    /**
     * @brief Set velocity write callback
     * @param cb Callback to set velocity target
     */
    void setSetVelocityCallback(SetVelocityCallback cb) { setVelocityCb_ = cb; }

    /**
     * @brief Set torque write callback
     * @param cb Callback to set torque target
     */
    void setSetTorqueCallback(SetTorqueCallback cb) { setTorqueCb_ = cb; }

    /**
     * @brief Set motor type identifier
     * @param type Motor type
     */
    void setMotorType(MotorType type) { motorType_ = type; }

    /**
     * @brief Set motor name
     * @param name Motor name string
     */
    void setName(const std::string& name) { name_ = name; }

private:
    // Callbacks
    InitCallback initCb_;
    DeinitCallback deinitCb_;
    EnableCallback enableCb_;
    UpdateCallback updateCb_;
    ControlCallback controlCb_;
    StateCallback stateCb_;
    PositionCallback positionCb_;
    VelocityCallback velocityCb_;
    SetPositionCallback setPositionCb_;
    SetVelocityCallback setVelocityCb_;
    SetTorqueCallback setTorqueCb_;

    // State
    MotorState state_;
    MotorParams params_;
    ControlMode controlMode_;
    bool enabled_;
    uint32_t errorCode_;

    // Identity
    MotorType motorType_;
    std::string name_;
};

}  // namespace omni::driver
