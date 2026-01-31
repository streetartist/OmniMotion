/**
 * @file multi_axis_controller.hpp
 * @brief Multi-axis coordinated motion controller
 */

#pragma once

#include "motor_controller.hpp"
#include "omni/motion/multi_axis_trajectory.hpp"
#include <vector>
#include <memory>

namespace omni::app {

/**
 * @brief Arc plane for 2D arc interpolation
 */
enum class ArcPlane {
    XY,
    XZ,
    YZ
};

/**
 * @brief Multi-axis motion controller
 *
 * Coordinates motion across multiple axes with synchronization
 * and interpolation capabilities.
 */
class MultiAxisController {
public:
    /**
     * @brief Construct multi-axis controller
     * @param numAxes Number of axes
     */
    explicit MultiAxisController(size_t numAxes)
        : numAxes_(numAxes)
        , multiTraj_(numAxes)
        , moving_(false)
        , elapsedTime_(0) {
        axes_.resize(numAxes, nullptr);
        ownedAxes_.resize(numAxes);
    }

    /**
     * @brief Add axis from existing controller
     * @param index Axis index
     * @param axis Motor controller
     */
    void addAxis(size_t index, MotorController* axis) {
        if (index < numAxes_) {
            axes_[index] = axis;
        }
    }

    /**
     * @brief Add axis from driver (creates internal controller)
     * @param index Axis index
     * @param driver Motor driver
     */
    void addAxis(size_t index, driver::IMotorDriver* driver) {
        if (index < numAxes_) {
            ownedAxes_[index] = std::make_unique<MotorController>(driver);
            axes_[index] = ownedAxes_[index].get();
        }
    }

    /**
     * @brief Get axis controller
     * @param index Axis index
     * @return Motor controller or nullptr
     */
    MotorController* getAxis(size_t index) {
        return (index < numAxes_) ? axes_[index] : nullptr;
    }

    /**
     * @brief Get number of axes
     */
    size_t getNumAxes() const { return numAxes_; }

    // === Synchronized Motion ===

    /**
     * @brief Move all axes to positions (synchronized)
     * @param positions Target positions
     */
    void moveTo(const std::vector<float>& positions) {
        if (positions.size() != numAxes_) return;

        std::vector<float> startPositions(numAxes_);
        for (size_t i = 0; i < numAxes_; i++) {
            if (axes_[i]) {
                startPositions[i] = axes_[i]->getPosition();
            }
        }

        multiTraj_.plan(startPositions, positions);
        moving_ = true;
        elapsedTime_ = 0;
    }

    /**
     * @brief Move to positions with specified velocity
     */
    void moveTo(const std::vector<float>& positions, float velocity) {
        // Set velocity constraints for all axes
        motion::MotionConstraints c;
        c.maxVelocity = velocity;
        c.maxAcceleration = velocity * 10.0f;
        c.maxJerk = velocity * 100.0f;

        for (size_t i = 0; i < numAxes_; i++) {
            multiTraj_.setAxisConstraints(i, c);
        }

        moveTo(positions);
    }

    // === Interpolated Motion ===

    /**
     * @brief Linear interpolated move
     * @param endPos End position for all axes
     */
    void linearMove(const std::vector<float>& endPos) {
        moveTo(endPos);  // Synchronized move is effectively linear
    }

    /**
     * @brief Linear move with feedrate
     * @param endPos End position
     * @param feedrate Feedrate (units/sec along path)
     */
    void linearMove(const std::vector<float>& endPos, float feedrate) {
        // Calculate path length
        float pathLength = 0;
        for (size_t i = 0; i < numAxes_ && i < endPos.size(); i++) {
            if (axes_[i]) {
                float d = endPos[i] - axes_[i]->getPosition();
                pathLength += d * d;
            }
        }
        pathLength = std::sqrt(pathLength);

        // Scale velocity
        float time = pathLength / feedrate;
        if (time > 0) {
            // TODO: Implement proper feedrate control
            moveTo(endPos);
        }
    }

    /**
     * @brief Arc interpolated move
     * @param endPos End position
     * @param center Arc center
     * @param plane Arc plane (XY, XZ, YZ)
     * @param clockwise Direction
     */
    void arcMove(const std::vector<float>& endPos,
                 const std::vector<float>& center,
                 ArcPlane plane, bool clockwise) {
        // Get axis indices for plane
        size_t axis1 = 0, axis2 = 1;
        switch (plane) {
            case ArcPlane::XY: axis1 = 0; axis2 = 1; break;
            case ArcPlane::XZ: axis1 = 0; axis2 = 2; break;
            case ArcPlane::YZ: axis1 = 1; axis2 = 2; break;
        }

        if (axis1 >= numAxes_ || axis2 >= numAxes_) return;
        if (!axes_[axis1] || !axes_[axis2]) return;

        // For now, just do a linear move
        // TODO: Implement proper arc interpolation
        moveTo(endPos);
        (void)center;
        (void)clockwise;
    }

    // === Trajectory Tracking ===

    /**
     * @brief Follow multi-axis trajectory
     * @param trajectory Vector of trajectory points for each axis
     */
    void followTrajectory(const std::vector<std::vector<motion::TrajectoryPoint>>& trajectory) {
        trajectory_ = trajectory;
        trajectoryIndex_ = 0;
        followingTrajectory_ = true;
        moving_ = true;
        elapsedTime_ = 0;
    }

    // === Synchronization ===

    /**
     * @brief Start synchronized motion
     */
    void startSynchronized() {
        for (auto* axis : axes_) {
            if (axis) axis->enable();
        }
    }

    /**
     * @brief Wait for all axes to complete motion
     */
    void waitForCompletion() {
        // Blocking wait - in real implementation would use proper synchronization
        while (moving_) {
            update(0.001f);
        }
    }

    /**
     * @brief Check if all axes are settled
     */
    bool allSettled() const {
        for (const auto* axis : axes_) {
            if (axis && !axis->isSettled()) {
                return false;
            }
        }
        return true;
    }

    // === Emergency ===

    /**
     * @brief Emergency stop all axes
     */
    void emergencyStopAll() {
        for (auto* axis : axes_) {
            if (axis) axis->emergencyStop();
        }
        moving_ = false;
    }

    // === Update ===

    /**
     * @brief Update all axes
     * @param dt Time step
     */
    void update(float dt) {
        if (moving_) {
            elapsedTime_ += dt;

            if (followingTrajectory_) {
                // Follow recorded trajectory
                updateTrajectoryFollowing(dt);
            } else {
                // Follow synchronized trajectory
                std::vector<motion::TrajectoryPoint> points;
                multiTraj_.evaluate(elapsedTime_, points);

                for (size_t i = 0; i < numAxes_; i++) {
                    if (axes_[i]) {
                        axes_[i]->moveTo(points[i].position);
                    }
                }

                if (elapsedTime_ >= multiTraj_.getDuration()) {
                    moving_ = false;
                }
            }
        }

        // Update individual axes
        for (auto* axis : axes_) {
            if (axis) axis->update(dt);
        }
    }

    /**
     * @brief Check if any motion is in progress
     */
    bool isMoving() const { return moving_; }

private:
    size_t numAxes_;
    std::vector<MotorController*> axes_;
    std::vector<std::unique_ptr<MotorController>> ownedAxes_;

    motion::MultiAxisTrajectory multiTraj_;

    bool moving_;
    float elapsedTime_;

    // Trajectory following
    std::vector<std::vector<motion::TrajectoryPoint>> trajectory_;
    size_t trajectoryIndex_ = 0;
    bool followingTrajectory_ = false;

    void updateTrajectoryFollowing(float dt) {
        (void)dt;

        if (trajectory_.empty() || trajectoryIndex_ >= trajectory_[0].size()) {
            followingTrajectory_ = false;
            moving_ = false;
            return;
        }

        // Send trajectory points to each axis
        for (size_t i = 0; i < numAxes_; i++) {
            if (axes_[i] && i < trajectory_.size() &&
                trajectoryIndex_ < trajectory_[i].size()) {
                axes_[i]->moveTo(trajectory_[i][trajectoryIndex_].position);
            }
        }

        trajectoryIndex_++;
    }
};

}  // namespace omni::app
