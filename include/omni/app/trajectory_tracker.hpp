/**
 * @file trajectory_tracker.hpp
 * @brief Trajectory tracking and playback controller
 */

#pragma once

#include "motor_controller.hpp"
#include "omni/motion/trajectory.hpp"
#include <vector>
#include <string>
#include <fstream>

namespace omni::app {

/**
 * @brief Trajectory tracker state
 */
enum class TrackerState {
    Idle,
    Running,
    Paused,
    Finished
};

/**
 * @brief Trajectory tracker
 *
 * Loads and executes pre-recorded or generated trajectories.
 */
class TrajectoryTracker {
public:
    /**
     * @brief Construct trajectory tracker
     * @param controller Motor controller to track with
     */
    explicit TrajectoryTracker(MotorController* controller)
        : controller_(controller)
        , state_(TrackerState::Idle)
        , elapsedTime_(0)
        , speedOverride_(1.0f)
        , looping_(false)
        , loopCount_(0)
        , currentLoop_(0) {}

    /**
     * @brief Load trajectory from generator
     * @param trajectory Trajectory generator
     */
    void loadTrajectory(motion::ITrajectoryGenerator* trajectory) {
        trajectory_ = trajectory;
        points_.clear();
        state_ = TrackerState::Idle;
        elapsedTime_ = 0;
    }

    /**
     * @brief Load trajectory from point list
     * @param points Trajectory points
     */
    void loadTrajectory(const std::vector<motion::TrajectoryPoint>& points) {
        points_ = points;
        trajectory_ = nullptr;
        state_ = TrackerState::Idle;
        elapsedTime_ = 0;
    }

    /**
     * @brief Load trajectory from CSV file
     * @param filename CSV file path
     * @return true if load successful
     */
    bool loadFromFile(const char* filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        points_.clear();
        std::string line;

        // Skip header
        std::getline(file, line);

        while (std::getline(file, line)) {
            motion::TrajectoryPoint point;
            if (parseCsvLine(line, point)) {
                points_.push_back(point);
            }
        }

        trajectory_ = nullptr;
        state_ = TrackerState::Idle;
        elapsedTime_ = 0;
        return !points_.empty();
    }

    /**
     * @brief Load trajectory from G-code file
     * @param filename G-code file path
     * @return true if load successful
     */
    bool loadFromGCode(const char* filename) {
        // Basic G-code parsing - just extract G0/G1 moves
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        points_.clear();
        float currentPos = 0;
        float feedrate = 1.0f;
        float time = 0;

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == ';') continue;

            // Parse G0/G1 commands
            if (line.find("G0") != std::string::npos ||
                line.find("G1") != std::string::npos) {

                // Extract X position (simplified)
                size_t xPos = line.find('X');
                if (xPos != std::string::npos) {
                    float newPos = std::stof(line.substr(xPos + 1));
                    float dist = std::abs(newPos - currentPos);
                    float dt = dist / feedrate;

                    motion::TrajectoryPoint point;
                    point.time = time + dt;
                    point.position = newPos;
                    points_.push_back(point);

                    currentPos = newPos;
                    time += dt;
                }

                // Extract feedrate
                size_t fPos = line.find('F');
                if (fPos != std::string::npos) {
                    feedrate = std::stof(line.substr(fPos + 1)) / 60.0f;  // mm/min to mm/s
                }
            }
        }

        trajectory_ = nullptr;
        state_ = TrackerState::Idle;
        elapsedTime_ = 0;
        return !points_.empty();
    }

    // === Playback Control ===

    /**
     * @brief Start playback
     */
    void start() {
        if (state_ == TrackerState::Idle || state_ == TrackerState::Finished) {
            elapsedTime_ = 0;
            currentLoop_ = 0;
        }
        state_ = TrackerState::Running;
    }

    /**
     * @brief Pause playback
     */
    void pause() {
        if (state_ == TrackerState::Running) {
            state_ = TrackerState::Paused;
        }
    }

    /**
     * @brief Resume playback
     */
    void resume() {
        if (state_ == TrackerState::Paused) {
            state_ = TrackerState::Running;
        }
    }

    /**
     * @brief Stop playback
     */
    void stop() {
        state_ = TrackerState::Idle;
        elapsedTime_ = 0;
    }

    /**
     * @brief Reset to beginning
     */
    void reset() {
        elapsedTime_ = 0;
        currentLoop_ = 0;
        if (state_ == TrackerState::Finished) {
            state_ = TrackerState::Idle;
        }
    }

    /**
     * @brief Set speed override factor
     * @param factor Speed multiplier (0.1 to 2.0)
     */
    void setSpeedOverride(float factor) {
        speedOverride_ = std::clamp(factor, 0.1f, 2.0f);
    }

    /**
     * @brief Get speed override
     */
    float getSpeedOverride() const { return speedOverride_; }

    /**
     * @brief Enable looping
     * @param count Number of loops (-1 for infinite)
     */
    void setLooping(int count) {
        looping_ = (count != 0);
        loopCount_ = count;
    }

    // === Status ===

    /**
     * @brief Get playback progress (0.0 to 1.0)
     */
    float getProgress() const {
        float duration = getDuration();
        if (duration <= 0) return 0;
        return std::clamp(elapsedTime_ / duration, 0.0f, 1.0f);
    }

    /**
     * @brief Get elapsed time
     */
    float getElapsedTime() const { return elapsedTime_; }

    /**
     * @brief Get remaining time
     */
    float getRemainingTime() const {
        return std::max(0.0f, getDuration() - elapsedTime_);
    }

    /**
     * @brief Check if running
     */
    bool isRunning() const { return state_ == TrackerState::Running; }

    /**
     * @brief Check if finished
     */
    bool isFinished() const { return state_ == TrackerState::Finished; }

    /**
     * @brief Get current state
     */
    TrackerState getState() const { return state_; }

    // === Update ===

    /**
     * @brief Update tracker
     * @param dt Time step
     */
    void update(float dt) {
        if (state_ != TrackerState::Running) return;

        elapsedTime_ += dt * speedOverride_;

        // Get current command
        motion::TrajectoryPoint cmd = getCurrentPoint();

        // Send to controller
        if (controller_) {
            controller_->moveTo(cmd.position);
        }

        // Check completion
        if (elapsedTime_ >= getDuration()) {
            if (looping_) {
                currentLoop_++;
                if (loopCount_ < 0 || currentLoop_ < loopCount_) {
                    elapsedTime_ = 0;  // Restart
                } else {
                    state_ = TrackerState::Finished;
                }
            } else {
                state_ = TrackerState::Finished;
            }
        }
    }

private:
    MotorController* controller_;
    motion::ITrajectoryGenerator* trajectory_ = nullptr;
    std::vector<motion::TrajectoryPoint> points_;

    TrackerState state_;
    float elapsedTime_;
    float speedOverride_;
    bool looping_;
    int loopCount_;
    int currentLoop_;

    float getDuration() const {
        if (trajectory_) {
            return trajectory_->getDuration();
        }
        if (!points_.empty()) {
            return points_.back().time;
        }
        return 0;
    }

    motion::TrajectoryPoint getCurrentPoint() const {
        if (trajectory_) {
            return trajectory_->evaluate(elapsedTime_);
        }

        if (points_.empty()) {
            return motion::TrajectoryPoint();
        }

        // Interpolate between points
        for (size_t i = 1; i < points_.size(); i++) {
            if (elapsedTime_ <= points_[i].time) {
                float t0 = points_[i - 1].time;
                float t1 = points_[i].time;
                float u = (elapsedTime_ - t0) / (t1 - t0);

                motion::TrajectoryPoint result;
                result.time = elapsedTime_;
                result.position = points_[i - 1].position * (1 - u) +
                                 points_[i].position * u;
                result.velocity = points_[i - 1].velocity * (1 - u) +
                                 points_[i].velocity * u;
                return result;
            }
        }

        return points_.back();
    }

    bool parseCsvLine(const std::string& line, motion::TrajectoryPoint& point) {
        // Simple CSV parsing: time,position,velocity,acceleration
        size_t pos = 0;
        int field = 0;

        while (pos < line.size()) {
            size_t next = line.find(',', pos);
            if (next == std::string::npos) next = line.size();

            std::string value = line.substr(pos, next - pos);
            float fval = std::stof(value);

            switch (field) {
                case 0: point.time = fval; break;
                case 1: point.position = fval; break;
                case 2: point.velocity = fval; break;
                case 3: point.acceleration = fval; break;
            }

            pos = next + 1;
            field++;
        }

        return field >= 2;  // At least time and position
    }
};

}  // namespace omni::app
