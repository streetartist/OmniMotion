/**
 * @file teach_recorder.hpp
 * @brief Teach pendant / trajectory recording functionality
 */

#pragma once

#include "motor_controller.hpp"
#include "multi_axis_controller.hpp"
#include <vector>
#include <string>
#include <fstream>

namespace omni::app {

/**
 * @brief Teach recorder for recording and playing back motion
 */
class TeachRecorder {
public:
    /**
     * @brief Construct for single axis
     * @param controller Motor controller
     */
    explicit TeachRecorder(MotorController* controller)
        : singleAxis_(controller)
        , multiAxis_(nullptr)
        , teaching_(false)
        , recording_(false)
        , playing_(false)
        , playbackIndex_(0)
        , recordInterval_(0.01f)
        , timeSinceRecord_(0) {}

    /**
     * @brief Construct for multiple axes
     * @param controller Multi-axis controller
     */
    explicit TeachRecorder(MultiAxisController* controller)
        : singleAxis_(nullptr)
        , multiAxis_(controller)
        , teaching_(false)
        , recording_(false)
        , playing_(false)
        , playbackIndex_(0)
        , recordInterval_(0.01f)
        , timeSinceRecord_(0) {}

    // === Teaching Mode ===

    /**
     * @brief Enter teaching mode (allows manual movement)
     */
    void startTeaching() {
        teaching_ = true;
        // Typically would disable position control or enable compliance
    }

    /**
     * @brief Exit teaching mode
     */
    void stopTeaching() {
        teaching_ = false;
        recording_ = false;
    }

    /**
     * @brief Check if in teaching mode
     */
    bool isTeaching() const { return teaching_; }

    // === Point Recording ===

    /**
     * @brief Record current position as a waypoint
     */
    void recordPoint() {
        if (singleAxis_) {
            SingleAxisPoint point;
            point.position = singleAxis_->getPosition();
            point.velocity = singleAxis_->getVelocity();
            point.time = points_.empty() ? 0 : points_.back().time + 1.0f;
            points_.push_back(point);
        } else if (multiAxis_) {
            MultiAxisPoint point;
            point.positions.resize(multiAxis_->getNumAxes());
            for (size_t i = 0; i < multiAxis_->getNumAxes(); i++) {
                auto* axis = multiAxis_->getAxis(i);
                point.positions[i] = axis ? axis->getPosition() : 0;
            }
            point.time = multiPoints_.empty() ? 0 : multiPoints_.back().time + 1.0f;
            multiPoints_.push_back(point);
        }
    }

    /**
     * @brief Start continuous trajectory recording
     * @param interval Recording interval in seconds
     */
    void recordTrajectory(float interval) {
        recordInterval_ = interval;
        recording_ = true;
        timeSinceRecord_ = interval;  // Record immediately
    }

    /**
     * @brief Stop continuous recording
     */
    void stopRecordTrajectory() {
        recording_ = false;
    }

    /**
     * @brief Check if recording
     */
    bool isRecording() const { return recording_; }

    // === Editing ===

    /**
     * @brief Delete a recorded point
     * @param index Point index
     */
    void deletePoint(size_t index) {
        if (index < points_.size()) {
            points_.erase(points_.begin() + index);
        }
        if (index < multiPoints_.size()) {
            multiPoints_.erase(multiPoints_.begin() + index);
        }
    }

    /**
     * @brief Insert a point (single axis)
     * @param index Insertion index
     * @param position Position value
     */
    void insertPoint(size_t index, float position) {
        SingleAxisPoint point;
        point.position = position;
        point.time = index > 0 ? points_[index - 1].time + 0.5f : 0;

        if (index >= points_.size()) {
            points_.push_back(point);
        } else {
            points_.insert(points_.begin() + index, point);
        }
    }

    /**
     * @brief Insert a point (multi axis)
     * @param index Insertion index
     * @param positions Position values
     */
    void insertPoint(size_t index, const std::vector<float>& positions) {
        MultiAxisPoint point;
        point.positions = positions;
        point.time = index > 0 ? multiPoints_[index - 1].time + 0.5f : 0;

        if (index >= multiPoints_.size()) {
            multiPoints_.push_back(point);
        } else {
            multiPoints_.insert(multiPoints_.begin() + index, point);
        }
    }

    /**
     * @brief Clear all recorded data
     */
    void clear() {
        points_.clear();
        multiPoints_.clear();
        playbackIndex_ = 0;
    }

    // === File I/O ===

    /**
     * @brief Save recorded data to file
     * @param filename Output filename
     * @return true if successful
     */
    bool saveToFile(const char* filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return false;

        file << "{\n  \"points\": [\n";

        if (singleAxis_) {
            for (size_t i = 0; i < points_.size(); i++) {
                file << "    {\"time\": " << points_[i].time
                     << ", \"position\": " << points_[i].position
                     << ", \"velocity\": " << points_[i].velocity << "}";
                if (i < points_.size() - 1) file << ",";
                file << "\n";
            }
        } else {
            for (size_t i = 0; i < multiPoints_.size(); i++) {
                file << "    {\"time\": " << multiPoints_[i].time
                     << ", \"positions\": [";
                for (size_t j = 0; j < multiPoints_[i].positions.size(); j++) {
                    file << multiPoints_[i].positions[j];
                    if (j < multiPoints_[i].positions.size() - 1) file << ", ";
                }
                file << "]}";
                if (i < multiPoints_.size() - 1) file << ",";
                file << "\n";
            }
        }

        file << "  ]\n}\n";
        return true;
    }

    /**
     * @brief Load recorded data from file
     * @param filename Input filename
     * @return true if successful
     */
    bool loadFromFile(const char* filename) {
        // Simplified JSON parsing - for production use a proper JSON library
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        clear();

        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

        // Basic parsing - find position values
        // This is a simplified implementation
        size_t pos = 0;
        while ((pos = content.find("\"position\":", pos)) != std::string::npos) {
            pos += 11;
            SingleAxisPoint point;
            point.position = std::stof(content.substr(pos));
            points_.push_back(point);
        }

        return !points_.empty();
    }

    // === Playback ===

    /**
     * @brief Start playback
     * @param speedFactor Speed multiplier
     */
    void playback(float speedFactor = 1.0f) {
        playbackIndex_ = 0;
        playbackSpeed_ = speedFactor;
        playbackTime_ = 0;
        playing_ = true;
    }

    /**
     * @brief Start looped playback
     * @param count Number of loops (-1 for infinite)
     */
    void playbackLoop(int count = -1) {
        loopCount_ = count;
        currentLoop_ = 0;
        playback();
    }

    /**
     * @brief Stop playback
     */
    void stopPlayback() {
        playing_ = false;
    }

    /**
     * @brief Check if playing
     */
    bool isPlaying() const { return playing_; }

    // === Data Access ===

    /**
     * @brief Get number of recorded points
     */
    size_t getPointCount() const {
        return singleAxis_ ? points_.size() : multiPoints_.size();
    }

    /**
     * @brief Get recorded point (single axis)
     * @param index Point index
     * @return Position value
     */
    float getPoint(size_t index) const {
        return (index < points_.size()) ? points_[index].position : 0;
    }

    /**
     * @brief Get recorded point (multi axis)
     * @param index Point index
     * @return Position values
     */
    std::vector<float> getMultiPoint(size_t index) const {
        return (index < multiPoints_.size()) ? multiPoints_[index].positions
                                              : std::vector<float>();
    }

    // === Update ===

    /**
     * @brief Update recorder (call in control loop)
     * @param dt Time step
     */
    void update(float dt) {
        // Handle continuous recording
        if (recording_ && teaching_) {
            timeSinceRecord_ += dt;
            if (timeSinceRecord_ >= recordInterval_) {
                recordPoint();
                timeSinceRecord_ = 0;
            }
        }

        // Handle playback
        if (playing_) {
            playbackTime_ += dt * playbackSpeed_;

            if (singleAxis_ && !points_.empty()) {
                // Find current point
                float pos = interpolateSingle(playbackTime_);
                singleAxis_->moveTo(pos);

                if (playbackTime_ >= points_.back().time) {
                    handlePlaybackComplete();
                }
            } else if (multiAxis_ && !multiPoints_.empty()) {
                std::vector<float> positions = interpolateMulti(playbackTime_);
                multiAxis_->moveTo(positions);

                if (playbackTime_ >= multiPoints_.back().time) {
                    handlePlaybackComplete();
                }
            }
        }
    }

private:
    struct SingleAxisPoint {
        float time = 0;
        float position = 0;
        float velocity = 0;
    };

    struct MultiAxisPoint {
        float time = 0;
        std::vector<float> positions;
    };

    MotorController* singleAxis_;
    MultiAxisController* multiAxis_;

    bool teaching_;
    bool recording_;
    bool playing_;

    std::vector<SingleAxisPoint> points_;
    std::vector<MultiAxisPoint> multiPoints_;

    size_t playbackIndex_;
    float playbackSpeed_ = 1.0f;
    float playbackTime_ = 0;
    int loopCount_ = 0;
    int currentLoop_ = 0;

    float recordInterval_;
    float timeSinceRecord_;

    float interpolateSingle(float time) const {
        if (points_.empty()) return 0;
        if (time <= points_.front().time) return points_.front().position;
        if (time >= points_.back().time) return points_.back().position;

        for (size_t i = 1; i < points_.size(); i++) {
            if (time <= points_[i].time) {
                float t0 = points_[i - 1].time;
                float t1 = points_[i].time;
                float u = (time - t0) / (t1 - t0);
                return points_[i - 1].position * (1 - u) + points_[i].position * u;
            }
        }
        return points_.back().position;
    }

    std::vector<float> interpolateMulti(float time) const {
        if (multiPoints_.empty()) return {};
        if (time <= multiPoints_.front().time) return multiPoints_.front().positions;
        if (time >= multiPoints_.back().time) return multiPoints_.back().positions;

        for (size_t i = 1; i < multiPoints_.size(); i++) {
            if (time <= multiPoints_[i].time) {
                float t0 = multiPoints_[i - 1].time;
                float t1 = multiPoints_[i].time;
                float u = (time - t0) / (t1 - t0);

                std::vector<float> result(multiPoints_[i].positions.size());
                for (size_t j = 0; j < result.size(); j++) {
                    result[j] = multiPoints_[i - 1].positions[j] * (1 - u) +
                               multiPoints_[i].positions[j] * u;
                }
                return result;
            }
        }
        return multiPoints_.back().positions;
    }

    void handlePlaybackComplete() {
        currentLoop_++;
        if (loopCount_ < 0 || currentLoop_ < loopCount_) {
            playbackTime_ = 0;  // Restart
        } else {
            playing_ = false;
        }
    }
};

}  // namespace omni::app
