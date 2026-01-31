/**
 * @file multi_axis_trajectory.hpp
 * @brief Multi-axis synchronized trajectory planning
 */

#pragma once

#include "trajectory.hpp"
#include "scurve_profile.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace omni::motion {

/**
 * @brief Multi-axis trajectory with synchronization
 *
 * Plans and synchronizes trajectories across multiple axes so all
 * axes reach their targets at the same time.
 */
class MultiAxisTrajectory {
public:
    /**
     * @brief Construct multi-axis trajectory
     * @param numAxes Number of axes
     */
    explicit MultiAxisTrajectory(size_t numAxes)
        : numAxes_(numAxes)
        , syncDuration_(0)
        , valid_(false) {
        profiles_.resize(numAxes);
        constraints_.resize(numAxes);

        // Default constraints
        MotionConstraints defaultConstraints{10.0f, 100.0f, 0, 1000.0f, 0};
        for (size_t i = 0; i < numAxes; i++) {
            constraints_[i] = defaultConstraints;
            profiles_[i] = new SCurveProfile(constraints_[i]);
        }
    }

    ~MultiAxisTrajectory() {
        for (auto* p : profiles_) {
            delete p;
        }
    }

    /**
     * @brief Set constraints for an axis
     * @param axis Axis index
     * @param constraints Motion constraints
     */
    void setAxisConstraints(size_t axis, const MotionConstraints& constraints) {
        if (axis < numAxes_) {
            constraints_[axis] = constraints;
            delete profiles_[axis];
            profiles_[axis] = new SCurveProfile(constraints);
        }
    }

    /**
     * @brief Plan synchronized motion
     * @param startPositions Starting positions for all axes
     * @param endPositions Ending positions for all axes
     * @return true if planning successful
     */
    bool plan(const std::vector<float>& startPositions,
              const std::vector<float>& endPositions) {
        if (startPositions.size() != numAxes_ ||
            endPositions.size() != numAxes_) {
            return false;
        }

        startPositions_ = startPositions;
        endPositions_ = endPositions;

        // Plan each axis individually
        for (size_t i = 0; i < numAxes_; i++) {
            profiles_[i]->plan(startPositions[i], endPositions[i]);
        }

        // Synchronize to longest duration
        syncDuration_ = 0;
        for (size_t i = 0; i < numAxes_; i++) {
            if (profiles_[i]->getDuration() > syncDuration_) {
                syncDuration_ = profiles_[i]->getDuration();
            }
        }

        // Replan with synchronized duration (scale velocity limits)
        for (size_t i = 0; i < numAxes_; i++) {
            if (profiles_[i]->getDuration() < syncDuration_) {
                // Calculate required scaling
                float ratio = profiles_[i]->getDuration() / syncDuration_;

                // Create scaled constraints
                MotionConstraints scaled = constraints_[i];
                scaled.maxVelocity *= ratio;
                scaled.maxAcceleration *= ratio * ratio;
                scaled.maxJerk *= ratio * ratio * ratio;

                delete profiles_[i];
                profiles_[i] = new SCurveProfile(scaled);
                profiles_[i]->plan(startPositions[i], endPositions[i]);
            }
        }

        valid_ = true;
        return true;
    }

    /**
     * @brief Evaluate all axes at given time
     * @param time Time since start
     * @param points Output trajectory points for all axes
     */
    void evaluate(float time, std::vector<TrajectoryPoint>& points) const {
        points.resize(numAxes_);

        for (size_t i = 0; i < numAxes_; i++) {
            if (valid_ && profiles_[i]->isValid()) {
                points[i] = profiles_[i]->evaluate(time);
            } else {
                points[i].position = startPositions_.empty() ? 0 : startPositions_[i];
            }
        }
    }

    /**
     * @brief Get synchronized duration
     */
    float getDuration() const { return syncDuration_; }

    /**
     * @brief Check if trajectory is valid
     */
    bool isValid() const { return valid_; }

    /**
     * @brief Get number of axes
     */
    size_t getNumAxes() const { return numAxes_; }

    /**
     * @brief Get profile for specific axis
     * @param axis Axis index
     * @return Trajectory generator for axis
     */
    const ITrajectoryGenerator* getAxisProfile(size_t axis) const {
        return (axis < numAxes_) ? profiles_[axis] : nullptr;
    }

private:
    size_t numAxes_;
    std::vector<SCurveProfile*> profiles_;
    std::vector<MotionConstraints> constraints_;
    std::vector<float> startPositions_;
    std::vector<float> endPositions_;
    float syncDuration_;
    bool valid_;
};

/**
 * @brief Path interpolator for multi-dimensional paths
 *
 * Provides linear and arc interpolation for multi-axis motion.
 */
class PathInterpolator {
public:
    /**
     * @brief Path segment types
     */
    enum class SegmentType {
        Line,
        Arc
    };

    /**
     * @brief Path segment
     */
    struct Segment {
        SegmentType type;
        std::vector<float> start;
        std::vector<float> end;
        std::vector<float> center;  // For arcs
        float length;
        bool clockwise;  // For arcs
    };

    PathInterpolator() : totalLength_(0) {}

    /**
     * @brief Add a line segment
     * @param startPoint Starting point
     * @param endPoint Ending point
     */
    void addLine(const std::vector<float>& startPoint,
                 const std::vector<float>& endPoint) {
        Segment seg;
        seg.type = SegmentType::Line;
        seg.start = startPoint;
        seg.end = endPoint;
        seg.length = calculateDistance(startPoint, endPoint);

        segments_.push_back(seg);
        totalLength_ += seg.length;
    }

    /**
     * @brief Add 2D arc segment
     * @param startX Start X
     * @param startY Start Y
     * @param endX End X
     * @param endY End Y
     * @param centerX Arc center X
     * @param centerY Arc center Y
     * @param clockwise Clockwise direction
     */
    void addArc2D(float startX, float startY,
                  float endX, float endY,
                  float centerX, float centerY,
                  bool clockwise) {
        Segment seg;
        seg.type = SegmentType::Arc;
        seg.start = {startX, startY};
        seg.end = {endX, endY};
        seg.center = {centerX, centerY};
        seg.clockwise = clockwise;

        // Calculate arc length
        float r = std::sqrt((startX - centerX) * (startX - centerX) +
                           (startY - centerY) * (startY - centerY));
        float startAngle = std::atan2(startY - centerY, startX - centerX);
        float endAngle = std::atan2(endY - centerY, endX - centerX);

        float angle = endAngle - startAngle;
        if (clockwise && angle > 0) angle -= 2 * 3.14159265f;
        if (!clockwise && angle < 0) angle += 2 * 3.14159265f;

        seg.length = std::abs(angle * r);
        segments_.push_back(seg);
        totalLength_ += seg.length;
    }

    /**
     * @brief Get total path length
     */
    float getPathLength() const { return totalLength_; }

    /**
     * @brief Evaluate position by path length
     * @param s Distance along path
     * @return Position vector
     */
    std::vector<float> evaluateByLength(float s) const {
        if (segments_.empty()) {
            return {};
        }

        s = std::clamp(s, 0.0f, totalLength_);

        float cumLength = 0;
        for (const auto& seg : segments_) {
            if (s <= cumLength + seg.length) {
                float u = (s - cumLength) / seg.length;
                return interpolateSegment(seg, u);
            }
            cumLength += seg.length;
        }

        return segments_.back().end;
    }

    /**
     * @brief Get number of segments
     */
    size_t getSegmentCount() const { return segments_.size(); }

    /**
     * @brief Clear all segments
     */
    void clear() {
        segments_.clear();
        totalLength_ = 0;
    }

private:
    std::vector<Segment> segments_;
    float totalLength_;

    static float calculateDistance(const std::vector<float>& a,
                                   const std::vector<float>& b) {
        float sum = 0;
        size_t n = std::min(a.size(), b.size());
        for (size_t i = 0; i < n; i++) {
            float d = b[i] - a[i];
            sum += d * d;
        }
        return std::sqrt(sum);
    }

    std::vector<float> interpolateSegment(const Segment& seg, float u) const {
        if (seg.type == SegmentType::Line) {
            std::vector<float> result(seg.start.size());
            for (size_t i = 0; i < result.size(); i++) {
                result[i] = seg.start[i] + u * (seg.end[i] - seg.start[i]);
            }
            return result;
        } else {
            // Arc interpolation (2D)
            float cx = seg.center[0];
            float cy = seg.center[1];
            float r = std::sqrt((seg.start[0] - cx) * (seg.start[0] - cx) +
                               (seg.start[1] - cy) * (seg.start[1] - cy));

            float startAngle = std::atan2(seg.start[1] - cy, seg.start[0] - cx);
            float endAngle = std::atan2(seg.end[1] - cy, seg.end[0] - cx);

            float angle = endAngle - startAngle;
            if (seg.clockwise && angle > 0) angle -= 2 * 3.14159265f;
            if (!seg.clockwise && angle < 0) angle += 2 * 3.14159265f;

            float currentAngle = startAngle + u * angle;

            return {cx + r * std::cos(currentAngle),
                    cy + r * std::sin(currentAngle)};
        }
    }
};

}  // namespace omni::motion
