/**
 * @file spline_trajectory.hpp
 * @brief Spline-based trajectory through multiple waypoints
 */

#pragma once

#include "trajectory.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace omni::motion {

/**
 * @brief Spline type
 */
enum class SplineType {
    CubicSpline,    // Natural cubic spline
    CatmullRom,     // Catmull-Rom spline
    BSpline,        // B-spline (not interpolating)
    Linear          // Linear interpolation
};

/**
 * @brief Spline trajectory through waypoints
 *
 * Generates smooth trajectories passing through multiple points.
 */
class SplineTrajectory : public ITrajectoryGenerator {
public:
    explicit SplineTrajectory(SplineType type = SplineType::CubicSpline)
        : type_(type)
        , totalTime_(0)
        , velocityLimit_(1.0f)
        , valid_(false) {}

    /**
     * @brief Add a waypoint
     * @param position Position at waypoint
     * @param time Time at waypoint (-1 for auto)
     */
    void addWaypoint(float position, float time = -1) {
        Waypoint wp;
        wp.position = position;
        wp.time = time;
        wp.velocity = 0;
        wp.autoTime = (time < 0);
        waypoints_.push_back(wp);
        valid_ = false;
    }

    /**
     * @brief Add waypoint with full state
     * @param point Trajectory point
     */
    void addWaypoint(const TrajectoryPoint& point) {
        Waypoint wp;
        wp.position = point.position;
        wp.time = point.time;
        wp.velocity = point.velocity;
        wp.autoTime = false;
        waypoints_.push_back(wp);
        valid_ = false;
    }

    /**
     * @brief Clear all waypoints
     */
    void clear() {
        waypoints_.clear();
        coeffs_.clear();
        valid_ = false;
    }

    /**
     * @brief Set velocity limit for auto time calculation
     * @param maxVel Maximum velocity
     */
    void setVelocityLimit(float maxVel) {
        velocityLimit_ = maxVel;
        valid_ = false;
    }

    /**
     * @brief Generate the spline trajectory
     * @return true if generation successful
     */
    bool generate() {
        if (waypoints_.size() < 2) {
            valid_ = false;
            return false;
        }

        // Auto-calculate times if needed
        calculateTimes();

        // Generate spline coefficients based on type
        switch (type_) {
            case SplineType::CubicSpline:
                generateCubicSpline();
                break;
            case SplineType::CatmullRom:
                generateCatmullRom();
                break;
            case SplineType::Linear:
                generateLinear();
                break;
            default:
                generateCubicSpline();
        }

        totalTime_ = waypoints_.back().time;
        valid_ = true;
        return true;
    }

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override {
        clear();
        TrajectoryPoint start(0, startPos, startVel);
        TrajectoryPoint end(1.0f, endPos, endVel);
        addWaypoint(start);
        addWaypoint(end);
        return generate();
    }

    TrajectoryPoint evaluate(float time) const override {
        TrajectoryPoint point;
        point.time = time;

        if (!valid_ || waypoints_.empty()) {
            return point;
        }

        if (time <= 0) {
            point.position = waypoints_.front().position;
            return point;
        }

        if (time >= totalTime_) {
            point.position = waypoints_.back().position;
            return point;
        }

        // Find segment
        size_t seg = 0;
        for (size_t i = 1; i < waypoints_.size(); i++) {
            if (time < waypoints_[i].time) {
                seg = i - 1;
                break;
            }
        }

        // Evaluate spline in segment
        float t0 = waypoints_[seg].time;
        float t1 = waypoints_[seg + 1].time;
        float u = (time - t0) / (t1 - t0);  // Normalized time [0, 1]

        if (type_ == SplineType::Linear) {
            point.position = waypoints_[seg].position * (1 - u) +
                            waypoints_[seg + 1].position * u;
            point.velocity = (waypoints_[seg + 1].position -
                             waypoints_[seg].position) / (t1 - t0);
        } else {
            // Cubic evaluation
            const auto& c = coeffs_[seg];
            float u2 = u * u;
            float u3 = u2 * u;

            point.position = c.a + c.b * u + c.c * u2 + c.d * u3;

            float dt = t1 - t0;
            point.velocity = (c.b + 2 * c.c * u + 3 * c.d * u2) / dt;
            point.acceleration = (2 * c.c + 6 * c.d * u) / (dt * dt);
        }

        return point;
    }

    float getDuration() const override { return totalTime_; }
    bool isValid() const override { return valid_; }

    float getStartPosition() const override {
        return waypoints_.empty() ? 0 : waypoints_.front().position;
    }

    float getEndPosition() const override {
        return waypoints_.empty() ? 0 : waypoints_.back().position;
    }

    /**
     * @brief Get number of waypoints
     */
    size_t getWaypointCount() const { return waypoints_.size(); }

    /**
     * @brief Get waypoint
     * @param index Waypoint index
     * @return Waypoint position
     */
    float getWaypointPosition(size_t index) const {
        return (index < waypoints_.size()) ? waypoints_[index].position : 0;
    }

    /**
     * @brief Set spline type
     */
    void setType(SplineType type) {
        type_ = type;
        valid_ = false;
    }

private:
    struct Waypoint {
        float position;
        float time;
        float velocity;
        bool autoTime;
    };

    struct CubicCoeffs {
        float a, b, c, d;  // a + b*u + c*u^2 + d*u^3
    };

    SplineType type_;
    std::vector<Waypoint> waypoints_;
    std::vector<CubicCoeffs> coeffs_;
    float totalTime_;
    float velocityLimit_;
    bool valid_;

    void calculateTimes() {
        waypoints_[0].time = 0;

        for (size_t i = 1; i < waypoints_.size(); i++) {
            if (waypoints_[i].autoTime) {
                float dist = std::abs(waypoints_[i].position -
                                     waypoints_[i - 1].position);
                float dt = dist / velocityLimit_;
                if (dt < 0.001f) dt = 0.001f;  // Minimum segment time
                waypoints_[i].time = waypoints_[i - 1].time + dt;
            }
        }
    }

    void generateLinear() {
        coeffs_.clear();
        // Linear doesn't need coefficients, evaluated directly
    }

    void generateCubicSpline() {
        size_t n = waypoints_.size();
        if (n < 2) return;

        coeffs_.resize(n - 1);

        // Natural cubic spline with zero second derivatives at ends
        std::vector<float> h(n - 1);
        for (size_t i = 0; i < n - 1; i++) {
            h[i] = waypoints_[i + 1].time - waypoints_[i].time;
        }

        // Solve tridiagonal system for second derivatives
        std::vector<float> alpha(n - 1);
        for (size_t i = 1; i < n - 1; i++) {
            alpha[i] = 3.0f / h[i] * (waypoints_[i + 1].position - waypoints_[i].position) -
                       3.0f / h[i - 1] * (waypoints_[i].position - waypoints_[i - 1].position);
        }

        std::vector<float> l(n), mu(n), z(n);
        l[0] = 1;
        mu[0] = z[0] = 0;

        for (size_t i = 1; i < n - 1; i++) {
            l[i] = 2 * (waypoints_[i + 1].time - waypoints_[i - 1].time) -
                   h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1;
        z[n - 1] = 0;

        std::vector<float> c(n), b(n - 1), d(n - 1);
        c[n - 1] = 0;

        for (int i = static_cast<int>(n) - 2; i >= 0; i--) {
            c[i] = z[i] - mu[i] * c[i + 1];
            b[i] = (waypoints_[i + 1].position - waypoints_[i].position) / h[i] -
                   h[i] * (c[i + 1] + 2 * c[i]) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);
        }

        // Store coefficients (transformed to [0,1] parameter)
        for (size_t i = 0; i < n - 1; i++) {
            float hi = h[i];
            coeffs_[i].a = waypoints_[i].position;
            coeffs_[i].b = b[i] * hi;
            coeffs_[i].c = c[i] * hi * hi;
            coeffs_[i].d = d[i] * hi * hi * hi;
        }
    }

    void generateCatmullRom() {
        size_t n = waypoints_.size();
        if (n < 2) return;

        coeffs_.resize(n - 1);

        for (size_t i = 0; i < n - 1; i++) {
            float p0 = (i > 0) ? waypoints_[i - 1].position : waypoints_[i].position;
            float p1 = waypoints_[i].position;
            float p2 = waypoints_[i + 1].position;
            float p3 = (i < n - 2) ? waypoints_[i + 2].position : waypoints_[i + 1].position;

            // Catmull-Rom to cubic coefficients
            coeffs_[i].a = p1;
            coeffs_[i].b = 0.5f * (-p0 + p2);
            coeffs_[i].c = 0.5f * (2 * p0 - 5 * p1 + 4 * p2 - p3);
            coeffs_[i].d = 0.5f * (-p0 + 3 * p1 - 3 * p2 + p3);
        }
    }
};

}  // namespace omni::motion
