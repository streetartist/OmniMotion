/**
 * @file polynomial_trajectory.hpp
 * @brief Polynomial trajectory generators
 */

#pragma once

#include "trajectory.hpp"
#include <cmath>
#include <array>

namespace omni::motion {

/**
 * @brief Polynomial trajectory with specified boundary conditions
 *
 * Supports cubic (4 coefficients), quintic (6 coefficients),
 * and septic (8 coefficients) polynomials.
 */
class PolynomialTrajectory : public ITrajectoryGenerator {
public:
    PolynomialTrajectory() : duration_(0), valid_(false) {
        coeffs_.fill(0);
    }

    /**
     * @brief Create cubic polynomial trajectory
     *
     * Specifies position and velocity at boundaries.
     *
     * @param duration Total time
     * @param startPos Starting position
     * @param endPos Ending position
     * @param startVel Starting velocity
     * @param endVel Ending velocity
     * @return PolynomialTrajectory
     */
    static PolynomialTrajectory cubic(float duration,
                                       float startPos, float endPos,
                                       float startVel = 0, float endVel = 0) {
        PolynomialTrajectory traj;
        traj.duration_ = duration;
        traj.startPos_ = startPos;
        traj.endPos_ = endPos;
        traj.order_ = 3;

        if (duration <= 0) {
            traj.valid_ = false;
            return traj;
        }

        float T = duration;
        float T2 = T * T;
        float T3 = T2 * T;

        // Solve for coefficients: p(t) = a0 + a1*t + a2*t^2 + a3*t^3
        // p(0) = startPos, p(T) = endPos
        // v(0) = startVel, v(T) = endVel
        traj.coeffs_[0] = startPos;
        traj.coeffs_[1] = startVel;
        traj.coeffs_[2] = (3.0f * (endPos - startPos) - T * (2.0f * startVel + endVel)) / T2;
        traj.coeffs_[3] = (2.0f * (startPos - endPos) + T * (startVel + endVel)) / T3;

        traj.valid_ = true;
        return traj;
    }

    /**
     * @brief Create quintic polynomial trajectory
     *
     * Specifies position, velocity, and acceleration at boundaries.
     *
     * @param duration Total time
     * @param startPos Starting position
     * @param endPos Ending position
     * @param startVel Starting velocity
     * @param endVel Ending velocity
     * @param startAccel Starting acceleration
     * @param endAccel Ending acceleration
     * @return PolynomialTrajectory
     */
    static PolynomialTrajectory quintic(float duration,
                                         float startPos, float endPos,
                                         float startVel = 0, float endVel = 0,
                                         float startAccel = 0, float endAccel = 0) {
        PolynomialTrajectory traj;
        traj.duration_ = duration;
        traj.startPos_ = startPos;
        traj.endPos_ = endPos;
        traj.order_ = 5;

        if (duration <= 0) {
            traj.valid_ = false;
            return traj;
        }

        float T = duration;
        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        // Coefficients for quintic polynomial
        traj.coeffs_[0] = startPos;
        traj.coeffs_[1] = startVel;
        traj.coeffs_[2] = startAccel / 2.0f;

        float dp = endPos - startPos;
        float dv = endVel - startVel;
        float da = endAccel - startAccel;

        traj.coeffs_[3] = (20.0f * dp - (8.0f * endVel + 12.0f * startVel) * T -
                          (3.0f * startAccel - endAccel) * T2) / (2.0f * T3);
        traj.coeffs_[4] = (-30.0f * dp + (14.0f * endVel + 16.0f * startVel) * T +
                          (3.0f * startAccel - 2.0f * endAccel) * T2) / (2.0f * T4);
        traj.coeffs_[5] = (12.0f * dp - 6.0f * (endVel + startVel) * T +
                          (endAccel - startAccel) * T2) / (2.0f * T5);

        traj.valid_ = true;
        return traj;
    }

    /**
     * @brief Create septic (7th order) polynomial trajectory
     *
     * Specifies position, velocity, acceleration, and jerk at boundaries.
     */
    static PolynomialTrajectory septic(float duration,
                                        float startPos, float endPos,
                                        float startVel = 0, float endVel = 0,
                                        float startAccel = 0, float endAccel = 0,
                                        float startJerk = 0, float endJerk = 0) {
        PolynomialTrajectory traj;
        traj.duration_ = duration;
        traj.startPos_ = startPos;
        traj.endPos_ = endPos;
        traj.order_ = 7;

        if (duration <= 0) {
            traj.valid_ = false;
            return traj;
        }

        float T = duration;

        // First 4 coefficients from initial conditions
        traj.coeffs_[0] = startPos;
        traj.coeffs_[1] = startVel;
        traj.coeffs_[2] = startAccel / 2.0f;
        traj.coeffs_[3] = startJerk / 6.0f;

        // Solve for remaining 4 coefficients using end conditions
        // This requires solving a 4x4 linear system
        // Simplified: using standard septic polynomial formulas

        float h = endPos - startPos - startVel * T - startAccel * T * T / 2.0f -
                 startJerk * T * T * T / 6.0f;
        float hd = endVel - startVel - startAccel * T - startJerk * T * T / 2.0f;
        float hdd = endAccel - startAccel - startJerk * T;
        float hddd = endJerk - startJerk;

        float T4 = T * T * T * T;
        float T5 = T4 * T;
        float T6 = T5 * T;
        float T7 = T6 * T;

        // Solve using closed-form solution
        traj.coeffs_[4] = (35.0f * h - 20.0f * hd * T + 5.0f * hdd * T * T -
                          hddd * T * T * T / 3.0f) / T4;
        traj.coeffs_[5] = (-84.0f * h + 45.0f * hd * T - 10.0f * hdd * T * T +
                          hddd * T * T * T) / T5;
        traj.coeffs_[6] = (70.0f * h - 36.0f * hd * T + 7.5f * hdd * T * T -
                          hddd * T * T * T) / T6;
        traj.coeffs_[7] = (-20.0f * h + 10.0f * hd * T - 2.0f * hdd * T * T +
                          hddd * T * T * T / 3.0f) / T7;

        traj.valid_ = true;
        return traj;
    }

    bool plan(float startPos, float endPos,
              float startVel = 0, float endVel = 0) override {
        // Default to quintic with zero acceleration boundaries
        *this = quintic(1.0f, startPos, endPos, startVel, endVel, 0, 0);
        return valid_;
    }

    /**
     * @brief Plan with specified duration
     */
    bool plan(float duration, float startPos, float endPos,
              float startVel = 0, float endVel = 0) {
        *this = quintic(duration, startPos, endPos, startVel, endVel, 0, 0);
        return valid_;
    }

    TrajectoryPoint evaluate(float time) const override {
        TrajectoryPoint point;
        point.time = time;

        if (!valid_) {
            point.position = startPos_;
            return point;
        }

        if (time <= 0) {
            point.position = startPos_;
            point.velocity = coeffs_[1];
            point.acceleration = 2.0f * coeffs_[2];
            point.jerk = 6.0f * coeffs_[3];
            return point;
        }

        if (time >= duration_) {
            point.position = endPos_;
            // Calculate end velocity/accel from polynomial
            float t = duration_;
            point.velocity = evaluateDerivative(t, 1);
            point.acceleration = evaluateDerivative(t, 2);
            point.jerk = evaluateDerivative(t, 3);
            return point;
        }

        // Evaluate polynomial and derivatives
        point.position = evaluatePolynomial(time);
        point.velocity = evaluateDerivative(time, 1);
        point.acceleration = evaluateDerivative(time, 2);
        point.jerk = evaluateDerivative(time, 3);

        return point;
    }

    float getDuration() const override { return duration_; }
    bool isValid() const override { return valid_; }
    float getStartPosition() const override { return startPos_; }
    float getEndPosition() const override { return endPos_; }

    /**
     * @brief Get polynomial order
     */
    int getOrder() const { return order_; }

    /**
     * @brief Get coefficient
     * @param i Coefficient index
     */
    float getCoefficient(int i) const {
        return (i >= 0 && i < 8) ? coeffs_[i] : 0;
    }

private:
    std::array<float, 8> coeffs_;  // Up to 7th order
    float duration_;
    float startPos_, endPos_;
    int order_ = 0;
    bool valid_;

    float evaluatePolynomial(float t) const {
        float result = 0;
        float tn = 1.0f;
        for (int i = 0; i <= order_; i++) {
            result += coeffs_[i] * tn;
            tn *= t;
        }
        return result;
    }

    float evaluateDerivative(float t, int deriv) const {
        if (deriv >= order_ + 1) return 0;

        float result = 0;
        float tn = 1.0f;

        for (int i = deriv; i <= order_; i++) {
            float coeff = coeffs_[i];
            // Multiply by factorial ratio
            for (int j = 0; j < deriv; j++) {
                coeff *= (i - j);
            }
            result += coeff * tn;
            tn *= t;
        }
        return result;
    }
};

}  // namespace omni::motion
