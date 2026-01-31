/**
 * @file math_utils.hpp
 * @brief Mathematical utilities for motor control
 */

#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace omni::utils {

/**
 * @brief Mathematical constants
 */
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float SQRT3 = 1.7320508075688772f;
constexpr float SQRT3_DIV_2 = SQRT3 / 2.0f;
constexpr float ONE_DIV_SQRT3 = 1.0f / SQRT3;

/**
 * @brief Normalize angle to [0, 2*PI)
 * @param angle Angle in radians
 * @return Normalized angle
 */
inline float normalizeAngle(float angle) {
    while (angle < 0) angle += TWO_PI;
    while (angle >= TWO_PI) angle -= TWO_PI;
    return angle;
}

/**
 * @brief Normalize angle to [-PI, PI)
 * @param angle Angle in radians
 * @return Normalized angle
 */
inline float normalizeAngleSigned(float angle) {
    while (angle < -PI) angle += TWO_PI;
    while (angle >= PI) angle -= TWO_PI;
    return angle;
}

/**
 * @brief Wrap angle difference to [-PI, PI]
 * @param a First angle
 * @param b Second angle
 * @return Shortest angular distance from a to b
 */
inline float angleDiff(float a, float b) {
    float diff = b - a;
    return normalizeAngleSigned(diff);
}

/**
 * @brief Convert degrees to radians
 */
inline float degToRad(float deg) {
    return deg * DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 */
inline float radToDeg(float rad) {
    return rad * RAD_TO_DEG;
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor [0, 1]
 * @return Interpolated value
 */
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * @brief Inverse linear interpolation
 * @param a Start value
 * @param b End value
 * @param v Value between a and b
 * @return Interpolation factor
 */
inline float invLerp(float a, float b, float v) {
    if (std::abs(b - a) < 1e-10f) return 0;
    return (v - a) / (b - a);
}

/**
 * @brief Remap value from one range to another
 * @param v Input value
 * @param inMin Input range minimum
 * @param inMax Input range maximum
 * @param outMin Output range minimum
 * @param outMax Output range maximum
 * @return Remapped value
 */
inline float remap(float v, float inMin, float inMax, float outMin, float outMax) {
    return lerp(outMin, outMax, invLerp(inMin, inMax, v));
}

/**
 * @brief Clamp value to range
 */
template<typename T>
inline T clamp(T value, T minVal, T maxVal) {
    return std::max(minVal, std::min(maxVal, value));
}

/**
 * @brief Sign function
 * @param value Input value
 * @return -1, 0, or 1
 */
inline float sign(float value) {
    if (value > 0) return 1.0f;
    if (value < 0) return -1.0f;
    return 0.0f;
}

/**
 * @brief Dead band function
 * @param value Input value
 * @param threshold Dead band threshold
 * @return 0 if within deadband, otherwise value with deadband subtracted
 */
inline float deadband(float value, float threshold) {
    if (std::abs(value) < threshold) return 0;
    return value - sign(value) * threshold;
}

/**
 * @brief Saturation function
 * @param value Input value
 * @param limit Saturation limit (symmetric)
 * @return Saturated value
 */
inline float saturate(float value, float limit) {
    return clamp(value, -limit, limit);
}

/**
 * @brief Smooth step function (Hermite interpolation)
 * @param edge0 Lower edge
 * @param edge1 Upper edge
 * @param x Input value
 * @return Smoothed value [0, 1]
 */
inline float smoothstep(float edge0, float edge1, float x) {
    x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

/**
 * @brief Fast inverse square root (Quake III algorithm)
 * @param x Input value
 * @return 1/sqrt(x) approximately
 */
inline float fastInvSqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

/**
 * @brief Fast sine approximation (valid for -PI to PI)
 * @param x Angle in radians
 * @return sin(x) approximately
 */
inline float fastSin(float x) {
    // Normalize to [-PI, PI]
    x = normalizeAngleSigned(x);

    // Parabola approximation
    const float B = 4.0f / PI;
    const float C = -4.0f / (PI * PI);
    float y = B * x + C * x * std::abs(x);

    // Refine with one iteration
    const float P = 0.225f;
    y = P * (y * std::abs(y) - y) + y;
    return y;
}

/**
 * @brief Fast cosine approximation
 * @param x Angle in radians
 * @return cos(x) approximately
 */
inline float fastCos(float x) {
    return fastSin(x + HALF_PI);
}

/**
 * @brief Calculate magnitude of 2D vector
 */
inline float magnitude2D(float x, float y) {
    return std::sqrt(x * x + y * y);
}

/**
 * @brief Calculate magnitude of 3D vector
 */
inline float magnitude3D(float x, float y, float z) {
    return std::sqrt(x * x + y * y + z * z);
}

/**
 * @brief Limit rate of change
 * @param current Current value
 * @param target Target value
 * @param maxRate Maximum rate of change per call
 * @return Rate-limited value
 */
inline float rateLimit(float current, float target, float maxRate) {
    float delta = target - current;
    if (std::abs(delta) <= maxRate) {
        return target;
    }
    return current + sign(delta) * maxRate;
}

/**
 * @brief Check if value is approximately zero
 */
inline bool isNearZero(float value, float epsilon = 1e-6f) {
    return std::abs(value) < epsilon;
}

/**
 * @brief Check if two values are approximately equal
 */
inline bool isNearEqual(float a, float b, float epsilon = 1e-6f) {
    return std::abs(a - b) < epsilon;
}

}  // namespace omni::utils
