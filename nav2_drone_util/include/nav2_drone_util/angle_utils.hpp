#pragma once

#include <cmath> // For standard math functions like fmod, floor
#include <algorithm> // For std::min and std::max

namespace nav2_drone_util {

constexpr double PI = 3.141592653589793238463;
constexpr double TWO_PI = 2.0 * PI;

/**
 * @brief Convert degrees to radians.
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
inline double from_degrees(double degrees) {
    return degrees * PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
inline double to_degrees(double radians) {
    return radians * 180.0 / PI;
}

/**
 * @brief Normalize an angle to the range [0, 2*PI).
 * @param angle Angle in radians.
 * @return Normalized angle in radians.
 */
inline double normalize_angle_positive(double angle) {
    const double result = fmod(angle, TWO_PI);
    return (result < 0.0) ? result + TWO_PI : result;
}

/**
 * @brief Normalize an angle to the range [-PI, PI).
 * @param angle Angle in radians.
 * @return Normalized angle in radians.
 */
inline double normalize_angle(double angle) {
    const double result = fmod(angle + PI, TWO_PI);
    return (result < 0.0) ? result + PI : result - PI;
}

/**
 * @brief Calculate the shortest angular distance between two angles.
 * The result is always in the range [-PI, PI).
 * @param from Starting angle in radians.
 * @param to Target angle in radians.
 * @return Shortest angular distance in radians.
 */
inline double shortest_angular_distance(double from, double to) {
    return normalize_angle(to - from);
}

/**
 * @brief Calculate the absolute shortest angular distance between two angles.
 * @param x First angle in radians.
 * @param y Second angle in radians.
 * @param c Maximum circular range (e.g., 2*PI for radians or 360 for degrees).
 * @return Shortest angular distance (signed).
 */
inline double getDiff2Angles(const double x, const double y, const double c) {
    double d = fabs(fmod(fabs(x - y), 2 * c));
    double r = (d > c) ? 2 * c - d : d;

    double sign = ((x - y >= 0.0) && (x - y <= c)) || ((x - y <= c) && (x - y > -2 * c)) ? 1.0 : -1.0;
    return sign * r;
}

/**
 * @brief Calculate the angle (in radians) between a line defined by two points and the x-axis.
 * @param x1 X-coordinate of the first point.
 * @param y1 Y-coordinate of the first point.
 * @param x2 X-coordinate of the second point.
 * @param y2 Y-coordinate of the second point.
 * @return Angle in radians.
 */
inline double angle(const double x1, const double y1, const double x2, const double y2) {
    return atan2(y2 - y1, x2 - x1); // atan2 handles quadrant-based calculation.
}

/**
 * @brief Convert radians to degrees.
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
inline double rad_to_deg(const double rad) {
    return (rad * 180.0) / PI;
}

/**
 * @brief Convert degrees to radians.
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
inline double deg_to_rad(const double deg) {
    return (deg * PI) / 180.0;
}

} // namespace nav_drone_util
