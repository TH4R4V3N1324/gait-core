#ifndef GAIT_CORE__TYPES_HPP
#define GAIT_CORE__TYPES_HPP

#include <Eigen/Dense>

/**
 * Scalar type used throughout the core. Defaults to float, which the ESP32 has hardware for and
 * which is much cheaper than double on the RP2040 (no FPU at all). The ROS build sets
 * GAIT_CORE_USE_DOUBLE so its maths matches what it was before the split.
*/
#ifndef GAIT_CORE_SCALAR
#define GAIT_CORE_SCALAR float
#endif

namespace gait_core {

using Scalar = GAIT_CORE_SCALAR;

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using AngleAxis = Eigen::AngleAxis<Scalar>;
using Rotation2D = Eigen::Rotation2D<Scalar>;

// M_PI is not standard C++ (MSVC and some embedded toolchains hide it)
inline constexpr Scalar kPi = static_cast<Scalar>(3.14159265358979323846);

/**
 * @brief Commanded body velocity, in the body frame
 * @note SI units, REP 103 body frame: +x forward, +y left, +z up, yaw positive anticlockwise.
 *       Replaces geometry_msgs/Twist inside the core; the ROS node copies Twist into it.
*/
struct VelocityCommand {
    Scalar vx = 0;    // m/s, forward
    Scalar vy = 0;    // m/s, left
    Scalar wz = 0;    // rad/s, anticlockwise
};

}  // namespace gait_core

#endif  // GAIT_CORE__TYPES_HPP
