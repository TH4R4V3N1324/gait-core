#ifndef GAIT_CORE__ROBOT_GEOMETRY_HPP
#define GAIT_CORE__ROBOT_GEOMETRY_HPP

#include <array>

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Where a leg's coxa joint sits on the body
*/
struct LegConfig {
    Scalar yaw;                // rad, rotation of the leg frame about the body z axis
    Vec3 translation_offset;   // m, position of the coxa joint in the body frame
};

/**
 * @brief Everything about the robot's shape that the gait maths needs
 * @note SI units, REP 103 body frame (+x forward, +y left, +z up). Legs are numbered 1-6:
 *       1-3 down the right side front to back, 4-6 up the left side back to front.
 *       legs[0] is unused so the arrays can be indexed by leg number.
*/
struct RobotGeometry {
    // Link lengths (m). The tibia length is the straight line from the knee axis to the foot.
    Scalar coxaLength;
    Scalar femurLength;
    Scalar tibiaLength;

    std::array<LegConfig, MAX_LEGS + 1> legs;

    // Neutral stance, in each leg's frame (m)
    Scalar standReach;    // how far out from the coxa joint the foot stands
    Scalar standHeight;   // foot height relative to the hips (negative = below)

    Scalar maxStride;     // m, furthest a foot moves from its neutral position

    /**
     * @brief Explorer (internal design V6.1), matching the URDF in hexapod-ros2
    */
    static RobotGeometry explorer();
};

}  // namespace gait_core

#endif  // GAIT_CORE__ROBOT_GEOMETRY_HPP
