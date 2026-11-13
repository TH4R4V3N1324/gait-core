#ifndef GAIT_CORE__ROBOT_GEOMETRY_HPP
#define GAIT_CORE__ROBOT_GEOMETRY_HPP

#include <array>
#include <cstdint>

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief The legs, clockwise from the front right seen from above, used to index every per-leg array
 * @note Hardware and URDF names count from 1 (leg1 is the front right), so leg i here is leg{i + 1} there.
*/
enum Leg : uint8_t {
    LEG_FRONT_RIGHT,
    LEG_MIDDLE_RIGHT,
    LEG_REAR_RIGHT,
    LEG_REAR_LEFT,
    LEG_MIDDLE_LEFT,
    LEG_FRONT_LEFT,
    NUM_LEGS
};

/**
 * @brief Where a leg's coxa joint sits on the body
*/
struct LegConfig {
    Scalar yaw;                // rad, rotation of the leg frame about the body z axis
    Vec3 translation_offset;   // m, position of the coxa joint in the body frame
};

/**
 * @brief Range each joint can safely move through (rad)
*/
struct JointLimits {
    Scalar coxaMax;    // either side of straight out
    Scalar femurMin;   // raised
    Scalar femurMax;   // lowered
    Scalar tibiaMin;   // straightened past in line with the femur
    Scalar tibiaMax;   // folded
};

/**
 * @brief Everything about the robot's shape that the gait maths needs
 * @note SI units, REP 103 body frame (+x forward, +y left, +z up). legs is indexed by Leg,
 *       clockwise from the front right: down the right side front to back, then up the left side
 *       back to front.
*/
struct RobotGeometry {
    // Link lengths (m). The tibia length is the straight line from the knee axis to the foot.
    Scalar coxaLength;
    Scalar femurLength;
    Scalar tibiaLength;

    std::array<LegConfig, NUM_LEGS> legs;

    // Neutral stance, in each leg's frame (m)
    Scalar standReach;    // how far out from the coxa joint the foot stands
    Scalar standHeight;   // foot height relative to the hips (negative = below)

    Scalar maxStride;     // m, furthest a foot moves from its neutral position

    JointLimits limits;

    /**
     * @brief Explorer (internal design V6.1), matching the URDF in hexapod-ros2
    */
    static RobotGeometry explorer();
};

}  // namespace gait_core

#endif  // GAIT_CORE__ROBOT_GEOMETRY_HPP
