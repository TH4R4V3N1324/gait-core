#ifndef GAIT_CORE__ROBOT_DEFINITION_HPP
#define GAIT_CORE__ROBOT_DEFINITION_HPP

#include <array>
#include <cstdint>

#include "gait_core/types.hpp"
#include "gait_core/leg_definition.hpp"

namespace gait_core {

/**
 * @brief Explorer's legs, clockwise from the front right seen from above
 * @note Names for the leg numbers in RobotDefinition::explorer() (and the hexapod gait schedules);
 *       generic code indexes legs by number, 0..numLegs - 1. Hardware and URDF names count from 1
 *       (leg1 is the front right), so leg i here is leg{i + 1} there.
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
 * @brief Everything about the robot that the gait needs: its legs, their links and limits, and stance
 * @note SI units, REP 103 body frame (+x forward, +y left, +z up). legs[0..numLegs - 1] are used;
 *       the gaits expect them clockwise from the front right seen from above, as in explorer().
*/
struct RobotDefinition {
    int numLegs = 0;                         // 1..kMaxLegs
    std::array<LegDefinition, kMaxLegs> legs;    // indexed by leg number

    // Link lengths (m), shared by every leg. The tibia length is the straight line from the knee
    // axis to the foot.
    Scalar coxaLength;
    Scalar femurLength;
    Scalar tibiaLength;

    // Neutral stance, in each leg's frame (m)
    Scalar standReach;    // how far out from the coxa joint the foot stands
    Scalar standHeight;   // foot height relative to the hips (negative = below)

    Scalar maxStride;     // m, furthest a foot moves from its neutral position

    JointLimits limits;   // shared by every leg

    /**
     * @brief Explorer (internal design V6.1), matching the URDF in hexapod-ros2
    */
    static RobotDefinition explorer();
};

using RobotGeometry = RobotDefinition;  // legacy name, kept for backwards compatibility

}  // namespace gait_core

#endif  // GAIT_CORE__ROBOT_DEFINITION_HPP
