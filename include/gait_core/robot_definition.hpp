#ifndef GAIT_CORE__ROBOT_DEFINITION_HPP
#define GAIT_CORE__ROBOT_DEFINITION_HPP

#include <array>
#include <cstdint>

#include "gait_core/gait_pattern.hpp"
#include "gait_core/leg_definition.hpp"
#include "gait_core/types.hpp"

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
 * @brief Explorer's gaits, by their index in RobotDefinition::explorer().gaits
 * @note Other robots number their own gaits; the engine takes any index into the robot's list.
*/
enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

/**
 * @brief Everything about the robot that the gait needs: its legs, their links and limits, and stance
 * @note SI units, REP 103 body frame (+x forward, +y left, +z up). legs[0..numLegs - 1] are used;
 *       the gaits expect them clockwise from the front right seen from above, as in explorer().
*/
struct RobotDefinition {
    int numLegs = 0;                         // 1..kMaxLegs
    std::array<LegDefinition, kMaxLegs> legs;    // indexed by leg number

    // Neutral stance, in each leg's frame (m)
    Scalar standReach;    // how far out from the coxa joint the foot stands
    Scalar standHeight;   // foot height relative to the hips (negative = below)

    Scalar maxStride;     // m, furthest a foot moves from its neutral position

    // The gaits it can walk in, selected by index (the protocol's and ROS's gait id). Each must
    // time exactly numLegs legs; the engine won't move a robot with a gait it can't run.
    int numGaits = 0;                          // 1..kMaxGaits
    std::array<GaitPattern, kMaxGaits> gaits;

    /**
     * @brief Explorer (internal design V6.1), matching the URDF in hexapod-ros2
    */
    static RobotDefinition explorer();
};

}  // namespace gait_core

#endif  // GAIT_CORE__ROBOT_DEFINITION_HPP
