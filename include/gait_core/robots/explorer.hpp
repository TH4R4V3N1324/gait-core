#ifndef GAIT_CORE__ROBOTS__EXPLORER_HPP
#define GAIT_CORE__ROBOTS__EXPLORER_HPP

#include <cstdint>

#include "gait_core/robot/robot_definition.hpp"

/**
 * Explorer (internal design V6.1): a hexapod with three-joint legs, matching the URDF in
 * hexapod-ros2. Part of gait_core::robots, not the gait core, which has no built-in robot.
 */
namespace gait_core::explorer {

/**
 * @brief Explorer's legs, clockwise from the front right seen from above
 * @note Names for its leg numbers. Hardware and URDF names count from 1 (leg1 is the front
 *       right), so leg i here is leg{i + 1} there.
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
 * @brief Explorer's gaits, by their index in robot().gaits
 * @note These are the gait ids controllers send (the protocol's Gait names) and ROS lists.
*/
enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

/**
 * @brief Explorer's definition: legs, joint chains and limits, stance and gaits
*/
RobotDefinition robot();

}  // namespace gait_core::explorer

#endif  // GAIT_CORE__ROBOTS__EXPLORER_HPP
