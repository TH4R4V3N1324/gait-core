#ifndef GAIT_CORE__TEST_COMMON_HPP
#define GAIT_CORE__TEST_COMMON_HPP

#include <algorithm>
#include <type_traits>

#include "gait_core/robot/robot_definition.hpp"
#include "gait_core/types.hpp"
#include "gait_core/robots/explorer.hpp"

/**
 * @brief Explorer, the robot most tests run (the gait core has none built in)
 * @note Only usable from tests linked against gait_core::robots.
*/
inline gait_core::RobotDefinition explorerRobot() {
    return gait_core::explorer::robot();
}

/**
 * @brief Tolerance for a check written for double precision
 * @param[in] doubleTolerance The tolerance the test used in the ROS (double) build
 * @return That tolerance in a double build; in a float build, no tighter than 20 micrometres /
 *         microradians, which is about float's resolution after a chain of trig on ~0.3 m values
*/
constexpr double tol(double doubleTolerance) {
    return std::is_same<gait_core::Scalar, double>::value ? doubleTolerance
                                                          : std::max(doubleTolerance, 2e-5);
}

#endif  // GAIT_CORE__TEST_COMMON_HPP
