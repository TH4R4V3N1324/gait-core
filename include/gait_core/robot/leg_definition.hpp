#ifndef GAIT_CORE__LEG_DEFINITION_HPP
#define GAIT_CORE__LEG_DEFINITION_HPP

#include <array>

#include "gait_core/types.hpp"
#include "gait_core/robot/joint_definition.hpp"

namespace gait_core {

/**
 * @brief Where a leg's coxa joint sits on the body
*/
struct LegConfig {
    Scalar yaw;                // rad, rotation of the leg frame about the body z axis
    Vec3 translation_offset;   // m, position of the coxa joint in the body frame
};

enum class LegSolver : uint8_t {
    Chain,          // a generic chain of revolute joints (ChainKinematics): works for any leg
    YawPitchPitch,  // the analytic solver (YawPitchPitchLeg) for a yaw joint then two pitch joints, e.g. coxa/femur/tibia
};

struct LegDefinition {
    LegConfig mount;                                        // as today: yaw + coxa position
    int numJoints = 0;
    std::array<JointDefinition, kMaxJointsPerLeg> joints;   // body to foot
    Vec3 foot;                                              // m, last joint to foot
    LegSolver solver = LegSolver::Chain;                    // which kinematics to use for this leg
    // m, leg frame: where this leg's foot stands relative to the robot's shared stance
    // (standReach, 0, standHeight). Kept through height and reach changes, sitting included.
    // Zero for legs that all stand alike, e.g. a radially symmetric hexapod.
    Vec3 stanceOffset = Vec3::Zero();
};

} // namespace gait_core

#endif  // GAIT_CORE__LEG_DEFINITION_HPP