#ifndef GAIT_CORE__JOINT_DEFINITION_HPP
#define GAIT_CORE__JOINT_DEFINITION_HPP

#include "gait_core/types.hpp"

namespace gait_core {

struct JointDefinition {
    Vec3 origin;      // m, from the previous joint (or the leg frame origin) at zero angles
    Vec3 axis;        // unit axis, in the leg frame at zero angles
    Scalar lower, upper;  // rad
};

}  // namespace gait_core

#endif  // GAIT_CORE__JOINT_DEFINITION_HPP