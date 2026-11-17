#ifndef GAIT_CORE__JOINT_DEFINITION_HPP
#define GAIT_CORE__JOINT_DEFINITION_HPP

#include <limits>

#include "gait_core/types.hpp"

namespace gait_core {

struct JointDefinition {
    Vec3 origin;      // m, from the previous joint (or the leg frame origin) at zero angles
    Vec3 axis;        // unit axis, in the leg frame at zero angles
    Scalar lower, upper;  // rad
    // rad/s, fastest this joint's actuator may turn; the gait slows so no joint exceeds its own.
    // Infinity (the default) means GaitParams::max_joint_speed applies instead.
    Scalar max_velocity = std::numeric_limits<Scalar>::infinity();

    /**
     * @brief The speed limit the gait holds this joint to
     * @param[in] fallback The limit for a joint that doesn't set max_velocity (rad/s)
     * @return max_velocity if set, otherwise fallback (rad/s)
    */
    Scalar velocityLimit(Scalar fallback) const {
        return (max_velocity > 0 && max_velocity < std::numeric_limits<Scalar>::infinity()) ? max_velocity : fallback;
    }
};

}  // namespace gait_core

#endif  // GAIT_CORE__JOINT_DEFINITION_HPP
