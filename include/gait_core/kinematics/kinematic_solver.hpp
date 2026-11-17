#ifndef GAIT_CORE__KINEMATIC_SOLVER_HPP
#define GAIT_CORE__KINEMATIC_SOLVER_HPP

#include "gait_core/robot/leg_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

struct JointAngles {
    Scalar coxa;
    Scalar femur;
    Scalar tibia;
};

/**
 * @brief Single-leg inverse and forward kinematics, in the leg frame
 * @note The ROS IK/FK services (KinematicSolverService) stay in hexapod_gait_controller and
 *       wrap this class.
*/
class KinematicSolver {
private:
    Scalar coxaLength;
    Scalar femurLength;
    Scalar tibiaLength;
public:
    /**
     * @brief Take the link lengths from a coxa/femur/tibia leg's chain
     * @param[in] leg The leg: joints[1].origin is the coxa link, joints[2].origin the femur, foot
     *            the tibia (see GaitEngine for the shape this solver needs)
    */
    explicit KinematicSolver(const LegDefinition& leg)
        : coxaLength(leg.joints[1].origin.norm()),
          femurLength(leg.joints[2].origin.norm()),
          tibiaLength(leg.foot.norm()) {}

    JointAngles solveIK(const Vec3& target, bool mirrored = false, bool* clamped = nullptr) const;
    Vec3 solveFK(const JointAngles& angles) const;
    Mat3 jacobian(const JointAngles& angles) const;
    bool groundForce(const JointAngles& angles, const Vec3& torques, Vec3& force) const;
};

}  // namespace gait_core

#endif  // GAIT_CORE__KINEMATIC_SOLVER_HPP
