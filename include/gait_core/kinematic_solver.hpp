#ifndef GAIT_CORE__KINEMATIC_SOLVER_HPP
#define GAIT_CORE__KINEMATIC_SOLVER_HPP

#include "gait_core/robot_definition.hpp"
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
    explicit KinematicSolver(const RobotDefinition& geometry = RobotDefinition::explorer())
        : coxaLength(geometry.coxaLength),
          femurLength(geometry.femurLength),
          tibiaLength(geometry.tibiaLength) {}

    JointAngles solveIK(const Vec3& target, bool mirrored = false, bool* clamped = nullptr) const;
    Vec3 solveFK(const JointAngles& angles) const;
    Mat3 jacobian(const JointAngles& angles) const;
    bool groundForce(const JointAngles& angles, const Vec3& torques, Vec3& force) const;
};

}  // namespace gait_core

#endif  // GAIT_CORE__KINEMATIC_SOLVER_HPP
