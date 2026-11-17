#ifndef GAIT_CORE__HEXAPOD_LEG_3DOF_HPP
#define GAIT_CORE__HEXAPOD_LEG_3DOF_HPP

#include "gait_core/kinematic_solver.hpp"
#include "gait_core/leg_kinematics.hpp"
#include "gait_core/robot_geometry.hpp"

namespace gait_core {

/**
 * @brief The hexapod's coxa-femur-tibia leg as a LegKinematics
 * @note An adapter over KinematicSolver: each method converts JointVec (coxa, femur, tibia) to and
 *       from JointAngles and calls the existing code, so the maths is unchanged. Not mirrored, as
 *       the engine never asks for a mirrored solve.
*/
class HexapodLeg3Dof : public LegKinematics {
private:
    KinematicSolver solver;
public:
    explicit HexapodLeg3Dof(const RobotGeometry& geometry = RobotGeometry::explorer())
        : solver(geometry) {}

    static constexpr int kNumJoints = 3;  // coxa, femur, tibia

    int numJoints() const override { return kNumJoints; }
    Vec3 fk(const JointVec& jointAngles) const override;
    bool ik(const Vec3& target, const JointVec& seed, JointVec& jointAngles) const override;
    LegJacobian jacobian(const JointVec& jointAngles) const override;
};

}  // namespace gait_core

#endif  // GAIT_CORE__HEXAPOD_LEG_3DOF_HPP
