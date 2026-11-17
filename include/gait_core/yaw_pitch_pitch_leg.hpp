#ifndef GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP
#define GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP

#include "gait_core/kinematic_solver.hpp"
#include "gait_core/leg_definition.hpp"
#include "gait_core/leg_kinematics.hpp"

namespace gait_core {

/**
 * @brief Exact, analytic kinematics for a leg of a yaw joint then two pitch joints, all in a line
 *        (a coxa about z, then femur and tibia about y, as on explorer and most hexapods)
 * @note The engine uses it for a leg whose definition asks for LegSolver::YawPitchPitch and has
 *       that shape; any other leg gets ChainKinematics. An adapter over KinematicSolver: each method converts JointVec (coxa, femur, tibia) to and
 *       from JointAngles and calls the existing code, so the maths is unchanged. Not mirrored, as
 *       the engine never asks for a mirrored solve.
*/
class YawPitchPitchLeg : public LegKinematics {
private:
    KinematicSolver solver;
public:
    /**
     * @brief Take the link lengths from the leg's chain (see KinematicSolver)
    */
    explicit YawPitchPitchLeg(const LegDefinition& leg)
        : solver(leg) {}

    static constexpr int kNumJoints = 3;  // coxa, femur, tibia

    int numJoints() const override { return kNumJoints; }
    Vec3 fk(const JointVec& jointAngles) const override;
    bool ik(const Vec3& target, const JointVec& seed, JointVec& jointAngles) const override;
    LegJacobian jacobian(const JointVec& jointAngles) const override;
};

}  // namespace gait_core

#endif  // GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP
