#ifndef GAIT_CORE__CHAIN_KINEMATICS_HPP
#define GAIT_CORE__CHAIN_KINEMATICS_HPP

#include "gait_core/leg_definition.hpp"
#include "gait_core/leg_kinematics.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Kinematics for any chain of revolute joints, from its LegDefinition
 * @note FK and the Jacobian are exact. IK is numerical (damped least squares from the seed),
 *       so it finds the solution nearest the seed and takes a bounded number of iterations.
 *       Use an analytic LegKinematics where there is one (HexapodLeg3Dof for explorer).
*/
class ChainKinematics : public LegKinematics {
public:
    /**
     * @brief Take a copy of the leg's chain
     * @param[in] leg The leg; its axes are normalised here, and numJoints kept to 0..kMaxJointsPerLeg
    */
    explicit ChainKinematics(const LegDefinition& leg);

    int numJoints() const override { return leg_.numJoints; }
    Vec3 fk(const JointVec& q) const override;
    bool ik(const Vec3& target, const JointVec& seed, JointVec& q) const override;
    LegJacobian jacobian(const JointVec& q) const override;

private:
    LegDefinition leg_;   // a copy, so this never depends on the RobotDefinition outliving it
};

}  // namespace gait_core

#endif  // GAIT_CORE__CHAIN_KINEMATICS_HPP
