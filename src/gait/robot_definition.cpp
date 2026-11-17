#include "gait_core/robot_definition.hpp"

namespace gait_core {

static_assert(kMaxLegs >= NUM_LEGS, "explorer() has 6 legs: GAIT_CORE_MAX_LEGS must be at least 6");

RobotDefinition RobotDefinition::explorer() {
    RobotDefinition g{};
    g.numLegs = NUM_LEGS;

    g.coxaLength  = static_cast<Scalar>(0.063);
    g.femurLength = static_cast<Scalar>(0.100);
    g.tibiaLength = static_cast<Scalar>(0.186);

    // Coxa joint yaw (rad) and position (m), matching the URDF: the middle legs 0.11836 m out
    // to the side, the corner legs at 45 degrees from (0.11916, 0.08952) on the slimmer body
    const Scalar m  = static_cast<Scalar>(0.11836);
    const Scalar cx = static_cast<Scalar>(0.11916);
    const Scalar cy = static_cast<Scalar>(0.08952);
    g.legs[LEG_FRONT_RIGHT]  = {-kPi / 4, Vec3(cx, -cy, 0)};
    g.legs[LEG_MIDDLE_RIGHT] = {-kPi / 2, Vec3(0, -m, 0)};
    g.legs[LEG_REAR_RIGHT]   = {-3 * kPi / 4, Vec3(-cx, -cy, 0)};
    g.legs[LEG_REAR_LEFT]    = {3 * kPi / 4, Vec3(-cx, cy, 0)};
    g.legs[LEG_MIDDLE_LEFT]  = {kPi / 2, Vec3(0, m, 0)};
    g.legs[LEG_FRONT_LEFT]   = {kPi / 4, Vec3(cx, cy, 0)};

    // Each foot straight out from its coxa (so the corner feet
    // sit at 45 degrees), 0.177 m out and 0.134 m below the hips
    g.standReach  = static_cast<Scalar>(0.177);
    g.standHeight = static_cast<Scalar>(-0.134);

    g.maxStride = static_cast<Scalar>(0.06);

    // The femur and tibia ranges match femur_lower/upper and tibia_lower/upper in the URDF (from the
    // CAD meshes); the coxa keeps +-1.4 rad until its range is worked out
    g.limits.coxaMax  = static_cast<Scalar>(1.4);
    g.limits.femurMin = static_cast<Scalar>(-1.6);
    g.limits.femurMax = static_cast<Scalar>(1.4);
    g.limits.tibiaMin = static_cast<Scalar>(-0.78);
    g.limits.tibiaMax = static_cast<Scalar>(2.63);
    return g;
}

}  // namespace gait_core
