// GaitEngine: terrain adaptation. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

/**
 * @brief Whether the foot forces can be trusted
 * @return true once the forces add up to a robot standing on its feet; false without effort data
 *         or with the robot lifted
*/
bool GaitEngine::forcesValid() {
    Scalar total = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        total += std::max(S(0.0), foot_forces[legNum].z());
    }
    return has_foot_forces && total > 3 * p.contact_force_on;
}

/**
 * @brief Follow uneven ground with the feet
 * @param[in] dt The time since the last tick (s)
 * @note A swinging foot that meets the ground in the second half of its swing stops there. One that
 *       finishes its swing without touching down keeps reaching down at seek_speed until it does.
 *       Each foot remembers its ground height (up to max_step_height either way). The body rises or
 *       falls at body_height_rate so the feet's average ground is back at nominal. Off, or without
 *       force data, the offsets ease back to zero.
*/
void GaitEngine::adaptToTerrain(Scalar dt) {
    if (!p.terrain_adaptation || !forcesValid()) {
        seeking.fill(false);
        landing_check.fill(false);
    } else {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            const Vec3& force = foot_forces[legNum];
            const Scalar sideways = force.head<2>().norm();
            const bool loaded = force.z() > p.touchdown_force;
            // Landing pushes up; catching an edge on the way across pushes back
            const bool landed = loaded && force.z() > p.touchdown_ratio * sideways;
            // Only once the foot is up, and pushed back without bearing weight, for 3 ticks
            const bool blockedFoot = leg_swinging[legNum] && phase_progress > S(0.2) && phase_progress < S(0.9) &&
                sideways > p.stumble_force && !loaded;
            stumble_ticks[legNum] = blockedFoot ? stumble_ticks[legNum] + 1 : 0;
            if (stumble_ticks[legNum] >= 3 && stumble_target[legNum] == 0) {
                stumble_target[legNum] = p.stumble_lift;
                emit(LogLevel::Debug, "Leg %d stumbled at s=%.2f (%.1f N sideways, %.1f N up)",
                    legNum + 1, static_cast<double>(phase_progress), static_cast<double>(sideways),
                    static_cast<double>(force.z()));
            }
            // Only well above where it was aiming: touching a little early is just a normal landing
            const bool aboveTarget = current_leg_positions[legNum].z() - swing_target[legNum].z() > S(0.01);
            // It has to look landed for 3 ticks in a row
            touchdown_ticks[legNum] = (leg_swinging[legNum] && phase_progress > S(0.6) && landed && aboveTarget)
                ? touchdown_ticks[legNum] + 1 : 0;
            if (touchdown_ticks[legNum] >= 3) {
                // Touched down early: plant the foot here
                touchdown_ticks[legNum] = 0;
                leg_swinging[legNum] = false;
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - config().startPosition.at(legNum).z(),
                    -p.max_step_height, p.max_step_height);
                emit(LogLevel::Debug, "Leg %d touched down %.1f mm high", legNum + 1,
                    static_cast<double>(terrain_offset[legNum] * 1000));
            } else if (landing_check[legNum] && !leg_swinging[legNum]) {
                // Its swing ended unloaded: if it is still unloaded once the weight has had time to
                // come onto it, the ground is lower than expected
                if (force.z() > p.contact_force_off) {
                    landing_check[legNum] = false;
                } else if ((unloaded_time[legNum] += dt) > S(0.15)) {
                    landing_check[legNum] = false;
                    seeking[legNum] = true;
                }
            } else if (seeking[legNum] && !leg_swinging[legNum]) {
                if (force.z() > p.contact_force_on || terrain_offset[legNum] <= -p.max_step_height) {
                    seeking[legNum] = false;
                    emit(LogLevel::Debug, "Leg %d found ground %.1f mm low", legNum + 1,
                        static_cast<double>(-terrain_offset[legNum] * 1000));
                } else {
                    terrain_offset[legNum] = std::max(-p.max_step_height, terrain_offset[legNum] - p.seek_speed * dt);
                }
            }
        }
    }

    // Body height follows the average ground (or the offsets unwind when adaptation is off)
    Scalar mean = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        mean += terrain_offset[legNum] / numLegs();
    }
    const Scalar shift = std::clamp(mean, -p.body_height_rate * dt, p.body_height_rate * dt);
    ground_level += shift;  // the body rises by shift
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        terrain_offset[legNum] -= shift;
        if (!p.terrain_adaptation || !forcesValid()) {
            terrain_offset[legNum] -= std::clamp(terrain_offset[legNum], -p.body_height_rate * dt, p.body_height_rate * dt);
        }
    }
}

}  // namespace gait_core
