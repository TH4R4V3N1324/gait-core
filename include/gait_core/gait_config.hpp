#ifndef GAIT_CORE__GAIT_CONFIG_HPP
#define GAIT_CORE__GAIT_CONFIG_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "gait_core/robot_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

static constexpr int MAX_RESOLUTION = 50;
// Explorer defaults; the stride actually used comes from RobotDefinition::maxStride
static constexpr Scalar MAX_STRIDE_LENGTH = static_cast<Scalar>(0.06);  // m
static constexpr Scalar LIFT_HEIGHT = static_cast<Scalar>(0.05);        // m

struct LocomotionOption {
    const char* name;
    uint8_t id;
};

enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    NUM_MODES
};

/**
 * @brief Which legs swing in each phase of a gait
 * @note A gait is a cycle of phases; in each, one group of legs swings while the rest stand.
 *       Every leg swings in exactly one phase per cycle.
*/
struct GaitSchedule {
    uint8_t phases = 0;                           // phases in a cycle, 0 if not set up
    std::array<uint8_t, kMaxLegs> phaseOf{};  // phase each leg swings in, indexed by leg number

    size_t size() const { return phases; }
    bool empty() const { return phases == 0; }
    /**
     * @brief Whether a leg swings in a phase
    */
    bool swingsIn(int legNum, size_t phase) const { return phaseOf[legNum] == phase; }

    /**
     * @brief Build a schedule from the legs that swing together in each phase, in order
     * @param[in] groups One list of legs per phase, the legs that swing together in that phase
     * @return The schedule; a leg left out never swings (the GaitSchedule test checks every gait swings each leg)
    */
    static constexpr GaitSchedule fromGroups(std::initializer_list<std::initializer_list<Leg>> groups) {
        GaitSchedule schedule;
        for (auto& phase : schedule.phaseOf) {
            phase = UINT8_MAX;
        }
        for (const auto& group : groups) {
            for (const Leg leg : group) {
                schedule.phaseOf[leg] = schedule.phases;
            }
            ++schedule.phases;
        }
        return schedule;
    }
};

class GaitConfig {
public:
    static constexpr std::array<LocomotionOption, NUM_GAITS> gaits = {{
        {"tripod", GAIT_TRIPOD},
        {"ripple", GAIT_RIPPLE},
        {"wave", GAIT_WAVE}
    }};
    static constexpr std::array<LocomotionOption, NUM_MODES> modes = {{
        {"normal", MODE_NORMAL},
        {"strafe", MODE_STRAFE},
        {"tilt", MODE_TILT}
    }};

    /**
     * @brief Set up the neutral stance from the robot's geometry
     * @param[in] geometry The robot whose stance to use
    */
    explicit GaitConfig(const RobotDefinition& geometry = RobotDefinition::explorer());

    /**
     * @brief Get the supported gaits
     * @return The list of supported gaits
    */
    const std::array<LocomotionOption, NUM_GAITS>& getGaits() const { return gaits; }
    /**
     * @brief Get the supported modes
     * @return The list of supported modes
    */
    const std::array<LocomotionOption, NUM_MODES>& getModes() const { return modes; }

    int numLegs() const { return legCount; }

    Scalar currentHeight;
    Scalar maxStride;
    Vec3 homePos;
    Vec3 startPos;
    std::array<Vec3, kMaxLegs> startPosition;   // indexed by leg number, 0..numLegs - 1
    Gait currentGait = GAIT_TRIPOD;
    Mode currentMode = MODE_NORMAL;
    bool gaitChangeRequested = false;
    Gait pendingGait = GAIT_TRIPOD;
    GaitSchedule getGaitConfig(Gait gait) const;
    void cycleGait();
    void setGait(Gait gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(Scalar height);
    void setReach(Scalar reach);

private:
    int legCount;
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_CONFIG_HPP
