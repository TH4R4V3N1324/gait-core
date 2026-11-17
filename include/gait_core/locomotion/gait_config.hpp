#ifndef GAIT_CORE__GAIT_CONFIG_HPP
#define GAIT_CORE__GAIT_CONFIG_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "gait_core/robot/gait_pattern.hpp"
#include "gait_core/robot/robot_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

struct LocomotionOption {
    const char* name;
    uint8_t id;
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    NUM_MODES
};

/**
 * @brief The robot's gaits as name and index, for listing (range-for over it)
 * @note The names point into the GaitConfig it came from, so use it while that's alive.
*/
struct GaitList {
    std::array<LocomotionOption, kMaxGaits> items{};
    size_t count = 0;

    const LocomotionOption* begin() const { return items.data(); }
    const LocomotionOption* end() const { return items.data() + count; }
    size_t size() const { return count; }
};

class GaitConfig {
public:
    static constexpr std::array<LocomotionOption, NUM_MODES> modes = {{
        {"normal", MODE_NORMAL},
        {"strafe", MODE_STRAFE},
        {"tilt", MODE_TILT}
    }};

    /**
     * @brief Set up the neutral stance and the gaits from the robot
     * @param[in] geometry The robot whose stance and gaits to use
    */
    explicit GaitConfig(const RobotDefinition& geometry);

    /**
     * @brief Get the robot's gaits
     * @return Each gait's name and index
    */
    GaitList getGaits() const;
    /**
     * @brief Get the supported modes
     * @return The list of supported modes
    */
    const std::array<LocomotionOption, NUM_MODES>& getModes() const { return modes; }

    int numLegs() const { return legCount; }
    int numGaits() const { return gaitCount; }
    const char* gaitName(uint8_t gait) const { return gait < gaitCount ? patterns[gait].name : "?"; }
    /**
     * @brief The first of the robot's gaits that the engine can't run on its legs
     * @return Its index, or -1 if every gait can run (and there is at least one)
    */
    int unrunnableGait() const;

    Scalar currentHeight;
    Scalar maxStride;
    Vec3 homePos;
    Vec3 startPos;
    std::array<Vec3, kMaxLegs> startPosition;   // indexed by leg number, 0..numLegs - 1: startPos + stanceOffset
    std::array<Vec3, kMaxLegs> stanceOffset;    // each leg's LegDefinition::stanceOffset
    uint8_t currentGait = 0;                    // index into the robot's gaits
    Mode currentMode = MODE_NORMAL;
    bool gaitChangeRequested = false;
    uint8_t pendingGait = 0;
    GaitSchedule getGaitConfig(uint8_t gait) const;
    void cycleGait();
    void setGait(uint8_t gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(Scalar height);
    void setReach(Scalar reach);

private:
    void placeFeet();

    int legCount;
    int gaitCount;
    std::array<GaitPattern, kMaxGaits> patterns;
    std::array<GaitSchedule, kMaxGaits> schedules;   // empty for a gait that can't run
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_CONFIG_HPP
