#include "gait_core/gait_config.hpp"

#include <algorithm>

namespace gait_core {

GaitConfig::GaitConfig(const RobotDefinition& geometry)
    : currentHeight(geometry.standHeight),
      maxStride(geometry.maxStride),
      homePos(geometry.standReach, 0, 0),
      startPos(geometry.standReach, 0, geometry.standHeight),
      legCount(geometry.numLegs),
      gaitCount(std::clamp(geometry.numGaits, 0, kMaxGaits)),
      patterns(geometry.gaits) {
    startPosition.fill(startPos);
    for (int gait = 0; gait < gaitCount; ++gait) {
        if (!patterns[gait].schedule(legCount, schedules[gait])) {
            schedules[gait] = GaitSchedule{};
        }
    }
}

GaitList GaitConfig::getGaits() const {
    GaitList list;
    for (int gait = 0; gait < gaitCount; ++gait) {
        list.items[gait] = {patterns[gait].name, static_cast<uint8_t>(gait)};
    }
    list.count = static_cast<size_t>(gaitCount);
    return list;
}

int GaitConfig::unrunnableGait() const {
    if (gaitCount == 0) {
        return 0;
    }
    for (int gait = 0; gait < gaitCount; ++gait) {
        if (schedules[gait].empty()) {
            return gait;
        }
    }
    return -1;
}

/**
 * @brief Which legs swing in each phase of a gait
 * @param[in] gait The gait's index in the robot's list
 * @return The schedule, empty for an unknown gait or one that can't run on the robot's legs
*/
GaitSchedule GaitConfig::getGaitConfig(uint8_t gait) const {
    return gait < gaitCount ? schedules[gait] : GaitSchedule{};
}

/**
 * @brief Changes to the next gait when called
*/
void GaitConfig::cycleGait() {
    if (gaitCount == 0) {
        return;
    }
    pendingGait = static_cast<uint8_t>((currentGait + 1) % gaitCount);
    gaitChangeRequested = true;
}

/**
 * @brief Sets the current gait to the specified gait
 * @param[in] gait The gait to switch to, an index into the robot's gaits (checked by the engine)
*/
void GaitConfig::setGait(uint8_t gait) {
    pendingGait = gait;
    gaitChangeRequested = true;
}

/**
 * @brief Changes to the next mode when called
*/
void GaitConfig::cycleMode() {
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
}

/**
 * @brief Sets the current mode to the specified mode
 * @param[in] mode The mode to switch to
*/
void GaitConfig::setMode(Mode mode) {
    currentMode = mode;
}

/**
 * @brief Set the height of the hexapod and update start positions accordingly
 * @param[in] height The new height to set (m, negative = feet below the hips)
*/
void GaitConfig::setHeight(Scalar height) {
    currentHeight = height;
    startPos.z() = currentHeight;
    for (int leg = 0; leg < numLegs(); ++leg) {
        startPosition[leg] = startPos;
    }
}

/**
 * @brief Set how far out from the hips the feet stand, and update start positions accordingly
 * @param[in] reach The feet's distance out from the hips (m)
*/
void GaitConfig::setReach(Scalar reach) {
    startPos.x() = reach;
    for (int leg = 0; leg < numLegs(); ++leg) {
        startPosition[leg] = startPos;
    }
}

}  // namespace gait_core
