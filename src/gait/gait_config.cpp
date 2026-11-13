#include "gait_core/gait_config.hpp"

namespace gait_core {

GaitConfig::GaitConfig(const RobotGeometry& geometry)
    : currentHeight(geometry.standHeight),
      maxStride(geometry.maxStride),
      homePos(geometry.standReach, 0, 0),
      startPos(geometry.standReach, 0, geometry.standHeight) {
    startPosition.fill(startPos);
}

/**
 * @brief Which legs swing in each phase of a gait
 * @param[in] gait The gait type
 * @return The schedule, empty for an unknown gait
*/
GaitSchedule GaitConfig::getGaitConfig(Gait gait) const {
    switch (gait) {
        case GAIT_TRIPOD:
            // Two alternating tripods, each stable on its own
            return GaitSchedule::fromGroups({
                {LEG_FRONT_RIGHT, LEG_REAR_RIGHT, LEG_MIDDLE_LEFT},
                {LEG_MIDDLE_RIGHT, LEG_REAR_LEFT, LEG_FRONT_LEFT},
            });
        case GAIT_RIPPLE:
            // Two legs at a time, one from each side
            return GaitSchedule::fromGroups({
                {LEG_REAR_RIGHT, LEG_FRONT_LEFT},
                {LEG_MIDDLE_RIGHT, LEG_REAR_LEFT},
                {LEG_FRONT_RIGHT, LEG_MIDDLE_LEFT},
            });
        case GAIT_WAVE:
            // One leg at a time: the right side rear to front, then the left side rear to front
            return GaitSchedule::fromGroups({
                {LEG_REAR_RIGHT}, {LEG_MIDDLE_RIGHT}, {LEG_FRONT_RIGHT},
                {LEG_REAR_LEFT}, {LEG_MIDDLE_LEFT}, {LEG_FRONT_LEFT},
            });
        default:
            return {};
    }
}

/**
 * @brief Changes to the next gait when called
*/
void GaitConfig::cycleGait() {
    pendingGait = static_cast<Gait>((currentGait + 1) % NUM_GAITS);
    gaitChangeRequested = true;
}

/**
 * @brief Sets the current gait to the specified gait
 * @param[in] gait The gait to switch to
*/
void GaitConfig::setGait(Gait gait) {
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
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        startPosition[leg] = startPos;
    }
}

/**
 * @brief Set how far out from the hips the feet stand, and update start positions accordingly
 * @param[in] reach The feet's distance out from the hips (m)
*/
void GaitConfig::setReach(Scalar reach) {
    startPos.x() = reach;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        startPosition[leg] = startPos;
    }
}

}  // namespace gait_core
