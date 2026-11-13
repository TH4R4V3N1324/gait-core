#include "gait_core/gait_config.hpp"

namespace gait_core {

GaitConfig::GaitConfig(const RobotGeometry& geometry)
    : currentHeight(geometry.standHeight),
      maxStride(geometry.maxStride),
      homePos(geometry.standReach, 0, 0),
      startPos(geometry.standReach, 0, geometry.standHeight) {
    startPosition.fill(startPos);
    startPosition[0] = Vec3::Zero();
}

/**
 * @brief Get the leg configuration based on the specified gait
 * @param[in] gait The gait type
 * @return The groups of legs that swing together, in phase order
*/
std::vector<std::vector<int>> GaitConfig::getGaitConfig(Gait gait) const {
    switch (gait) {
        case GAIT_TRIPOD:
            return {{1, 3, 5}, {2, 4, 6}};
        case GAIT_RIPPLE:
            return {{3, 6}, {2, 4}, {1, 5}};
        case GAIT_WAVE:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
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
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        startPosition[leg] = startPos;
    }
}

/**
 * @brief Set how far out from the hips the feet stand, and update start positions accordingly
 * @param[in] reach The feet's distance out from the hips (m)
*/
void GaitConfig::setReach(Scalar reach) {
    startPos.x() = reach;
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        startPosition[leg] = startPos;
    }
}

}  // namespace gait_core
