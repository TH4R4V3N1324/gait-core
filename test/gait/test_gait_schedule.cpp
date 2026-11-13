#include <gtest/gtest.h>

#include "gait_core/gait_config.hpp"

using gait_core::Gait;
using gait_core::GaitConfig;
using gait_core::GaitSchedule;
using gait_core::NUM_GAITS;
using gait_core::NUM_LEGS;

TEST(GaitSchedule, FromGroupsNumbersThePhasesInOrder) {
    const GaitSchedule tripod = GaitSchedule::fromGroups({
        {gait_core::LEG_FRONT_RIGHT, gait_core::LEG_REAR_RIGHT, gait_core::LEG_MIDDLE_LEFT},
        {gait_core::LEG_MIDDLE_RIGHT, gait_core::LEG_REAR_LEFT, gait_core::LEG_FRONT_LEFT},
    });
    EXPECT_EQ(tripod.size(), 2u);
    EXPECT_TRUE(tripod.swingsIn(gait_core::LEG_FRONT_RIGHT, 0));
    EXPECT_TRUE(tripod.swingsIn(gait_core::LEG_MIDDLE_LEFT, 0));
    EXPECT_TRUE(tripod.swingsIn(gait_core::LEG_FRONT_LEFT, 1));
    EXPECT_FALSE(tripod.swingsIn(gait_core::LEG_FRONT_LEFT, 0));
}

TEST(GaitSchedule, EveryGaitSwingsEachLegExactlyOnce) {
    GaitConfig config;
    for (int g = 0; g < NUM_GAITS; ++g) {
        const GaitSchedule schedule = config.getGaitConfig(static_cast<Gait>(g));
        ASSERT_FALSE(schedule.empty()) << "gait " << g;
        int swings[NUM_LEGS] = {};
        for (size_t phase = 0; phase < schedule.size(); ++phase) {
            int inPhase = 0;
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                if (schedule.swingsIn(leg, phase)) {
                    ++swings[leg];
                    ++inPhase;
                }
            }
            EXPECT_GT(inPhase, 0) << "gait " << g << " phase " << phase << " swings no legs";
        }
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            EXPECT_EQ(swings[leg], 1) << "gait " << g << " leg " << leg;
        }
    }
}

TEST(GaitSchedule, UnknownGaitIsEmpty) {
    GaitConfig config;
    EXPECT_TRUE(config.getGaitConfig(NUM_GAITS).empty());
}
