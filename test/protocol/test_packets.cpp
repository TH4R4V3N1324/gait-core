// Checks the controller <-> robot packet layout, and that its values match the gait engine's

#include <gtest/gtest.h>

#include <cstddef>

#include "gait_core/gait_config.hpp"
#include "hexapod_protocol/packets.h"

extern "C" {
size_t hexapod_c_control_size(void);
size_t hexapod_c_hex_size(void);
size_t hexapod_c_control_command_offset(void);
size_t hexapod_c_hex_flags_offset(void);
int hexapod_c_feature_count(void);
}

// The robot passes Gait and Mode values straight to the engine
static_assert(static_cast<int>(::GAIT_TRIPOD) == gait_core::GAIT_TRIPOD, "");
static_assert(static_cast<int>(::GAIT_RIPPLE) == gait_core::GAIT_RIPPLE, "");
static_assert(static_cast<int>(::GAIT_WAVE) == gait_core::GAIT_WAVE, "");
static_assert(static_cast<int>(::NUM_GAITS) == gait_core::NUM_GAITS, "");
static_assert(static_cast<int>(::MODE_NORMAL) == gait_core::MODE_NORMAL, "");
static_assert(static_cast<int>(::MODE_STRAFE) == gait_core::MODE_STRAFE, "");
static_assert(static_cast<int>(::MODE_TILT) == gait_core::MODE_TILT, "");
// MODE_CONFIG is the firmware's own, just past the engine's modes
static_assert(static_cast<int>(::MODE_CONFIG) == gait_core::NUM_MODES, "");

TEST(Packets, SizesAreFixed) {
    EXPECT_EQ(sizeof(ControlPacket), 22u);
    EXPECT_EQ(sizeof(HexPacket), 12u);
    EXPECT_EQ(sizeof(Command), 1u);
    EXPECT_EQ(sizeof(Gait), 1u);
    EXPECT_EQ(sizeof(Mode), 1u);
}

TEST(Packets, VersionComesFirst) {
    EXPECT_EQ(offsetof(ControlPacket, version), 0u);
    EXPECT_EQ(offsetof(HexPacket, version), 0u);
}

TEST(Packets, CAndCppAgree) {
    EXPECT_EQ(hexapod_c_control_size(), sizeof(ControlPacket));
    EXPECT_EQ(hexapod_c_hex_size(), sizeof(HexPacket));
    EXPECT_EQ(hexapod_c_control_command_offset(), offsetof(ControlPacket, command));
    EXPECT_EQ(hexapod_c_hex_flags_offset(), offsetof(HexPacket, flags));
    EXPECT_EQ(hexapod_c_feature_count(), NUM_FEATURES);
}

TEST(Packets, CommandValuesNeverMove) {
    // Appending is fine; changing these breaks every controller and robot built before
    EXPECT_EQ(CMD_NONE, 0);
    EXPECT_EQ(CMD_SET_GAIT, 1);
    EXPECT_EQ(CMD_SET_MODE, 2);
    EXPECT_EQ(CMD_SET_CONFIG, 3);
    EXPECT_EQ(CMD_HOME_STANCE, 4);
    EXPECT_EQ(CMD_REQUEST_CONFIG, 5);
    EXPECT_EQ(CMD_SIT, 6);
    EXPECT_EQ(CMD_POWER_DOWN, 7);
    EXPECT_EQ(CMD_SET_FEATURE, 8);
}

TEST(Packets, FlagsDontOverlapAndFitInAByte) {
    unsigned all = HEX_FLAG_SEATED | HEX_FLAG_SERVO_POWER;
    for (int f = 0; f < NUM_FEATURES; ++f) {
        EXPECT_EQ(all & HEX_FLAG_FEATURE(f), 0u) << "feature " << f;
        all |= HEX_FLAG_FEATURE(f);
    }
    EXPECT_LE(all, 0xFFu);
}
