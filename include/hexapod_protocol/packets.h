/**
 * Packets between a handheld controller (or gamepad receiver) and a microcontroller robot:
 * ESP-NOW for hexapod-esp32, I2C (through the receiver) for hexapod-servo2040.
 *
 * Header-only and dependency-free, and valid C as well as C++, so any controller firmware can
 * include it without the gait library or Eigen. Every repo that sends or receives these packets
 * includes this one file, so they can't drift apart.
 *
 * Changing the layout: bump HEXAPOD_PROTOCOL_VERSION and the size checks at the bottom, append
 * enum values rather than reordering them, and release gait-core so every repo moves together.
 */
#ifndef HEXAPOD_PROTOCOL__PACKETS_H
#define HEXAPOD_PROTOCOL__PACKETS_H

#include <stdint.h>

/** First byte of every packet. A receiver drops packets with any other version. */
#define HEXAPOD_PROTOCOL_VERSION 4

/** Stick and tilt values run from -HEXAPOD_AXIS_FULL_SCALE to +HEXAPOD_AXIS_FULL_SCALE - 1. */
#define HEXAPOD_AXIS_FULL_SCALE 128
/** Controller orientation fields are in radians multiplied by this value. */
#define HEXAPOD_ANGLE_MILLIRAD_SCALE 1000
/** Most joints in one leg that HexPacket.legConfigs and CMD_SET_CONFIG can address (version 4). */
#define HEXAPOD_MAX_JOINTS_PER_LEG 6

#ifdef __cplusplus
#define HEXAPOD_ENUM(name) enum name : uint8_t
#else
/* C has no fixed-width enums before C23: the type is a uint8_t, the values an enum */
#define HEXAPOD_ENUM(name) typedef uint8_t name; enum
#endif

/**
 * A gait, as an index into the robot's gait list (0 .. HexPacket.numGaits - 1). The names are
 * explorer's gaits and equal gait_core::Gait; other robots number their own.
 */
HEXAPOD_ENUM(Gait) {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

/** Modes; the values equal gait_core::Mode. MODE_CONFIG is handled by the firmware itself. */
HEXAPOD_ENUM(Mode) {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    MODE_CONFIG,         /* every joint to its centre plus its calibration offset */
    NUM_MODES
};

/**
 * Gait engine features that can be switched on and off (GaitParams). The order matches the
 * gamepad's A, B, X, Y buttons (hexapod_teleop::Feature).
 */
HEXAPOD_ENUM(GaitFeature) {
    FEATURE_TERRAIN_ADAPTATION,
    FEATURE_WALK_LEVELING,
    FEATURE_LOAD_BALANCING,
    FEATURE_TILT_LEVELING,
    NUM_FEATURES
};

HEXAPOD_ENUM(Command) {
    CMD_NONE = 0,
    CMD_SET_GAIT,        /* [0] gait index, 0 .. HexPacket.numGaits - 1 */
    CMD_SET_MODE,        /* [0] mode */
    CMD_SET_CONFIG,      /* [0] leg, [1] joint 0 .. HEXAPOD_MAX_JOINTS_PER_LEG - 1 (body to foot), [2] offset in degrees (clamped to +-60) */
    CMD_HOME_STANCE,     /* stand up */
    CMD_REQUEST_CONFIG,  /* [0] leg; its offsets come back in HexPacket.legConfigs */
    CMD_SIT,             /* [0] 1 sit down, 0 stand up (which also powers the servos up) */
    CMD_POWER_DOWN,      /* sit down, then turn servo power off */
    CMD_SET_FEATURE      /* [0] GaitFeature, [1] 1 on, 0 off. A robot without the sensors refuses "on". */
};

#undef HEXAPOD_ENUM

/* HexPacket.flags */
#define HEX_FLAG_SEATED       (1u << 0)  /* seated, or sitting down or standing up */
#define HEX_FLAG_SERVO_POWER  (1u << 1)  /* the servos are powered */
#define HEX_FLAG_FEATURE(f)   (1u << (2 + (f)))  /* GaitFeature f is on */

#pragma pack(push, 1)

/**
 * Controller -> robot, every 10 ms. The robot applies a command once, when it or its arguments
 * change, so send each one for a few packets and then CMD_NONE.
 */
typedef struct ControlPacket {
    uint8_t version;          /* HEXAPOD_PROTOCOL_VERSION */
    int16_t joystick1X;       /* left stick, X right: strafe (or turn, by mode); tilt: shift right */
    int16_t joystick1Y;       /* left stick, Y forward: walk forward; tilt: shift forward */
    int16_t joystick2X;       /* right stick, X right: turn right; tilt: roll (lowers the right side) */
    int16_t joystick2Y;       /* right stick, Y forward: tilt: pitch (lowers the nose) */
    int16_t tiltYaw;          /* tilt: yaw, positive turns left */
    int16_t tiltZ;            /* tilt: raise the body */
    int16_t currentHeight;    /* body height in mm, 0 = unchanged */
    Command command;
    int16_t commandArgs[3];
    /* Controller orientation in milliradians. Valid while controllerTilt is non-zero. */
    int16_t controllerRoll;
    int16_t controllerPitch;
    int16_t controllerYaw;
    uint8_t controllerTilt;  /* left bumper held and a controller IMU is available */
} ControlPacket;

/** Robot -> controller, in reply to each ControlPacket. */
typedef struct HexPacket {
    uint8_t version;          /* HEXAPOD_PROTOCOL_VERSION */
    int16_t legConfigs[HEXAPOD_MAX_JOINTS_PER_LEG];  /* offsets for the leg asked for with CMD_REQUEST_CONFIG, body to foot; 0 past its last joint (version 4: was 3) */
    int16_t currentHeight;    /* body height in mm */
    Gait currentGait;         /* the gait it's in, or changing to */
    Mode currentMode;
    uint8_t flags;            /* HEX_FLAG_* */
    uint8_t numGaits;         /* how many gaits the robot has, for cycling through them (version 4) */
} HexPacket;

#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(ControlPacket) == 29, "ControlPacket layout changed: bump HEXAPOD_PROTOCOL_VERSION");
static_assert(sizeof(HexPacket) == 19, "HexPacket layout changed: bump HEXAPOD_PROTOCOL_VERSION");
#else
typedef char hexapod_control_packet_size_check[(sizeof(ControlPacket) == 29) ? 1 : -1];
typedef char hexapod_hex_packet_size_check[(sizeof(HexPacket) == 19) ? 1 : -1];
#endif

#endif /* HEXAPOD_PROTOCOL__PACKETS_H */
