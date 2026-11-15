# hexapod-gait-core

The hexapod gait and behaviours as plain C++17 libraries with no ROS, Arduino or Pico SDK
dependency, so every robot walks and reacts to its sensors the same way. It also holds what every
stack has to agree on to be driven the same way: the controller packets and the gamepad layout.

How this fits with the robots and the other repositories is documented in
[hexapod-docs](https://github.com/TH4R4V3N1324/hexapod-docs).

## Contents

Two libraries: the gait (`gait_core`, headers in `include/gait_core/`) and the behaviours built on
it (`gait_core::behaviour`, headers in `include/hexapod_behaviour/`). The gait doesn't depend on
the behaviours, so a robot can use the gait alone.

Two header-only ones with no dependencies at all (not even Eigen), so controllers can use them
too: the controller packets (`gait_core::protocol`, `include/hexapod_protocol/`) and the gamepad
layout (`gait_core::teleop`, `include/hexapod_teleop/`).

### Gait

| Header | Contents |
| --- | --- |
| `gait_engine.hpp` | `GaitEngine`: the whole gait (startup, walking, sit/stand, tilt mode, leveling, load balancing, terrain adaptation, leg odometry). Feed it joint states, IMU and velocity; call `update(dt, now)` each tick for 18 joint targets, and `servoPower` to drive a servo power switch (unpowered until the first stand with `power_on_startup` off; `powerDown()` sits then switches off; losing the servos switches off too). Settings in `GaitParams` |
| `types.hpp` | `Scalar`, `Vec2/Vec3/Mat3`, `kPi`, `Timestamp`, `LogLevel`, `VelocityCommand` |
| `robot_geometry.hpp` | `Leg` enum; `RobotGeometry`: link lengths, coxa mounts, neutral stance, max stride, joint limits. Presets: `explorer()` |
| `kinematic_solver.hpp` | Single-leg IK/FK, Jacobian, foot force from joint torques |
| `frame_converter.hpp` | Leg frame ↔ body frame |
| `gait_config.hpp` | Gaits, modes, neutral stance, height/reach |
| `trajectory_generator.hpp` | Swing Béziers, stance lines, foot displacement, body pose |
| `leg_odometry.hpp` | Body motion and climb from planted feet |

### Behaviours

Each behaviour takes sensor readings and returns what to do; the robot does it with its own
hardware (a service call or `GaitEngine::sit`, a controller rumble, a beep, an LED). A robot
without a sensor just never feeds it, and that behaviour stays idle.

| Header | Contents |
| --- | --- |
| `battery_monitor.hpp` | `BatteryMonitor`: filtered voltage from up to 4 power monitors, OK / warning / critical with hysteresis, alert when it gets worse, sit once at critical. Settings in `BatteryParams` |

### Protocol

The packets between a handheld controller or gamepad receiver and a microcontroller robot:
ESP-NOW for hexapod-esp32, I²C for hexapod-servo2040, both sent by hexapod-controller. Valid C as
well as C++, since some controller firmware is C.

| Header | Contents |
| --- | --- |
| `packets.h` | `ControlPacket` (sticks, tilt yaw and height, optional controller orientation hold), `HexPacket` (height, gait, mode, seated / servo power / feature flags, calibration offsets), `Command`, `Gait`, `Mode` and `GaitFeature`. Both start with `HEXAPOD_PROTOCOL_VERSION` (3); a receiver drops any other version |

`Gait` and `Mode` equal the engine's values (checked in the tests). To change the layout, bump the
version and the size checks, and append rather than reorder enum values.

### Teleop

| Header | Contents |
| --- | --- |
| `gamepad_teleop.hpp` | `GamepadTeleop`: the gamepad layout. Feed it a normalised `GamepadState` and the robot's `RobotState` (seated, tilt mode); it returns velocities and a tilt pose as fractions of the robot's limits, plus events: next mode/gait, feature toggles, sit/stand, power down, height steps. `stepHeight()` picks the next body height. Settings in `TeleopParams` |

The layout, used by hexapod-ros2's `controller_teleop` and hexapod-controller's receiver:

| Input | Walking (normal / strafe) | Tilt mode |
| --- | --- | --- |
| Left stick | Forward/back, left/right | Shift the body forward/back, left/right |
| Right stick X | Turn | Roll |
| Right stick Y | — | Pitch (forward lowers the nose) |
| LT / RT [L2 / R2] | — | Yaw left / right |
| D-pad up / down | Body height up / down a step, repeating while held (not while seated) | Raise / lower the body |
| RB [R1] | On release: sit down if standing, otherwise stand up. Held 2 s: power down instead | The same |
| LB [L1] | Hold to follow the controller IMU orientation, release to recenter | The same |
| Start [Options] | Next mode | Next mode |
| Back [Create] | Next gait | Next gait |
| A, B, X, Y [Cross, Circle, Square, Triangle] | Terrain adaptation, walk leveling, load balancing, tilt leveling on/off | The same |

## API conventions

- SI units: metres, radians, seconds.
- Body frame as in [Coordinate frames](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/frames.md).
  Each leg frame has +X pointing out along the leg from its coxa joint.
- Per-leg arrays have `NUM_LEGS` entries indexed by the `Leg` enum, which runs clockwise from
  `LEG_FRONT_RIGHT` (0) to `LEG_FRONT_LEFT` (5). The hardware and URDF names count from 1
  (see [Legs](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/legs.md)),
  so `Leg` 0 is `leg1`.
- Callers convert to and from other units or frames at their own edges, never inside the core.

## Building and testing

```sh
git clone --recursive https://github.com/TH4R4V3N1324/hexapod-gait-core.git
cd hexapod-gait-core
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

GoogleTest is used from the system if installed, otherwise downloaded.

### CMake options

| Option | Default | Effect |
| --- | --- | --- |
| `GAIT_CORE_USE_DOUBLE` | `OFF` | Make `Scalar` `double` instead of `float` |
| `GAIT_CORE_BUNDLED_EIGEN` | `OFF` | Always use `third_party/eigen`, even if a system Eigen is found. Always on when cross-compiling |
| `GAIT_CORE_BUILD_TESTS` | `ON` when top-level | Build the test suite |

`Scalar` defaults to `float` because the microcontroller targets have no or single-precision
FPUs. CI runs the tests with both `float` and `double`, and cross-compiles for Cortex-M0+.

## Linking

- **CMake subproject:** `add_subdirectory(path/to/gait_core)` then link `gait_core::gait_core`,
  `gait_core::behaviour`, `gait_core::protocol` and/or `gait_core::teleop`.
- **Installed package (e.g. colcon):** `find_package(gait_core REQUIRED)` then link the same
  targets. `package.xml` lets colcon build it as a plain CMake package.
- **PlatformIO:** `library.json` is provided, so it works from `lib/` or `lib_deps`.
- **Just the protocol or teleop headers** (a controller that doesn't need the gait): add
  `include/` to the include path. They need nothing else, so the submodule can be cloned without
  its own Eigen submodule.

## Versioning

Releases are tagged `vMAJOR.MINOR.PATCH`. MINOR changes when the API changes; PATCH is for
fixes that keep the API.

## License

Apache-2.0. See [LICENSE](LICENSE).
