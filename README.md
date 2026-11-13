# hexapod-gait-core

The hexapod gait maths (kinematics, frames, trajectories and leg odometry) as a plain C++17
library with no ROS, Arduino or Pico SDK dependency.

How this fits with the robots and the other repositories is documented in
[hexapod-docs](https://github.com/TH4R4V3N1324/hexapod-docs).

## Contents

| Header | Contents |
| --- | --- |
| `gait_engine.hpp` | `GaitEngine`: the whole gait (startup, walking, sit/stand, tilt mode, leveling, load balancing, terrain adaptation, leg odometry). Feed it joint states, IMU and velocity; call `update(dt, now)` each tick for 18 joint targets. Settings in `GaitParams` |
| `types.hpp` | `Scalar`, `Vec2/Vec3/Mat3`, `kPi`, `VelocityCommand` |
| `robot_geometry.hpp` | `RobotGeometry`: link lengths, coxa mounts, neutral stance, max stride, joint limits. Presets: `explorer()` |
| `kinematic_solver.hpp` | Single-leg IK/FK, Jacobian, foot force from joint torques |
| `frame_converter.hpp` | Leg frame ↔ body frame |
| `gait_config.hpp` | Gaits, modes, neutral stance, height/reach |
| `trajectory_generator.hpp` | Swing Béziers, stance lines, foot displacement, body pose |
| `leg_odometry.hpp` | Body motion and climb from planted feet |

## API conventions

- SI units: metres, radians, seconds.
- Body frame as in [Coordinate frames](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/frames.md).
  Each leg frame has +X pointing out along the leg from its coxa joint.
- Legs are numbered 1–6 (see [Legs](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/legs.md)).
  Per-leg arrays have `MAX_LEGS + 1` entries and are indexed by leg number; entry 0 is unused.
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

- **CMake subproject:** `add_subdirectory(path/to/gait_core)` then link `gait_core`.
- **Installed package (e.g. colcon):** `find_package(gait_core REQUIRED)` then link
  `gait_core::gait_core`. `package.xml` lets colcon build it as a plain CMake package.
- **PlatformIO:** `library.json` is provided, so it works from `lib/` or `lib_deps`.

## Versioning

Releases are tagged `vMAJOR.MINOR.PATCH`. MINOR changes when the API changes; PATCH is for
fixes that keep the API.

## License

Apache-2.0. See [LICENSE](LICENSE).
