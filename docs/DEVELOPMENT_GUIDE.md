# OpenFlightComputer Flight Simulation — Development Guide

This guide explains how to build `flight-simulation`, a closed-loop software-in-the-loop (SIL) simulator for the OpenFlightComputer firmware. It covers the architecture, the conventions every file must follow, the build setup, reference code for the difficult parts, a milestone plan with definitions of done, and a roadmap for after the first version.

- Firmware reference snapshot: `flight-computer-firmware` at commit `521d62e` (2026-09-23). Re-check quoted signatures whenever the submodule is bumped.
- Status: M0 in progress. Repository and host-build scaffolding exist; the two C/C++ interoperability tests remain hands-on tasks. Add simulator modules as their milestones begin rather than pre-creating the full roadmap in source files.

---

## 1. Goal and non-goals

**Goal.** Run the real, unmodified flight firmware (the manual behavior, the flight-control core, and the IMU processing and attitude estimator) in a closed loop against a physics model of the vehicle, on a laptop, faster than real time and deterministically. The simulator should also do three things:

1. Characterize the controller with inputs that have never been flown, such as steps, disturbances and failsafe events.
2. Be validated against real flight logs from the blackbox.
3. Be extensible later: more detailed aerodynamics, fault injection, new behaviors, and hardware-in-the-loop.

**Non-goals for the first version:**

- A general-purpose simulation framework or game-engine graphics.
- High-fidelity aerodynamics. Start with the simplest model that is still honest, and add fidelity only when a validation result shows it is needed.
- Real-time or live 3D rendering. Visualization is offline and driven by the logs.
- Any change to firmware behavior. The firmware is linked as it is. The only firmware changes allowed are small testability refactors (Section 5.3), and none of them may change what the firmware does.

**Working agreement.** The physics code (rigid body, integrator, propulsion, contact, sensor model) and the validation comparison are written by hand and must be explainable line by line. AI coding agents may scaffold the build, generate test boilerplate and review code. Document this in the README's "How this was built" section.

---

## 2. Architecture

### 2.1 The closed loop

```
                  ┌──────────────────────────────────────────────┐
                  │            ObjectiveProducer (C++)           │
                  │  Manual │ Scripted │ StubAutonomous │ (later)│
                  └──────────────────────┬───────────────────────┘
                                         │ control_objective_t
                                         ▼
              ┌───────────────────────────────────────────────┐
  REAL C  →   │ flight_control_update()  (flight_control_core)│
              │ attitude ctrl → rate PID → quad-X mixer       │
              └──────────────────────┬────────────────────────┘
                                     │ motor_command_t.throttle[4]  (0..1)
                                     ▼
              ┌───────────────────────────────────────────────┐
  NEW C++ →   │ Propulsion: command → rotor speed (lag)       │
              │            → thrust + reaction torque          │
              ├───────────────────────────────────────────────┤
              │ Rigid body 6-DOF + RK4 integrator             │
              │ + ground contact                               │
              └──────────────────────┬────────────────────────┘
                                     │ true state (position, velocity,
                                     │ attitude, body rates, specific force)
                                     ▼
              ┌───────────────────────────────────────────────┐
  NEW C++ →   │ IMU model: true state → BMI270-shaped counts  │
              │ (scale, bias, noise, quantization, saturation)│
              └──────────────────────┬────────────────────────┘
                                     │ imu_sample_snapshot_t
                                     ▼
              ┌───────────────────────────────────────────────┐
  REAL C  →   │ imu_processing_pipeline_process()             │
              │ filters + complementary attitude estimator     │
              └──────────────────────┬────────────────────────┘
                                     │ attitude_snapshot_t → vehicle_state_t
                                     └──────► back to the producer / core
```

**What is real and what is new.** Everything marked REAL is the firmware's own C source, compiled for the host and linked through `extern "C"`. Everything marked NEW is simulator code. The ObjectiveProducer layer is mixed. The manual producer wraps the real `receiver_failsafe_update()` and `manual_easy_behavior_update()`. The scripted and stub-autonomous producers are new code.

**Ground truth never enters the loop.** Producers and the firmware only ever see what real sensors could provide. True position, true velocity and the true attitude quaternion are logged as ground truth but are never passed back into the loop. The vehicle has no GPS, so the firmware has no position information either, and the simulator must not invent any. The `ProducerContext` type (Section 6.7) enforces this: it has no ground-truth fields.

### 2.2 One control tick, in order

A tick lasts one control period (1000 µs by default, which matches the firmware's 1 kHz control task).

1. **Sense.** The IMU model samples the true state at time `t`. The result is an `imu_sample_snapshot_t` with a strictly increasing `sequence` and `acquired_at_us`.
2. **Estimate.** The real `imu_processing_pipeline_process()` consumes the sample. `vehicle_state_t` is then built exactly the way the firmware's `read_stabilization_inputs()` builds it.
3. **Decide.** The active producer returns a status: an objective, hold, stop, or idle.
4. **Control.** If there is an objective, the real `flight_control_update()` produces the four motor commands. Stop and idle produce zero commands. Hold keeps the previous commands.
5. **Actuate and integrate.** The commands are held constant (zero-order hold) while the plant integrates from `t` to `t + dt`. The period can be split into several physics substeps.
6. **Log.** Telemetry and ground truth for this tick are written to the log.
7. **Advance.** `t += control_period_us`.

No step checks for liftoff. Liftoff happens when summed thrust exceeds weight and the integrated position leaves the ground (Section 6.4).

---

## 3. Conventions

These conventions are enforced in code review. Most simulator bugs come from breaking one of them.

### 3.1 Frames

| Frame | Axes | Used for |
| --- | --- | --- |
| Body (FRD) | +X forward, +Y right, +Z down | Forces, torques, body rates, IMU output. It matches the firmware (`docs/bmi270.md`: body forward = +X, right = +Y, down = +Z). |
| World (NED) | +X north, +Y east, +Z down, with the origin on the ground | Position and velocity. "Up" is **negative z**. The ground is the plane `z = 0`, and the vehicle is airborne when `z < 0`. |

**Attitude** is stored as a unit quaternion `q` that rotates body vectors into the world frame: `v_world = q * v_body`. Internally the simulator never integrates Euler angles.

**Euler angles** (only for comparison and logging) use the aerospace ZYX sequence, yaw then pitch then roll. The signs match the firmware: positive roll puts the right side down, positive pitch puts the nose up, and positive yaw turns the nose right (clockwise seen from above).

**Gravity** in NED is `g_world = (0, 0, +9.80665) m/s²`.

**Specific force** is what an accelerometer measures: `f_body = qᵀ ⊗ (a_world − g_world)`. When the vehicle is at rest and level, `f_body = (0, 0, −9.80665)`, which is −1 g on z. The firmware relies on this: `accelerometer_attitude_calculate()` computes `roll = atan2(−a_y, −a_z)` with the comment "static specific force is up".

### 3.2 Units and naming

- SI units internally: metres, seconds, kilograms, radians, newtons.
- Degrees appear only at the firmware boundary, because the firmware uses degrees and degrees per second.
- Every physical quantity carries its unit in its name: `position_m`, `velocity_mps`, `rate_rad_s`, `thrust_n`, `torque_nm`, `time_us`, `rate_dps`.
- Time is always an integer `uint64_t` in microseconds, the same as the firmware's timestamps. Convert to `double` seconds only inside the integrator.

### 3.3 Precision

- The plant uses `double`.
- The firmware uses `float`.
- Convert explicitly, and only at the boundary between the two (the IMU model output, reading the motor commands, and logging).

### 3.4 Motor geometry: derived from the firmware mixer, must be verified

`quad_x_mixer_prepare()` defines each motor's coefficients `{roll, pitch, yaw}`. For `PROPELLER_LAYOUT_PROPS_IN`, the yaw coefficient is −1:

```
motor 0: {+1, +1, -1}
motor 1: {+1, -1, +1}
motor 2: {-1, +1, +1}
motor 3: {-1, -1, -1}
```

A positive correction on an axis must produce a positive torque on that axis. With the sign conventions from Section 3.1 this gives:

| Logical motor | Position (body) | x (fwd) | y (right) | Prop spin seen from above | `spin_direction` (sign of rotation about +Z down) |
| --- | --- | --- | --- | --- | --- |
| 0 | front-left | +a | −a | CW | +1 |
| 1 | rear-left | −a | −a | CCW | −1 |
| 2 | front-right | +a | +a | CCW | −1 |
| 3 | rear-right | −a | +a | CW | +1 |

Here `a = arm_radius / √2`, and `arm_radius` is the distance from the centre of mass to a motor axis. The layout agrees with the firmware comment "Positive yaw torque raises the CCW pair for props-in", and with Betaflight's default props-in layout.

**This table is derived from the mixer, not measured.** Before trusting any closed-loop result:

1. Check it against the real airframe: which ESC output goes to which corner, the `motor_mapping` permutation, the configured motor directions, and how the props are mounted.
2. Add the verified table to the firmware's documentation. At the moment it exists only on the vehicle.
3. Keep the mixer and plant consistency test from Section 7.2 passing at all times. The pitch-feedback sign error found during bench validation would have been caught by exactly this kind of test.

### 3.5 Throttle semantics (easy to get wrong)

- The objective's `throttle` is the shaped stick value in the range 0 to 1.
- The core maps it to a motor baseline: `motor_baseline = armed_idle + throttle × (1 − armed_idle)`. With the default `armed_idle = 0.05`, a stick throttle of 0.20 gives a baseline of 0.24.
- A stick throttle of exactly 0 while armed produces `armed_idle` on all motors (`mix_idle`), so the motors spin slowly.
- A motor command of 0 means the motor is stopped. That happens only when disarmed or in failsafe stop.

---

## 4. Repository layout

```
flight-simulation/
├── CMakeLists.txt
├── CMakePresets.json               # debug, release, asan-ubsan
├── external/
│   └── flight-computer-firmware/   # git submodule, pinned commit
├── cmake/
│   └── FirmwareCore.cmake          # builds the ofc_firmware_core static library
├── include/ofcsim/                 # public headers of the simulator library
│   ├── math.hpp                    # Eigen aliases, quaternion helpers
│   ├── rigid_body.hpp              # RigidBodyState, derivative, RK4
│   ├── propulsion.hpp              # motor lag, thrust and torque, geometry
│   ├── contact.hpp                 # ground contact
│   ├── imu_model.hpp               # true state → imu_sample_snapshot_t
│   ├── vehicle_config.hpp          # physical parameters, JSON loading
│   ├── firmware_stack.hpp          # owns all firmware C state structs
│   ├── producers.hpp               # ObjectiveProducer and its implementations
│   ├── receiver_input.hpp          # synthetic and log-replay receiver sources
│   ├── scenario.hpp                # scenario JSON loading
│   ├── simulator.hpp               # the tick loop
│   └── log_writer.hpp              # JSON Lines: telemetry + ground truth
├── src/                            # implementations of the above
├── apps/
│   └── ofcsim_run.cpp              # CLI: ofcsim run <scenario.json> -o <log.jsonl>
├── tests/                          # GoogleTest
├── vehicles/                       # vehicle parameter files (JSON)
├── scenarios/                      # scenario files (JSON)
├── tools/                          # Python (uv): loader, plots, renderer, validation
├── docs/
│   ├── DEVELOPMENT_GUIDE.md        # this file
│   ├── decisions/                  # short decision records
│   └── validation/                 # one report per validated flight log
└── .github/workflows/ci.yml
```

The simulator is a library (`ofcsim`) plus a thin CLI. Tests link the library directly. Later, Monte Carlo runners and HIL bridges will link the same library.

---

## 5. Build setup

### 5.1 Toolchain and dependencies

- C++20 and C11. CMake 3.25 or newer. Ninja. Clang or GCC.
- **Eigen 3.4 or newer**: vectors, matrices and quaternions. It is header-only. Install it with the system package manager (`brew install eigen` or `apt install libeigen3-dev`), with a `FetchContent` fallback.
- **nlohmann/json**: scenario and vehicle files, and log output.
- **GoogleTest**: unit and scenario tests.
- **Python 3.11+ with uv** for `tools/`: numpy, matplotlib and scipy. This matches the firmware's `host_tools`.

### 5.2 Top-level CMake

```cmake
cmake_minimum_required(VERSION 3.25)
project(ofcsim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING_SAVED ${BUILD_TESTING})
  set(BUILD_TESTING OFF)
  FetchContent_MakeAvailable(eigen)
  set(BUILD_TESTING ${BUILD_TESTING_SAVED})
endif()

FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

include(cmake/FirmwareCore.cmake)          # defines ofc_firmware_core

add_library(ofcsim
  src/rigid_body.cpp src/propulsion.cpp src/contact.cpp src/imu_model.cpp
  src/vehicle_config.cpp src/firmware_stack.cpp src/producers.cpp
  src/receiver_input.cpp src/scenario.cpp src/simulator.cpp src/log_writer.cpp)
target_include_directories(ofcsim PUBLIC include)
target_link_libraries(ofcsim PUBLIC ofc_firmware_core Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_options(ofcsim PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror)

add_executable(ofcsim_run apps/ofcsim_run.cpp)
target_link_libraries(ofcsim_run PRIVATE ofcsim)

include(CTest)
if(BUILD_TESTING)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
  FetchContent_MakeAvailable(googletest)
  add_subdirectory(tests)
endif()
```

### 5.3 Firmware changes the simulator needs

These are small testability changes in `flight-computer-firmware`. Make each one on its own branch with its own commit, and keep the firmware's host tests passing.

| ID | When | Change | Why |
| --- | --- | --- | --- |
| F1 | M0, required | In `cmake/FlightConfiguration.cmake`, resolve paths from `${CMAKE_CURRENT_LIST_DIR}/..` instead of `${PROJECT_SOURCE_DIR}`, and let `OFC_DEFAULT_CONFIGURATION_FILE` be overridden if it is already defined. | The simulator can then `include()` the firmware's own configuration generator. `flight_configuration_defaults()` is then byte-for-byte the firmware's defaults, with nothing copied. |
| F2 | M5, recommended | Move the runtime preparation from `app/flight_configuration_service.c` (`apply_runtime()` and `apply_imu_processing_configuration()`) into a hardware-independent function in the flight layer. Move `BMI270_GYROSCOPE_COUNTS_PER_DPS` into `bmi270_configuration.h`. | Until then, `FirmwareStack` (Section 6.6) has to copy about 40 lines of this logic, and the copy can silently drift from the firmware. |
| F3 | M5, recommended | Move the `vehicle_state_t` builder out of `app/flight_control_task.c` (`read_stabilization_inputs()`) into a pure function. | Same reason. The simulator should call the firmware's own code instead of re-implementing it. |
| F4 | M8, required for exact replay | Make the configuration-snapshot decoder public and hardware-independent. The decoder already exists as static `decode()` in `app/board_flight_configuration_storage.c`. Extracting encode and decode into a pure codec file would do. | Each blackbox log contains the exact configuration the vehicle flew with, as `configuration_snapshot_hex`. The simulator must replay a log with that configuration, not with today's defaults. |
| F5 | Any time | Document the verified motor positions and spin directions (Section 3.4) in the firmware docs. | This is physical truth that the simulator depends on. |

F1 as a sketch:

```cmake
# flight-computer-firmware/cmake/FlightConfiguration.cmake (top of file)
set(OFC_FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
if(NOT DEFINED OFC_DEFAULT_CONFIGURATION_FILE)
  set(OFC_DEFAULT_CONFIGURATION_FILE
      "${OFC_FIRMWARE_ROOT}/config/default-flight-configuration.json")
endif()
# ...and at the bottom, use ${OFC_FIRMWARE_ROOT}/cmake/flight_configuration_defaults.h.in
```

### 5.4 Building the firmware as a host library

The source list below is taken from the firmware's own `tests/CMakeLists.txt` at the reference commit. When the firmware adds a dependency, the linker will say so.

```cmake
# cmake/FirmwareCore.cmake
set(OFC_FW "${CMAKE_CURRENT_SOURCE_DIR}/external/flight-computer-firmware")
include("${OFC_FW}/cmake/FlightConfiguration.cmake")   # needs F1; writes generated/flight_configuration_defaults.h
set(FW "${OFC_FW}/firmware")

add_library(ofc_firmware_core STATIC
  # control core
  ${FW}/flight/control/flight_control_core.c
  ${FW}/flight/control/quad_x_mixer.c
  ${FW}/flight/control/roll_attitude_controller.c
  ${FW}/flight/control/pitch_attitude_controller.c
  ${FW}/flight/control/yaw_attitude_controller.c
  ${FW}/flight/control/rate_pid.c
  ${FW}/flight/control/rate_controller.c
  ${FW}/flight/actuators/motor_command.c
  # manual behavior and input shaping
  ${FW}/flight/behavior/manual_easy/manual_easy_behavior.c
  ${FW}/flight/control/control_curve.c
  ${FW}/flight/control/control_input_shaping.c
  ${FW}/flight/control/takeoff_leveling.c
  # receiver failsafe
  ${FW}/flight/receiver/receiver_failsafe.c
  # IMU processing and estimation
  ${FW}/flight/sensors/imu_sample.c
  ${FW}/flight/sensors/acceleration_filter.c
  ${FW}/flight/sensors/gyro_filter.c
  ${FW}/flight/estimation/accelerometer_attitude.c
  ${FW}/flight/estimation/attitude_estimator.c
  ${FW}/flight/estimation/imu_processing_pipeline.c
  # configuration (defaults and validation)
  ${FW}/flight/configuration/flight_configuration.c
  ${FW}/flight/behavior/common/flight_behavior_configuration.c
  ${FW}/flight/actuators/motor_configuration.c
)
set_target_properties(ofc_firmware_core PROPERTIES
  C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_include_directories(ofc_firmware_core PUBLIC
  ${FW}/flight/control ${FW}/flight/actuators ${FW}/flight/behavior/common
  ${FW}/flight/behavior/manual_easy ${FW}/flight/receiver ${FW}/flight/sensors
  ${FW}/flight/estimation ${FW}/flight/configuration ${FW}/peripherals/bmi270
  ${CMAKE_BINARY_DIR}/generated)
target_compile_options(ofc_firmware_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(ofc_firmware_core PUBLIC m)
```

**The C code compiles as C, not C++.** It uses compound literals and other C11 constructs that are not valid C++. CMake compiles `.c` files with the C compiler automatically. Never rename them.

**Include the headers through `extern "C"`.** The firmware headers have no `__cplusplus` guards, so every inclusion from C++ must be wrapped:

```cpp
// include/ofcsim/firmware_c.hpp — the only place firmware headers are included
#pragma once
extern "C" {
#include "flight_configuration.h"
#include "flight_control_core.h"
#include "control_input_shaping.h"
#include "imu_processing_pipeline.h"
#include "imu_sample.h"
#include "manual_easy_behavior.h"
#include "receiver_failsafe.h"
#include "bmi270_configuration.h"
}
```

### 5.5 CI and sanitizers

- GitHub Actions on ubuntu-latest: check out with submodules, configure, build, run `ctest --output-on-failure`, and run a short smoke scenario.
- A preset that builds with `-fsanitize=address,undefined`. Run it in CI too. The simulator hands many raw C structs across the C/C++ boundary, and the sanitizers catch misuse of them.
- `clang-format` checks the C++ code. Do not reformat the firmware submodule.

---

## 6. Core components, with reference code

The snippets in this section are reference implementations for the parts that are hard to get right. They show structure, signs and pitfalls. Type them in yourself and understand each line (see the working agreement in Section 1). Don't paste them blindly.

### 6.1 Rigid-body state and its derivative

```cpp
// include/ofcsim/rigid_body.hpp
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace ofcsim {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;
using MotorArray = std::array<double, 4>;

inline const Vec3 kGravityNed{0.0, 0.0, 9.80665};   // NED: +Z is down

struct RigidBodyState {
    Vec3 position_m = Vec3::Zero();      // world NED; airborne when z < 0
    Vec3 velocity_mps = Vec3::Zero();    // world NED
    Quat attitude = Quat::Identity();    // rotates body (FRD) vectors into world (NED)
    Vec3 rate_rad_s = Vec3::Zero();      // body FRD, what an ideal gyro would read
    MotorArray rotor_rad_s{};            // actual rotor speeds (they lag the command)
};

struct StateDerivative {
    Vec3 position = Vec3::Zero();        // = velocity
    Vec3 velocity = Vec3::Zero();        // = acceleration (world)
    Vec4 attitude_wxyz = Vec4::Zero();   // quaternion rate, stored explicitly as w,x,y,z
    Vec3 rate = Vec3::Zero();            // angular acceleration (body)
    MotorArray rotor{};                  // rotor angular acceleration
};

}  // namespace ofcsim
```

**The equations of motion.** Each `derivative()` call evaluates them once. RK4 calls it four times per step.

```
ṗ = v
v̇ = (q ⊗ F_body) / m + g_world
q̇ = ½ · q ⊗ (0, ω)
ω̇ = I⁻¹ · (τ_body − ω × (I·ω))
Ω̇ᵢ = (Ω_cmd,ᵢ − Ωᵢ) / τ_motor,ᵢ          (first-order rotor lag)
```

```cpp
// src/plant.cpp (excerpt)
Vec4 quaternion_rate_wxyz(const Quat& q, const Vec3& w_body) {
    // Eigen pitfall: the Quat constructor takes (w, x, y, z), but coeffs() stores (x, y, z, w).
    const Quat p = q * Quat(0.0, w_body.x(), w_body.y(), w_body.z());
    return 0.5 * Vec4(p.w(), p.x(), p.y(), p.z());
}

StateDerivative Plant::derivative(const RigidBodyState& s, const MotorArray& command) const {
    StateDerivative d;
    const Wrench w = propulsion_.wrench(s.rotor_rad_s);           // body-frame force and torque

    for (std::size_t i = 0; i < 4; ++i) {
        const double target = propulsion_.commanded_speed_rad_s(i, command[i]);
        d.rotor[i] = (target - s.rotor_rad_s[i]) / vehicle_.motors[i].time_constant_s;
    }
    d.position = s.velocity_mps;
    d.velocity = s.attitude * w.force_n / vehicle_.mass_kg + kGravityNed;   // q * v rotates v into the world frame
    d.attitude_wxyz = quaternion_rate_wxyz(s.attitude, s.rate_rad_s);
    const Vec3& om = s.rate_rad_s;
    d.rate = vehicle_.inertia_inv_kgm2 * (w.torque_nm - om.cross(vehicle_.inertia_kgm2 * om));
    return d;
}
```

`inertia_inv_kgm2` is computed once when the vehicle is loaded. Never invert a matrix inside the loop.

### 6.2 RK4 integrator

```cpp
// include/ofcsim/rk4.hpp
RigidBodyState advance(const RigidBodyState& s, const StateDerivative& d, double h) {
    RigidBodyState out = s;
    out.position_m += h * d.position;
    out.velocity_mps += h * d.velocity;
    const Vec4 q = Vec4(s.attitude.w(), s.attitude.x(), s.attitude.y(), s.attitude.z())
                   + h * d.attitude_wxyz;
    out.attitude = Quat(q[0], q[1], q[2], q[3]).normalized();    // project back onto unit quaternions
    out.rate_rad_s += h * d.rate;
    for (std::size_t i = 0; i < 4; ++i) out.rotor_rad_s[i] += h * d.rotor[i];
    return out;
}

StateDerivative rk4_average(const StateDerivative& k1, const StateDerivative& k2,
                            const StateDerivative& k3, const StateDerivative& k4) {
    auto avg = [](const auto& a, const auto& b, const auto& c, const auto& e) {
        return (a + 2.0 * b + 2.0 * c + e) / 6.0;
    };
    StateDerivative d;
    d.position = avg(k1.position, k2.position, k3.position, k4.position);
    d.velocity = avg(k1.velocity, k2.velocity, k3.velocity, k4.velocity);
    d.attitude_wxyz = avg(k1.attitude_wxyz, k2.attitude_wxyz, k3.attitude_wxyz, k4.attitude_wxyz);
    d.rate = avg(k1.rate, k2.rate, k3.rate, k4.rate);
    for (std::size_t i = 0; i < 4; ++i)
        d.rotor[i] = avg(k1.rotor[i], k2.rotor[i], k3.rotor[i], k4.rotor[i]);
    return d;
}

template <typename DerivativeFn>
RigidBodyState rk4_step(const RigidBodyState& s, double h, DerivativeFn&& f) {
    const StateDerivative k1 = f(s);
    const StateDerivative k2 = f(advance(s, k1, 0.5 * h));
    const StateDerivative k3 = f(advance(s, k2, 0.5 * h));
    const StateDerivative k4 = f(advance(s, k3, h));
    return advance(s, rk4_average(k1, k2, k3, k4), h);
}
```

Why RK4 rather than explicit Euler: the attitude and rate dynamics oscillate, and explicit Euler adds artificial energy to oscillating systems. At 1 kHz the difference is small but measurable. Section 7.1 has a test that shows it.

Why the quaternion is normalized at every stage: adding `h·q̇` moves the quaternion slightly off unit length. Renormalizing is the standard practical fix. A quaternion that is not unit length stretches the rotation matrix and quietly corrupts the forces.

### 6.3 Propulsion (v1: one combined model per motor)

```cpp
// include/ofcsim/vehicle_config.hpp
struct MotorConfig {
    Vec3 position_m;            // rotor hub relative to the centre of mass, body FRD (see Section 3.4)
    int spin_direction;         // +1: CW seen from above (+ about +Z down), -1: CCW
    double thrust_coeff;        // kT  [N / (rad/s)^2]
    double torque_coeff;        // kQ  [N·m / (rad/s)^2]
    double max_speed_rad_s;     // rotor speed at command 1.0
    double time_constant_s;     // spin-up and spin-down lag
};

struct VehicleConfig {
    double mass_kg;
    Mat3 inertia_kgm2;          // about the centre of mass, body FRD
    Mat3 inertia_inv_kgm2;      // precomputed
    std::array<MotorConfig, 4> motors;   // index = firmware logical motor index
};
```

```cpp
// src/propulsion.cpp
double Propulsion::commanded_speed_rad_s(std::size_t i, double command) const {
    // v1 assumption: rotor speed is proportional to the normalized command (fixed battery voltage).
    // A command of 0 stops the motor. Armed idle (0.05) keeps it spinning slowly.
    return std::clamp(command, 0.0, 1.0) * vehicle_.motors[i].max_speed_rad_s;
}

Wrench Propulsion::wrench(const MotorArray& rotor_rad_s) const {
    Wrench w;
    for (std::size_t i = 0; i < 4; ++i) {
        const MotorConfig& m = vehicle_.motors[i];
        const double speed_sq = rotor_rad_s[i] * rotor_rad_s[i];
        const Vec3 thrust_n(0.0, 0.0, -m.thrust_coeff * speed_sq);       // thrust points up = -Z (FRD)
        w.force_n += thrust_n;
        w.torque_nm += m.position_m.cross(thrust_n);                     // roll and pitch from off-centre thrust
        w.torque_nm.z() += -m.spin_direction * m.torque_coeff * speed_sq; // yaw from the rotor's drag reaction
    }
    return w;
}
```

Sign checks you can do in your head (they are also unit tests):

- For a left motor (`y < 0`), `r × (0, 0, −T)` has an x component of `−y·T > 0`. That is a positive roll (right side down), matching the mixer's `+1` roll coefficient for motors 0 and 1.
- A CW rotor (`spin_direction = +1`) drags the body CCW, so its yaw torque is negative. Increasing the CCW pair (motors 1 and 2) gives positive yaw, as the firmware comment says.

**Worked hover check.** This uses the placeholder vehicle from Section 6.10: mass 0.65 kg, kT = 1.6e-6, and a maximum speed of 3140 rad/s.

- Hover needs `4·kT·Ω² = m·g`, so `Ω = √(0.65 · 9.807 / (4 · 1.6e-6)) ≈ 998 rad/s`.
- That is a motor command of about 998 / 3140 ≈ 0.32.
- Undoing the baseline mapping from Section 3.5 gives a stick throttle of about (0.32 − 0.05) / 0.95 ≈ 0.28.

If your real vehicle hovers far from 28% stick throttle, the placeholder numbers are wrong. That alone is a useful first calibration.

### 6.4 Ground contact and specific force

Without a ground model the vehicle falls through the floor at t = 0, before the motors spin up. The v1 model is deliberately crude. It is a constraint, not a spring.

```cpp
// src/contact.cpp
bool GroundContact::on_ground(const RigidBodyState& s) const {
    return s.position_m.z() >= config_.ground_z_m - 1e-9;      // NED: the ground is z = 0, up is negative
}

void GroundContact::enforce(RigidBodyState& s) const {
    if (s.position_m.z() < config_.ground_z_m) return;          // airborne: nothing to do
    s.position_m.z() = config_.ground_z_m;
    if (s.velocity_mps.z() > 0.0) s.velocity_mps.z() = 0.0;     // cannot move into the ground
    s.velocity_mps.x() = 0.0;                                   // v1: static friction, no sliding
    s.velocity_mps.y() = 0.0;
    s.rate_rad_s.setZero();                                     // v1: no tipping on the legs (see R3)
}

Vec3 GroundContact::constrain_acceleration(const RigidBodyState& s, Vec3 a_world) const {
    if (!on_ground(s)) return a_world;
    if (a_world.z() > 0.0) a_world.z() = 0.0;                   // the normal force cancels net downward acceleration
    a_world.x() = 0.0;
    a_world.y() = 0.0;
    return a_world;
}
```

```cpp
// src/plant.cpp
void Plant::step(const MotorArray& command, double dt_s) {
    state_ = rk4_step(state_, dt_s, [&](const RigidBodyState& s) { return derivative(s, command); });
    contact_.enforce(state_);
    // The IMU needs the acceleration at the end of the step, with the ground's normal force applied.
    const Vec3 a_world = contact_.constrain_acceleration(state_, derivative(state_, command).velocity);
    specific_force_body_mps2_ = state_.attitude.conjugate() * (a_world - kGravityNed);
}
```

This is where liftoff comes from. While `m·g` exceeds the summed thrust, every step pushes `z` above 0 and `enforce()` clamps it back. Once the thrust exceeds the weight, the vertical acceleration becomes negative (upward in NED), `z` goes below 0, and the clamp stops acting. No step contains any logic about liftoff.

Sanity check: at rest and level, `a_world = 0`, so `specific_force_body = (0, 0, −9.807)`. The IMU model turns that into −4096 counts on z, and the firmware estimator reports a roll and pitch of 0.

Known limitations of v1, recorded honestly: no tipping or pivoting on the landing gear, no bounce, no sliding, and no tether. These are listed as roadmap item R3. They matter for validating against tethered hops (Section 9).

### 6.5 IMU model

```cpp
// src/imu_model.cpp
imu_sample_snapshot_t ImuModel::sample(const Vec3& specific_force_body_mps2,
                                       const Vec3& rate_body_rad_s,
                                       uint64_t time_us) {
    constexpr double kStandardGravity = 9.80665;
    constexpr double kAccelCountsPerG = BMI270_ACCELERATION_COUNTS_PER_G;   // 4096, ±8 g range
    constexpr double kGyroCountsPerDps = 16.384;   // ±2000 °/s; firmware keeps this in app/ (see F2)
    constexpr double kRadToDeg = 180.0 / std::numbers::pi;

    std::array<int32_t, 3> accel{}, gyro{};
    for (int a = 0; a < 3; ++a) {
        const double accel_g = specific_force_body_mps2[a] / kStandardGravity
                               + noise_.accel_bias_g[a] + noise_.accel_noise_std_g * unit_(rng_);
        const double gyro_dps = rate_body_rad_s[a] * kRadToDeg
                                + noise_.gyro_bias_dps[a] + noise_.gyro_noise_std_dps * unit_(rng_);
        accel[a] = to_counts(accel_g * kAccelCountsPerG);
        gyro[a] = to_counts(gyro_dps * kGyroCountsPerDps);
    }

    imu_sample_snapshot_t s{};
    s.acceleration_x = accel[0]; s.acceleration_y = accel[1]; s.acceleration_z = accel[2];
    s.gyroscope_x = gyro[0];     s.gyroscope_y = gyro[1];     s.gyroscope_z = gyro[2];
    s.acquired_at_us = time_us;  // must strictly increase, or the pipeline rejects the sample
    s.sequence = ++sequence_;    // must strictly increase, or the rate controller sees "no new sample"
    s.valid = true;
    return s;
}

int32_t ImuModel::to_counts(double value) {
    // The BMI270 registers are int16: clamping here *is* the sensor's saturation.
    const long rounded = std::lround(value);
    return static_cast<int32_t>(std::clamp(rounded, -32768L, 32767L));
}
```

- **Where the noise numbers come from.** Datasheet noise densities are only the lower bound. On a flying quad, motor vibration dominates the noise. Fit `accel_noise_std_g` and `gyro_noise_std_dps` from blackbox segments: disarmed on the ground for the sensor floor, and armed at idle on the ground for the vibration floor. Record both in the vehicle file.
- **Determinism.** Seed `std::mt19937_64` from the scenario seed, so the same scenario and seed give identical runs on the same platform. Be aware that `std::normal_distribution` is not required to produce identical sequences across standard libraries, and libstdc++ and libc++ do differ. Cross-platform golden tests must therefore compare metrics within tolerances, never exact samples. For bit-exact output everywhere, write your own Box–Muller transform.
- **Sensor axes.** v1 produces body-axis samples directly. Roadmap R4 generates raw sensor-axis samples instead and runs them through the real `imu_map_raw_sample()` with the V1 mapping (+X, −Y, −Z), which exercises that firmware code as well.

### 6.6 FirmwareStack: owning the firmware's state

`FirmwareStack` owns every firmware C struct and prepares them the same way `apply_runtime()` and `apply_imu_processing_configuration()` do in `app/flight_configuration_service.c`. Until F2 lands, this is a copy of that logic. Keep a comment pointing to the source, and re-check it whenever the submodule is bumped.

```cpp
// include/ofcsim/firmware_stack.hpp
class FirmwareStack {
public:
    explicit FirmwareStack(const flight_configuration_t& configuration);

    // core_.rate_controller points into this object, so copying or moving it would leave a dangling pointer.
    FirmwareStack(const FirmwareStack&) = delete;
    FirmwareStack& operator=(const FirmwareStack&) = delete;
    FirmwareStack(FirmwareStack&&) = delete;
    FirmwareStack& operator=(FirmwareStack&&) = delete;

    void process_imu(const imu_sample_snapshot_t& sample, imu_freshness_t freshness);
    vehicle_state_t vehicle_state() const;
    flight_control_core_result_t run_core(const control_objective_t& objective,
                                          const vehicle_state_t& state, uint64_t now_us,
                                          flight_control_output_t& output);
    void reset_core() { flight_control_core_reset(&core_); }

    const flight_configuration_t& configuration() const { return config_; }
    const prepared_control_input_shaping_t& prepared_control() const { return control_; }
    const imu_processing_pipeline_t& imu_pipeline() const { return imu_; }

private:
    flight_configuration_t config_{};
    prepared_control_input_shaping_t control_{};
    prepared_quad_x_mixer_t mixer_{};
    prepared_control_profile_t profile_{};
    rate_controller_t rate_controller_{};
    flight_control_core_t core_{};
    imu_processing_pipeline_t imu_{};
};
```

```cpp
// src/firmware_stack.cpp — mirrors app/flight_configuration_service.c at 3174bbe
FirmwareStack::FirmwareStack(const flight_configuration_t& configuration) : config_(configuration) {
    if (!flight_configuration_is_valid(&config_))
        throw std::invalid_argument("flight configuration failed firmware validation");

    const float maximum_rate_dps[RATE_CONTROLLER_AXIS_COUNT] = {
        config_.control.roll.maximum_rate_dps,
        config_.control.pitch.maximum_rate_dps,
        config_.control.yaw.maximum_rate_dps,
    };
    // C++20 designated initializers must follow declaration order, as they do here.
    const imu_processing_config_t imu_config = {
        .acceleration_filter = {.type = ACCELERATION_FILTER_FIRST_ORDER_LOW_PASS,
                                .cutoff_hz = config_.acceleration_filter.cutoff_hz},
        .gyro_filter = {.type = GYRO_FILTER_FIRST_ORDER_LOW_PASS,
                        .cutoff_hz = config_.gyro_filter.cutoff_hz},
        .attitude_estimator = {.type = ATTITUDE_ESTIMATOR_COMPLEMENTARY,
                               .accelerometer_correction_time_constant_s =
                                   config_.attitude_estimator.accelerometer_correction_time_constant_s},
        .maximum_gap_us = config_.attitude_estimator.maximum_gap_us,
        .acceleration_counts_per_g = BMI270_ACCELERATION_COUNTS_PER_G,
        .gyroscope_counts_per_dps = 16.384F,
        .level_roll_trim_degrees =
            config_.level_calibration.calibrated ? config_.level_calibration.roll_trim_degrees : 0.0F,
        .level_pitch_trim_degrees =
            config_.level_calibration.calibrated ? config_.level_calibration.pitch_trim_degrees : 0.0F,
    };

    const bool ok =
        control_input_shaping_prepare(&config_.control, &control_) &&
        quad_x_mixer_prepare(config_.propeller_layout, &mixer_) &&
        flight_control_profile_prepare(&config_.roll_attitude_controller,
                                       &config_.pitch_attitude_controller, maximum_rate_dps,
                                       &mixer_, easy_mode_armed_idle(&config_.easy_mode), &profile_) &&
        rate_controller_initialize(&rate_controller_, &config_.rate_controller) &&
        flight_control_core_initialize(&core_, &rate_controller_) &&
        imu_processing_pipeline_initialize(&imu_, &imu_config);
    if (!ok) throw std::runtime_error("firmware runtime preparation failed");
}

void FirmwareStack::process_imu(const imu_sample_snapshot_t& sample, imu_freshness_t freshness) {
    // v1 passes a zero gyro bias. R4 runs the firmware's real gyro calibration during a simulated still phase.
    static constexpr int32_t kZeroBias[3] = {0, 0, 0};
    (void)imu_processing_pipeline_process(&imu_, &sample, freshness, kZeroBias);
}

vehicle_state_t FirmwareStack::vehicle_state() const {
    // Mirrors read_stabilization_inputs() in app/flight_control_task.c (see F3).
    attitude_snapshot_t a{};
    vehicle_state_t v{};
    if (!imu_processing_pipeline_latest(&imu_, &a) || !a.valid) return v;   // v.valid stays false
    v.attitude_degrees[RATE_CONTROLLER_AXIS_ROLL] = a.roll_degrees;
    v.attitude_degrees[RATE_CONTROLLER_AXIS_PITCH] = a.pitch_degrees;       // no yaw attitude: the firmware has none
    for (int axis = 0; axis < RATE_CONTROLLER_AXIS_COUNT; ++axis)
        v.angular_rate_dps[axis] = a.filtered_gyroscope_dps[axis];
    v.acquired_at_us = a.acquired_at_us;
    v.source_sequence = a.source_sequence;
    v.valid = true;
    return v;
}

flight_control_core_result_t FirmwareStack::run_core(const control_objective_t& objective,
                                                     const vehicle_state_t& state, uint64_t now_us,
                                                     flight_control_output_t& output) {
    return flight_control_update(&core_, &profile_, state.valid ? &state : nullptr,
                                 &objective, now_us, &output);
}
```

Getting a configuration: `flight_configuration_t c; flight_configuration_defaults(&c);` gives exactly the firmware's defaults (after F1). For replaying a log, decode `configuration_snapshot_hex` with the F4 decoder.

### 6.7 Objective producers

```cpp
// include/ofcsim/producers.hpp
enum class ProducerStatus {
    Objective,   // run the core with this objective
    Hold,        // no new command this tick; motors keep the previous command (firmware: WAITING_FOR_STATE)
    Stop,        // failsafe stop: all motors 0 (firmware: STOP_REQUESTED, INVALID_STATE, invalid control)
    Idle,        // behavior inactive: core reset, motors 0 (firmware: INACTIVE)
};

struct ProducerOutput {
    ProducerStatus status = ProducerStatus::Idle;
    control_objective_t objective{};
};

// Everything a producer may know. No ground truth, on purpose: real behaviors don't have it.
struct ProducerContext {
    uint64_t now_us;
    const vehicle_state_t* vehicle_state;   // nullptr while the estimate is invalid
    imu_freshness_t imu_freshness;
};

class ObjectiveProducer {
public:
    virtual ~ObjectiveProducer() = default;
    virtual ProducerOutput update(const ProducerContext& context) = 0;
    virtual std::string_view name() const = 0;
};
```

**Manual producer.** This wraps the real receiver failsafe and the real manual behavior. The status mapping mirrors `execute_manual_easy_behavior()` in `app/flight_control_task.c`.

```cpp
class ManualProducer final : public ObjectiveProducer {
public:
    ManualProducer(const FirmwareStack& firmware, std::unique_ptr<ReceiverInput> input, uint64_t start_us)
        : firmware_(firmware), input_(std::move(input)) {
        if (!receiver_failsafe_initialize(&failsafe_, &firmware_.configuration().receiver_failsafe, start_us) ||
            !manual_easy_behavior_initialize(&behavior_, &firmware_.configuration().easy_mode))
            throw std::runtime_error("manual producer initialization failed");
    }

    ProducerOutput update(const ProducerContext& ctx) override {
        const receiver_control_snapshot_t rc = input_->at(ctx.now_us);
        (void)receiver_failsafe_update(&failsafe_, &rc, ctx.now_us, &last_decision_);

        ProducerOutput out;
        const flight_behavior_result_t result = manual_easy_behavior_update(
            &behavior_, &firmware_.prepared_control(), &firmware_.configuration().easy_mode,
            &last_decision_, ctx.vehicle_state, ctx.imu_freshness, ctx.now_us, &out.objective);

        switch (result) {
        case FLIGHT_BEHAVIOR_OBJECTIVE_READY:   out.status = ProducerStatus::Objective; break;
        case FLIGHT_BEHAVIOR_WAITING_FOR_STATE: out.status = ProducerStatus::Hold; break;
        case FLIGHT_BEHAVIOR_INACTIVE:          out.status = ProducerStatus::Idle; break;
        default:                                out.status = ProducerStatus::Stop; break;
        }
        return out;
    }

    std::string_view name() const override { return "manual_easy"; }
    const receiver_failsafe_decision_t& last_decision() const { return last_decision_; }
    const manual_easy_behavior_t& behavior() const { return behavior_; }   // for logging the setpoint

private:
    const FirmwareStack& firmware_;
    std::unique_ptr<ReceiverInput> input_;
    receiver_failsafe_t failsafe_{};
    manual_easy_behavior_t behavior_{};
    receiver_failsafe_decision_t last_decision_{};
};
```

The receiver input must reproduce how the firmware's receiver service presents the link. A new frame advances `received_at_us` and `source_sequence`. A lost link means both stop advancing while the last snapshot is still being reported. **Check this against `receiver_failsafe.c` and the receiver service before building link-loss scenarios**, because the failsafe's timing depends on it.

```cpp
class ReceiverInput {
public:
    virtual ~ReceiverInput() = default;
    virtual receiver_control_snapshot_t at(uint64_t now_us) = 0;
};
// SyntheticReceiver: keyframes {t, roll, pitch, yaw, throttle, arm} held between frames,
//                    a configurable frame period, and optional link-loss windows.
// LogReplayReceiver: the decoded blackbox "receiver" values at 100 Hz, held between samples
//                    (M8). Check the order of the four values in the firmware's diagnostics capture.
```

**Scripted producer.** It commands the core directly and bypasses the receiver and behavior layers. Use it to characterize the controller.

```cpp
struct ObjectiveKeyframe {
    uint64_t at_us;
    std::array<control_objective_axis_mode_t, 3> mode;   // roll, pitch, yaw
    std::array<float, 3> value;                          // degrees for ANGLE, °/s for RATE
    float throttle;                                      // 0..1 (stick throttle, before the armed-idle mapping)
};

ProducerOutput ScriptedProducer::update(const ProducerContext& ctx) {
    const ObjectiveKeyframe& k = keyframe_at(ctx.now_us);   // the last keyframe with at_us <= now (held)
    ProducerOutput out{ProducerStatus::Objective, {}};
    for (int a = 0; a < 3; ++a) {
        out.objective.axis_mode[a] = k.mode[a];
        out.objective.axis_value[a] = k.value[a];
    }
    out.objective.throttle = k.throttle;
    out.objective.produced_at_us = ctx.now_us;      // the core requires produced <= now <= valid_until
    out.objective.valid_until_us = ctx.now_us;
    out.objective.valid = true;
    return out;
}
```

**Stub autonomous producer.** This is a placeholder for a real autonomous firmware behavior. It may only use `vehicle_state`, so its limits are the same ones a real on-board behavior would have.

```cpp
ProducerOutput StubAutonomousProducer::update(const ProducerContext& ctx) {
    if (ctx.vehicle_state == nullptr) return {ProducerStatus::Hold, {}};
    const vehicle_state_t& v = *ctx.vehicle_state;
    // Simple on-board safety rule, using estimated state only.
    if (std::abs(v.attitude_degrees[0]) > 45.0F || std::abs(v.attitude_degrees[1]) > 45.0F)
        return {ProducerStatus::Stop, {}};

    const double t_s = (ctx.now_us - armed_at_us_) * 1e-6;
    float throttle = 0.0F;
    if (t_s < ramp_s_)                          throttle = static_cast<float>(hover_throttle_ * t_s / ramp_s_);
    else if (t_s < ramp_s_ + hold_s_)           throttle = hover_throttle_;
    else if (t_s < 2 * ramp_s_ + hold_s_)       throttle = static_cast<float>(hover_throttle_ * (2 * ramp_s_ + hold_s_ - t_s) / ramp_s_);

    ProducerOutput out{ProducerStatus::Objective, {}};
    out.objective.axis_mode[0] = CONTROL_OBJECTIVE_AXIS_ANGLE;  out.objective.axis_value[0] = 0.0F;
    out.objective.axis_mode[1] = CONTROL_OBJECTIVE_AXIS_ANGLE;  out.objective.axis_value[1] = 0.0F;
    out.objective.axis_mode[2] = CONTROL_OBJECTIVE_AXIS_RATE;   out.objective.axis_value[2] = 0.0F;
    out.objective.throttle = throttle;
    out.objective.produced_at_us = out.objective.valid_until_us = ctx.now_us;
    out.objective.valid = true;
    return out;
}
```

When the firmware gets its real behavior arbitration and autonomous behaviors, they replace these producers: link the new C modules and wrap them as the manual behavior is wrapped. See R9.

### 6.8 The simulator loop

```cpp
// src/simulator.cpp
void Simulator::run() {
    while (now_us_ < end_us_) tick();
}

void Simulator::tick() {
    // 1. Sense: the IMU samples the true state at t.
    const imu_sample_snapshot_t imu = imu_.sample(plant_.specific_force_body_mps2(),
                                                  plant_.state().rate_rad_s, now_us_);
    const imu_freshness_t freshness = fault_injector_.imu_freshness(now_us_);   // FRESH unless a fault is scripted
    firmware_.process_imu(imu, freshness);
    const vehicle_state_t estimate = firmware_.vehicle_state();

    // 2. Decide and 3. control (only after the scenario's arm time; before that the motors are stopped).
    flight_control_output_t core_out{};
    if (now_us_ >= arm_at_us_) {
        const ProducerOutput p = producer_->update({now_us_, estimate.valid ? &estimate : nullptr, freshness});
        switch (p.status) {
        case ProducerStatus::Objective:
            if (firmware_.run_core(p.objective, estimate, now_us_, core_out) == FLIGHT_CONTROL_CORE_UPDATED
                && core_out.mixer_output_valid) {
                for (std::size_t i = 0; i < 4; ++i)
                    command_[i] = core_out.mixer_output.command.throttle[i];
            } else {
                command_.fill(0.0);                // the firmware treats a core error as a failsafe entry
                firmware_.reset_core();
            }
            break;
        case ProducerStatus::Hold: break;          // keep the previous command
        case ProducerStatus::Stop:
        case ProducerStatus::Idle:
            command_.fill(0.0);
            firmware_.reset_core();
            break;
        }
    }

    // 4. Actuate and integrate: hold the commands across the control period, split into physics substeps.
    const double h = control_period_s_ / physics_substeps_;
    for (int k = 0; k < physics_substeps_; ++k) plant_.step(command_, h);

    // 5. Log.
    if (logger_.due(now_us_)) logger_.write(now_us_, plant_, firmware_, *producer_, core_out, command_);

    // 6. Advance time.
    now_us_ += control_period_us_;
}
```

Two timing assumptions are made on purpose here. They are listed as roadmap item R5 and should be recorded in the README.

- The IMU sample and the control computation happen at the same instant, with no computation delay and no actuation delay.
- The IMU rate equals the control rate.

### 6.9 Log format (JSON Lines)

- The first line is a header object. Every following line is one sample.
- Each sample has two top-level sections, `telemetry` and `ground_truth`, so the difference between the two stays explicit (Section 2.1).
- **`telemetry`** uses the same keys as the firmware's decoded blackbox samples (`host_tools/openflightcomputer/blackbox.py`, function `_sample()`), so a single loader can read both. Only fill fields the simulator really has. The values come from the pipeline observation (`imu_processing_pipeline_latest_observation()`), the manual behavior's `last_setpoint`, and the core output.
- **`ground_truth`** holds only simulation quantities. No real sensor measures them.

```json
{"type":"header","schema_version":1,"scenario":"roll_step_10deg","seed":42,
 "firmware_commit":"3174bbe","vehicle":{"...":"full vehicle config"},
 "flight_configuration_source":"firmware_default","control_period_us":1000,"physics_substeps":4}
{"type":"sample","timestamp_us":1000,
 "telemetry":{"imu_sequence":1,"raw_acceleration":[0,0,-4096],"raw_gyroscope":[0,0,0],
   "filtered_acceleration_g":[0,0,-1],"filtered_gyroscope_dps":[0,0,0],
   "accelerometer_attitude_degrees":[0,0],"gyro_predicted_attitude_degrees":[0,0],
   "attitude_degrees":[0,0],"accelerometer_weight":0.002,
   "setpoint":[0,0,0,0],"desired_rates_dps":[0,0,0],
   "pid":[[0,0,0,0],[0,0,0,0],[0,0,0,0]],"motors":[0,0,0,0],"motor_baseline":0,
   "producer":"scripted","producer_status":"idle"},
 "ground_truth":{"position_m":[0,0,0],"velocity_mps":[0,0,0],"attitude_wxyz":[1,0,0,0],
   "euler_deg":[0,0,0],"rate_dps":[0,0,0],"rotor_rad_s":[0,0,0,0],"thrust_n":[0,0,0,0],
   "specific_force_g":[0,0,-1],"estimator_error_deg":[0,0],"on_ground":true}}
```

`estimator_error_deg` is the estimated roll and pitch minus the true roll and pitch. It is one of the most informative quantities the simulator produces (Section 9.4).

Converting a quaternion to Euler angles. Don't use `Eigen::Matrix3d::eulerAngles()` for this, because it returns angles in ranges that can flip. Use the explicit ZYX formulas:

```cpp
Vec3 euler_zyx_deg(const Quat& q) {   // returns {roll, pitch, yaw}
    const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    const double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return Vec3(roll, pitch, yaw) * (180.0 / std::numbers::pi);
}
```

Log rate: 1 kHz by default. 100 Hz matches the blackbox rate and is useful for side-by-side comparison. At 1 kHz a 10 s run is about 10k lines, which is fine. For Monte Carlo (R7), switch to a binary format such as Parquet or a flat binary file.

### 6.10 Scenario and vehicle files

The vehicle file below uses **placeholder values for a typical 5-inch quad**. Replace every value with a measurement or a sourced estimate, and record the source (Section 11).

```json
{
  "schema_version": 1,
  "name": "ofc_v1_5inch_placeholder",
  "mass_kg": 0.65,
  "inertia_kgm2": [[0.0030, 0, 0], [0, 0.0035, 0], [0, 0, 0.0060]],
  "arm_radius_m": 0.1125,
  "motors": [
    {"corner": "front_left",  "spin_direction":  1},
    {"corner": "rear_left",   "spin_direction": -1},
    {"corner": "front_right", "spin_direction": -1},
    {"corner": "rear_right",  "spin_direction":  1}
  ],
  "motor_defaults": {
    "thrust_coeff": 1.6e-6,
    "torque_coeff": 2.5e-8,
    "max_speed_rad_s": 3140,
    "time_constant_s": 0.03
  },
  "imu_noise": {
    "accel_noise_std_g": 0.02, "gyro_noise_std_dps": 0.5,
    "accel_bias_g": [0, 0, 0], "gyro_bias_dps": [0, 0, 0]
  },
  "sources": {
    "mass_kg": "TODO: kitchen scale incl. battery",
    "thrust_coeff": "TODO: motor/prop datasheet, later fitted from a hop (M8)"
  }
}
```

The loader turns `corner` and `arm_radius_m` into `position_m` using Section 3.4, and precomputes the inverse inertia. Keep `sources` in the file. It is how the simulator shows which values are measured and which are guessed.

```json
{
  "schema_version": 1,
  "name": "roll_step_10deg",
  "seed": 42,
  "duration_s": 6.0,
  "control_period_us": 1000,
  "physics_substeps": 4,
  "log_rate_hz": 1000,
  "vehicle": "vehicles/ofc_v1_5inch_placeholder.json",
  "flight_configuration": {"source": "firmware_default"},
  "initial_state": {"position_m": [0, 0, 0], "euler_deg": [0, 0, 0]},
  "arm_at_s": 0.5,
  "producer": {
    "type": "scripted",
    "keyframes": [
      {"t_s": 0.5, "modes": ["angle", "angle", "rate"], "values": [0, 0, 0], "throttle": 0.35},
      {"t_s": 3.0, "modes": ["angle", "angle", "rate"], "values": [10, 0, 0], "throttle": 0.35}
    ]
  },
  "faults": []
}
```

The CLI should accept overrides, for example `--set motor_defaults.thrust_coeff=1.8e-6`. The parameter fit in M8 needs this, and it avoids ever having to edit files from inside a script.

### 6.11 Python tools (`tools/`, uv project)

**Loader.** A single function reads both simulator logs and decoded blackbox logs:

```python
# tools/ofcsim_tools/load.py
import json
from pathlib import Path
import pandas as pd

def load_run(path: Path) -> tuple[dict, pd.DataFrame, pd.DataFrame | None]:
    """Return (header, telemetry, ground_truth). ground_truth is None for real flights."""
    if path.suffix == ".jsonl":                                   # simulator log
        lines = path.read_text().splitlines()
        header = json.loads(lines[0])
        rows = [json.loads(line) for line in lines[1:]]
        t = [r["timestamp_us"] for r in rows]
        tel = pd.json_normalize([r["telemetry"] for r in rows]).assign(timestamp_us=t)
        gt = pd.json_normalize([r["ground_truth"] for r in rows]).assign(timestamp_us=t)
        return header, tel, gt
    doc = json.loads(path.read_text())                            # `./ofc flight-log decode` output
    header = {k: v for k, v in doc.items() if k != "samples"}
    return header, pd.DataFrame(doc["samples"]), None
```

**Offline 3D renderer.** It uses ground-truth position and attitude:

```python
# tools/ofcsim_tools/render.py
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

def quat_to_matrix(w, x, y, z):
    """Rotation matrix for a body->world quaternion (scalar first, like the log's attitude_wxyz).
    Pitfall: scipy's Rotation.from_quat expects scalar LAST by default."""
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def ned_to_plot(v):
    """NED -> plot axes (x = east, y = north, z = up) so the plot reads naturally."""
    return np.array([v[1], v[0], -v[2]])

def render(gt, arm_m=0.1125, fps=30, out="run.mp4"):
    a = arm_m / np.sqrt(2)
    motors_body = np.array([[a, -a, 0], [-a, -a, 0], [a, a, 0], [-a, a, 0]])   # Section 3.4 order
    frames = gt.iloc[:: max(1, int(round(1e6 / fps / gt.timestamp_us.diff().median())))]

    fig = plt.figure(figsize=(6, 6)); ax = fig.add_subplot(projection="3d")
    arm1, = ax.plot([], [], [], lw=3); arm2, = ax.plot([], [], [], lw=3)
    nose, = ax.plot([], [], [], "o"); trail, = ax.plot([], [], [], lw=1, alpha=0.5)
    trail_pts = []

    def draw(i):
        row = frames.iloc[i]
        R = quat_to_matrix(*row["attitude_wxyz"]); p = np.asarray(row["position_m"])
        m = np.array([ned_to_plot(p + R @ mb) for mb in motors_body])
        arm1.set_data_3d(*m[[0, 3]].T); arm2.set_data_3d(*m[[1, 2]].T)   # the two diagonals
        n = ned_to_plot(p + R @ np.array([arm_m, 0, 0])); nose.set_data_3d([n[0]], [n[1]], [n[2]])
        trail_pts.append(ned_to_plot(p)); trail.set_data_3d(*np.array(trail_pts).T)
        c = ned_to_plot(p); ax.set_xlim(c[0]-0.5, c[0]+0.5); ax.set_ylim(c[1]-0.5, c[1]+0.5); ax.set_zlim(0, 1.0)
        return arm1, arm2, nose, trail

    FuncAnimation(fig, draw, frames=len(frames), interval=1000 / fps).save(out, writer=FFMpegWriter(fps=fps))
```

Use the render as a debugging tool as well as a demo. A sign error in the geometry is visible within a second of watching the motion.

---

## 7. Testing strategy

There are four layers of tests. Every milestone in Section 8 ends with its tests passing in CI.

### 7.1 Physics unit tests (analytic checks)

Each test compares the plant against a closed-form solution. These tests are what make the simulator trustworthy, so write them before the code they test.

| Test | Setup | Expected |
| --- | --- | --- |
| Free fall | no thrust, no contact, start at rest | `z(t) = ½·g·t²` within 1e-6 m after 1 s |
| Constant thrust climb | thrust fixed at 1.5·m·g, motor lag disabled, level | `z(t) = −¼·g·t²` (net upward acceleration of ½·g) |
| Constant torque spin-up | torque τ about body x only, zero thrust, gravity off | `ω_x(t) = τ/I_xx · t`, other axes stay 0 |
| Torque-free tumbling | asymmetric inertia, initial ω on all axes, gravity off | the magnitude of the world-frame angular momentum and the rotational kinetic energy are conserved (drift below 1e-6 over 10 s at dt = 1 ms) |
| Quaternion norm | any long run | `abs(norm(q) − 1) < 1e-12` after every step |
| RK4 vs Euler | torque-free tumbling | RK4's energy drift is at least 100× smaller than a reference explicit-Euler step (this documents why RK4 is used) |
| Motor lag | step command | the rotor reaches 63.2% of the step after `τ_motor` |
| Specific force at rest | on the ground, level; then static roll of 30° and pitch of 20° | `f_body/g = (0, 0, −1)`, and the real `accelerometer_attitude_calculate()` returns the true roll and pitch within 0.01° |

### 7.2 Consistency tests between the firmware and the plant

These tests catch the most expensive kind of bug: a sign or ordering mismatch between the real firmware and the simulated vehicle.

- **Mixer ↔ plant sign test (the single most important test).**
  1. Start at hover rotor speeds.
  2. For each axis (roll, pitch, yaw) and each sign (±), apply a small unit correction through the real `quad_x_mixer_apply_prepared()`.
  3. Turn the resulting four motor commands into rotor speeds, then compute `Propulsion::wrench()`.
  4. Assert that the torque on that axis has the same sign as the correction, and that the other two axes change by less than 1% of it.
- **IMU round trip.** Put in a known true state, run the IMU model, then run the real pipeline for 2 s while the vehicle stays still. The estimated roll and pitch must converge to the true values.
- **Frame round trip.** Convert a quaternion to Euler angles and back for random attitudes, away from ±90° pitch.

### 7.3 Closed-loop scenario tests

These run whole scenarios and check metrics, not individual samples.

- **Hover hold.** Scripted producer, level objective, hover throttle. The altitude drift is bounded in the chosen direction, the attitude stays within ±1°, and nothing becomes NaN.
- **Roll step 10°.** Report the rise time, the overshoot and the settling time to within 2%, and assert bounds on them. The first run sets the bounds, and each is recorded with the firmware commit it came from.
- **Yaw rate step.** Check that the rate is tracked and that roll and pitch are disturbed only within a bound.
- **Receiver loss during hover.** Manual producer with a synthetic loss window. The failsafe must go through the states in the right order: hold last, then stage one (throttle 0.05 and level), then stage two stop. The timings are taken from the configuration in `receiver_failsafe`.
- **Takeoff leveling.** Start tilted by a few degrees on the ground. The manual behavior's takeoff leveling must bring the vehicle level after the first throttle input that isn't zero.
- **Determinism.** Two runs with the same seed must produce identical logs on the same platform.

### 7.4 Golden metrics

A small `tests/golden/*.json` file stores the metric bounds for each scenario. After a firmware bump (a submodule update), CI shows which metrics moved. That turns the simulator into a regression test for firmware changes, which is a significant benefit of the whole project.

---

## 8. Milestones

The effort figures include learning time. Every milestone ends with a definition of done (DoD) and something you can publish, such as a README update, a plot or a blog post.

### M0 — Repository, build and C/C++ interop (8–12 h)

Goal: the real firmware code runs inside a C++ test.

- [ ] Create the repository and add `flight-computer-firmware` as a submodule pinned to a commit. Add the license and a README skeleton.
- [ ] Firmware change F1 (Section 5.3), merged upstream.
- [ ] `ofc_firmware_core` builds with `-Werror`. `ofcsim` builds as an empty library.
- [ ] GoogleTest is wired up. One test calls `flight_configuration_defaults()` and checks that the result passes `flight_configuration_is_valid()`. A second test reproduces one case from the firmware's own `flight_control_core_tests.c`, called from C++.
- [ ] Debug, release and asan-ubsan presets. CI runs build and tests. clang-format is set up.
- **DoD:** CI is green, and running `ctest` executes real firmware code from C++.
- **C++ to learn here:** `extern "C"` and linkage, CMake targets and properties, RAII, `std::array`, value semantics versus pointers.

### M1 — Rigid body and RK4 (15–20 h)

Goal: a vehicle that falls and spins correctly.

- [ ] `RigidBodyState`, `StateDerivative`, `derivative()`, `advance()`, `rk4_step()` (Sections 6.1 and 6.2).
- [ ] `euler_zyx_deg()`, plus quaternion helpers with their own tests.
- [ ] All the analytic tests in Section 7.1 except the motor and specific-force ones.
- **DoD:** the analytic tests pass. `docs/decisions/0001-frames-and-units.md` records the frame and unit conventions.
- **C++ to learn here:** Eigen basics and the pitfalls noted in the code comments, templates with a callable parameter, `constexpr`, lambdas.

### M2 — Propulsion, geometry and vehicle files (10–15 h)

Goal: motor commands become the right forces and torques.

- [ ] `VehicleConfig` JSON loading, including the corner-to-position mapping and precomputed inverse inertia.
- [ ] `Propulsion` (Section 6.3), with the rotor lag inside the RK4 state.
- [ ] The mixer ↔ plant sign test (Section 7.2), the motor lag test, and a hover-thrust test (the analytic hover command matches the worked example in Section 6.3).
- [ ] Section 3.4 checked against the real airframe (F5 started).
- **DoD:** the sign test passes against the real firmware mixer, and the placeholder values are marked as placeholders in the vehicle file.

### M3 — Ground contact and open-loop liftoff (5–8 h)

Goal: the vehicle rests on the ground and lifts off when it should.

- [ ] `GroundContact` and specific force (Section 6.4).
- [ ] Open-loop tests with no firmware yet: a command below the hover command keeps the vehicle on the ground indefinitely, and a command above it produces liftoff. Check the time to reach 0.5 m against the analytic value, with motor lag disabled in the test.
- **DoD:** tests pass. The first plot of altitude against time goes in the README.

### M4 — IMU model (8–12 h)

Goal: the firmware estimator sees realistic sensor data.

- [ ] `ImuModel` (Section 6.5): scale, bias, seeded noise, quantization and saturation.
- [ ] Tests: counts at rest, static tilt through the real `accelerometer_attitude_calculate()`, saturation clamping, and identical output for the same seed.
- [ ] A first noise fit from a disarmed and an idle blackbox segment. It can be a simple standard deviation calculated in Python.
- **DoD:** the IMU round-trip test from Section 7.2 passes through the real pipeline.

### M5 — Firmware in the loop, scripted producer (15–20 h)

Goal: the real flight software stabilizes the simulated vehicle. This is the first result worth publishing.

- [ ] `FirmwareStack` (Section 6.6). Do F2 and F3 upstream if you have the time, otherwise keep the mirrored code with a comment pointing to its source.
- [ ] `ObjectiveProducer`, `ScriptedProducer` and the `Simulator` loop (Sections 6.7 and 6.8), with arming at a set time.
- [ ] `LogWriter` in JSON Lines (Section 6.9). `ofcsim_run` CLI with `--set` overrides.
- [ ] Scenario tests: hover hold, roll step and yaw rate step (Section 7.3).
- [ ] Quick plots of true attitude against estimated attitude against setpoint.
- **DoD:** the scenario tests pass in CI, and the README shows a roll-step plot generated by the real firmware controller.
- **Expected first finding:** the firmware estimator's gyro prediction adds the body rates `p` and `q` directly to roll and pitch. The exact Euler-rate kinematics are `φ̇ = p + (q·sinφ + r·cosφ)·tanθ` and `θ̇ = q·cosφ − r·sinφ`, so the prediction is exact only near level or without rotation about the other axes, for example yaw rate while banked. The accelerometer correction limits the error, but it doesn't remove it. The simulator can now measure that error (`estimator_error_deg`). Document it. This is exactly the kind of finding the simulator exists for.
- **Publish:** blog post "Running my flight controller's real firmware in a simulator".

### M6 — Manual producer, receiver path and failsafe (8–12 h)

Goal: the full manual stack, including the real receiver failsafe, runs in the loop.

- [ ] `ReceiverInput` interface and `SyntheticReceiver`, after confirming the link-loss semantics (Section 6.7).
- [ ] `ManualProducer` wrapping the real `receiver_failsafe_update()` and `manual_easy_behavior_update()`.
- [ ] Scenario tests: receiver loss during hover, and takeoff leveling.
- **DoD:** the failsafe sequence is checked against the configured timings in CI.
- **Publish:** a failsafe demo as a plot and a render.

### M7 — Analysis tools and offline renderer (8–12 h)

Goal: every run can be inspected and turned into a video with one command.

- [ ] `tools/` uv project: the loader that reads both formats, standard plots (attitude, rates, motors, estimator error, altitude), and the renderer (Section 6.11).
- [ ] `ofcsim_tools plot <log>` and `ofcsim_tools render <log>`.
- **DoD:** the README contains an embedded clip of a scenario, rendered from a simulator log.

### M8 — Validation against a real hop (15–25 h)

Goal: show how close the simulator comes to the real vehicle, and fit the parameters that are still guessed. The method is in Section 9.

- [ ] Firmware change F4, so the exact configuration from the log can be used.
- [ ] `LogReplayReceiver` and a replay scenario type (initial attitude and arm time taken from the log).
- [ ] Comparison script with metrics and plots, and a parameter fit.
- [ ] `docs/validation/<log-id>.md` with honest results, including where the model is wrong and why.
- **DoD:** at least one validation report for a real hop, and fitted parameters recorded in the vehicle file together with their source.
- **Publish:** blog post "How close is my simulator to my real drone?"

### M9 — Stub autonomy, scenario library and regression suite (8–12 h)

Goal: the simulator becomes a lasting test tool for the firmware.

- [ ] `StubAutonomousProducer` (Section 6.7).
- [ ] A scenario library covering hover, steps in each axis, failsafe, takeoff leveling, and a disturbance torque pulse.
- [ ] Golden metrics (Section 7.4), checked in CI.
- [ ] A final README: architecture diagram, results, the validation summary, the rendered clip, "How this was built", and known limitations.
- **DoD:** a firmware submodule bump in CI reports changed metrics automatically.

**Total: about 110–160 h.** M5 is the first point at which the project can be shown. M8 is what makes it credible.

---

## 9. Validating against real flights

### 9.1 What to compare

Compare **estimated state with estimated state**. The real log contains the firmware's *estimates* (`attitude_degrees`, `filtered_gyroscope_dps`) and its *commands* (`motors`), but no true state. So the valid comparison is simulated estimate against logged estimate, with both produced by the same estimator code. Simulated ground truth takes part only in the second step, where you explain differences.

Primary signals, in order of usefulness:

1. `filtered_gyroscope_dps` for roll, pitch and yaw. This is the most direct measure of the rotational dynamics.
2. `attitude_degrees` for roll and pitch.
3. `motors`: does the simulated controller need the same commands to fly the same inputs? This is the most sensitive check of thrust and inertia.
4. `pid` terms, especially the integral terms, which reveal constant biases such as an off-centre centre of gravity or motor mismatch.

### 9.2 Procedure

1. Pick a log, decode it with `./ofc flight-log decode`, and write down the firmware version and build ID from its header.
2. Build a replay scenario:
   - use the configuration snapshot from the log (F4);
   - drive the manual producer with the logged receiver values;
   - set the initial attitude to the first logged estimate;
   - set the arm time from the logged `system_state` transition.
3. Run it. Interpolate the simulated telemetry onto the log's timestamps, which are at 100 Hz.
4. Compute metrics per signal: RMS error, peak error, and the time offset that maximizes cross-correlation. A large offset points to delays that are not modelled (R5).
5. Plot the simulated signal over the real one for each primary signal.

```python
# tools/ofcsim_tools/validate.py (core of the comparison)
import numpy as np

def compare(t_real_us, real, t_sim_us, sim):
    sim_on_real = np.interp(t_real_us, t_sim_us, sim)
    err = sim_on_real - real
    lags = np.arange(-50, 51)                                   # ±0.5 s at 100 Hz
    xc = [np.corrcoef(np.roll(sim_on_real, k), real)[0, 1] for k in lags]
    return {"rms": float(np.sqrt(np.mean(err**2))),
            "peak": float(np.max(np.abs(err))),
            "best_lag_ms": int(lags[int(np.argmax(xc))] * 10)}
```

### 9.3 Parameter identification (fitting from flight data)

Fit only parameters that are really uncertain, and only a few at a time: a thrust coefficient scale, the motor time constant, an inertia scale for roll and pitch, and the gyro noise level. Minimize the weighted RMS error of the primary signals, calling the simulator through the CLI with `--set` overrides:

```python
from scipy.optimize import minimize

def cost(x):
    kt_scale, tau_s, inertia_scale = x
    log = run_sim(overrides={"motor_defaults.thrust_coeff": KT0 * kt_scale,
                             "motor_defaults.time_constant_s": tau_s,
                             "inertia_scale_rp": inertia_scale})
    return weighted_rms(log, real)

result = minimize(cost, x0=[1.0, 0.03, 1.0], method="Nelder-Mead")
```

Report the fitted values together with the error before and after the fit. Check the fit on a second log that was not used for fitting, otherwise you are only fitting noise. If you swap motors or props later, one new hop plus this fit gives the new parameters without a thrust stand.

### 9.4 Caveats to state openly

- **The tether.** The hops were tethered or otherwise constrained. The tether adds forces that the simulator does not model. Either validate only the segments before the tether goes taut (the ground-truth altitude shows when that happens in simulation, and the log's motor and gyro signals show it in reality), or add a tether model first (R3).
- **Ground phase.** The v1 contact model cannot tip. Compare from the moment of liftoff onwards.
- **Battery voltage.** v1 assumes constant voltage, and real thrust at a given command drops as the battery sags. Expect a slow drift in the fitted thrust over a long log (R1).
- **Timing.** v1 has no computation delay or actuation delay (R5). A consistent cross-correlation offset of a few milliseconds is expected and should be reported, not hidden.

---

## 10. Roadmap after M9

Each item builds on the existing interfaces. None of them should require a rewrite. Pick items when a validation result or a firmware feature calls for them, not ahead of time.

| ID | Item | What it adds | Built on |
| --- | --- | --- | --- |
| R1 | Refined propulsion | Split motor, propeller and ESC: propeller `C_T`/`C_Q` and diameter using `T = C_T·ρ·n²·D⁴` and `Q = C_Q·ρ·n²·D⁵`; motor Kv and winding resistance; ESC response and current limit; a battery model (voltage sag under load); air density as an environment parameter | `Propulsion`, `VehicleConfig` |
| R2 | Aerodynamics | Rotor drag (a linear drag term proportional to rotor speed), body drag, and ground effect using the Cheeseman–Bennett approximation `T_IGE/T_OGE = 1 / (1 − (R/4z)²)`, which matters for hops close to the ground | `derivative()` |
| R3 | Contact and tether | Four landing-gear points modelled as spring-dampers (so the vehicle can tip, bounce and settle), plus a tether modelled as a one-sided spring-damper between an anchor and the airframe | `GroundContact` |
| R4 | Sensor realism | Vibration at the rotor frequency and its harmonics, fitted from armed logs; raw sensor-axis output through the real `imu_map_raw_sample()`; the real `gyro_calibration.c` running during a simulated still phase at startup; temperature-dependent bias drift | `ImuModel`, `FirmwareStack` |
| R5 | Timing realism | Computation and actuation delay, DShot latency, an IMU rate different from the control rate, and scheduler jitter taken from the firmware timing analysis | `Simulator::tick()` |
| R6 | Fault injection | IMU dropout or freeze (through `imu_freshness_t`), loss of a motor or prop (a thrust scale), receiver loss patterns, and sensor bias steps, all declared in the scenario's `faults` list, together with a matrix of fault tests | `fault_injector_`, scenarios |
| R7 | Monte Carlo | Scatter the parameters (mass, inertia, `kT`, noise, centre-of-gravity offset) and run many scenarios in parallel with a thread pool, reporting robustness statistics such as the settling-time distribution and failure rate. This needs the binary log format. | `ofcsim` library |
| R8 | Barometer and altitude | A BMP388 model (pressure noise and a lag), a vertical estimator, and an altitude-hold autonomous producer. The board already has a BMP388. | `ImuModel` pattern, producers |
| R9 | Behavior arbitration testbed | When the firmware's arbitration and autonomous behaviors exist, link the real C modules and test the transitions: which producer is active, cases where freshness is lost, and no glitches in motor output | `ObjectiveProducer` |
| R10 | Hardware in the loop | Replace the IMU model output and the motor command input with a USB link to the real board: a firmware build option injects samples and reports commands. Rev 0.2's IMU bus switch later allows injection at the SPI level. The scenario files stay the same. | `Simulator` seams |
| R11 | Live visualization (optional) | Stream the state to a viewer while the simulation runs. Options are a Rerun viewer, which has a C++ SDK, or a three.js page shared with the ground-station dashboard. | `LogWriter` sink |
| R12 | Performance | A benchmark of real-time factor, profiling, and a fixed-capacity binary logger | all |

---

## 11. Parameters to measure and open questions

Fill this in as you go. Every value in the vehicle file needs a source.

| Item | How to get it | Status |
| --- | --- | --- |
| Motor corners and spin directions | Inspect the airframe, check the ESC wiring against `motor_mapping`, and check the configured directions (Section 3.4) | open |
| Mass, including battery and O4 unit | Kitchen scale | open |
| Arm radius and centre-of-gravity position | Measure motor-to-motor diagonals, and use a balance point test for the centre of gravity | open |
| Inertia | First a point-mass estimate: `I_xx ≈ Σ mᵢ·yᵢ²` over motors, battery and board. Later a bifilar pendulum test (hang the quad from two strings and time the swing) | open |
| Motor and prop part numbers | Build notes, then manufacturer thrust tables giving `kT`, `kQ` and maximum speed at your voltage | open |
| Motor time constant | Starting estimate 20–40 ms. Fitted in M8. | open |
| IMU noise | Blackbox segments disarmed on the ground and armed at idle on the ground (M4) | open |
| Link-loss semantics of the receiver snapshot | Read `receiver_failsafe.c` and the receiver service (M6) | open |
| Order of the four blackbox `receiver` values | Read the firmware diagnostics capture (M8) | open |
| Tether geometry and length during the hops | Your test setup notes (M8, R3) | open |

---

## 12. References

- Firmware sources quoted in this guide: `flight/control/flight_control_core.{h,c}`, `flight/control/quad_x_mixer.c`, `flight/behavior/manual_easy/manual_easy_behavior.c`, `flight/estimation/imu_processing_pipeline.{h,c}`, `flight/estimation/accelerometer_attitude.c`, `app/flight_control_task.c`, `app/flight_configuration_service.c`, `docs/bmi270.md`, `docs/blackbox.md`, and `host_tools/openflightcomputer/blackbox.py`, all at commit `3174bbe`.
- Rigid-body dynamics and quaternion kinematics: any flight-dynamics text covering 6-DOF equations of motion. Beard and McLain, *Small Unmanned Aircraft: Theory and Practice*, is a good match for this project.
- Propeller coefficients: manufacturer performance data (for example APC's published propeller data) and standard propeller theory for `C_T` and `C_Q`.
- C++: Stroustrup, *A Tour of C++* (3rd ed.); the C++ Core Guidelines; cppreference.com; the Eigen documentation, in particular its pages on common pitfalls and alignment.
