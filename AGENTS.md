# Repository Guidelines

## Project Structure & Module Organization

This repository is currently in the planning stage; `docs/DEVELOPMENT_GUIDE.md` is the authoritative design and implementation roadmap. The intended project is a C++20/C11 software-in-the-loop flight simulator. Place public headers in `include/ofcsim/`, implementations in `src/`, the CLI in `apps/`, and GoogleTest suites in `tests/`. Store vehicle and scenario JSON in `vehicles/` and `scenarios/`; keep Python analysis and validation utilities in `tools/`. The pinned firmware dependency belongs in `external/flight-computer-firmware/` and must remain unmodified except for documented testability refactors.

## Build, Test, and Development Commands

Build infrastructure has not yet been committed. Once the documented CMake presets exist, use:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan
```

The first two commands configure and build the simulator; `ctest` runs unit and scenario tests. The sanitizer preset checks memory errors and undefined behavior. Python tooling should use Python 3.11+ through `uv`. Do not assume a command works until its referenced configuration has been added.

## Coding Style & Naming Conventions

Use C++20 for simulator code and C11 at firmware boundaries. Format C++ with `clang-format`; never reformat the firmware submodule. Compile with strict warnings (`-Wall -Wextra -Wpedantic -Wconversion -Werror`). Use SI units internally, `double` for plant physics, and explicit `float` conversions at firmware boundaries. Encode units in names, for example `velocity_mps`, `rate_rad_s`, and `time_us`. Preserve the documented body-FRD/world-NED frames and integrate unit quaternions rather than Euler angles.

## Testing Guidelines

Use GoogleTest. Name tests by observable behavior, such as `RigidBody.FreeFallMatchesAnalyticSolution`. Cover analytic physics checks, mixer/plant sign consistency, deterministic closed-loop scenarios, and invalid JSON/configuration. Every bug fix should add a regression test. Run the full suite and the sanitizer build before submitting changes.

## Commit & Pull Request Guidelines

No Git history is present here, so no repository-specific commit convention can be inferred. Use short, imperative subjects (for example, `Add RK4 rigid-body integrator`) and keep commits focused. Pull requests should explain the behavior change, list verification commands, link relevant issues or design decisions, and include plots or log excerpts when simulation results change. Call out firmware-submodule updates and any changed physical assumptions explicitly.

## Safety & Validation

Ground truth must never feed the control loop. Treat motor geometry as unverified until checked against the real airframe, and document parameter sources and validation evidence in `docs/validation/`.
