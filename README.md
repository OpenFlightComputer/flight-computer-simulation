# OpenFlightComputer Flight Simulation

`flight-computer-simulation` is a deterministic software-in-the-loop simulator
for the OpenFlightComputer firmware. It links the real flight-control and IMU
processing C modules to a C++20 vehicle model so controller behaviour can be
characterized, regression-tested, and compared with physical flight logs.

The project is at Milestone M0. The repository and host build are in place;
the first C/C++ interoperability exercises are ready to implement. See
`docs/DEVELOPMENT_GUIDE.md` for the architecture, conventions, milestones,
and validation plan.

## Build and test

Initialize the pinned firmware dependency after cloning:

```sh
git submodule update --init --recursive
```

Configure, build, and test a development build:

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Use `release` for an optimized build or `asan-ubsan` for AddressSanitizer and
UndefinedBehaviorSanitizer checks.

## Repository layout

- `include/ofcsim/`: initial public library interface and C firmware wrapper.
- `src/`: library implementation; add modules here as milestones begin.
- `apps/`: thin command-line applications.
- `tests/`: initial firmware interop exercises; add suites alongside features.
- `external/flight-computer-firmware/`: firmware submodule pinned to a reviewed commit.
- `vehicles/` and `scenarios/`: versioned JSON inputs.
- `tools/`: Python analysis, plotting, rendering, and validation utilities.
- `docs/decisions/` and `docs/validation/`: engineering decisions and physical comparisons.

## Development agreement

The flight dynamics, integration, propulsion, contact, sensor model, and
validation reasoning are developed by hand and must remain explainable line by
line. Coding agents may provide build infrastructure, file and test scaffolding,
mechanical plumbing, code review, and debugging support. Generated assistance
must not replace understanding of the physical model or its evidence.

## License

This project is licensed under the MIT License. See `LICENSE`.
