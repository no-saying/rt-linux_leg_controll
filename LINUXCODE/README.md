# Wheel-Legged Control Framework

This is a self-contained source handoff of the current control framework. The
selected controller is RT-Thread ESO observation plus three decoupled nonlinear
NMPC blocks for roll, pitch, and yaw/wheel control. The NMPC QP path uses the
bundled acados dense-QP adapter, HPIPM, and BLASFEO. OSQP is not used.

## Contents

- `source/`: application, AMP codec boundary, static message/logging/threading
  interfaces, ESO reference implementation, three-axis NMPC, bundled solver
  sources, and framework unit tests.
- `doc/amp/`: AMP protocol and RT-Thread/BSP handoff, including the requirement
  that RT-Thread publishes ESO estimates with the matching sensor snapshot.
- `doc/control/`: selected algorithm baseline.
- `doc/experiments/three_nmpc/`: solver integration and experiment records,
  including the latest reduced CAD/RK4 retest. That retest completed QP solves
  but did not pass closed-loop stability acceptance.
- `tools/`: standalone board-architecture path test. It uses deterministic
  algorithm doubles and does not claim to validate actuator behavior.

The simulator and its run logs are intentionally outside this framework
package. The framework's CMake tests below exercise the actual bundled solver
and three-axis adapter; the separate architecture test exercises AMP parsing,
application state handling, and data routing.

## Build and test the framework

```bash
cmake -S source -B build/control_stack -DCMAKE_BUILD_TYPE=Release
cmake --build build/control_stack -j
ctest --test-dir build/control_stack --output-on-failure
```

## Run the board-architecture test

```bash
cmake -S tools -B build/board_arch_test -DCMAKE_BUILD_TYPE=Release
cmake --build build/board_arch_test -j
ctest --test-dir build/board_arch_test --output-on-failure
```

RT-Thread owns AMP hardware transport and runs the production ESO task. The
framework receives synchronized sensor and ESO values through AMP; it must not
run a second ESO in the production path. Board transport, actuator drivers,
calibration, and real-time acceptance remain board-integration work described
in `doc/amp/`.
