# Wheel control stack

This is the statically allocated application and platform framework for the wheel-legged controller. It is independent of RT-Thread and owns no device registers.

## Data path

```text
RT-Thread sensor + remote sample (ESO already updated on RT-Thread)
  -> AMP frame codec and validation (commands `0x01`, sensor snapshot `0x12`)
  -> ControlRuntime (sequence, timeout, safety state, odometry, recovery stages)
  -> ControlApplication (AMP ESO state + local perception -> green-wave plan -> selected controller)
  -> fixed-size torque output
  -> board motor writer
```

AMP is the only input path represented here. USB is an unrelated external-peripheral link and is outside this control stack. The RT-Thread AMP channel and sensor/motor drivers are board-owned adapters, not replacements for the protocol or application logic.

## Responsibilities

- `application`: command-to-state interpretation, timeout handling, latched fault/ESTOP, odometry accumulation, recovery sequence, and controller profile selection.
- RT-Thread runs the Global ESO. Its roll/pitch/yaw lumped-disturbance estimate is carried in AMP sensor snapshot `0x12`; this framework consumes the received values and does not run a second ESO.
- `algorithms/perception`: currently only an interface. The framework has no registered velocity/contact or per-wheel slip implementation. ESO is not owned by this local perception interface.
- `algorithms/control`: one controller interface selected with Balance, Locomotion, or Recovery profile. RL and hybrid policies fit behind the same control boundary.
- `algorithms/greenwave`: optional command timing/planning between perception and control.
- `platform/amp`: fixed command/sensor frame formats, CRC, float16 conversion and validation.
- `platform/logging`, `platform/threading`, and `message_center`: fixed-data interfaces, static SPSC queues, and board scheduling hooks.

The framework supplies `ThreeNmpcControlAlgorithm`, an `IControlAlgorithm` implementation that calls the selected three-axis `DecoupledController`. CMake compiles and links it into the application library. Board initialization must create the native solver contexts and provide a mapper for the robot's verified axis states and references, then inject it into `ControlApplication`. `ControlRuntime` inhibits output when AMP bit3 marks the RT-Thread ESO estimate invalid; the adapter forwards those AMP disturbance values into the NMPC prediction input. The target HPIPM adapter is external to this workspace. Host tests use fake solver callbacks, not native solver timing or physical control validation.

## Current inventory

- Implemented and compiled: AMP command/sensor codec, CRC and binary16 conversion; application safety/state-machine/odometry/recovery orchestration; static message queues and logging/thread interfaces.
- Called by `ControlApplication`: injected perception, green-wave, then control. The selected `ThreeNmpcControlAlgorithm` is supplied and linked; the board instantiates it with verified axis mapping and native solver contexts.
- Interface only: perception and green-wave. RL remains a policy interface and is not selected or called.
- Legacy perception: wheel slip observer/projection and velocity Kalman filter under `source/control/balance` are called by the old balance path, not this framework. Python motion-IMU, odometry, jump-support, and speed-confidence components are called by simulation/research applications and are excluded from this deployment source.
- Retained but uncompiled reference components: `estimation.*`, `control_pipeline.*`, `state_reconstruction.*`, and `scheduler.*` from the wider C++ runtime work.
- Board integration still needed: AMP transport backend, motor writer, RT-Thread thread backend, axis state/reference mapper, and native HPIPM adapter/library.

## Static execution

Framework storage uses fixed arrays and bounded queues. The thread API accepts a function pointer plus context (no `std::function`), and control-cycle interfaces do not allocate. Each `StaticRingBuffer` is SPSC: one producer and one consumer only. `clear()` is for initialization while producers/consumers are stopped.

## Build

```bash
cmake -S source/control_stack -B build/control_stack -DCMAKE_BUILD_TYPE=Release
cmake --build build/control_stack -j
cmake --install build/control_stack --prefix build/control_stack/install
```

The standalone board architecture test is in `tools/control_stack_board_test`; see its README. It remains outside this framework directory.

Protocol and board integration details are in `doc/amp/AMP_PROTOCOL.md` and `doc/amp/AMP_BSP_HANDOFF.md`. Control baseline is in `doc/control/CONTROL_ALGORITHM_BASELINE.md`.
