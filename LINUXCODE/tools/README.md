# Board architecture test

This is a standalone verification project. It is outside `source/control_stack` and links that framework as a CMake subdirectory. The test drives fixed AMP frames, fake sensor snapshots, and deterministic algorithm doubles through the public application API.

It checks frame encode/decode, CRC rejection, duplicate sequence rejection, AMP ESO estimate forwarding to the controller input, local perception wiring, application state/profile choice, ESTOP latch, recovery handoff, and command timeout. It does not execute the NMPC solver or verify actuator wiring, sensor calibration, or robot stability.

## Build and run on a Linux-capable board

```bash
cmake -S tools -B build/board_arch_test -DCMAKE_BUILD_TYPE=Release
cmake --build build/board_arch_test -j
ctest --test-dir build/board_arch_test --output-on-failure
```

For a bare-metal board, configure with its CMake toolchain file and run the resulting `control_stack_board_test` image. The test currently writes its report with `printf`; redirect that output to the board console/UART in the board toolchain. No RT-Thread or simulator is needed for this architecture check.

## Connect the actual board runtime

RT-Thread owns the AMP hardware channel and device drivers. Pass each complete AMP command (`0x01`) or sensor snapshot (`0x12`) frame to `ControlRuntime::ingestAmpBytes(frame, size, local_monotonic_received_us)`. The runtime rejects malformed, stale, and duplicate frames, and its 50 ms timeout uses the local receive clock rather than the sender timestamp. Each control cycle calls `ControlRuntime::tick`. Write the six torques only when `ControlOutput.valid` is true. Keep the AMP receive callback bounded and nonblocking; queue frames into a fixed SPSC buffer if the control cycle runs in another thread.

Before enabling motors, replace the deterministic test algorithm doubles with adapters for the selected production controller, perception estimator, and planner, then rerun this architecture test and perform actuator-disabled board checks.
