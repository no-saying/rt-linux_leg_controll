# RT-Thread ESO Reference

`decoupled_control.h/.cpp` retain `GlobalEso` and the selected three-NMPC
integration. Run `GlobalEso::update` in RT-Thread and pass its synchronized state into
the RT-Thread task that owns the synchronized sensor sample, then publish
`EsoState::disturbance[roll,pitch,yaw]` with that sample in AMP sensor snapshot
`0x12`. Set the ESO-valid flag only after initialization and valid updates.

The framework and controller consume the transmitted disturbance state and do
not run another ESO. The three NMPC rollouts include the axis disturbance as an
additive acceleration term. These algorithm files build as a separate target;
the AMP/application adapter maps sensor, reference, and ESO fields into
`DecoupledInput`.
