# Plant and Lockstep Runtime Contract

## Plant

- Rotor commands are normalized values in `[0, 1]` and map linearly to the configured static maximum thrust and yaw torque.
- Static thrust uses `Ct * rho * n^2 * D^4` at `rho = 1.225 kg/m^3`.
- `torque_coefficient_convention` is explicit. Existing profiles use `legacy_2_5_pi`; new profiles may select `power_coefficient_2_pi`.
- Rotor locations and rigid-body wrenches use Unity body axes internally: forward, up, left. Allocation uses right-handed body FLU. External world state uses ENU.
- Saturation preserves collective first, roll/pitch second, and desaturates yaw first. Telemetry reports the allocation request, achieved result, residual, axis scales, matrix rank/condition estimate, and saturated motors.

## Timing and observations

- `step_index` is authoritative. `sim_time_ns` is `start_time_ns + step_index * fixed_step_ns`.
- A request with `count = N` targets resulting step `current_step + N`.
- Strict lockstep requires a paused simulation and an exact `target_step`. Missing, late/duplicate, count-mismatched, and skipped targets are rejected before commands are applied.
- A fixed step runs controller/actuation, Unity physics, then sensor sampling. Composite command-step responses therefore contain post-step observations.
- Sensor, state, image, event, and telemetry payloads include episode ID, step index, integer simulation time, source frame, and sequence number.
- Episode reset clears time, actuator lag, controller/command state, sensors, contacts/events, and message sequences. Drones, managers, scene objects, and emitted events use stable ordering.
- Fast in-place episode resets are compared with a `1e-6` absolute state tolerance to account for PhysX floating-point residue. Bitwise replay claims require a fresh physics scene or player process.

QuadSimLib's `LockstepLoop` supplies strict target steps automatically. Its replay format stores canonical actions and states with a SHA-256 digest chain; `quadsim replay` verifies a recorded run and `quadsim compare` reports the first divergent field.
