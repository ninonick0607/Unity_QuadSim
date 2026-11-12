# Engineering backlog

Started 2026-10-28 from a full read of all three repositories. Each item says
how it was established. "Verified" means checked in source or reproduced, not
fixed. Mark items done with the commit that fixes them.

Ownership follows `CLAUDE.md`: **core** items are written by Nicolas with AI
review, and **delegated** items may be done by the AI.

## Bugs

| # | Item | Evidence | Area |
|---|---|---|---|
| B1 | **The SDK socket is stuck after any timeout.** `_transport.py` keeps the same REQ socket after a `recv` timeout. ZeroMQ then rejects every later send ("Operation cannot be accomplished in current state"), so one slow call breaks the whole session until reconnect. The fix is the ZeroMQ guide's Lazy Pirate pattern: close and recreate the REQ socket after a timeout. | Reproduced with a minimal pyzmq script; source at `QuadSimLib/quadsim/_transport.py` `_send_unlocked` | core, RPC (learning exercise) |
| B2 | **The YAML `runtime:` block is ignored by Unity.** `physics_rate_hz` and `controller_rate_hz` are validated by the SDK but never parsed in C#. The physics rate comes from the `SimulationManager.baseHz` inspector field, and the controller runs every physics step. | `grep` finds no reader in `Runtime/Scripts` | core, timing |
| B3 | **Hardcoded velocity limits.** `CascadedController` clamps position-mode velocity setpoints with literal `4.0f, 10f, 8f` (`// TODO: DEFINE THESE IN CONFIG`), while YAML declares `max_velocity_xy` / `max_velocity_z`. | Source | core, control |
| B4 | **Latent double CoM rebase.** `QuadPawn.ApplyPhysicalConfig` stores CoM-rebased rotors back into `_config`. `ReapplyConfig()` then rebases them again. Nothing in the runtime calls `QuadSimApi.ReapplyConfig()` today, and the bundled configs only have vertical CoM offsets (no torque effect), so there is no current impact. | Source (`DroneConfig` is a struct; write-back at `ApplyPhysicalConfig`) | core, plant |
| B5 | **Stale test.** `QuadSimLib/Testing/test_high_level.py` imports `quadsim.api`, which no longer exists. CI only runs a fixed file list, so this went unnoticed. | Collection error under pytest | delegated |
| B6 | **The PID integrator leaks.** `QuadPIDController.Calculate` multiplies the integral by `DecayFactor = 0.99` on every call (nothing overrides it), on top of conditional integration and back-calculation. At 250 Hz the integral decays with τ ≈ 0.4 s, so it settles at `0.4 s × error` and cannot remove steady-state error. The decay is also per call, so it changes with the controller rate. **Done in `731a7ea`.** | Source (`QuadPIDController.cs:70`); see M0 in `02-cpp-core-plan.md` (local roadmap) | core, control |
| B7 | **The D term differentiates raw error without a filter.** Setpoint steps cause derivative kick. The position loop differentiates 10 Hz sample-and-hold GPS at 250 Hz, so D is zero on 24 of 25 steps and about 25× too large on the update step; sensor noise is amplified by 1/dt. Flightmare and Standard use position D = 0.5–1.0. **Done in `731a7ea`** (derivative on measurement; position D set to 0 in both profiles). | Source (`QuadPIDController.cs:88`, `SensorManager.gpsHz = 10`); profile gains | core, control |
| B8 | **The controller mixes measured and true state.** Rate, velocity, and position loops read `SensorManager.Latest` (raw, noisy when enabled, GPS at 10 Hz, no estimator), while heading and collective read `_body.transform`, which may be the interpolated render pose (`rb.interpolation = Interpolate`). | Source (`CascadedController.cs:515`, `:884`; `QuadPawn.cs:380`) | core, control |
| B9 | **Heading rotation sign is suspect.** `OutputFrameVelToHeadingVel` and `DesiredForceToCollective` apply a counter-clockwise rotation formula using Unity `eulerAngles.y` (clockwise-positive). By reading, a southward ENU setpoint at yaw 90° becomes a backward body command. Confirmed 2026-11-12 by live runs (Flightmare sank 2 m facing North/South, held facing East/West; position step went the wrong way facing North); `SensorManager.HeadingVel` had the same formula. **Done in `2df652c`.** | Source; live SDK runs | core, control |
| B10 | **Thrust vector → attitude uses two independent `atan2`s.** Roll and pitch come from `atan2(y, z)` and `atan2(x, z)` separately (`AccelerationToAttitude`). This is exact only when one angle is zero; at large combined tilts the thrust does not point where it was asked to. | Source (`CascadedController.cs:853–854`) | core, control |
| B12 | **FLU yaw angle is clockwise-positive, so position-mode yaw hold diverges.** `Frames.TransformAttitude` negates FLU roll and pitch but passes Unity's clockwise yaw through unchanged, while FLU yaw rate (`TransformAngularVelocity`) is counter-clockwise-positive. The yaw angle loop and yaw rate loop therefore disagree in sign. Measured 2026-11-12 (Flightmare, position mode, setpoint 0°): yaw −0.05° at 2 s, −2.7° at 3 s, −78° at 4.5 s. Facing ENU north, `imu_attitude` reports −90°. Fixing it changes the public `imu_attitude` yaw, the UI yaw setpoint (`ImuAttitude.z`), and SDK yaw handling together. The SDK already assumes CCW-positive yaw (`_control_loops.py`). **FLU done in `31dff22`.** The FRD branch had roll and yaw signs that disagreed with FRD angular rate, and yaw referenced to East instead of North (static check 2026-11-12: roll +20° read −20°; facing North read 90° instead of 0°). The PX4 bridge does not read it (`quadsim/px4.py` uses PX4's own ATTITUDE). **FRD done in `a827b4f`.** | Live run via SDK; static attitude check; source (`Frames.cs` `TransformAttitude`) | core, frames |
| B13 | **Attitude Euler angles are approximate when roll and pitch are both non-zero.** `IMUSensor.SampleAttitude` reorders Unity's Euler angles (Y-X-Z application order) into roll/pitch/yaw instead of decomposing the rotation in ZYX order. Exact for single-axis rotations; at yaw 30°, pitch 15°, roll 10° the yaw reads 27.4° (expected 30°) in both FLU and FRD. Fix: compute ZYX angles from the body rotation in the target frame inside `Frames`. | Static attitude check via SDK, 2026-11-12 | core, frames |
| B14 | **`reset_pose` in FRD mode does not convert the quaternion.** Position is read as NED, but `Frames.InverseTransformQuaternion` has no FRD branch, so the quaternion is applied as a raw Unity rotation (the "KNOWN GAP" noted in `Frames.cs`). | Static attitude check via SDK, 2026-11-12; source (`ExternalRpcAdapter.HandleResetPose`) | core, frames |
| B15 | **`torque_coef` holds a torque-to-thrust ratio in two profiles.** `crazyflie2_quad.yaml` sets `torque_coef` to Bitcraze's `THRUST2TORQUE` (0.00699 m, per its own comment), but QuadSim reads it as a dimensionless coefficient (legacy 2.5π convention), giving a torque-to-thrust ratio of 0.2 mm instead of 7.0 mm, so yaw authority is about 35× too small. Flightmare's `0.016` gives 2.6 mm; if it is Flightmare's 0.016 m ratio (not yet checked against Flightmare's source), yaw authority is about 6× too small. At hover Flightmare can only produce 72 °/s² of yaw acceleration, a main contributor to its 19–40° yaw-step overshoot. | Source; offline allocator calculation from the YAML values, 2026-11-12 | core, plant config |
| B16 | **Pitch and yaw inertia are swapped in two profiles.** Crazyflie2 and Flightmare list `inertia` in body roll/pitch/yaw order (the largest value, yaw, last), but Unity reads (forward, up, left), so the yaw value lands on pitch. Sentinel and Standard permute correctly (Sentinel's comment shows how). | Source; Crazyflie values match published Ixx/Iyy/Izz | core, plant config |
| B17 | **Most rate gains are far beyond what a 250 Hz loop supports.** In physical units (`1edd62d`) the rate-loop bandwidths are: Flightmare 395/254/564 1/s (roll/pitch/yaw), Standard 143/142/111, Crazyflie2 106/60/45, Sentinel 16.5/16.5/14.5. Above roughly 100 1/s, P × dt approaches 1; these loops stay bounded only because the allocator saturates, so yaw behaves bang-bang (Flightmare yaw saturates above 0.13 °/s of rate error). Retune toward Sentinel-like values (roughly 15–30 1/s roll/pitch, 5–10 yaw) after B15/B16. | Exact gain conversion in `1edd62d` | core, tuning |
| B11 | **Controller cleanup.** `MinIdleThrottle = 0.04` is defined twice; `ControlAllocator.PostProcessMotorOutputs` is never called; new `float[]` arrays are allocated every step in the allocator and mixer. | Source | core, control |

## Architecture

| # | Item | Notes |
|---|---|---|
| A1 | **Separate the plant from the controller.** Motor lag and the rotor-to-wrench model currently run inside `CascadedController`, so a second controller would have to duplicate them. The C# `ControllerKind.Geometric` slot is empty. Target: a controller produces actuator commands, and a per-vehicle plant turns them into a wrench. | Prerequisite for controller extensions and for B2 (a controller sub-rate needs a plant that runs every step). Note: `QuadPawn.ApplyBufferedForces` skips steps where the wrench generation did not change, so a sub-rate controller would silently drop thrust today. |
| A2 | **Make the rate-loop output physical.** Rate PID output (deg/s) is divided by the max rate and used as a normalized torque. Gains do not transfer between airframes with different inertia. **Done in `1edd62d`** (output is angular acceleration, torque = J·α; gains converted exactly) **and `fed2651`** (optional `rate_gain_units: normalized_torque`, PX4-style). | Decide together with A1. |
| A3 | **RPC throughput is tied to the render loop.** Requests are drained in `ExternalRpcAdapter.Update()` (up to `maxRequestsPerFrame = 10`), so each lockstep round trip costs at least one player frame. | Measure before changing anything (see P2). |
| A4 | **ECS-style platform layer.** Entity manifests, component registration, and a modkit (AtomEngine model). | Proposal stage; see `platform-extension-ideas.md`. Do not adopt DOTS without a decision. |

## Fidelity gaps (paper reviewers will ask)

- Thrust is quadratic in rotor speed and motor lag acts on rotor speed
  (`2cbefc1`). Remaining gap: motor commands (`send_motors`, the allocator
  output) are thrust fractions with an ideal speed controller. There is no
  ESC throttle curve, no RPM command, and no RPM telemetry.
- Aerodynamic drag is Unity Rigidbody damping. There is no rotor drag or blade
  flapping, no ground effect, no battery or voltage model, and air density is
  fixed.
- The IMU has white noise only: no bias, no bias random walk, and no lever-arm
  offset from the center of mass.
- Standard quad: the declared `propulsion.max_thrust` differs from the
  coefficient-derived value by about 45% (already noted in
  `phase-3-4-validation.md`).

## Paper evidence to produce

| # | Item |
|---|---|
| P1 | **Ease-of-use evidence**, since this is the core claim: install steps, time to first flight, and lines of code for a standard task, compared against Colosseum / Project AirSim / Pegasus / Flightmare where feasible. |
| P2 | **Throughput benchmark script:** lockstep and free-run steps per second, faster-than-real-time ratio, and scaling from 1 to N drones, recording hardware and versions. The reported ~10x (Linux workstation) and ~21x (M5 MacBook Pro) faster-than-real-time figures were measured by hand; they need a checked-in script. |
| P3 | **Accuracy at speed:** show that the replay digest of a trajectory is identical at 1x and at maximum speed. Fixed-step design should guarantee this; the paper should demonstrate it. |
| P4 | **Plant validation against an external reference** (ProjectAirSim comparison planned in `phase-1-2-validation.md`). |

## Release and hygiene

- The Unity EditMode/PlayMode suites have not been executed by CI. The CI
  license secrets are not configured. 0.3.0 / 0.16.0 are not tagged
  (`release-checklist.md`).
- Committed files that should not be: `QuadSimLib/.idea/`; in
  Resnet-GeometricController, `.idea/`, `.vscode/`, and `__pycache__/`
  (all matched by `.gitignore` but tracked).
- The Resnet-GeometricController git history is ~292 MB.
- `jax_resnet` (the ResNet library) is neither pinned nor vendored. Pin it
  before the controller paper, for reproducibility.

## Pending decisions

- **License.** Released code is MIT and public, and MIT grants for already
  published versions cannot be revoked. Closing source later means only future
  versions (e.g. an open SDK/modkit with a closed core). This interacts with
  paper reproducibility expectations, so decide before the paper.
- **Paper headline.** The working claim is ease of use + verifiable
  determinism + one API across standalone/plugin/headless.
