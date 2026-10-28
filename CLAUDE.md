# QuadSim — working rules (master file)

This file governs AI-assisted work across all three QuadSim repositories.
`QuadSimLib/CLAUDE.md` and `Resnet-GeometricController/CLAUDE.md` import it.
Keep it short: it is loaded at the start of every session.

## What we are building

QuadSim is a robotics simulator owned by Nicolas Nino Fernandez. It is meant to
be usable three ways with one API: a standalone application, a Unity package
(plugin) inside a user's own project, and a headless player. Unreal is a later
goal, not current work.

- **Core claim: ease of use for researchers.** Installing, flying, and scripting
  from Python should take minutes. AirSim is deprecated, and Colosseum and
  Project AirSim are hard to set up and integrate; labs have left them for that
  reason. Every feature is judged partly by how much setup it costs a user.
- **Direction: an AtomEngine-style platform.** A base simulator, entities with
  components, and a modkit so researchers extend QuadSim without forking it.
  See `Documentation~/platform-extension-ideas.md`.
- **Near-term deliverables:** a QuadSim paper, and a geometric-controller +
  ResNet paper that uses QuadSim as its testbed.

## Repositories (clone side by side)

| Repo | Role |
|---|---|
| `Unity_QuadSim` | UPM package: simulation runtime, physics/plant, controllers, sensors, RPC server, HUD |
| `QuadSimLib` | Python SDK `quadsim-sdk`: transport, `QuadSim`/`Drone`, lockstep, replay, YAML tooling, PX4 bridge, RL helpers |
| `Resnet-GeometricController` | Research consumer: Lee geometric controller, adaptive ResNet compensator, ROS2 node, PX4 SITL tuning |

## How we work together

- **Core areas: Nicolas writes, the AI explains, proposes, and reviews.** These
  areas are physics/plant, controllers, allocation, timing and stepping, frames,
  the RPC protocol and transport, and the SDK's public API. The AI writes core
  code only when Nicolas explicitly hands that task over in the session. Even
  then, it explains the change well enough for him to defend it.
- **Delegated to the AI:** the HUD (UI Toolkit workspace), ImGui developer
  tools, editor windows, CI plumbing, and helper scripts.
- **Nicolas is rebuilding hands-on programming skill.** Point to `file:line`,
  ask a guiding question before giving an answer, and prefer exercises over
  finished code in core areas. He knows ROS well; explain networking through
  ROS analogies. The current study track is `Documentation~/learning-rpc.md`.
  Quiz him when asked.
- **Be a partner, not an assistant.** Push back when he is wrong and keep the
  position if the evidence supports it. Say plainly when unsure. Do not hedge
  facts that were verified.

## Invariants (change only after an explicit design decision)

- **`SimulationManager` owns time.** Simulation code takes `dt`/`nowNanos` from
  `PrePhysicsStep`/`PostPhysicsStep` or `ClockFactory`, never `UnityEngine.Time`.
  Presentation code may use wall time.
- **Step order:** PrePhysics (controller → actuation → wrench), then
  `Physics.Simulate(dt)`, then PostPhysics (sensors). Step RPCs return
  post-step observations.
- **`MathUtil/Frames.cs` is the only frame-conversion boundary.** World data on
  the wire is always ENU. Body data is FLU, or FRD in PX4 mode. NED appears only
  at the MAVLink boundary and in explicitly named fields.
- **Determinism:** the same seed and commands give the same state (in-place
  resets within `1e-6`). The simulation path must not use wall-clock time,
  unseeded RNG, or unordered iteration.
- **RPC:** binds to loopback by default. A breaking wire change bumps the
  protocol major version and updates `RpcProtocol.cs` and the SDK's
  `quadsim/protocol.py` in the same change set. `package.json` version must
  equal `UnityPackageVersion` (CI checks this).
- **Drone YAML (schema v1):** files in `Runtime/Resources/QuadSim/Configs/Drones/`
  must be byte-identical to the `Samples~/QuadSim Starter/...` mirror (CI checks
  this). Schema changes update `Documentation~/drone-yaml-schema-v1.md` and the
  SDK's `quadsim/drone_config.py` together.
- **Package hygiene:** runtime code never references `UnityEditor`. Every new
  file outside `~`- or `.`-suffixed/prefixed folders needs a `.meta`.
  Headless runs create no HUD.
- **No migration to Unity DOTS/Entities without a decision.** It would replace
  PhysX and reset the determinism baseline. The current proposal is ECS-style
  *contracts* (entity manifests, component registration) on GameObjects.

## Evidence rules

- Never call something validated, passing, or working unless it was run in the
  current session. State what was run and what was not.
- Unity cannot run in cloud sessions, so C# changes made there are
  **unverified** until they are compiled and tested in a host project. Say so.
- Every number in a paper needs a checked-in script that regenerates it.
- Validation docs record what was executed (versions, counts, platform), in
  the style of the existing `phase-*-validation.md` files.

## Scope discipline

- Do the task that was asked. Record other findings in
  `Documentation~/engineering-backlog.md` instead of fixing them silently.
- Make small, reviewable changes, one concern per commit.
- Core architecture changes start as a written proposal that gets discussed
  before any code.

## Workflow

- Commit messages are imperative and specific ("Read runtime rates from drone
  YAML"), never "Push".
- Never commit build output, IDE folders, `__pycache__`, or large run data.
- **SDK tests:** run `pip install -e '.[test]'`, then
  `python -m pytest -q Testing --ignore=Testing/verify_lockstep_unity.py --ignore=Testing/test_high_level.py`
  (`test_high_level.py` is stale; see the backlog).
- **Unity tests:** use a host project with `com.nino.quadsim` in the manifest
  `testables`, and run EditMode and PlayMode from the Test Runner.
- **Headless player:**
  `./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556`.
  `ci~/built_player_smoke.py` is the scripted smoke test.

## Code map (Unity runtime, `Runtime/Scripts/`)

| Concern | Start here |
|---|---|
| Clock and stepping | `SimCore/SimulationManager.cs`, `SimCore/SimClock.cs`, `SimCore/ClockFactory.cs` |
| Drone entity | `DroneCore/QuadPawn.cs`, `DroneCore/Core/DroneManager.cs` |
| Control cascade and current plant | `DroneCore/Controllers/CascadedController.cs`, `DroneCore/Controls/ControlAllocator.cs`, `DroneCore/Controls/Quadactuationmodel.cs` |
| Rotor math and config | `MathUtil/RotorMath.cs`, `Yaml/Drone/DroneConfigStructs.cs`, `Yaml/Drone/YamlParser.cs` |
| Sensors | `RobotCore/SensorManager.cs`, `RobotCore/Sensors/` |
| Frames | `MathUtil/Frames.cs` |
| RPC | `SimCore/Rpc/RpcServerThread.cs` (network thread), `SimCore/Rpc/ExternalRpcAdapter.cs` (main-thread handlers), `SimCore/Rpc/RpcProtocol.cs` |
| Authority | `SimCore/ControlAuthorityManager.cs`, `SimCore/SimCoreApi.cs` |
| RL and scenario objects | `RL/` |
| HUD | `RuntimeUI/Workspace/` (default), `RuntimeUI/ImGui/` (developer only) |
