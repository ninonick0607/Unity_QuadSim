# Unity QuadSim — CLAUDE.md

## Project Overview

A quadrotor drone flight simulator in Unity, ported from an Unreal Engine original. It features a deterministic physics stepping system, cascaded PID flight controller, YAML-based drone configuration, and a telemetry HUD. The architecture closely mirrors the Unreal source: types, naming conventions, and pipeline structure are intentionally preserved.

**Engine:** Unity 6000.3.6f1 · **Pipeline:** HDRP 17.3.0 · **Language:** C#

---

## Repository Layout

```
Assets/
├── Scripts/
│   ├── DroneCore/          # Drone entity, controllers, actuation
│   │   ├── Common/         # QuadPIDController
│   │   ├── Controllers/    # CascadedController + AxisControllers
│   │   ├── Controls/       # ControlAllocator, QuadActuationModel
│   │   ├── Core/           # DroneManager
│   │   ├── Debug/          # DroneDebugDraw
│   │   └── Interfaces/     # ICommandSource, FlightCommandProxy
│   ├── RobotCore/          # SensorManager, IMU/GPS sensors, SensorData
│   ├── SimCore/            # SimulationManager, SimClock, ClockFactory
│   │   └── Common/         # CommonStructs (Axis4, GoalMode, FlightCommand)
│   ├── MathUtil/           # Frames.cs — coordinate frame transforms
│   ├── UI/                 # SimHUDController, TelemetryDeckController, etc.
│   └── Yaml/               # DroneConfigLoader, DroneConfigStructs, YamlParser
├── Configs/Drones/         # YAML drone configs (StandardDrone_Config.yaml)
├── Prefabs/                # QuadPawnPrefab, SimRoot, HUD
└── Maps/                   # Terrain/environment
```

---

## Architecture

### Simulation Loop

`SimulationManager` (execution order −1000) owns the entire physics loop. Unity's built-in `FixedUpdate` is **disabled** (`Physics.simulationMode = Script`). Every rendered frame, `Update()` accumulates sim-time and runs N fixed steps:

```
SimulationManager.Update()
  └─ RunFixedSteps(n)
       ├─ ClockFactory.StepOnce()          // advance deterministic clock
       ├─ ISimulatable.PrePhysicsStep()    // controllers compute wrench
       ├─ Physics.Simulate(dt)             // Unity physics with fixed dt
       └─ ISimulatable.PostPhysicsStep()   // sensors sample, logging
```

Default rate: **250 Hz** (dt = 0.004 s). Configurable via `SimulationManager.baseHz`. Two modes: `FreeRun` (wall-time × timeScale) and `Manual` (explicit `StepOnce()` / `StepMany()` calls).

### Drone Step Pipeline

`DroneManager` implements `ISimulatable` and drives all `QuadPawn` instances in `PrePhysicsStep`:

```
PrePhysicsStep → QuadPawn.PhysicsStep(dt, now)
  1. SensorManager.UpdateSensors()         // read IMU, GPS
  2. CascadedController.StepController()   // compute motor commands
  3. QuadPawn.ApplyBufferedForces()        // write wrench to Rigidbody
```

### Cascaded Controller

`CascadedController` implements a four-stage cascade, mirroring `UQuadDroneController`:

| Stage | Active in modes | Output |
|-------|----------------|--------|
| Position | `Position` | Velocity setpoint (world) |
| Velocity | `Position`, `Velocity`, `Angle` (Z only) | Acceleration → attitude cmd |
| Angle | all except `Rate` | Rate setpoint (deg/s) |
| Rate | always | Torque demand (normalized) |

Control modes (`GoalMode`): `None`, `Position`, `Velocity`, `Angle`, `Rate`, `Passthrough`, `Wrench`.

`ThrustMixer` → `ControlAllocator.AllocateYawLimited()` → `QuadActuationModel.ComputeBodyWrench()` → `QuadPawn.SetBodyWrench()`.

### Coordinate Frames

`MathUtil/Frames.cs` handles all frame conversions. The project uses three frames:

| Frame | Convention | Basis (+X, +Y, +Z) |
|-------|-----------|---------------------|
| `UnityBody` | Unity local | forward, up, left |
| `FLU` | ROS | forward, left, up |
| `FRD` | PX4 | forward, right, down |

Always pass vectors through `Frames.Transform*()` before crossing frame boundaries. Angular velocity transforms negate all components (left-hand → right-hand).

---

## Key Files

| File | Purpose |
|------|---------|
| `Scripts/SimCore/SimulationManager.cs` | Authoritative sim loop; owns `Physics.Simulate` |
| `Scripts/SimCore/Common/CommonStructs.cs` | `Axis4`, `GoalMode`, `FlightCommand`, `InputSource` enums |
| `Scripts/DroneCore/QuadPawn.cs` | Drone entity — owns Rigidbody, orchestrates step pipeline |
| `Scripts/DroneCore/Core/DroneManager.cs` | Registry, spawning, drives QuadPawns; implements `ISimulatable` |
| `Scripts/DroneCore/Controllers/CascadedController.cs` | Cascaded PID flight controller |
| `Scripts/DroneCore/Controls/ControlAllocator.cs` | Effectiveness-matrix motor allocation |
| `Scripts/DroneCore/Controls/Quadactuationmodel.cs` | Body wrench from per-motor thrust/torque |
| `Scripts/RobotCore/SensorManager.cs` | Manages IMU and GPS sensors; exposes `TargetFrame` |
| `Scripts/MathUtil/Frames.cs` | Frame transform utilities (UnityBody ↔ FLU ↔ FRD) |
| `Scripts/Yaml/Drone/DroneConfigLoader.cs` | Loads `DroneConfig` from YAML |
| `Assets/Configs/Drones/StandardDrone_Config.yaml` | Reference drone: 1.28 kg, 250 mm frame, PID gains |

---

## Drone Configuration (YAML)

```yaml
physical_properties:
  mass_kg: 1.28
  arm_length: 0.1789      # meters, center-to-motor
  linear_drag_coef: 0.1
  angular_drag_coef: 0.05

propulsion:
  prop_diameter: 0.127    # meters
  max_rpm: 28000
  max_thrust: 4.179       # Newtons per motor
  thrust_coef: 0.11
  torque_coef: 0.040164
  propellers:             # id, x/y/z offset, dir (+1 CCW / -1 CW)
    - {id: 0, x: 0.211, y: 0.0117, z:  0.211, dir:  1}  # FL
    - {id: 1, x: 0.211, y: 0.0117, z: -0.211, dir: -1}  # FR
    - {id: 2, x:-0.211, y: 0.0117, z:  0.211, dir: -1}  # BL
    - {id: 3, x:-0.211, y: 0.0117, z: -0.211, dir:  1}  # BR

controller:
  position_gains: ...
  velocity_gains: ...
  attitude_gains: ...
  rate_gains: ...
```

Motor canonical order: `[0]=FL [1]=FR [2]=BL [3]=BR`.

---

## Interfaces & Contracts

```csharp
// Any system that needs to participate in the sim loop implements:
public interface ISimulatable
{
    int ExecutionOrder { get; }      // lower = earlier
    void OnSimulationStart(SimulationManager sim);
    void OnSimulationReset(SimulationManager sim);
    void PrePhysicsStep(double dtSec, long nowNanos);
    void PostPhysicsStep(double dtSec, long nowNanos);
}

// Command sources (keyboard, UI, external API) implement:
public interface ICommandSource
{
    GoalMode GetGoalMode();
    Axis4 GetCommandValue();
    InputSource GetActiveSource();
    SourceStatus GetSourceStatus();
}
```

`FlightCommandProxy` is the concrete `ICommandSource` attached to each `QuadPawn`. Controllers read from it, external inputs write to it.

---

## Coding Conventions

- **Namespace per folder** — `DroneCore`, `DroneCore.Controllers`, `SimCore`, `SimCore.Common`, `MathUtil`, `RobotCore`, `Yaml.Drone`.
- **Mirror UE naming** — class/method names track Unreal originals (e.g., `SetBodyWrench`, `ApplyConfig`, `StepController`). Preserve this when porting new features.
- **No `FixedUpdate`** — all drone/sim logic must run inside the `ISimulatable` pipeline. `Update()` is only for visuals/UI.
- **Buffered wrench pattern** — controllers call `QuadPawn.SetBodyWrench()` during `PrePhysicsStep`; `ApplyBufferedForces()` writes to Rigidbody immediately before `Physics.Simulate`.
- **Frame discipline** — convert vectors at frame boundaries using `Frames.Transform*()`. Document which frame each value is in.
- **Config-driven** — drone physical and controller parameters live in YAML, not hard-coded. Defaults are acceptable only as fallbacks.
- **`[DisallowMultipleComponent]`** on `QuadPawn` and `CascadedController` — enforce one per GameObject.

---

## Common Pitfalls

- **Do not call `Physics.Simulate` anywhere else.** `SimulationManager` is the sole owner.
- **Axis4 semantic overloading** — in `Position`/`Velocity` mode W is altitude/Vz; in `Angle`/`Rate` mode W is throttle. Check `GoalMode` before interpreting.
- **Angular velocity sign** — Unity uses left-hand convention. `Frames.TransformAngularVelocity` negates all components when converting to FLU/FRD.
- **Hover throttle** — computed dynamically from mass via `RotorPhysicsDerived.ComputeHoverThrottle()`; do not hard-code.
- **Inertia tensor disabled** — `rb.inertiaTensor` assignment is commented out pending body-response validation. Do not uncomment without testing roll/pitch coupling.
- **`_axesBuilt` guard** — `SetRotorPhysics` is called before `Initialize` in some paths. The guard prevents double axis construction; preserve it.

---

## Planned / Stub Areas

- `CameraRig` — `ForceFPVCameraActive()` / `SwitchCamera()` are stubs on `QuadPawn`.
- Geometric controller — `ControllerKind.Geometric` case in `QuadPawn.RunController` is stubbed.
- `commandProxy.ResetToSafe()` — commented out, intended for command proxy safe-state reset on init/reset.
- Velocity constraint constants (`4.0f`, `10f`, `8f`) in `FlightController` — marked `TODO: DEFINE THESE IN CONFIG`.
- Yaw attitude PID — commented note in YAML; only rate-level yaw is tuned.
