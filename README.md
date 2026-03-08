<p align="center">
  <h1 align="center">QuadSim</h1>
  <p align="center">
    Research-oriented quadrotor simulation built in Unity.
    <br />
    Modular controllers, explicit control authority, and scriptable at every level.
  </p>
</p>

<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#scene-setup">Scene Setup</a> •
  <a href="#control-sources">Control Sources</a> •
  <a href="#flight-modes">Flight Modes</a> •
  <a href="#drone-configuration">Drone Configuration</a> •
  <a href="#python-sdk">Python SDK</a> •
  <a href="#architecture">Architecture</a>
</p>

---

## What is QuadSim?

QuadSim is a simulation platform for quadrotor research. It supports classical cascaded controllers, swappable control sources, and a Python SDK for external scripting. The goal is a platform where controls engineers and autonomy researchers can prototype, test, and validate flight algorithms with full transparency into what the system is doing and why.

**Key features:**
- Cascaded PID controller (Position, Velocity, Angle, Rate) ported from a proven Unreal Engine implementation
- Three control sources: in-sim UI (gamepad/keyboard), Internal C# scripts, External Python via ZeroMQ RPC
- Explicit authority system where only one source controls the drone at a time
- YAML-based drone configuration (mass, inertia, rotor geometry, PID gains, flight limits)
- Deterministic physics stepping with pause/step/resume
- Real-time telemetry streaming over PUB/SUB
- FLU coordinate frame output for standard robotics conventions

---

## Getting Started

### Requirements

- Unity 2022.3 LTS or newer (2023.x works too)
- NuGet packages (included): NetMQ, MessagePack-CSharp, YamlDotNet

### Clone

```
git clone https://github.com/ninonick0607/Unity_QuadSim.git
```

Open the project in Unity Hub. Let it import and compile.

---

## Scene Setup

QuadSim is designed to drop into any Unity scene. You need two things:

### 1. SimRoot Prefab

Drag the **SimRoot** prefab into your scene hierarchy. This is the simulation backbone. It contains:

- **SimulationManager** — physics stepping, time control, deterministic loop
- **ControlAuthorityManager** — who has control (UI, Internal, External)
- **ExternalRpcAdapter** — ZeroMQ server for Python connections
- **DroneManager** — spawning, selection, config loading

SimRoot auto-resolves its internal references on Awake. You don't need to wire anything up manually.

### 2. QuadPawn Prefab

The **QuadPawn** prefab is your drone. DroneManager spawns it automatically using the config specified in its inspector. You don't manually place drones in the scene. On play, DroneManager checks if any drones exist, and if not, spawns one at the default spawn origin with the selected YAML config.

That's it. Hit Play and you have a flying drone.

### Using the Default Scene

The project includes a ready-to-go scene with SimRoot and a ground plane already configured. Open it from `Assets/Scenes/` and hit Play. The drone spawns, the HUD telemetry panel shows up, and you can fly immediately with whatever control source is set as default.

---

## Control Sources

QuadSim has three control sources. Only one has authority at a time.

### UI (Default)

Gamepad or keyboard input. This is the default if you don't change anything. Pick up a controller and fly, or use the on-screen controls.

### Internal (C# Scripts)

Drop a MonoBehaviour into your scene that uses `SimCoreApi` and `QuadSimApi` to send commands. This is for writing flight behaviors, test scripts, or autonomy logic directly in Unity.

```csharp
// Get the APIs
var sim = SimCoreApi.Get();
var drone = QuadSimApi.Selected();

// Register as Internal and request authority
sim.NotifyInternalConnected();
sim.RequestAuthority(InputSource.Internal);

// Fly
drone.SetMode(GoalMode.Rate);
drone.SendCommand(InputSource.Internal, new Axis4(0, 0, 30, 0.44f));

// Read sensors
var sensors = drone.GetSensorData();
Debug.Log($"Altitude: {sensors.GpsPosition.y}");
```

See `Assets/Scripts/Examples/InternalApiExample.cs` for a full working demo that exercises mode switching, velocity commands, resets, and time control.

### External (Python via RPC)

Connect from Python using the [QuadSim Python SDK](https://github.com/ninonicok0607/QuadSimLib). The ExternalRpcAdapter on SimRoot runs a ZeroMQ server (REQ/REP on port 5555, PUB/SUB on port 5556) that bridges Python commands to the Unity main thread.

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()
    drone.takeoff(3.0)
    drone.fly_to(x=5, y=0, z=3)
    drone.land()
```

More on this in the [Python SDK](#python-sdk) section.

### Switching Authority

Set the default source in the **ControlAuthorityManager** inspector on SimRoot:

| Setting | What happens |
|---------|-------------|
| **Default Source = UI** | Gamepad/keyboard active immediately |
| **Default Source = Internal** | Waits for an Internal script to call `NotifyInternalConnected()` |
| **Default Source = External** | Waits for a Python client to connect, auto-accepts |

If the default source doesn't connect within the timeout (configurable, default 60s), the HUD shows an option to switch to UI.

You can also switch at runtime. The authority system is explicit: calling `RequestAuthority(InputSource.Internal)` only succeeds if that source is in a Connected state. No silent takeovers.

---

## Flight Modes

The cascaded controller supports these goal modes, selectable per-source:

| Mode | What you command | What the controller does |
|------|-----------------|------------------------|
| **Rate** | Roll/pitch/yaw rates + throttle | Innermost loop only. Direct rate control. |
| **Angle** | Roll/pitch angles + yaw rate + throttle | Angle PID outputs rates, rate PID stabilizes. |
| **Velocity** | Vx, Vy, Vz + yaw rate | Velocity PID outputs angles, cascades down. |
| **Position** | X, Y, Z + yaw | Full cascade. Position PID outputs velocity, cascades all the way down. |
| **Passthrough** | Motor commands [0,1] per motor | Bypasses the entire controller. Direct motor control. |
| **Wrench** | Force + torque vectors | Bypasses controller and actuation model. Raw physics. |

Each mode enters the cascade at a different level. Rate is the innermost loop, Position is the outermost. The controller automatically runs all stages below whatever you set.

---

## Drone Configuration

Drone parameters live in YAML files at `Assets/Configs/Drones/`. The default is `StandardDrone_Config.yaml`. DroneManager loads the config specified in its inspector field.

### YAML Structure

```yaml
name: "StandardDrone"

physical_properties:
  mass_kg: 1.0
  com_offset: { x: 0, y: 0, z: 0 }
  inertia: { x: 0.01, y: 0.02, z: 0.01 }
  arm_length: 0.22
  linear_drag_coef: 0.1
  angular_drag_coef: 0.05

propulsion:
  prop_diameter: 0.254
  max_rpm: 10000
  thrust_coef: 0.025
  torque_coef: 0.0005
  propellers:
    - { id: 0, x: 0.155, y: 0, z: 0.155, dir: 1 }    # FL, CW
    - { id: 1, x: 0.155, y: 0, z: -0.155, dir: -1 }   # FR, CCW
    - { id: 2, x: -0.155, y: 0, z: 0.155, dir: -1 }   # BL, CCW
    - { id: 3, x: -0.155, y: 0, z: -0.155, dir: 1 }   # BR, CW

flight_limits:
  max_vel_xy: 5.0
  max_vel_z: 3.0
  max_angle: 35.0
  max_rate_rp: 220.0
  max_rate_yaw: 200.0

controller:
  position_gains:
    x: { p: 1.0, i: 0.0, d: 0.0 }
    y: { p: 1.0, i: 0.0, d: 0.0 }
    z: { p: 1.5, i: 0.0, d: 0.0 }
  velocity_gains:
    x: { p: 2.0, i: 0.1, d: 0.0 }
    y: { p: 2.0, i: 0.1, d: 0.0 }
    z: { p: 5.0, i: 0.5, d: 0.0 }
  attitude_gains:
    roll:  { p: 6.0, i: 0.0, d: 0.0 }
    pitch: { p: 6.0, i: 0.0, d: 0.0 }
  rate_gains:
    roll:  { p: 0.15, i: 0.0, d: 0.002 }
    pitch: { p: 0.15, i: 0.0, d: 0.002 }
    yaw:   { p: 0.3,  i: 0.0, d: 0.0 }
```

### What Each Section Controls

**physical_properties** — Mass, center of mass offset, diagonal inertia tensor, arm length, and drag coefficients. These feed directly into the Rigidbody and the control allocator.

**propulsion** — Rotor model (diameter, RPM, thrust/torque coefficients) and individual rotor placement. Each rotor has a body-frame position and spin direction (+1 CW, -1 CCW). The control allocator builds its effectiveness matrix from this.

**flight_limits** — Maximum velocities, angles, and angular rates. These clamp the output at each cascade stage so the controller doesn't demand physically unreasonable setpoints.

**controller** — PID gains for each cascade stage (position, velocity, attitude, rate) per axis. Tuning these is where most of the flight behavior comes from.

### Hot Reload

You can edit the YAML while the sim is running. The drone config can be reloaded through `QuadPawn.ReloadYaml()` or through the inspector. PID gains can also be tuned live through the RPC interface from Python.

---

## Python SDK

The [QuadSim Python SDK](https://github.com/ninonicok0607/QuadSimLib) (`quadsim-sdk`) provides full programmatic control:

```bash
pip install -e path/to/QuadSimLib
```

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()

    # High-level
    drone.takeoff(altitude=3.0)
    drone.fly_to(x=5, y=0, z=3, speed=2.0)

    # Low-level on the same object
    drone.set_mode("rate")
    drone.send_command(roll=0, pitch=0, yaw=30, throttle=0.44)
    sensors = drone.get_sensors()

    drone.land()
```

The SDK communicates over ZeroMQ (REQ/REP for commands, PUB/SUB for streaming telemetry). All data uses FLU coordinates. See the [SDK README](https://github.com/ninonicok0607/QuadSimLib) for full API docs.

### Connection Setup

1. Set **Default Source = External** on the ControlAuthorityManager (SimRoot inspector)
2. Hit Play in Unity
3. Run your Python script — it connects automatically
4. Authority is granted to External on connection

If Default Source is UI or Internal, an External client can still connect but will need explicit authority handoff (either from the UI popup or programmatically).

---

## Architecture

```
SimRoot (GameObject)
├── SimulationManager        — physics stepping, time control
├── ControlAuthorityManager  — source lifecycle, authority switching
├── ExternalRpcAdapter       — ZeroMQ server, Python bridge
└── DroneManager             — spawning, selection, config loading
    └── QuadPawn (spawned)   — the drone itself
        ├── Rigidbody
        ├── SensorManager    — IMU + GPS at configurable rates
        ├── CascadedController — Position→Velocity→Angle→Rate PID cascade
        ├── FlightCommandProxy — validated command routing
        ├── ModeCoordinator  — goal mode state machine
        └── QuadSimApi       — per-drone API facade
```

### Key Design Principles

**Explicit registration over scene searching.** Subsystems use serialized field injection and direct registration. No `FindObjectOfType` at runtime.

**Controllers are authority-agnostic.** The CascadedController reads commands through `FlightCommandProxy` and never knows if the command came from a gamepad, a C# script, or Python.

**Layer separation.** `SimCore` handles sim-wide concerns (time, authority). `DroneCore` handles per-drone concerns (control, sensors, physics). SimCore never references DroneCore directly.

**One authority at a time.** The `ControlAuthorityManager` enforces that exactly one source has control. Switching is deliberate and visible. The `FlightCommandProxy` rejects commands from non-authority sources.

---

## Related

- **QuadSim Python SDK** — [github.com/ninonicok0607/QuadSimLib](https://github.com/ninonicok0607/QuadSimLib)

---

## License

MIT
