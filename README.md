<p align="center">
  <h1 align="center">QuadSim Unity Plugin</h1>
  <p align="center">
    Installable Unity quadrotor simulation package with Python SDK control.
    <br />
    Deterministic stepping, headless RPC, sensors, control allocation, and sample scenes.
  </p>
</p>

<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#scene-setup">Scene Setup</a> •
  <a href="#python-sdk">Python SDK</a> •
  <a href="#control-overview">Control Overview</a> •
  <a href="#headless--lockstep">Headless / Lockstep</a> •
  <a href="#sensors--frames">Sensors & Frames</a> •
  <a href="#drone-configuration">Drone Configuration</a> •
  <a href="#architecture">Architecture</a>
</p>

---

## What is QuadSim?

QuadSim is a research-oriented quadrotor simulation platform built in Unity for controls research, autonomy development, headless tuning, and sim-to-real workflows.

This repository is the **Unity runtime/plugin**. It provides the simulation scene components, drone prefab, sensors, controller plumbing, control allocation, physics stepping, and RPC bridge needed to run QuadSim inside a Unity project.

The Python package is separate: **`quadsim-sdk`** connects to a running QuadSim scene over ZeroMQ and is where user-facing Python control APIs live.

```text
Unity QuadSim Plugin       ← simulator runtime, prefabs, physics, sensors, RPC
quadsim-sdk / QuadSimLib   ← Python interface, high-level flight, raw modes, lockstep control
```

Use this README to install and run the Unity simulator. Use the Python SDK docs for detailed control-mode usage.

- Python SDK repo: [github.com/ninonick0607/QuadSimLib](https://github.com/ninonick0607/QuadSimLib)
- Detailed Python control modes: [QuadSimLib/docs/control_modes.md](https://github.com/ninonick0607/QuadSimLib/blob/master/docs/control_modes.md)
- Reinforcement learning guide: [QuadSimLib/docs/reinforcement_learning.md](https://github.com/ninonick0607/QuadSimLib/blob/master/docs/reinforcement_learning.md)

---

## Key Features

- Installable Unity Package Manager package.
- Drop-in `SimRoot` + `QuadPawn` scene setup.
- Deterministic fixed-step physics using scripted simulation.
- Headless Linux execution with RPC ports configurable from the command line.
- External Python control over ZeroMQ / MessagePack.
- Composite lockstep stepping for faster-than-real-time studies.
- Explicit control authority between UI, Internal C#, and External Python.
- Cascaded controller modes: position, velocity, angle, and rate.
- Low-level access: direct motors, allocated wrench, and direct rigidbody wrench.
- Sensor stack: IMU, GPS, barometer, magnetometer.
- ENU world / FLU body frames for Python-facing robotics workflows (FRD body + NED accessors for PX4).
- YAML-based drone model and controller configuration.

---

## Getting Started

### Requirements

- Unity 6 / Unity 6000.x
- An HDRP-configured project. The QuadSim scene and terrain use HDRP shaders; the easiest starting point is Unity Hub's **High Definition 3D** template.
- Git
- Git LFS installed and available on `PATH` (the package contains large drone assets)
- Python 3.10+ for Python SDK control
- A QuadSim scene containing `SimRoot`, `ExternalRpcAdapter`, and at least one configured `QuadPawn` for RPC control

### Install the Unity Package

1. Open **Window → Package Manager**.
2. Click **+** and choose **Add package from git URL...**.
3. Paste:

```text
https://github.com/ninonick0607/Unity_QuadSim.git#v0.3.0
```

QuadSim includes its pinned UImGui 7.1.0 runtime, matching ImGui.NET managed binding, and Windows, Linux, and universal macOS native libraries. Do not install UImGui or ImGui.NET separately; a second copy can cause duplicate or mismatched assembly errors.

Installing the HDRP package does not by itself assign an HDRP pipeline asset to an existing Built-in project. If the scene is pink, open **Window → Rendering → HDRP Wizard**, select the HDRP tab, and run **Fix All** before importing or reopening the sample scene.

For active development, you can omit the tag and install the latest `main` branch:

```text
https://github.com/ninonick0607/Unity_QuadSim.git
```

For reproducible research or lab machines, prefer a tagged version like `#v0.3.0`. The tag must exist before consumers can use that URL; until the release is tagged, use a local checkout of the prepared release or install `main` with the untagged URL.

### Import the Sample Scene

After the package installs:

1. Open **Package Manager**.
2. Select **QuadSim**.
3. Open the **Samples** section.
4. Import **QuadSim Starter**.
5. Open `Assets/Samples/QuadSim/0.3.0/QuadSim Starter/Maps/Land.unity` and press Play.

Delete older imported copies such as `Assets/Samples/QuadSim/0.2.0` before importing a newer sample version. Unity samples are copied into `Assets` and are not upgraded in place.

The imported sample owns the files users are expected to edit: the Land scene and terrain, `Configuration/Default_SceneConfig`, `Prefabs/DroneCatalog`, the base and environment prefabs, every drone under `Prefabs/Quad Prefabs`, the example C# script, and YAML overrides under `StreamingAssets/Configs/Drones`. Simulation code, default fallback configs, models/materials, environment internals, and third-party plugins remain package-owned under `Packages/com.nino.quadsim/Runtime/`.

Do not edit files directly under `Packages/`; Git-installed package changes can be replaced on update. Edit the imported copies under `Assets/Samples/QuadSim/...` instead.

---

## Scene Setup

QuadSim is designed to drop into a Unity scene with a small number of required objects.

### `SimRoot`

`SimRoot` is the simulation backbone. It should contain or reference:

| Component | Purpose |
|----------|---------|
| `SimulationManager` | Owns scripted fixed-step physics, pause/resume, and stepping |
| `SimCoreApi` | Global simulation API facade |
| `ControlAuthorityManager` | Tracks whether UI, Internal, or External owns control |
| `ExternalRpcAdapter` | ZeroMQ server for Python clients |
| `DroneManager` | Spawns and registers configured drones |

### `QuadPawn`

`QuadPawn` is the drone vehicle. It owns the Rigidbody, motor transforms, controller, sensors, command proxy, and per-drone API.

A typical drone prefab includes:

| Component | Purpose |
|----------|---------|
| `Rigidbody` | Physical body simulated by Unity |
| `CascadedController` | Position → velocity → angle → rate controller |
| `SensorManager` | IMU, GPS, barometer, magnetometer |
| `FlightCommandProxy` | Current command and goal mode |
| `ModeCoordinator` | Mode transitions and routing |
| `QuadSimApi` | Per-drone API used by C# and RPC |

`DroneManager` can spawn a configured `QuadPawn` automatically. For most users, the sample scene is the fastest starting point.

---

## Python SDK

Install the SDK separately from the Unity package:

```bash
pip install quadsim-sdk
```

From source:

```bash
git clone https://github.com/ninonick0607/QuadSimLib.git
cd QuadSimLib
pip install -e .
```

Start the Unity scene or headless build first, then verify the connection:

```python
from quadsim import QuadSim

with QuadSim() as sim:
    print(sim.get_status())
```

Basic flight:

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()

    drone.takeoff(altitude=3.0)
    drone.fly_to(x=5, y=0, z=3)
    drone.hover(duration=2.0)
    drone.yaw_to(heading_deg=180)
    drone.land()
```

For detailed mode-by-mode Python usage, see the SDK docs:

```text
QuadSimLib/docs/control_modes.md
```

---

## Reinforcement Learning

The Unity runtime provides the deterministic physics side of the RL boundary:
atomic multi-drone reset/stepping, stable identities, privileged RL state,
dedicated per-drone image sensors, pooled runtime obstacles/goals, and physical
collision events. Scenario logic, rewards, observations, and training remain in
Python.

Setup, API schemas, obstacle spawning, image decoding, complete SDK command
reference, and training-loop examples are documented in the SDK repository:

- [QuadSim Reinforcement Learning Guide](https://github.com/ninonick0607/QuadSimLib/blob/master/docs/reinforcement_learning.md)

---

## Control Overview

QuadSim separates **who owns control** from **what kind of command is being sent**.

### Control Sources

Only one source has authority at a time.

| Source | Used for |
|--------|----------|
| `UI` | Keyboard/gamepad/manual testing |
| `Internal` | C# scripts running inside Unity |
| `External` | Python SDK over ZeroMQ RPC |

The authority system prevents silent takeovers. External Python commands only apply when the Python client is connected and has authority.

### Control Interfaces

This README intentionally keeps the mode list short. The detailed Python command syntax lives in the SDK docs.

| Interface | What you command | Typical use |
|----------|------------------|-------------|
| High-level SDK flight | `takeoff`, `fly_to`, `hover`, `land` | Scripted demos, simple autonomy |
| Position mode | FLU position + yaw | Waypoints and navigation |
| Velocity mode | FLU velocity + yaw rate | Outer-loop guidance |
| Angle mode | Roll/pitch angle + yaw rate + throttle/altitude stage | Manual-style stabilized control |
| Rate mode | Body rates + throttle | Inner-loop controller work |
| Passthrough motors | Four normalized motor commands | PX4 bridge, mixer tests, actuator studies |
| Allocated wrench | Body torques + thrust through allocator | Geometric controller and physical feasibility tests |
| Direct wrench | Body force/torque applied to Rigidbody | Idealized controller sanity checks |
| Acceleration helper | Desired local acceleration + yaw rate through SDK helper | Policy/guidance interfaces that do not want to command torques |

Important distinction:

- **Allocated wrench** goes through motor allocation and saturation.
- **Direct wrench** bypasses the allocator and applies the requested force/torque directly to the Rigidbody.
- **Passthrough** bypasses the cascaded controller but still goes through the actuation model.

Detailed examples for switching modes from Python are in:

- [Python SDK control modes](https://github.com/ninonick0607/QuadSimLib/blob/main/docs/control_modes.md)

---

## Headless / Lockstep

QuadSim can run without a Unity window for tuning, regression tests, and faster-than-real-time studies.

Build your scene for Linux, then run:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

The simulator uses scripted physics stepping, so Python can pause the sim and advance it deterministically:

```python
from quadsim import QuadSim

CONTROL_HZ = 50.0

with QuadSim() as sim:
    drone = sim.drone()
    sim.pause()

    status = sim.get_status()
    steps_per_tick = max(1, round((1.0 / CONTROL_HZ) / status.fixed_dt))

    sensors = drone.get_sensors()
    for _ in range(1000):
        command = my_controller(sensors)
        sensors = drone.step_with_command(command, count=steps_per_tick)

    sim.resume()
```

At the default 250 Hz physics rate, `count=5` gives a 50 Hz controller update.

Why lockstep matters:

- Physics advances only when requested.
- Trials are reproducible.
- Headless studies can run faster than real time.
- Composite step calls reduce RPC round trips.

---

## Sensors & Frames

QuadSim distinguishes **world** frames (fixed to the map) from **body**
frames (attached to the vehicle). The wire is canonical: world data is
always ENU; body data is FLU by default and FRD in PX4 mode.

### Unity <-> robotics frame mapping

Unity is left-handed; all robotics frames below are right-handed.
`Frames.cs` is the single conversion boundary — no other file converts.

```
Unity WORLD:  X = East,    Y = Up,  Z = North
Unity BODY:   X = forward, Y = up,  Z = left

WORLD frames:
  ENU (E, N, U) = ( u.x,  u.z,  u.y )    # canonical world frame
  NED (N, E, D) = ( u.z,  u.x, -u.y )    # MAVLink wire / explicit fields only

BODY frames:
  FLU (F, L, U) = ( b.x,  b.z,  b.y )    # canonical body frame (ROS)
  FRD (F, R, D) = ( b.x, -b.z, -b.y )    # PX4 HIL body frame

Pseudovectors (angular velocity, torque): relabel + negate-all
(left-handed -> right-handed chirality correction).
Quaternion (Unity -> body-FLU/world-ENU): (-q.x, -q.z, -q.y, q.w)
```

Note NED is an X/Y **swap** of ENU plus a Z flip — not a sign flip of
FLU-style axes. This is why world and body conventions are documented
separately.

### Sensor fields

| Field | Frame | Notes |
|-------|-------|-------|
| `gps_position` | World **ENU** | Always ENU in every mode. `z` = altitude. |
| `gps_lat/lon/alt` | WGS84 geodetic | Ellipsoidal altitude, computed in-sim. |
| `gps_vel_ned` | World **NED** | `(vn, ve, vd)` — PX4 `HIL_GPS` wire quantity. |
| `imu_vel` | **Body** FLU/FRD | Body-frame velocity — not world. |
| `imu_ang_vel` | Body FLU/FRD | rad/s. |
| `imu_accel` | Body FLU/FRD | Specific force (gravity included), as PX4 expects. |
| `imu_attitude` | Body convention | RPY degrees, display-oriented. |
| `imu_orientation` | body(FLU) -> world(ENU) | Quaternion. FLU mode only (no FRD branch in `TransformQuaternion` yet). |
| `mag_field` | Body FLU/FRD | Gauss, WMM-derived. |
| `baro_*` | Scalars | ISA model. |
| `frame` | — | Payload self-description: `"flu"` or `"frd"` body convention. |

The `outputFrame` toggle (SensorManager / runtime UI) switches the
**body** convention only, for the PX4 HIL bridge. World-frame data never
changes convention; NED exists only at the MAVLink boundary (`Px4Link`)
and in the explicitly named `gps_vel_ned` field. The Python SDK exposes
explicit accessors (`position_ned`, `velocity_enu`, `vel_body_frd`, ...)
derived by pure relabels.

---

## Drone Configuration

Drone parameters are loaded from a versioned YAML configuration and applied to
the spawned `QuadPawn`. Schema v1 defines the mass, inertia, center of mass,
rotor geometry and direction, propulsion coefficients, motor response,
controller gains, limits, and runtime rates in explicit units.

Create and validate a complete conventional-quad profile with the Python SDK:

```bash
quadsim create-drone my_quad.yaml --name MyQuad
quadsim validate-drone my_quad.yaml
quadsim inspect-drone my_quad.yaml --output my_quad.report.json
```

Use `extends` for a small tuning overlay, then run `quadsim resolve-drone` before
copying the self-contained result into `StreamingAssets/Configs/Drones`. The
Unity runtime does not resolve external inheritance paths in a player build.

Open **Tools → QuadSim → Drone YAML Inspector** to preview the rotor layout,
coefficient-derived plant values, effectiveness rank, and condition estimate.
The full schema and calibration workflow are in `Documentation~/`.

---

## Visualization

### In Unity

The sample scene can visualize the drone, ground plane, telemetry HUD, and optional ghost/setpoint markers.

The runtime camera supports Follow, FPV, Orbit, and Free modes. Follow and FPV track the drone's rendered pose exactly so Rigidbody interpolation does not compound with camera damping or make the drone appear to hitch ahead of the view. Orbit mode retains position and rotation damping for smooth user-controlled movement.

### Headless UDP Viewer

For headless runs, stream the state from Python to a separate UDP viewer:

```python
from quadsim import UdpViz

viz = UdpViz(source="study-0")
viz.path(planned_points)

# inside your control loop
viz.sample(t, pos, target=target_pos, err=err)
```

Run the viewer in a second terminal:

```bash
python live_viewer.py --port 14660
```

The viewer uses FLU `[x, y, z]` directly.

---

## Architecture

```text
Python SDK
  └── ZeroMQ / MessagePack RPC
        └── ExternalRpcAdapter
              └── SimCoreApi / QuadSimApi
                    ├── ControlAuthorityManager
                    ├── SimulationManager
                    └── QuadPawn
                          ├── SensorManager
                          ├── CascadedController
                          ├── ControlAllocator
                          └── Rigidbody
```

### Simulation Timing

`SimulationManager` sets Unity physics to scripted mode and advances the sim through a fixed-step pipeline:

```text
PrePhysicsStep → Physics.Simulate(dt) → PostPhysicsStep
```

The default base rate is 250 Hz. Controllers, sensors, telemetry, and external clients can run at subrates using deterministic tick timing.

### RPC

The external interface uses two sockets:

| Socket | Default port | Purpose |
|--------|--------------|---------|
| REQ/REP | `5555` | Commands, queries, mode changes, stepping |
| PUB/SUB | `5556` | Telemetry streaming |

Both sockets bind to `127.0.0.1` by default. The headless executable can
override the address and ports explicitly:

```bash
-rpcBindAddress 127.0.0.1 -rpcPort 5555 -telemetryPort 5556
```

Remote binding is not authenticated or encrypted and is outside the supported
0.3.0 configuration.

---

## Project Structure

Package-root repository layout:

```text
Unity_QuadSim/
├── package.json
├── Runtime/
│   ├── Scripts/
│   ├── Prefabs/
│   ├── Resources/
│   └── Plugins/ThirdParty/
├── Editor/
├── Samples~/
│   ├── QuadSim Starter/
│   └── PX4 SITL Tools/
└── Documentation~/
```

The repository root is the installable UPM package. Develop and test it from a separate Unity host project by adding this repository as a local or Git package dependency.

---

## Related

- **Python SDK / QuadSimLib**: [github.com/ninonick0607/QuadSimLib](https://github.com/ninonick0607/QuadSimLib)
- **Unity plugin**: [github.com/ninonick0607/Unity_QuadSim](https://github.com/ninonick0607/Unity_QuadSim)
- **Research controller repo**: `Resnet-GeometricController`

---

## License

MIT
