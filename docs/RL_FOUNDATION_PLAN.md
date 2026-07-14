# QuadSim RL Foundation Implementation Plan

## Purpose

This document defines the minimum foundation needed to begin reinforcement-learning experiments in QuadSim without turning the simulator into a large RL-specific framework.

The first milestone is intentionally small:

- Two quadrotors.
- One deterministic 1v1 scenario.
- State-vector observations only.
- Continuous acceleration + yaw-rate actions.
- No weapons or projectiles.
- A virtual capture cone with a dwell timer.
- One scripted opponent before self-play.
- Headless deterministic lockstep execution.

The architecture should still support later 2v2 self-play, visual observations, weapons, domain randomization, and centralized critics.

---

## Architectural Boundary

Keep the simulator and task logic separated.

### Unity owns

- Drone rigidbody physics.
- Applying all agents' actions atomically before a shared physics step.
- Episode-wide physical reset.
- Exact ground-truth state capture at one simulation timestamp.
- Collision and contact facts.
- Deterministic sensor reseeding.
- Human-view camera rigs.

### Python owns

- Scenario definitions and spawn-geometry generation.
- Observation construction and normalization.
- Reward functions.
- Capture-cone and dwell-timer logic.
- Episode termination and truncation.
- Scripted opponents.
- PettingZoo/RLlib/MARLlib adapters.

This keeps combat rules easy to iterate on and prevents Unity from becoming coupled to one RL library or one task.

---

# Phase 1: Minimum RL Foundation

## 1. Stable Multi-Agent Identity

Every controllable drone must have a stable agent identity that does not depend on which drone is currently selected in the UI.

Required fields:

- `agent_id`, such as `red_0` or `blue_0`.
- `team_id`, such as `red` or `blue`.
- `drone_index` or persistent `drone_id` used by RPC.

The first implementation may use two already-spawned drones, but the RPC protocol must not assume exactly two agents.

### Required behavior

- `list_drones` returns all available drones in deterministic order.
- Every multi-agent request identifies drones explicitly.
- The order of actions in a request must not change the physical result.

---

## 2. Atomic Multi-Agent RPC

The current single-drone composite step cannot be called once per agent because that would advance physics between agents. A multi-agent step must apply every action first and advance the world only once.

### New RPC: `step_multi`

Example request:

```text
{
  "method": "step_multi",
  "count": 5,
  "actions": [
    {
      "agent_id": "red_0",
      "mode": "acceleration",
      "x": 1.0,
      "y": 0.0,
      "z": 0.2,
      "w": 10.0
    },
    {
      "agent_id": "blue_0",
      "mode": "acceleration",
      "x": -0.5,
      "y": 0.4,
      "z": 0.0,
      "w": -5.0
    }
  ]
}
```

Required sequence inside Unity:

1. Validate all agent IDs and actions.
2. Convert every command through the existing frame boundary.
3. Apply every command without stepping physics.
4. Advance exactly `count` fixed physics steps.
5. Capture all RL states after the final step.
6. Return all states with one shared `sim_time`, `episode_id`, and `step_index`.

Example response shape:

```text
{
  "episode_id": 12,
  "step_index": 48,
  "sim_time": 0.96,
  "agents": {
    "red_0": { ... compact RL state ... },
    "blue_0": { ... compact RL state ... }
  },
  "events": [ ... ]
}
```

### Validation rules

- Reject duplicate actions for one agent.
- Reject unknown agents.
- Require one action per active agent for the first version.
- Reject invalid action dimensions or non-finite values.
- Clamp only at the documented action boundary, not silently in several layers.
- Missing actions may be supported later through an explicit neutral-action policy.

---

## 3. Episode Reset

Add one episode-level reset instead of issuing separate reset calls for every drone.

### New RPC: `reset_episode`

Python sends an already-resolved episode specification containing exact spawn poses and seeds.

Example request:

```text
{
  "method": "reset_episode",
  "seed": 12345,
  "agents": [
    {
      "agent_id": "red_0",
      "team_id": "red",
      "drone_index": 0,
      "position_enu": [-5.0, 0.0, 5.0],
      "yaw_deg": 0.0
    },
    {
      "agent_id": "blue_0",
      "team_id": "blue",
      "drone_index": 1,
      "position_enu": [5.0, 0.0, 5.0],
      "yaw_deg": 180.0
    }
  ],
  "settle_steps": 2
}
```

Required reset sequence:

1. Pause simulation stepping.
2. Increment `episode_id` and reset `step_index`.
3. Clear all buffered commands and wrenches.
4. Reset controller state and motor-lag state.
5. Reset rigidbody linear and angular velocity.
6. Apply every pose before any physics step.
7. Reseed each drone and each sensor stream deterministically.
8. Clear collision/contact/event state.
9. Reset episode-local simulation time.
10. Run a small deterministic number of settle steps.
11. Return a fresh state snapshot for all agents.

The reset must be atomic from Python's perspective.

---

## 4. Independent Deterministic Seeding

Each episode needs repeatability without giving every drone identical noise streams.

Use deterministic seed derivation:

```text
episode_seed
  -> agent_seed(agent_index)
      -> imu_seed
      -> gps_seed
      -> baro_seed
      -> mag_seed
```

A stable integer mixing function should derive seeds from:

- Master episode seed.
- Agent index or stable agent ID.
- Sensor stream ID.

### Required code changes

- `GaussianRng` needs a `Reset(seed)` or reseed operation.
- Each sensor exposes a reseed method.
- `SensorManager` exposes one episode-seed method and applies unique stream offsets.
- Resetting the same episode seed must reproduce the same noise sequence.
- Different agents must not receive identical samples merely because they were spawned from the same prefab.

---

## 5. Compact RL State

Add a dedicated RL state path instead of expanding normal sensor telemetry with privileged ground truth.

### Proposed `RLAgentState`

Required initial fields:

```text
agent_id
team_id
position_enu[3]
velocity_enu[3]
orientation_flu_to_enu[4]
body_velocity_flu[3]
body_angular_velocity_flu[3]
forward_enu[3]
up_enu[3]
is_valid
has_collision
is_grounded
```

Optional later fields:

```text
linear_acceleration
motor_outputs
remaining_energy
health
weapon_state
camera_visibility
```

### Rules

- The state is captured directly from the rigidbody/transform after the shared physics step.
- World quantities are always ENU.
- Body quantities are always FLU.
- Frame conversion remains centralized in `Frames.cs`.
- All states in one `step_multi` response share the same timestamp.
- Privileged RL state remains separate from the ordinary sensor API used for sim-to-real controllers.

---

## 6. Collision and Episode Events

The first task needs a small event stream, not a full damage system.

Initial event types:

```text
ground_contact
agent_collision
arena_exit
invalid_state
```

Each event should contain:

```text
type
agent_id
other_agent_id (optional)
sim_time
```

Capture-cone success will initially be computed in Python from agent states. Unity only needs to report physical facts that Python cannot infer reliably, such as collisions.

---

# Scenario and Spawn Geometry

## Scenario definitions live in Python

A scenario should resolve into exact spawn poses before `reset_episode` is called. Unity should not contain hard-coded concepts such as “head-on duel” or “tail chase.”

### Core data classes

```python
@dataclass(frozen=True)
class SpawnSlot:
    agent_id: str
    team_id: str
    drone_index: int
    position_enu: tuple[float, float, float]
    yaw_deg: float

@dataclass(frozen=True)
class ArenaSpec:
    center_enu: tuple[float, float, float]
    radius_m: float
    floor_z_m: float
    ceiling_z_m: float

@dataclass(frozen=True)
class CaptureSpec:
    max_range_m: float
    half_angle_deg: float
    dwell_time_s: float

@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    control_hz: float
    max_episode_s: float
    arena: ArenaSpec
    capture: CaptureSpec
    spawns: tuple[SpawnSlot, ...]
```

### Geometry generators

Convenience functions generate the `SpawnSlot` list:

```text
head_on_duel(...)
tail_chase(...)
parallel_start(...)
crossing_start(...)
random_ring(...)
```

Only `head_on_duel` is required for the first milestone.

### First spawn geometry

Use a symmetric head-on start:

```text
Arena center:       ENU (0, 0, 5)
Initial separation: 10 m
Red position:       (-5, 0, 5), yaw 0 degrees
Blue position:      ( 5, 0, 5), yaw 180 degrees
Arena radius:       20 m
Floor:              0.75 m
Ceiling:            15 m
```

Optional randomization for later:

- Small lateral offset.
- Small altitude offset.
- Small yaw perturbation.
- Initial velocity perturbation.

Randomization should be disabled until the deterministic base task works.

---

# First RL Task: 1v1 Capture-Cone Duel

## Action space

Use a continuous normalized action:

```text
[ax_body, ay_body, az_body, yaw_rate]
```

The environment maps `[-1, 1]` into documented limits, then sends acceleration mode commands through the existing controller stack.

Suggested initial limits:

```text
ax:       +/- 5 m/s^2
ay:       +/- 5 m/s^2
az:       +/- 4 m/s^2
yaw rate: +/- 120 deg/s
```

These are starting values, not permanent physical limits.

Do not begin with motor commands or allocated wrench control. The first experiment should test task learning rather than forcing the policy to learn stabilization and combat simultaneously.

## Observation space

Each policy receives ego-centric features.

Initial observation:

```text
Ego:
- altitude
- sin/cos roll
- sin/cos pitch
- body velocity xyz
- body angular velocity xyz

Opponent relative to ego:
- relative position in ego FLU xyz
- relative velocity in ego FLU xyz
- opponent forward vector in ego FLU xyz
- normalized range
- normalized closing speed
- ego boresight angle to opponent
- opponent boresight angle to ego
```

For later centralized critics, Python may additionally construct a privileged global state from both `RLAgentState` objects.

## Capture rule

A drone captures the opponent when all conditions remain true continuously:

```text
range <= max_range_m
boresight angle <= half_angle_deg
line of sight is valid, when occlusion is introduced
condition held for dwell_time_s
```

Initial values:

```text
max range: 10 m
half angle: 15 degrees
dwell time: 0.5 s
```

## Initial reward

Keep the reward understandable and log every component separately.

Suggested terms:

```text
+ terminal capture
- terminal captured
+ decrease in ego boresight angle
- decrease in opponent boresight angle toward ego
+ movement toward useful engagement range
- arena violation
- ground or agent collision
- excessive control effort
- time penalty for inactivity
```

Do not use a large collection of shaping terms initially. The first test should reveal whether the observation/action interface and capture geometry are learnable.

## Initial episode termination

Terminate when:

- Either agent completes capture dwell.
- Either agent crashes or becomes invalid.
- Both agents collide severely.

Truncate when:

- Episode reaches `max_episode_s`.
- Either agent exits the arena for longer than a short grace period.

Use separate `terminated` and `truncated` values in the Python environment.

---

# Camera Foundation

Human-view cameras and future visual RL observations must remain separate concerns.

## `DroneCameraRig`

Add a pure component to each drone prefab. It should not control the drone, read commands, or participate in physics.

Responsibilities:

- Expose named mount transforms:
  - FPV mount.
  - Chase/follow target.
  - Orbit target.
  - Optional payload or gimbal mount.
- Expose the owning drone ID.
- Allow a global camera director to bind to the rig.
- Remain inactive and inexpensive in headless mode.

Suggested API:

```csharp
public sealed class DroneCameraRig : MonoBehaviour
{
    public Transform FpvMount { get; }
    public Transform FollowTarget { get; }
    public Transform OrbitTarget { get; }
}
```

Remove direct camera ownership from `QuadPawn` over time. `QuadPawn` may expose the component reference, but it should not implement camera behavior.

## `CameraDirector`

A single optional scene-level component controls the active human camera mode:

```text
Free
Orbit
Follow
FPV
```

It binds to the selected drone's `DroneCameraRig` but remains independent of `DroneManager` physics and RL logic.

## Future `DroneVisionSensor`

Visual RL should later use a separate component that renders into a `RenderTexture` at an explicit simulation rate. It should not reuse the human-view camera because:

- Human view selection changes during a run.
- Headless training may disable display cameras.
- RL image timing must be deterministic.
- Multiple agents may need simultaneous observations.

`DroneVisionSensor` is deferred from the first state-vector RL milestone.

---

# Python SDK Foundation

Keep this small and independent of MARLlib.

## SDK additions

### Existing-file changes

- `quadsim/sim.py`
  - `list_drones()`
  - `reset_episode(...)`
  - `step_multi(...)`
- `quadsim/types.py`
  - `RLAgentState`
  - `MultiAgentStepResult`
  - `EpisodeResetResult`

### New RL files

Suggested minimal package:

```text
quadsim/rl/
├── scenario.py       # scenario dataclasses and spawn generators
└── dogfight_env.py   # environment, observation, reward, termination
```

A third file may be added when scripted opponents grow:

```text
quadsim/rl/baselines.py
```

## Environment API

Implement a small native environment first, with a PettingZoo-style parallel interface:

```python
obs, info = env.reset(seed=123)
obs, rewards, terminated, truncated, info = env.step(action_dict)
```

Agent IDs remain stable for the entire episode:

```text
red_0
blue_0
```

Then add thin adapters for RLlib or MARLlib. Do not make QuadSim depend on MARLlib's older Ray/Gym versions.

---

# Unity File Plan

## New files

```text
Assets/QuadSim/Scripts/RL/RLAgentState.cs
Assets/QuadSim/Scripts/RL/RLEpisodeManager.cs
Assets/QuadSim/Scripts/DroneCore/Camera/DroneCameraRig.cs
Assets/QuadSim/Scripts/RuntimeUI/Camera/CameraDirector.cs   # optional in first pass
```

## Existing files to extend

```text
ExternalRpcAdapter.cs
- list_drones
- reset_episode
- step_multi
- shared multi-agent response serialization

RpcProtocol.cs
- safe helpers for arrays/lists of maps

QuadSimApi.cs
- GetRLState()
- explicit lookup by stable ID/index

DroneManager.cs
- stable lookup methods
- optional agent metadata registration

SensorManager.cs and sensor RNG classes
- episode reseeding

Frames.cs
- explicit world-vector conversion helper if needed
```

No Unity reward classes, policy classes, or MARLlib-specific classes are required for the first milestone.

---

# Testing and Acceptance Criteria

## Transport and timing

- Two agents receive actions in one request.
- Physics advances exactly once per environment step.
- Reversing action order in the request produces the same result.
- Every returned state has the same timestamp.
- `step_index` increases by exactly one environment step.

## Reset determinism

- Same scenario seed produces identical initial states.
- Same scenario seed and action sequence produce identical trajectories.
- Different agent seeds produce different noise streams.
- Repeating reset does not preserve stale velocity, controller, collision, or motor state.

## Frames

- RL world position and velocity are ENU.
- RL body velocity and angular velocity are FLU.
- Relative observations are invariant to global translation.
- A known yaw rotation produces the expected ego-frame opponent vector.

## First task

- Scripted pursuit agent can complete capture.
- Hovering and spinning in place cannot exploit the reward.
- Arena and ground termination work.
- A random policy can run thousands of steps without RPC or reset failure.

## Headless performance

- Rendering and camera components are disabled in headless training.
- No per-tick logging in the training path.
- One Unity process can complete repeated episodes without restart.
- Existing disjoint RPC/telemetry port arguments continue to support parallel workers.

---

# Explicitly Deferred

The following are not required before the first RL run:

- Missiles or projectiles.
- Damage and health systems.
- 2v2 teams.
- Communication actions.
- Self-play leagues or population-based training.
- Camera observations.
- Occlusion-aware vision.
- Dynamic spawning/despawning during an episode.
- PX4 in the RL training loop.
- Motor-level policy learning.
- Domain randomization.
- Curriculum scheduling.
- Replay viewer and dataset export.

The protocol and state shapes should leave room for these features, but they should not be implemented preemptively.

---

# Recommended Implementation Order

## Milestone A: Multi-agent simulator boundary

1. Stable agent lookup.
2. `RLAgentState`.
3. `list_drones`.
4. `step_multi`.
5. `reset_episode`.
6. Per-episode reseeding.
7. Determinism integration tests.

## Milestone B: First Python environment

1. `ScenarioSpec` and `head_on_duel()`.
2. Python SDK wrappers for reset and step.
3. Ego-centric observation builder.
4. Capture-cone state machine.
5. Reward and termination.
6. Scripted pursuit opponent.
7. Random-policy smoke test.

## Milestone C: First learning run

1. Train one agent against the scripted opponent.
2. Evaluate capture rate from fixed seeds.
3. Add symmetric policy control for both agents.
4. Begin simple self-play only after the environment is stable.

## Milestone D: Camera cleanup

1. Add `DroneCameraRig` to the drone prefab.
2. Move FPV/follow/orbit mount ownership out of `QuadPawn`.
3. Add optional `CameraDirector` for human viewing.
4. Keep visual-RL capture deferred to `DroneVisionSensor`.

---

# Expected Scope

This foundation should remain relatively small.

### Unity

- Approximately three required new classes.
- One optional camera-director class.
- Focused modifications to RPC, drone lookup, state capture, frames, and sensor reseeding.

### Python SDK and environment

- Two small RL files.
- A few additions to `sim.py` and `types.py`.
- One integration test module.

The largest conceptual change is the atomic multi-agent RPC. Once that boundary is correct, the initial scenario, observation, reward, and environment code should remain ordinary Python and be easy to iterate on.