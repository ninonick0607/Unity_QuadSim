# QuadSim platform and extension ideas

Date: 2026-10-01. Status: future-work idea sheet, not an implementation commitment.

## Direction

Borrow AtomEngine's separation of a base simulator, entity capabilities, and independently authored extension packages. Let researchers add and share functionality without maintaining a fork of QuadSim.

Support the same capabilities through a standalone application with UI/Python control and an installable Unity package. A future Unreal backend could implement shared QuadSim contracts. Extensions should work across the supported delivery modes where their dependencies allow it.

## Proposed boundaries

- **Simulation foundation:** entity identity, clock/stepping, lifecycle/reset, control authority, units/frames, and command/observation contracts.
- **Extensions:** controllers, sensors, vehicle definitions, disturbances, experiment tasks, metrics, and optional UI panels. Components describe per-entity capabilities/state; systems implement behavior on a defined simulation schedule.
- **Python:** experiment orchestration, autonomy, learning, evaluation, and control through the simulation API.
- **Engine integrations:** C# for Unity-specific physics, rendering, and UI; potentially C++/Blueprints for Unreal. Expose useful extension operations and observations to Python.
- **Modkit:** starter packages, documented registration interfaces, examples, validation, and package metadata covering versions, dependencies, and supported backends.

QuadSim already has simulation/drone API facades, lifecycle and pre/post-physics hooks, YAML vehicle definitions, and a separate Python SDK. Sensor construction and controller selection still contain fixed internal choices. First prove one separately packaged controller or sensor plus an experiment that uses it without editing the simulator internals.

Build the standalone app using the public package APIs. Start with project-installed engine extensions and external Python/configuration packages; downloadable executable mods require a separate loading design. Standardize portable contracts before attempting Unreal support; matching APIs do not guarantee matching physics. ROS can be another integration layer.

## Candidate extension: swarm choreography

The user wants to pursue this direction. See the separate [swarm choreography and night-show lighting idea](swarm-show-idea.md) for the first demonstration and proposed light component.

Create a reusable package for formation shapes, drone-to-position assignment, time-indexed trajectories, synchronized colors, playback, and evaluation. It could serve visual drone shows first, then formation-control and decentralized-swarm research.

The inspected code already supports multiple registered drones, stable identities, batched per-agent commands with shared stepping, seeded episode resets, and collision/state reporting. Python resets existing drones; it does not create a fleet through `reset_episode`.

Start with a 16-drone grid-to-ring demonstration: bounded trajectories, separation checks throughout transitions, coordinated colors, tracking-error metrics, and repeatable resets. This is a proposed test size, not a measured capacity. Benchmark larger fleets before promising scale; consider separate full-physics and lightweight visual-preview modes. Decentralized research would additionally need restricted observations and modeled communication delay/loss; close-proximity fidelity may require aerodynamic interaction models.

## Entity/model authoring idea

Explore a readable entity manifest joining a stable type ID/version, model reference, physics configuration, named attachment frames, and optional components such as sensors and show lights. Keep entity definitions separate from spawned instance IDs and scenario state. Current authoring already separates YAML physics from catalog-selected Unity prefabs; the improvement is a complete, validated description of their relationship.

Keep current YAML calibration/propulsion settings and Unity prefabs working. Prototype USD as an optional source for model hierarchy, transforms, materials, and attachment frames, with an explicit mapping to QuadSim components. USDA supports readable text; USDC supports binary assets. glTF/GLB is another candidate for visual asset delivery. These are authoring/import choices, not evidence of better physics throughput.

Define one authority for each physical property and mount transform; generated Unity assets should not become a competing source. Validate units, coordinate conversion, inertia, rotor locations, and sensor/light mounts during import. Existing schema-v1 geometry uses Unity body axes, so a future portable convention requires versioned conversion. Test one vehicle before selecting or migrating formats; ordinary asset import does not establish Isaac/Unity physics or material equivalence.

References: [OpenUSD introduction](https://openusd.org/release/intro.html), [USD file encodings](https://openusd.org/release/usdfaq.html), [Unity USD Importer 1.0 documentation](https://docs.unity3d.com/Packages/com.unity.importer.usd@1.0/manual/index.html), [glTF overview](https://www.khronos.org/gltf/).

## Success criteria

A researcher can install an extension, configure it, run it in the standalone app or another Unity project, and reproduce a recorded experiment using its package versions, configuration, seed, and timing settings. Verify repeatability in a named runtime/platform; do not promise cross-platform bitwise determinism.

## Source pointers

- `Runtime/Scripts/SimCore/SimulationManager.cs`: lifecycle and stepping contract.
- `Runtime/Scripts/RobotCore/SensorManager.cs` and `Runtime/Scripts/DroneCore/QuadPawn.cs`: current sensor/controller coupling.
- `Runtime/Scripts/SimCore/Rpc/ExternalRpcAdapter.cs`: multi-agent command and reset handlers.
- Companion QuadSimLib: `quadsim/sim.py` and `docs/reinforcement_learning.md`.

Assessment based on source/documentation inspection; no live swarm benchmark performed.
