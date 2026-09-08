# QuadSim runtime UI and camera handoff

Last updated: 2026-09-08  
Repository: `Unity_QuadSim` (`com.nino.quadsim`, package version 0.3.0)  
Current branch / base: `test` at `28731b4` (`UI Setup`)  
Primary development scene used for visual checks: the imported `Land` sample in a separate Unity project  

## Read this first

This document is intended for another Codex agent continuing the runtime simulator interface on another computer. Preserve the current worktree. At the time of handoff, the latest camera and config-driven model work is **not committed**. A clone of the remote repository will therefore not contain all work described here until the user commits/pushes it or transfers the working tree/patch.

Do not run `git reset --hard`, discard untracked files, or replace package files from the imported Unity sample. Inspect `git status` and `git diff` before editing. The untracked `output/` directory contains release-review documents and is unrelated to the HUD/camera changes; do not assume it should be committed.

The temporary diagnostic file `Editor/TemporaryCameraProbe.cs` was deleted. If an old Unity Console row still shows its former `UnityEngine.InputSystem` compile error, that is console history. A subsequent import compiled successfully, and the file no longer exists. `[DroneRootBootstrap] Built drone components on ...` is an informational `Debug.Log` emitted by `OnValidate`, not an error.

Read these focused documents after this one:

- `Documentation~/runtime-workspace.md` — complete native HUD endpoint inventory and performance design.
- `Documentation~/camera-navigation.md` — global display camera behavior and controls.
- `Documentation~/drone-model-catalog.md` — config-to-prefab mapping and FPV mount rules.
- `README.md` — package installation, simulation, RPC/Python, frames and broader architecture.

### Starter prompt for the next Codex agent

Paste this after the full worktree is available on the other computer:

> Continue the QuadSim native runtime UI and camera work. First read `Documentation~/codex-runtime-ui-handoff.md`, then `Documentation~/runtime-workspace.md`, `Documentation~/camera-navigation.md`, and `Documentation~/drone-model-catalog.md`. Inspect `git status` and preserve all existing changes and Unity `.meta` files. Verify the current compile plus Editor/PlayMode tests before editing. Start with the remaining manual checks listed in the handoff—especially sustained WASD/Q/E movement, distinct config-driven drone spawning, FPV mount placement, and a standalone build—then fix any issues you find. Keep the HUD optional/headless-safe, preserve deterministic simulation ordering and authority checks, and build on the existing UI Toolkit architecture rather than replacing it.

## Product direction

QuadSim is a research-oriented Unity quadcopter simulator designed for faster-than-real-time and deterministic simulation, reinforcement learning, multi-drone experiments, autonomy research, a Python SDK and future ROS2/networking workflows. The standalone build should feel like a lightweight robotics simulation editor in the family of Gazebo, Webots, RViz, Blender and the Unity Editor. It should remain an engineering tool rather than a game HUD.

The runtime interface is replacing the existing ImGui presentation while preserving the established simulation APIs and behavior. Frontend placeholders are acceptable for backends that do not exist yet, but visible controls should have a real use. Avoid redesigning physics, controllers or RPC unless required for the interface connection.

The chosen frontend is Unity UI Toolkit. It provides retained UI, styling, virtualized lists and low presentation overhead. The workspace is created at runtime and owns its `UIDocument`, `PanelSettings`, shell and window lifecycle. It temporarily disables the legacy `ImGuiRuntimeBootstrap` and UImGui renderer; disabling the workspace restores them.

## Simulator mechanics that the UI must respect

### Fixed-step simulation

`SimulationManager` is the authoritative clock and stepping owner. It sets `Physics.simulationMode = Script`, initializes `ClockFactory`, and runs a deterministic pipeline for every fixed step:

1. Advance the simulation clock.
2. Call registered `ISimulatable.PrePhysicsStep` in execution order.
3. Call `Physics.Simulate(baseDt)`.
4. Call `ISimulatable.PostPhysicsStep` in execution order.

The default base rate is 250 Hz (`dt = 0.004 s`). `FreeRun` converts unscaled wall time multiplied by the simulation time scale into fixed steps. `Manual` advances only through explicit steps. Pause stops normal accumulation, while queued single steps still work. The safety cap is 1000 steps per rendered frame.

Do not drive controller or sensor logic from HUD refresh frequency. The HUD uses wall time for display and camera input. `WorkspaceFlightSession` is an `ISimulatable` at order 5, before `DroneManager` at order 10, so an engaged UI command is renewed for every simulated step even during accelerated batches. `SensorManager` runs later at order 50.

### Control authority

`ControlAuthorityManager` arbitrates `UI`, `Internal` and `External` input. The native HUD must only mutate commands, pose, environment and other protected state when the current API/authority checks allow it. External connection attempts can create an Accept/Deny request. Authority transitions, simulation reset, pose reset, reload and frame changes invalidate retained UI commands to prevent stale control replay.

Selecting a drone, controlling a drone, highlighting it and targeting it with the camera are separate concepts. Do not couple them implicitly. The one deliberate possession behavior is a double-click/follow action or an explicit camera command.

### Drones, controllers and frames

`DroneManager` owns registration, spawning, selection and the fixed-step drone loop. A `QuadPawn` owns the Rigidbody, rotors, `CascadedController`, `SensorManager`, `FlightCommandProxy`, `ModeCoordinator`, `QuadSimApi`, `DroneCameraRig` and image sensor integration.

Control modes are Position, Velocity, Angle and Acro/rate. The native flight session preserves the existing Axis4 packing and configured limits. Position uses ENU position with yaw and altitude in the expected slots; Angle/Acro use hover-throttle estimates. Only an explicitly engaged, currently selected drone receives continuous HUD commands.

Robotics-facing world coordinates are ENU. The normal body frame is FLU, with FRD/NED accessors for PX4-oriented workflows. The Unity drone model convention is local `+X` forward, `+Y` up and `+Z` left. Camera mounts and model-facing code must honor the `+X` nose convention.

Sensors include IMU, GPS, barometer and magnetometer. Image/depth capture uses `DroneImageSensor`, which creates a separate disabled `TrainingCamera` below the drone's FPV mount and renders on demand. Never reuse the training camera as the global display camera.

### RPC and Python

`ExternalRpcAdapter` exposes ZeroMQ/MessagePack endpoints consumed by the separate `quadsim-sdk`/QuadSimLib repository. The HUD Connections and Control Authority panels read the live adapter/authority state. Headless and lockstep behavior should remain independent of UI Toolkit. Runtime UI code is intentionally inactive in batch/headless execution.

## Native HUD implemented so far

The main implementation is `Runtime/Scripts/RuntimeUI/Workspace/RuntimeWorkspace.cs`, split into partial files for layout, flight/PID, level settings and selection. It is added to a scene with **GameObject → QuadSim → Runtime Workspace (Preview)** through `Editor/RuntimeWorkspaceMenu.cs`.

The top application-style bar has Scene, Simulation and Window menus plus centered pause/step/reset/speed controls and compact Level Settings / Control Authority shortcuts. The application title and old panel-selection dropdown were removed. Window menu entries show a check when the panel is open.

Implemented windows and surfaces include:

- Scene with a virtualized drone list, config catalog, runtime spawning, reload/default config actions, and object/marker controls.
- Inspector with applied config, mass/inertia, flight limits and transform editing.
- Telemetry with sensor/controller readouts and smooth vertical motor/thruster bars.
- Flight controls with Position, Velocity, Angle and Acro commands, command sticks, gamepad support, FLU/FRD selection, hover, resets and debugging.
- Camera with Free, Orbit, Follow and FPV modes, distance/FOV/fly speed, framing, projection and axis views.
- Logs with bounded retained output.
- Connections and Control Authority with live endpoints and authority actions.
- Level Settings for simulation mode/origin, sensor noise/seed, wind, objects/markers and workspace/startup preferences.

Panels can be docked left/right/bottom, floated, collapsed, closed and resized when floating. Dragging a title exposes edge docking zones. Floating panels snap to neighboring floating-panel edges. Layout, sizes, panel-open state, UI scale and theme are persisted with `PlayerPrefs`. A first run presents the taskbar; subsequent runs restore prior window state. Window → Restore default layout restores Scene/Inspector/Logs.

Window → Hide all UI and the Settings equivalent hide all chrome, leaving a translucent restore button at bottom-right. “Hide all UI on startup” persists. Both light and dark themes exist. The design uses compact spacing, subtle borders, modest translucency and code-drawn vector icons; no external font or raster icon dependency was introduced.

The HUD backend boundary is `IWorkspaceBackend` / `WorkspaceBackend`. Keep view code on that boundary rather than reaching into simulation systems everywhere. `WorkspaceFlightSession` owns retained command state, `WorkspaceLevelService` validates scene mutations, and `WorkspacePidHistory` preserves compatible PID CSV behavior.

### ImGui feature migration

The native workspace connects the old ImGui controls rather than leaving decorative UI. The full mapping is in `runtime-workspace.md`. It includes transport/time scale, YAML config discovery and reload, drone selection/spawn, authority and external approval, every flight mode and axis, hover, gamepad, coordinate frames, debug visualization, pose resets, camera modes, sensor telemetry, motor outputs, command sticks, PID editing/save/history/apply, startup preferences, level/environment settings, and runtime cubes/spheres/cylinders/goals.

The old ImGui implementation remains as a fallback/reference under `Runtime/Scripts/RuntimeUI/ImGui`. Do not delete it until native feature parity and standalone validation are complete.

## Selection and object behavior

Clicking a drone row or drone collider explicitly selects and highlights it. Selection on startup does not highlight anything. Clicking empty viewport space clears the highlight while retaining backend selection. Double-clicking a drone row/collider follows it. Double-clicking another collider frames that object.

The selection effect is `WorkspaceSelectionPass`, an HDRP custom pass rendered after post-processing and filtered to the global display camera. It caches the selected entity's mesh renderers and draws a bright cyan silhouette with depth testing disabled, so the selected drone remains visible through walls. This avoids per-frame bounds jitter, material mutation and a fullscreen outline buffer. Cost is one draw per selected mesh submesh.

`ScenarioObjectManager` is the runtime object backend. Level Settings can add/remove pooled cubes, spheres, cylinders and goal/marker objects when editing is allowed. Scene-object entries include explicit Frame and Follow camera actions. Extend this service/catalog when adding obstacle types; do not scatter primitive spawning through UI callbacks.

## Global camera system

`CameraDirector` now owns a single independent display camera under the workspace host. It copies the initial scene camera and HDRP camera settings but never reparents a drone/sensor camera. Legacy drone display cameras are disabled; `DroneImageSensor.TrainingCamera` remains independent.

Targets are generic `Transform`s. `Bind(QuadPawn)` remains for legacy callers and reads `DroneCameraRig`; `BindEntity(Transform, optionalMount)` supports arbitrary scene entities. Follow and FPV track exactly to avoid Rigidbody interpolation lag. Orbit is damped and uses a stable world-oriented pivot, so drone yaw does not rotate the user's inspection orbit. Removing a target retains the display camera and falls back safely.

Current controls:

| Input | Behavior |
| --- | --- |
| Middle drag | Orbit |
| Shift + middle drag | Pan |
| Ctrl + middle drag | Dolly/zoom |
| Mouse wheel | Zoom |
| Right drag | Enter free mode and look |
| W / S | Forward / backward |
| A / D | Left / right |
| E / Q | World up / down |
| Shift while moving | Boost |
| Shift + grave | Enter free mode |
| F or numpad decimal | Frame selected/current target |
| Numpad 1 / 3 / 7 | Front / right / top orthographic view |
| Ctrl + numpad 1 / 3 / 7 | Opposite axis |
| Numpad 5 | Perspective/orthographic |

The viewport must have focus for keyboard navigation. Clicking a camera mode now focuses it. Movement reads Input System keyboard state during `RuntimeWorkspace.Update`, uses `Time.unscaledDeltaTime`, and therefore works while the simulation is paused. Tests cover all six movement directions, diagonal normalization, boost and released-input behavior. Automated Windows key taps were too brief to provide a trustworthy live movement check; manually test sustained keys in Game view on the next machine.

FPV mounts for Standard, Flightmare and Crazyflie were moved to model-specific positions just ahead of visible geometry and point along local `+X`. A live Flightmare FPV check confirmed the camera matches the mount and no longer sits inside the body. These corrected mounts also move the training image-sensor origin, which is desired for consistent extrinsics but has not been regression-compared against prior sensor images.

## Config-driven drone models

Previously every spawn instantiated `DroneManager.quadPawnPrefab` regardless of selected YAML. Configs now parse both `body_mesh` and a new `prefab_key` into `ModelType`. `DroneManager.SpawnDroneWithConfig` resolves the key through `Runtime/Resources/QuadSim/DronePrefabCatalog.asset` before instantiation.

Current mappings:

| Config key | Prefab | Note |
| --- | --- | --- |
| `standard_quad` | `Standard_quad.prefab` | Bundled visual |
| `flightmare_quad` | `flightmare_pawn.prefab` | Bundled visual |
| `crazyflie2_quad` | `crazyflie_prefab.prefab` | Bundled visual |
| `sentinel_quad` | `Standard_quad.prefab` | Explicit fallback; Sentinel mesh is not bundled, physics remains Sentinel config |

An explicit unknown `prefab_key` fails spawning with an actionable error instead of silently showing the wrong model. Legacy YAML without `prefab_key` resolves by config source filename, then `body_mesh`, then the manager's legacy prefab with a warning. Serialized prefab references ensure standalone builds include assets; no `AssetDatabase` appears in runtime code.

Both package fallback YAMLs under `Runtime/Resources/QuadSim/Configs/Drones` and editable sample overrides under `Samples~/QuadSim Starter/StreamingAssets/Configs/Drones` received `prefab_key`. All `DroneManager` spawning paths use the resolver, including HUD spawn/reload and SDK calls that reach `SpawnDroneWithConfig`. Existing drones placed directly in a scene do not morph into another prefab when only a config is applied; replacing an existing model requires the spawn/reload path.

Historical Flightmare and Standard YAMLs share `name: Standard_Quad_V1`; do not use the display name as the asset key. The explicit key fixes that ambiguity.

## Performance constraints and decisions

- UI text updates at 5 Hz wall time by default. Closed, collapsed and inactive panels skip expensive formatting.
- Motor bar fills update at render cadence with an 80 ms smoothing constant; numbers remain unsmoothed at text cadence.
- The drone list uses virtualized fixed-height rows.
- Logs and PID history retain at most 200 entries; logs truncate individual messages.
- Config/history disk IO occurs on explicit user actions.
- Hidden UI skips presentation work.
- Camera navigation uses unscaled render time and does not enter the fixed-step simulation loop.
- Selection caches renderer/submesh references only when selection changes.
- No polling or rendering work should be added to `PrePhysicsStep`/`PostPhysicsStep` unless it is simulation-critical.
- Batch/headless runs must not construct or render the native workspace.

## Important files

| Area | Files |
| --- | --- |
| Workspace shell/input/panels | `Runtime/Scripts/RuntimeUI/Workspace/RuntimeWorkspace.cs` and partial files |
| Docking and persistence | `WorkspaceWindow.cs`, `WorkspaceDocking.cs`, `RuntimeWorkspace.Layout.cs` |
| Simulation adapter | `WorkspaceBackend.cs`, `WorkspaceFlightSession.cs`, `WorkspaceLevelService.cs`, `WorkspacePidHistory.cs` |
| Styles and popup themes | `Runtime/Scripts/RuntimeUI/Workspace/Resources/QuadSimWorkspace*.uss` and `*.tss` |
| Vector icons/readouts | `WorkspaceIcon.cs`, `WorkspaceReadout.cs` |
| Selection | `RuntimeWorkspace.Selection.cs`, `WorkspaceSelectionPass.cs`, `QuadSimSelectionOverlay.shader` |
| Camera | `Runtime/Scripts/DroneCore/Camera/CameraDirector.cs`, `DroneCameraRig.cs`, `DroneImageSensor.cs` |
| Config model mapping | `DronePrefabCatalog.cs`, `Runtime/Resources/QuadSim/DronePrefabCatalog.asset`, YAML parser/structs |
| Drone lifecycle | `Runtime/Scripts/DroneCore/Core/DroneManager.cs`, `Runtime/Scripts/DroneCore/QuadPawn.cs` |
| Core simulation | `SimulationManager.cs`, `SimCoreApi.cs`, `ControlAuthorityManager.cs`, `ClockFactory.cs` |
| RPC | `Runtime/Scripts/SimCore/Rpc/ExternalRpcAdapter.cs` |
| Tests | `Tests/Editor/WorkspaceTests.cs`, `CameraDirectorTests.cs`, `DronePrefabCatalogTests.cs`, `Tests/PlayMode/WorkspaceIntegrationTests.cs` |

## Current validation state

Latest automated results:

- 70 Editor tests passed in Unity 6000.4.10f1 / HDRP 17.4.
- 3 PlayMode tests passed.
- `git diff --check` passes; only CRLF conversion warnings are emitted.
- Camera tests cover global camera ownership, preservation of entity sensor cameras, generic targets, framing, orbit pan/zoom, axis/projection changes, explicit mount attachment, all six free-movement directions while paused, released input and boosted diagonal speed.
- Model tests cover distinct Standard/Flightmare/Crazyflie resolution, body-mesh parsing, explicit unknown-key rejection, legacy source-name fallback and forward/external FPV mounts.
- PlayMode integration covers mixed Flightmare/Crazyflie spawning through `DroneManager`, FPV pose/orientation, selection/follow, 200 accelerated physics steps without UI Update, stale-ID rejection, authority transitions, environment edits, PID persistence, runtime objects and pose-preserving reload.

Visual checks in Unity 6000.3.10f1 covered the runtime workspace, layouts, explicit through-wall highlighting, camera shelf, Follow/Free/FPV transitions and corrected Flightmare FPV. The Unity console was clean after the temporary probe was removed. The following remain unverified or only partially verified:

- Sustained physical WASD/Q/E movement feel and mouse sensitivity on another machine.
- A standalone Windows/Linux build.
- Real gamepad hardware.
- Live Python socket traffic after this UI/camera iteration.
- Sensor-image before/after comparison for the corrected FPV extrinsics.
- Large multi-drone profiling.
- Sentinel-specific visual asset (none is currently present).

## Working-tree inventory at handoff

The uncommitted camera/model phase includes modifications to:

- Three drone prefabs for FPV mount placement.
- Four package YAML configs and four sample YAML configs for `prefab_key`.
- `CameraDirector.cs`, `DroneManager.cs`, YAML structs/parser.
- Runtime workspace camera/input, object-camera actions and viewport framing.
- `WorkspaceIntegrationTests.cs`.

It adds:

- `DronePrefabCatalog.cs` and its `.meta`.
- `Runtime/Resources/QuadSim/DronePrefabCatalog.asset` and its `.meta`.
- `CameraDirectorTests.cs` and its `.meta`.
- `DronePrefabCatalogTests.cs` and its `.meta`.
- The three focused documents under `Documentation~`, including this handoff.

Run `git status --short` for the authoritative list. Preserve Unity `.meta` files alongside every asset/script. The broader native HUD implementation is already represented in base commit `28731b4`, while the changes above are layered on top.

## Suggested continuation order

1. Transfer the complete dirty worktree or create a reviewable commit/branch and push it. A remote clone alone currently loses the newest work.
2. Open/import the package on the destination computer, let Unity finish compilation, clear old Console history, and confirm no current errors.
3. Run Editor and PlayMode tests before changing behavior. Record Unity/HDRP versions and result XML paths.
4. In Game view, focus the viewport and hold W/S/A/D/E/Q; verify speed, world-vertical behavior, pause behavior, Shift boost and release. Tune `freeMoveSpeed`, sensitivity and acceleration only after this manual check.
5. Spawn Standard, Flightmare and Crazyflie from the native Scene config catalog and visually confirm distinct models, collider alignment, rotor animation, selection highlighting, Follow and FPV.
6. Add a real Sentinel pawn/model or keep the explicit fallback warning. Do not silently claim Sentinel visual support.
7. Build a standalone player and repeat window persistence, UI hide/restore, spawning, camera controls and authority flow.
8. Profile the workspace with representative multi-drone counts. Focus on formatting allocations, telemetry reads and selected-entity custom-pass draws.
9. Continue camera polish: optional acceleration/deceleration, configurable keymap, navigation gizmo, camera bookmarks and multiple saved views. Preserve the single global display-camera ownership model.
10. Continue HUD feature work through `IWorkspaceBackend` and retained services. Likely next panels are sensor-image views, ROS2/network diagnostics, scenario save/load and richer object catalogs.

## Guardrails for the next agent

- Keep the runtime UI modular and optional. Headless simulation must not depend on it.
- Do not use `UnityEditor` APIs in runtime assemblies.
- Preserve deterministic simulation ordering and use simulation time inside fixed-step systems.
- Keep presentation refreshes on wall time and outside physics callbacks.
- Keep camera target, selected entity, highlighted entity and controlled drone independent.
- Never mutate render materials to highlight selection; use the existing custom pass.
- Never adopt/reparent a drone or training camera as the global display camera.
- Route all spawned drone models through `DronePrefabCatalog`; explicit unknown keys should fail loudly.
- Preserve backward compatibility for older YAML and ImGui while migration remains in preview.
- Use the existing frame-conversion helpers. Do not guess mappings between Unity, ENU, FLU, FRD and NED.
- Validate UI commands against current authority and selected agent ID immediately before mutation.
- Do not present placeholder controls as functional. Wire them through existing APIs or label them clearly.

## Definition of a solid next milestone

A good next milestone is a standalone build in which a user can launch with only the taskbar, restore their saved layout, spawn each available drone config with the correct model, select/highlight any drone, double-click to follow, enter correctly mounted FPV, use full Blender-style orbit/pan/frame controls, free-fly with sustained WASD/Q/E input while paused or running, manipulate runtime objects, inspect telemetry, and switch authority without stale commands. It should pass Editor/PlayMode tests and remain inactive in headless mode.
