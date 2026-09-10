# Runtime workspace

QuadSim's default HUD is the UI Toolkit workspace. It starts automatically in graphical simulation scenes through `RuntimeWorkspaceStartup`, reuses an authored workspace when present, and creates none for batch/headless runs. Fresh installations open Scene and Inspector; existing layouts and display preferences are retained.

## Controls and editing

- The Scene hierarchy combines Scene Objects (preserving parent/child structure) and Drones. Expand the catalogs to spawn assets or drone configurations.
- Hierarchy selection enables the mesh highlight. Viewport picking and spawning select for inspection without tinting. Clear selection or Escape clears selection; Escape during a handle drag cancels the drag.
- Enable scene editing pauses simulation and takes UI authority. Transform fields apply on Enter or focus loss. Position follows ENU/NED/Unity world coordinates; rotation is Unity Euler degrees; scale is local Unity XYZ. Drone scale comes from its model configuration.
- The transparent XYZ gizmo translates in World or Body space using Sensor/FLU/FRD/Unity conventions. Ctrl snaps to 0.25 m; Shift gives precision.
- MMB orbits, Shift+MMB pans, wheel zooms, RMB enters fly look, WASD/QE moves the active Free camera, F frames selection. HUD clicks deactivate camera input; WASD is excluded from UI navigation.
- Scene objects spawn in view by default. Disable Spawn in view to use the entered ENU position. Collision-free markers remain selectable.

## Code map

All workspace code is in `Runtime/Scripts/RuntimeUI/Workspace/`.

| Area | Files | Framework connection |
| --- | --- | --- |
| Startup and shell | `RuntimeWorkspaceStartup`, `RuntimeWorkspace` partials | Initializes presentation only; no simulation clock ownership. Partials separate layout, scene editing, level settings, selection, and flight/PID panels. |
| Backend | `IWorkspaceBackend`, `WorkspaceBackend` | Resolves `SimCoreApi`, `DroneManager`, authority and simulation status. |
| Scene editing | `WorkspaceSceneService`, `WorkspaceHierarchy`, `WorkspaceTransformGizmo` | Aggregates authored assets and scenario objects; validates pause, authority, selection and transforms. |
| Object lifecycle | `WorkspaceLevelService` | Delegates spawning/pooling/removal to `Runtime/Scripts/RL/ScenarioObjectManager`; snapshots read current transforms. |
| Flight and tuning | `WorkspaceFlightSession`, `WorkspacePidHistory` | Commands flow through existing drone APIs and fixed-step hooks. Selection/authority changes clear stale command state. |
| Cameras and input | `WorkspaceInputRouting`, `WorkspaceNavigationBindings` | Uses `DroneCore/Camera/CameraDirector`; display camera is independent of sensor cameras. Bindings are restored when the workspace closes. |
| Presentation | `WorkspaceWindow`, `WorkspaceDocking`, `WorkspaceReadout`, `WorkspaceIcon`, `Resources/` | Docked/floating panels, saved layouts, USS/themes and camera-filtered HDRP selection pass. No render-material mutation for highlighting. |
| Drone models | `DroneCore/Core/DronePrefabCatalog`, `Runtime/Resources/QuadSim/DronePrefabCatalog.asset` | YAML model keys resolve to the correct prefab, geometry and camera mounts. |

Text refresh runs at 5 Hz wall time; motor bars, camera navigation and gizmos refresh per rendered frame. Closed panels skip presentation work. Logs and PID history are bounded. Simulation timing stays with SimulationManager. Camera target, scene selection and controlled drone remain separate.

## Developer ImGui

ImGui is retained under `Runtime/Scripts/RuntimeUI/ImGui/` with its pinned vendor dependencies under `Runtime/ThirdParty/UImGui/`. Do not install a second ImGui binding.

In Editor Play Mode, use **QuadSim > Developer > Use ImGui HUD** or **Use Workspace HUD**. Development players can opt in with `--quadsim-dev-imgui`. Release players ignore that flag. `Editor/RuntimeWorkspaceBuildProcessor` removes legacy HUD components from release scene copies; development scene copies keep them disabled for explicit opt-in. Source scenes are not modified. Vendor libraries remain available for development tooling.

## Build and verification

Build a host project containing the imported starter scene; this repository is a UPM package, not a standalone Unity project. Runtime code contains no UnityEditor dependencies. UI resources and the shared scenario material live under Resources for player inclusion.

Regression coverage is under `Tests/Editor/` and `Tests/PlayMode/`: frame mappings, camera ownership, prefab resolution, authority boundaries, spawning, selection, numeric text commits, hierarchy updates, and keyboard isolation. Host projects must list `com.nino.quadsim` in manifest `testables` to discover package tests.

Scenario save/load UI, ROS2 integration and sensor-image panels are not implemented. See [camera navigation](camera-navigation.md), [drone catalog](drone-model-catalog.md), and [networking](networking.md) for those subsystems.

## Verified build — 2026-09-10

73 Editor and 6 PlayMode tests passed. The Windows x64 release player built successfully with zero errors (411 MB) and initialized simulation, drone sensors/controllers and RPC in a startup smoke check. Output in the host project: `Builds/WorkspaceRelease/QuadSim.exe`.

The build reports 18 warnings, including Unity API deprecations, legacy vendor code, hangar collision import settings and a System.Windows.Forms dependency. Player startup falls back to basic text segmentation because runtime-created PanelSettings have no ICU data; advanced international text has not been validated. Pipeline remote control is disabled in the release player because the host has no RuntimePipelineConfig. These are remaining build hygiene items, not a claim of warning-free release certification.
`RuntimeWorkspaceStartup` also invokes initialization after Play startup, independently of `sceneLoaded`, so disabling domain and scene reload does not prevent HUD creation. Verified in two Play cycles without an authored RuntimeWorkspace component.
