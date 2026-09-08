# Runtime workspace

## Enable

In a QuadSim scene, choose **GameObject → QuadSim → Runtime Workspace (Preview)** and save the scene. The component creates its own UI Toolkit document and temporarily disables the scene's ImGui bootstrap/renderer. Disabling it restores ImGui. Native rendering and subscriptions remain disabled in headless/batch applications.

Taskbar menus open the matching dock tab and expand collapsed panels. Drag a panel title to a workspace edge to dock it; Escape cancels. Window → Restore default layout restores Scene, Inspector and Logs. Window → Hide all UI hides all chrome, leaving a translucent bottom-right UI restore button. Settings → Workspace display & startup also provides Hide all UI now and a persistent Hide all UI on startup checkbox. Flight commands continue when their panel is closed, collapsed or hidden.

## Visual system and presentation

The light translucent theme uses shared USS palette variables, compact controls, aligned property rows, subtle section rules and an original 16-unit vector icon family (`WorkspaceIcon`). UI Toolkit draws icons when their geometry needs repainting; no external fonts, texture atlases or raster dependencies are required. `WorkspaceReadout` retains grouped property rows across value updates, including collapsed groups.

Catalog and enum popups use the workspace-owned `QuadSimWorkspaceTheme.tss`: 24-unit rows, 2-by-6-unit item padding, a small selection-check gutter and the same light palette. These rules live in the panel theme because Unity attaches popup lists outside the document root. Font size and the user's UI scale are preserved.

Scene lists drone names and agent IDs. The collapsible Drone catalog groups spawning and configuration actions. Inspector, Telemetry and Control Authority show labeled values with units. Level Settings groups simulation, sensors, wind, objects and workspace preferences.

**Showcase** opens a viewport-plus-telemetry arrangement. **Window → Restore default layout** returns Scene / Inspector / Logs. A compact vertical tool shelf at the viewport left provides Camera settings, Follow, Orbit and Free shortcuts with active-state feedback. The display camera renders inside the available canvas so docked panels do not obscure the followed drone; disabling the workspace restores its previous camera rectangle. Sensor cameras are unchanged.

**Level Settings → Workspace display & startup → UI scale** adjusts the pixel-based interface scale (80–200%, default 125%). Buttons retain text and tooltips; color supplements running-state and external-authority labels.

## Connected endpoint inventory

| Existing ImGui feature | Native destination and connection |
| --- | --- |
| Play/pause, slower/faster, speed, step, reset | Taskbar → SimCoreApi. Step works while paused or in Manual mode. Reset restores ×1 and resumes, matching ImGui. Status shows FPS, fixed dt, sim time, run mode, authority and measured real-time factor. |
| Spawn, config selection, YAML catalog refresh | Scene → Drone catalog → DroneManager. Selection sets the next spawn/reload config. Newly spawned drones become selected. |
| YAML reload and applied configuration | Reload replaces the selected drone after successful spawning, preserving its pose and selecting its replacement. Pause and UI authority required. Inspector shows applied model, mass, inertia and flight limits. YAML remains the configuration source of truth. |
| Default configuration | Scene → Drone catalog → Use this config by default uses the existing PlayerPrefs preference. Explicit scene configuration still overrides it at startup. |
| Drone selection / camera possession | Single click in Scene selects and shows a bright through-wall mesh silhouette in the display camera. Double-click a row or drone collider in the viewport to select and follow it. Single selection does not move the camera. |
| Authority display / switch to UI | Control Authority shows effective source, source statuses, actual RPC server/client status and addresses/ports. Take control in this app calls the existing API. |
| External connection approval | Accept/Deny modal → SimCoreApi. Pending state is reconciled with the authority manager so expired/cancelled requests cannot leave a stale modal. |
| Position / Velocity / Angle / Acro | Flight controls → QuadSimApi.SetMode and WorkspaceFlightSession. Position seeds from the current ENU sample; Angle/Rate use the existing mass/thrust hover estimate. |
| Sliders, numeric fields, per-axis zero | Validated Axis4 values use the selected drone's configured limits. Position packs yaw in Z and altitude in W; Angle/Rate pack throttle in W. |
| Hover / hover altitude | Position mode holds current XY/yaw at the entered altitude. Disabling hover retains the last position command for subsequent editing, matching ImGui. |
| Continuous UI commands | WorkspaceFlightSession renews only the engaged, selected drone's command before DroneManager physics. No rendering or device polling in the physics hook. |
| Gamepad checkbox | Reads Input System Gamepad in Angle/Acro. Left: yaw/throttle (center 50%); right: roll/pitch. No device disables activation. Disconnect/focus loss returns to neutral axes and hover-throttle estimate; re-enabling is required. The old checkbox only changed labels. |
| FLU/FRD | Flight controls → SetSensorOutputFrame, allocator rebuild and controller reset. GPS stays ENU. Frame changes invalidate prior UI setpoints. |
| Debug visualization | Selected CascadedController.DebugVisualsEnabled. |
| Reset high, low, rotation | Existing pose/rotation APIs. High = spawn origin + 5 Unity-Y meters; low = spawn origin. Retained UI setpoints are cleared. |
| Follow / FPV, Free / Orbit, FOV, follow distance | Existing CameraDirector. Targeted modes bind the selected drone. Orbit yaw/pitch/reset, free follow and target/source readouts are included. |
| Viewport camera input | Right-drag/wheel, WASD/QE/Shift and F through UI Toolkit focus/capture. Editing fields does not steer the camera. |
| Motors and state telemetry | Four output/thrust bars plus IMU validity/time, attitude/rates, velocity/force, GPS, magnetometer, barometer and controller targets. |
| Command sticks | Retained yaw/throttle and roll/pitch dots in Angle/Rate show commanded input, not measured motion. |
| Four PID stages / linked XY and RP | Existing GetPIDGains, SetPIDGainTerm and IsPIDStageActive hooks. Authority and selected-ID checks; invalid/nonfinite/negative values rejected. |
| PID save/history/apply | Existing persistentDataPath/PIDGains.csv schema; latest 200 rows, selected values and full header/row validation. Integrators reset once after successful load. |
| Persist control/state panels | Startup toggles preserve QuadSim.UI.PersistControlPanel and QuadSim.UI.PersistStateHUD. |
| Obstacle placeholder | Level Settings adds/removes cubes, spheres, cylinders and goals through ScenarioObjectManager without replacing unrelated objects; existing pooling retained. World fields explicitly use ENU. |
| Level controls | Run mode, spawn origin, geodetic origin, sensor noise/master seed and wind volume enable/speed/heading. Environment applies to current and future drones; pause and UI authority required. |

## Command ownership and performance

Choosing a flight mode or editing a command explicitly engages streaming. Opening the HUD alone does not send throttle. Only the selected drone is streamed, matching ImGui; other drones retain existing stale-command fallback behavior. Reselecting a commanded drone restores its per-drone values unless reset or authority transition invalidated them.

Authority changes and simulation resets clear native sessions. Returning to UI authority cannot replay old setpoints automatically. Pose resets, reloads and frame changes clear affected sessions. Writes check current authority and expected agent ID; delayed edits for a previous selection are rejected.

The command service fixes an ImGui timing limitation: a command sent once per rendered frame can expire during an accelerated batch of physics steps. It registers through ISimulatable at order 5, ahead of DroneManager (10), renews through the public API and leaves clock, expiry policy and controller algorithms intact.

RuntimeWorkspace composes the shell, panel lifecycle and camera input; partial files separate flight, PID, level and selection. IWorkspaceBackend adapts simulation APIs; WorkspaceFlightSession owns commands, WorkspaceLevelService validates scene mutations, and WorkspacePidHistory owns compatible file IO.

Text presentation stays at 5 Hz wall time. Four vertical motor fills refresh at render cadence with an 80 ms exponential smoothing constant; numeric samples remain unsmoothed at text cadence. Hidden UI skips presentation updates. Closed/collapsed/inactive tabs skip expensive formatting. Selection is explicit: startup and automatic drone activation do not highlight. Clicking a drone or its list row enables a bright cyan mesh silhouette; clicking empty scene space clears it. A camera-filtered HDRP custom pass draws only cached renderers of that entity after post-processing with depth testing disabled, so walls do not hide the highlight. This adds one draw per selected mesh submesh, without a fullscreen outline buffer or material mutation. Other cameras are excluded. Follow framing shifts only the display-camera projection; the full frame is rendered beneath translucent chrome to prevent stale UI pixels. Sensor-camera projections are unchanged. Scene uses virtualized fixed-height rows. Logs retain at most 200 messages of 500 characters; PID history retains 200 rows. Catalog/history IO occurs on explicit actions. Devices are sampled once per rendered frame, and destroyed command states are pruned at presentation cadence.

## Verification and boundaries

Unity 6000.4.10f1 / HDRP 17.4.0 is the headless verification environment. The PlayMode integration test uses real drone prefabs and exercises selection/follow, 200 physics steps without UI Update, stale-ID rejection, limits/nonfinite rejection, frames, authority loss/restoration, environment application, PID CSV round-trip/rejection, additive objects, pose-preserving reload and external denial. Editor tests cover command packing, validation, docking and retained readout state.

Graphical testing uses the open Unity 6000.3.10f1 Land scene. Live checks cover Showcase, telemetry docking to the left edge, default layout recovery, Free-to-Follow by double-clicking a drone row, grouped level settings, and the compact menu at 1366 × 768. The editor regression suite passed 53 tests. Real gamepad hardware, live Python socket traffic, a standalone build and large-swarm profiling are separate verification boundaries; no coverage of those is claimed by the headless tests.

Environment/scenario save-load, generic transform gizmos and ROS2 transport were never functional ImGui endpoints and are not presented as working controls. The legacy vendor package remains installed for fallback.

The first launch starts with the taskbar and scene only. Subsequent sessions restore open panels, active dock tabs, collapse state, dock region sizes, and floating window positions/sizes using versioned PlayerPrefs (`QuadSim.UI.Workspace.v1`). Settings offers a persistent Dark theme toggle; light and dark modes share control spacing, section headers and compact popup geometry. Hide all UI on startup remains an independent override. Layout is saved on focus loss and shutdown; this saves workspace presentation, not scenario objects or simulation state.

Drag dock boundaries to resize regions, or use a floating window's lower-right corner. Floating windows stay within the viewport and snap to nearby floating window edges. Dropping a title onto an occupied left/right/bottom region joins its tab strip. This is a three-region tab workspace, not an arbitrary nested split-window system.

Scene → Objects & markers exposes cube, sphere, cylinder and goal spawning. Enable scene editing explicitly pauses the simulation and requests UI authority; Spawn remains disabled until both conditions hold. Position and size use world ENU metres. The object selector supports removing spawned scenario objects.

The final popup-theme import and C# compilation passed in Unity 6000.4.10f1. A follow-up graphical pass in Unity 6000.3.10f1 verified catalog and run-mode menus, enabled flight sliders, checked debug toggles, foldout expansion and Showcase. This pass corrected foldout arrow contrast, checkbox check contrast and clipped section headings in narrow panels. Game view was already set to Free Aspect and remains there; Play mode was stopped after testing. The earlier graphical QA also used the 1366 × 768 preset.

The light-theme pass adds open-window checks, balanced centered transport, and Scene object spawning (cube, sphere, cylinder and goal marker). Follow framing, vertical meters, full hide/restore and menu checks were exercised in Unity 6000.3.10f1; the duplicate permanent help strip was removed (instructions are available as viewport hover help).

Startup hiding was verified across a Play-mode restart, followed by restoration via the corner button. The test preference was returned to off and Unity left in Edit mode. Hidden external requests remain pending and are presented when the interface is restored; hiding does not approve requests or stop ongoing simulation commands.


The theme/layout/selection pass was exercised in Unity 6000.3.10f1: taskbar-only layout, dark/light switching, uniform catalog headers, dock and floating resizing, joining dock tabs by dragging, session restoration, scene-edit activation, cube spawn/removal and camera-relative placement. The mesh surface highlight was visually verified on the body and moving propellers. The 54-test Editor suite passed in Unity 6000.4.10f1; the material restoration test was rerun after the final contrast adjustment and passed. No standalone, large-swarm profile or sensor-image comparison is claimed.

The explicit-selection pass was checked live in Unity 6000.3.10f1: no startup highlight, viewport click activation, empty-space clearing, list selection through a fully occluding cube, and opening Camera settings from the left tool shelf. The temporary cube was removed. Both palettes now use more opaque panels and compact tool buttons. The previous property-block tint implementation and its obsolete test were removed.

