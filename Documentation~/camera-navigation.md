# Runtime display camera

`CameraDirector` owns one display camera under the workspace host. It copies the initial scene camera's rendering settings, including HDRP camera data, without reparenting a drone camera. Legacy drone display cameras are disabled; training cameras remain separate. Camera movement uses unscaled time so navigation continues while simulation is paused.

`Bind(QuadPawn)` retains the legacy API and the drone's +X forward convention. `BindEntity(Transform, Transform mount = null, bool snap = true)` targets any entity, using +Z forward for generic follow mode and an optional mount for attached/FPV mode. No input or camera component is required on that entity. Selection, camera target, and flight control authority remain independent. Removing a target preserves the display camera; follow/FPV fall back to free view, while orbit retains its last pivot.

| Input | Action |
| --- | --- |
| Middle drag | Orbit around target/pivot |
| Shift + middle drag | Pan in the camera plane |
| Ctrl + middle drag / wheel | Zoom |
| Right drag | Enter free mode and look |
| WASD / Q, E | Free movement / down, up |
| Shift while moving | Boost |
| Shift + grave | Enter free mode |
| F / numpad decimal | Frame selected entity (or current target) |
| Numpad 1 / 3 / 7 | Front / right / top orthographic view |
| Ctrl + numpad 1 / 3 / 7 | Opposite axis view |
| Numpad 5 | Perspective / orthographic |
| Double-click drone | Follow drone |
| Double-click other collider | Frame object |

Keyboard controls require an explicit viewport click or camera-mode action and viewport focus. HUD clicks deactivate camera movement; WASD is excluded from UI navigation while the workspace is active. Arrow keys and gamepad navigation remain available to the HUD. Right mouse down enters Free immediately. Editing a tracked object's transform releases camera tracking and keeps the view stationary. The Camera panel exposes framing, axes, projection, FOV, distance, and fly speed. Scene objects have explicit Frame and Follow actions. Orbit uses world orientation, so an entity's yaw does not rotate the inspection view. Bounds are gathered only when framing, not each frame. This implements Blender-style navigation, not the full Blender navigation preference system or fly-mode confirm/cancel behavior.

Validation: 58 Editor tests passed during implementation, including five camera tests covering ownership, sensor-component preservation, bounds framing, axis/projection, pan/zoom, and mount attachment. No standalone visual or sensor-image comparison was performed. Live mouse feel still needs evaluation in the main project.
