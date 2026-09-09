using System.Collections.Generic;
using Experimental;
using SimCore;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed partial class RuntimeWorkspace
    {
        private VisualElement levelEdits;
        private Label levelStatus;
        private DoubleField latitude, longitude, altitude;
        private IntegerField noiseSeed;
        private Toggle noise;
        private Vector3Field spawnOrigin;
        private EnumField runMode;
        private DropdownField windChoice;
        private Toggle windOn;
        private FloatField windSpeed, windYaw;
        private readonly List<WindModule> windZones = new List<WindModule>();
        private bool levelInitialized;
        private DropdownField objectChoice;
        private Button spawnObjectButton, removeObjectButton;
        private Label objectEditStatus;
        private void BuildLevelControls(VisualElement parent)
        {
            levelStatus = Text(parent, "Pause and take UI control to change level settings.");
            AddButton(parent, "Read current settings", ReadLevel);
            levelEdits = new VisualElement(); parent.Add(levelEdits);
            Section(levelEdits, "Simulation");
            runMode = new EnumField("Run mode", SimulationManager.RunMode.FreeRun);
            runMode.RegisterValueChangedCallback(e => backend.Level?.SetRunMode((SimulationManager.RunMode)e.newValue)); levelEdits.Add(runMode);
            spawnOrigin = new Vector3Field("Spawn origin (Unity m)"); levelEdits.Add(spawnOrigin);
            Section(levelEdits, "All drones · sensor environment");
            latitude = new DoubleField("Latitude (deg)") { isDelayed = true }; levelEdits.Add(latitude);
            longitude = new DoubleField("Longitude (deg)") { isDelayed = true }; levelEdits.Add(longitude);
            altitude = new DoubleField("Altitude (m)") { isDelayed = true }; levelEdits.Add(altitude);
            noise = new Toggle("Sensor noise"); levelEdits.Add(noise);
            noiseSeed = new IntegerField("Noise seed") { isDelayed = true }; levelEdits.Add(noiseSeed);
            AddButton(levelEdits, "Apply environment", () =>
            {
                bool applied = backend.Level != null && backend.Level.Apply(noiseSeed.value, noise.value, latitude.value, longitude.value, altitude.value, spawnOrigin.value);
                if (applied) Flight?.Clear();
                levelStatus.text = applied ? "Environment applied to current drones and future spawns. Sensor samples reset." : "Not applied. Pause, use UI authority and enter valid coordinates.";
            });
            Section(parent, "Wind volumes");
            windChoice = new DropdownField(); windChoice.RegisterValueChangedCallback(_ => ReadWind()); parent.Add(windChoice);
            AddButton(parent, "Refresh wind volumes", RefreshWind);
            windOn = new Toggle("Wind enabled"); parent.Add(windOn);
            windSpeed = new FloatField("Speed (m/s)") { isDelayed = true }; parent.Add(windSpeed);
            windYaw = new FloatField("Heading (deg)") { isDelayed = true }; parent.Add(windYaw);
            AddButton(parent, "Apply wind", () =>
            {
                int index = windChoice.index;
                bool ok = index >= 0 && index < windZones.Count && backend.Level != null && backend.Level.ApplyWind(windZones[index], windOn.value, windSpeed.value, windYaw.value);
                levelStatus.text = ok ? "Wind volume updated." : "Select an existing wind volume; pause and use UI authority.";
            });
            Section(parent, "Workspace display & startup");
            var uiScale = new Slider("UI scale", 0.8f, 2f) { showInputField = true, value = settings.scale };
            uiScale.RegisterValueChangedCallback(e => { if (!WorkspaceBackend.IsFinite(e.newValue)) return; settings.scale = Mathf.Clamp(e.newValue, 0.8f, 2f); PlayerPrefs.SetFloat("QuadSim.UI.Scale", settings.scale); }); parent.Add(uiScale);
            var dark = new Toggle("Dark theme") { value = PlayerPrefs.GetInt("QuadSim.UI.DarkTheme", 0) == 1 };
            dark.RegisterValueChangedCallback(e => SetDarkTheme(e.newValue)); parent.Add(dark);
            StartupToggle(parent, "Hide all UI on startup", "QuadSim.UI.StartHidden");
            AddButton(parent, "Hide all UI now", () => SetUiHidden(true));
            Text(parent, "Open panels, dock sizes and theme are remembered between sessions.");
            GroupSections(levelEdits);
            GroupSections(parent.contentContainer);
        }
        private void BuildObjectControls(VisualElement parent)
        {
            var group = new Foldout { text = "Scene objects & assets", value = true }; group.AddToClassList("readout-group"); parent.Add(group);
            parent = group;
            var objectStatus = Text(parent);
            objectEditStatus = Text(parent);
            AddButton(parent, "Enable scene editing", EnableSceneEditing);
            var shape = new DropdownField("Shape", new List<string> { "cube", "sphere", "cylinder", "goal" }, 0); parent.Add(shape);
            var objectPosition = new Vector3Field("Position (ENU m)") { value = new Vector3(2, 0, 1) }; parent.Add(objectPosition);
            var objectSize = new Vector3Field("Size (ENU m)") { value = Vector3.one }; parent.Add(objectSize);
            AddButton(parent, "Place ahead of camera", () =>
            {
                var camera = cameraDirector?.ViewCamera;
                if (camera == null) return;
                float distance = Mathf.Max(3, objectSize.value.magnitude * 2);
                var point = camera.transform.position + camera.transform.forward * distance;
                objectPosition.value = MathUtil.Frames.TransformWorldPosition(point, MathUtil.SimFrame.FLU);
            });
            var collision = new Toggle("Collision") { value = true }; parent.Add(collision);
            var placeInView = new Toggle("Spawn in view") { value = true }; parent.Add(placeInView);
            spawnObjectButton = AddButton(parent, "Spawn object", () =>
            {
                if (placeInView.value && cameraDirector?.ViewCamera != null)
                {
                    var camera = cameraDirector.ViewCamera;
                    float distance = Mathf.Max(3, objectSize.value.magnitude * 2);
                    var ray = new Ray(camera.transform.position, camera.transform.forward);
                    var point = Physics.Raycast(ray, out var hit, distance) ? hit.point + hit.normal * objectSize.value.magnitude * .5f : ray.GetPoint(distance);
                    objectPosition.SetValueWithoutNotify(MathUtil.Frames.TransformWorldPosition(point, MathUtil.SimFrame.FLU));
                }
                bool ok = backend.Level.AddObject(shape.value, objectPosition.value, objectSize.value, collision.value, out var created);
                if (ok) { SelectSceneObject(created.transform); cameraDirector?.FrameTarget(created.transform); }
                objectStatus.text = ok ? $"Added {created.Shape} · selected in Inspector." : "Pause, use UI authority and enter a positive object size.";
                RefreshObjects(); nextRefresh = 0;
            });
            objectChoice = new DropdownField("Scene selection"); parent.Add(objectChoice);
            objectChoice.RegisterValueChangedCallback(_ =>
            {
                int index = objectChoice.index;
                if (index >= 0 && index < sceneObjects.Count) SelectSceneObject(sceneObjects[index]);
            });
            void TargetObject(bool follow)
            {
                if (backend.Level == null || cameraDirector == null) return;
                int index = objectChoice.index;
                if (index < 0 || index >= sceneObjects.Count || sceneObjects[index] == null) return;
                var target = sceneObjects[index]; SelectSceneObject(target);
                cameraDirector.BindEntity(target, snap: false);
                if (follow) cameraDirector.SetMode(DroneCore.CameraSystem.CameraViewMode.Follow);
                else cameraDirector.FrameTarget(target);
            }
            var cameraActions = new VisualElement(); cameraActions.AddToClassList("action-row"); parent.Add(cameraActions);
            AddButton(cameraActions, "Frame", () => TargetObject(false));
            AddButton(cameraActions, "Follow", () => TargetObject(true));
            AddButton(parent, "Refresh objects", () => { backend.Scene?.RefreshAuthoredObjects(); RefreshObjects(); });
            removeObjectButton = AddButton(parent, "Remove selected object", () =>
            {
                var item = backend.Scene?.Selected != null ? backend.Scene.Selected.GetComponent<QuadSim.RL.RLSceneObject>() : null;
                bool ok = item != null && backend.Level.RemoveObject(item.ObjectId);
                if (ok) SelectSceneObject(null);
                objectStatus.text = ok ? "Object removed." : "Pause, use UI authority and select an existing object.";
                RefreshObjects();
            });
        }
        private void RefreshObjectControls()
        {
            bool ready = backend.Level != null && backend.Level.CanEdit;
            spawnObjectButton.SetEnabled(ready);
            removeObjectButton.SetEnabled(ready && backend.Scene?.Selected != null && backend.Scene.Selected.GetComponent<QuadSim.RL.RLSceneObject>() != null);
            objectEditStatus.text = ready ? "Scene editing enabled · world ENU coordinates." : "Enable scene editing before spawning: simulation must be paused with UI control.";
        }
        private void RefreshObjects()
        {
            if (backend.Level == null) return;
            var previous = backend.Scene.Selected;
            backend.Scene.ReadObjects(sceneObjects);
            var choices = new List<string>();
            int selected = -1;
            for (int i = 0; i < sceneObjects.Count; i++)
            {
                choices.Add($"{i + 1} · {sceneObjects[i].name}");
                if (sceneObjects[i] == previous) selected = i;
            }
            objectChoice.choices = choices;
            objectChoice.SetValueWithoutNotify(selected >= 0 ? choices[selected] : "");
            RefreshObjectControls();
        }
        private static void StartupToggle(VisualElement parent, string label, string key)
        {
            var toggle = new Toggle(label) { value = PlayerPrefs.GetInt(key, 0) == 1 };
            toggle.RegisterValueChangedCallback(e => { PlayerPrefs.SetInt(key, e.newValue ? 1 : 0); PlayerPrefs.Save(); }); parent.Add(toggle);
        }
        private void RefreshLevel()
        {
            if (!levelInitialized && backend.Level != null) ReadLevel();
            levelEdits.SetEnabled(backend.Level != null && backend.Level.CanEdit);
            if (backend.Level != null && !Editing(runMode)) runMode.SetValueWithoutNotify(backend.Level.Sim.RunMode);
        }
        private void ReadLevel()
        {
            var level = backend.Level; if (level == null || level.Sim == null || !level.Sim.IsInitialized || level.Sim.Geo == null || level.Drones == null) return;
            latitude.SetValueWithoutNotify(level.Sim.Geo.OriginLatDeg);
            longitude.SetValueWithoutNotify(level.Sim.Geo.OriginLonDeg);
            altitude.SetValueWithoutNotify(level.Sim.Geo.OriginHeight);
            spawnOrigin.SetValueWithoutNotify(level.Drones.SpawnOrigin);
            noise.SetValueWithoutNotify(level.Drones.SensorEnvironmentNoise);
            noiseSeed.SetValueWithoutNotify(level.Drones.SensorEnvironmentSeed);
            runMode.SetValueWithoutNotify(level.Sim.RunMode); levelInitialized = true; RefreshWind();
        }
        private void RefreshWind()
        {
            windZones.Clear(); var choices = new List<string>();
            foreach (var wind in WindModule.Active) if (wind != null) { windZones.Add(wind); choices.Add($"{windZones.Count}: {wind.name}"); }
            windChoice.choices = choices; windChoice.SetValueWithoutNotify(choices.Count > 0 ? choices[0] : ""); ReadWind();
        }
        private void ReadWind()
        {
            int i = windChoice.index; if (i < 0 || i >= windZones.Count || windZones[i] == null) return;
            windOn.SetValueWithoutNotify(windZones[i].IsEnabled);
            windSpeed.SetValueWithoutNotify(windZones[i].WindSpeed);
            windYaw.SetValueWithoutNotify(windZones[i].transform.eulerAngles.y);
        }
        private void BuildCameraDetails(VisualElement body)
        {
            Section(body, "Navigation");
            var navigation = new VisualElement(); navigation.AddToClassList("action-row"); body.Add(navigation);
            AddButton(navigation, "Frame target", () => cameraDirector?.FrameTarget());
            AddButton(navigation, "Perspective / Ortho", () => cameraDirector?.ToggleProjection());
            var axes = new VisualElement(); axes.AddToClassList("action-row"); body.Add(axes);
            AddButton(axes, "Front", () => cameraDirector?.AxisView(Vector3.forward));
            AddButton(axes, "Right", () => cameraDirector?.AxisView(Vector3.left));
            AddButton(axes, "Top", () => cameraDirector?.AxisView(Vector3.down));
            var speed = new FloatField("Fly speed (m/s)"); body.Add(speed);
            speed.RegisterValueChangedCallback(e => { if (cameraDirector != null && WorkspaceBackend.IsFinite(e.newValue)) cameraDirector.FreeMoveSpeed = e.newValue; });
            Text(body).text = "MMB orbit · Shift+MMB pan · Ctrl+MMB zoom\nRMB fly look · WASD / Q E move · Shift boost\nF / Num . frame · Num 1/3/7 axis · Num 5 projection";
            var yaw = new Slider("Orbit yaw (deg)", -180, 180) { showInputField = true };
            var pitch = new Slider("Orbit pitch (deg)", -80, 80) { showInputField = true };
            yaw.RegisterValueChangedCallback(e => { if (cameraDirector != null && WorkspaceBackend.IsFinite(e.newValue)) cameraDirector.OrbitYawDeg = e.newValue; });
            pitch.RegisterValueChangedCallback(e => { if (cameraDirector != null && WorkspaceBackend.IsFinite(e.newValue)) cameraDirector.OrbitPitchDeg = e.newValue; });
            body.Add(yaw); body.Add(pitch);
            AddButton(body, "Toggle free follow", () => cameraDirector?.ToggleFreeFollow(Flight?.Pawn));
            var details = Text(body);
            body.schedule.Execute(() =>
            {
                if (cameraDirector == null || !windows["Camera"].ShouldRefresh) return;
                if (!Editing(speed)) speed.SetValueWithoutNotify(cameraDirector.FreeMoveSpeed);
                if (!Editing(yaw)) yaw.SetValueWithoutNotify(cameraDirector.OrbitYawDeg);
                if (!Editing(pitch)) pitch.SetValueWithoutNotify(cameraDirector.OrbitPitchDeg);
                details.text = $"View: {cameraDirector.Mode} · {(cameraDirector.Orthographic ? "Orthographic" : "Perspective")}\nTarget: {cameraDirector.Target?.name ?? "None"}\nCamera: Global display rig\nFree follow: {cameraDirector.FreeFollowTarget?.DroneID ?? "Off"}";
            }).Every(200);
        }
    }
}

