using System;
using System.Collections.Generic;
using DroneCore;
using DroneCore.Core;
using DroneCore.CameraSystem;
using QuadSim.RuntimeUI.ImGuiHud;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Opt-in runtime workspace. UI work runs on wall time, never on physics steps.</summary>
    [DisallowMultipleComponent, DefaultExecutionOrder(1100)]
    public sealed partial class RuntimeWorkspace : MonoBehaviour
    {
        [SerializeField, Range(1, 20)] private int refreshHz = 5;
        private IWorkspaceBackend backend;
        private UIDocument document;
        private PanelSettings settings;
        private VisualElement root, left, right, bottom, floating, viewport, approval, toolbar;
        private Label status, logText, feedback, sceneCount, sceneEmpty, viewLabel;
        private WorkspaceReadout connections;
        private WorkspaceReadout inspector, telemetry;
        private WorkspaceIcon transportIcon;
        private Button play, step, spawn, applyPose;
        private FloatField speed;
        private Vector3Field position, rotation;
        private DropdownField configs;
        private ListView entityList;
        private readonly List<WorkspaceEntity> entities = new List<WorkspaceEntity>();
        private readonly List<WorkspaceEntity> scratch = new List<WorkspaceEntity>();
        private readonly Dictionary<string, WorkspaceWindow> windows = new Dictionary<string, WorkspaceWindow>();
        private readonly Dictionary<WorkspaceDock, WorkspaceWindow> activeTabs = new Dictionary<WorkspaceDock, WorkspaceWindow>();
        private VisualElement transport, menuRegion, toolbarActions;
        private Button restoreUi;
        private bool uiHidden, restoringLayout, quitting;
        private VisualElement viewTools;
        private StyleSheet darkStyle;
        private Button authorityShortcut;
        private WorkspaceDocking docking;
        private VisualElement taskMenu;
        private readonly Queue<string> logs = new Queue<string>();
        private readonly HashSet<KeyCode> keys = new HashSet<KeyCode>();
        private readonly List<Behaviour> suspendedLegacy = new List<Behaviour>();
        private CameraDirector cameraDirector;
        private string poseEntity;
        private double nextRefresh;
        private bool logsDirty, configLoaded;
        private int lookPointer = -1;
        private VisualElement workspaceShell;
        private float framesPerSecond;
        private double lastStatusWall, lastStatusSim;

        private void OnEnable()
        {
            quitting = false;
            // Avoid any retained tree, camera creation, or subscriptions in headless jobs.
            if (Application.isBatchMode || SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null) return;
            if (FindObjectsByType<RuntimeWorkspace>(FindObjectsSortMode.None).Length > 1)
            {
                Debug.LogError("Only one RuntimeWorkspace may be active."); enabled = false; return;
            }
            backend = new WorkspaceBackend();
            backend.ExternalApprovalRequested += ShowApproval;
            backend.Connect();
            settings = ScriptableObject.CreateInstance<PanelSettings>();
            settings.themeStyleSheet = Resources.Load<ThemeStyleSheet>("QuadSimWorkspaceTheme");
            settings.scaleMode = PanelScaleMode.ConstantPixelSize;
            settings.scale = Mathf.Clamp(PlayerPrefs.GetFloat("QuadSim.UI.Scale", 1.25f), 0.8f, 2f);
            settings.referenceResolution = new Vector2Int(1440, 900);
            settings.match = 0.5f;
            // Own only the document created here; no mutation of a pre-existing document.
            var host = new GameObject("QuadSim Workspace Document"); host.transform.SetParent(transform, false);
            document = host.AddComponent<UIDocument>(); document.panelSettings = settings; document.sortingOrder = 100;
            root = document.rootVisualElement;
            root.styleSheets.Add(Resources.Load<StyleSheet>("QuadSimWorkspace"));
            root.AddToClassList("quadsim-workspace");
            root.AddToClassList("light-theme");
            darkStyle = Resources.Load<StyleSheet>("QuadSimWorkspaceDark");
            SetDarkTheme(PlayerPrefs.GetInt("QuadSim.UI.DarkTheme", 0) == 1);
            Build();
            cameraDirector = CameraDirector.GetOrCreate(gameObject);
            foreach (var legacy in FindObjectsByType<ImGuiRuntimeBootstrap>(FindObjectsSortMode.None))
            {
                if (!legacy.enabled) continue;
                suspendedLegacy.Add(legacy); legacy.enabled = false;
                var renderer = legacy.GetComponent<UImGui.UImGui>();
                if (renderer != null && renderer.enabled) { suspendedLegacy.Add(renderer); renderer.enabled = false; }
            }
            Application.logMessageReceived += OnLog;
            nextRefresh = 0;
        }

        private void OnDisable()
        {
            SaveWorkspace();
            ReleaseSelectionHighlight();
            RestoreViewportCamera();
            Application.logMessageReceived -= OnLog;
            if (backend != null) { backend.ExternalApprovalRequested -= ShowApproval; backend.Dispose(); backend = null; }
            keys.Clear();
            foreach (var window in windows.Values) window.Cancel();
            if (viewport != null && lookPointer >= 0) viewport.ReleasePointer(lookPointer);
            lookPointer = -1;
            if (document != null) { document.enabled = false; Destroy(document.gameObject); }
            if (settings != null) Destroy(settings);
            for (int i = suspendedLegacy.Count - 1; !quitting && i >= 0; i--)
                if (suspendedLegacy[i] != null) suspendedLegacy[i].enabled = true;
            suspendedLegacy.Clear(); windows.Clear(); activeTabs.Clear(); entities.Clear(); scratch.Clear(); logs.Clear();
            configLoaded = false; poseEntity = null; root = null;
            levelInitialized = false; windZones.Clear(); historyRows.Clear(); lastStatusWall = lastStatusSim = 0;
        }

        private void Build()
        {
            toolbar = new VisualElement(); toolbar.AddToClassList("workspace-toolbar"); root.Add(toolbar);
            menuRegion = new VisualElement(); menuRegion.AddToClassList("toolbar-side"); toolbar.Add(menuRegion);
            AddMenu("Scene", ("Scene objects / Spawn drone", () => OpenPanel("Scene")),
                ("Selected object inspector", () => OpenPanel("Inspector")),
                ("Level Settings", () => OpenPanel("Level Settings")));
            AddMenu("Simulation", ("Control Authority", () => OpenPanel("Control Authority")),
                ("Flight controls", () => OpenPanel("Flight controls")),
                ("PID tuning", () => OpenPanel("PID tuning")),
                ("Telemetry & sensors", () => OpenPanel("Telemetry")));
            AddMenu("Window", ("Scene", () => OpenPanel("Scene")),
                ("Inspector", () => OpenPanel("Inspector")), ("Camera", () => OpenPanel("Camera")),
                ("Telemetry", () => OpenPanel("Telemetry")), ("Logs", () => OpenPanel("Logs")),
                ("Level Settings", () => OpenPanel("Level Settings")),
                ("Control Authority", () => OpenPanel("Control Authority")),
                ("Flight controls", () => OpenPanel("Flight controls")), ("PID tuning", () => OpenPanel("PID tuning")),
                ("Showcase layout", ShowcaseLayout), ("Hide all UI", ToggleHud), ("Restore default layout", ResetLayout));
            transport = new VisualElement(); transport.AddToClassList("transport-controls"); toolbar.Add(transport);
            play = AddButton(transport, "Pause", () => backend.TogglePause());
            transportIcon = play.Q<WorkspaceIcon>();
            AddButton(transport, "−", () => backend.SetSpeed(Mathf.Max(0.05f, (float)backend.Status.TimeScale - 0.1f))).tooltip = "Slower";
            step = AddButton(transport, "Step", () => backend.Step());
            AddButton(transport, "Reset", () => backend.Reset()).tooltip = "Reset simulation and drones, resume at ×1 (same as the legacy HUD)";
            speed = new FloatField("Speed ×") { isDelayed = true };
            speed.AddToClassList("speed-field");
            speed.RegisterValueChangedCallback(e => backend.SetSpeed(e.newValue)); transport.Add(speed);
            AddButton(transport, "+", () => backend.SetSpeed(Mathf.Min(100f, (float)backend.Status.TimeScale + 0.25f))).tooltip = "Faster";
            toolbarActions = new VisualElement(); toolbarActions.AddToClassList("toolbar-side"); toolbarActions.AddToClassList("toolbar-actions"); toolbar.Add(toolbarActions);
            var showcaseShortcut = AddButton(toolbarActions, "Showcase", ShowcaseLayout);
            showcaseShortcut.tooltip = "Viewport and live telemetry; restore editing from Window";
            root.RegisterCallback<GeometryChangedEvent>(_ => showcaseShortcut.style.display = root.layout.width < 950 ? DisplayStyle.None : DisplayStyle.Flex);
            AddButton(toolbarActions, "Settings", () => OpenPanel("Level Settings"));
            authorityShortcut = AddButton(toolbarActions, "Control: —", () => OpenPanel("Control Authority"));
            authorityShortcut.tooltip = "Who controls the drones? Open Control Authority";
            var shell = workspaceShell = new VisualElement(); shell.AddToClassList("workspace-shell"); root.Add(shell);
            var work = new VisualElement(); work.AddToClassList("workspace-area"); shell.Add(work);
            left = new VisualElement(); left.AddToClassList("dock-left"); work.Add(left);
            var center = new VisualElement(); center.AddToClassList("workspace-center"); work.Add(center);
            viewport = new VisualElement { focusable = true }; viewport.AddToClassList("workspace-viewport"); center.Add(viewport);
            viewTools = new VisualElement(); viewTools.AddToClassList("viewport-tools"); viewport.Add(viewTools);
            viewLabel = new Label("SCENE VIEW"); viewLabel.style.display = DisplayStyle.None;
            void CameraTool(string icon, string hint, Action action, CameraViewMode? mode = null)
            {
                var button = new Button(action) { tooltip = hint, userData = mode };
                button.AddToClassList("camera-tool"); button.Add(new WorkspaceIcon(icon)); viewTools.Add(button);
            }
            CameraTool("Camera", "Camera settings", () => OpenPanel("Camera"));
            CameraTool("Follow", "Follow selected drone", () => backend.FollowSelected(), CameraViewMode.Follow);
            CameraTool("Orbit", "Orbit selected drone", () => { if (cameraDirector == null) return; if (Flight?.Pawn != null) cameraDirector.Bind(Flight.Pawn, false); cameraDirector.SetMode(CameraViewMode.Orbit); }, CameraViewMode.Orbit);
            CameraTool("Free", "Free camera · WASD / Q E", () => cameraDirector?.SetMode(CameraViewMode.Free), CameraViewMode.Free);
            viewTools.RegisterCallback<PointerDownEvent>(e => e.StopPropagation());
            viewTools.RegisterCallback<WheelEvent>(e => e.StopPropagation());
            viewport.tooltip = "Select: click · Follow: double-click · Look: right-drag · Move: WASD / Q E · Boost: Shift";
            bottom = new VisualElement(); bottom.AddToClassList("dock-bottom"); center.Add(bottom);
            right = new VisualElement(); right.AddToClassList("dock-right"); work.Add(right);
            floating = new VisualElement { pickingMode = PickingMode.Ignore }; floating.AddToClassList("floating-layer"); work.Add(floating);
            docking = new WorkspaceDocking(floating, center);
            status = new Label("Waiting for simulation…"); status.AddToClassList("status-bar"); root.Add(status);
            BuildViewportInput();

            var scene = AddWindow("Scene");
            var catalog = new Foldout { text = "Drone catalog", value = false }; catalog.AddToClassList("readout-group"); scene.Body.Add(catalog);
            configs = new DropdownField { tooltip = "Drone configuration" }; configs.AddToClassList("config-field"); catalog.Add(configs);
            configs.RegisterValueChangedCallback(e => backend.SetConfig(e.newValue));
            var spawnRow = new VisualElement(); spawnRow.AddToClassList("action-row"); catalog.Add(spawnRow);
            spawn = AddButton(spawnRow, "Spawn", () => backend.Spawn(configs.value));
            AddButton(spawnRow, "Refresh", RefreshConfigs).tooltip = "Refresh drone catalog";
            AddButton(catalog, "Reload selected from YAML", () =>
            {
                feedback.text = backend.ReloadSelected(poseEntity) ? "Drone reloaded from YAML." : "Pause, take UI control, and select a drone before reloading.";
                nextRefresh = 0;
            });
            AddButton(catalog, "Use this config by default", () => backend.SaveDefaultConfig());
            BuildObjectControls(scene.Body);
            sceneCount = Text(scene.Body, "DRONES  ·  0"); sceneCount.AddToClassList("section-heading");
            entityList = new ListView(entities, 30, MakeEntityRow, (row, index) =>
            {
                row.Q<Label>("entity-name").text = entities[index].Name;
                row.Q<Label>("entity-id").text = entities[index].Id;
                row.tooltip = entities[index].Id;
                row.userData = entities[index].Id;
            }) { selectionType = SelectionType.Single, virtualizationMethod = CollectionVirtualizationMethod.FixedHeight };
            entityList.AddToClassList("entity-list");
            entityList.style.flexGrow = 1; entityList.style.minHeight = 100;
            scene.Body.contentContainer.style.flexGrow = 1;
            entityList.selectionChanged += selection =>
            {
                foreach (WorkspaceEntity entity in selection) { backend.Select(entity.Id); HighlightEntity(entity.Id); nextRefresh = 0; }
            };
            scene.Body.Add(entityList);
            sceneEmpty = Text(scene.Body, "No drones in the scene. Open Drone catalog to spawn one.");
            var sceneHelp = Text(scene.Body, "Click to inspect · Double-click to follow"); sceneHelp.AddToClassList("panel-hint");

            var inspectWindow = AddWindow("Inspector");
            inspector = new WorkspaceReadout(); inspectWindow.Body.Add(inspector);
            Section(inspectWindow.Body, "Transform · World / Y up");
            position = new Vector3Field("Position (m)"); rotation = new Vector3Field("Euler (deg)");
            inspectWindow.Body.Add(position); inspectWindow.Body.Add(rotation);
            position.Query<FloatField>().ForEach(field => field.formatString = "0.###");
            rotation.Query<FloatField>().ForEach(field => field.formatString = "0.###");
            AddButton(inspectWindow.Body, "Read current pose", ReadPose);
            applyPose = AddButton(inspectWindow.Body, "Apply pose", () =>
            {
                feedback.text = backend.ApplyPose(poseEntity, position.value, rotation.value)
                    ? "Pose applied; physics state reset." : "Pose not applied. Pause, use UI authority, and read the selected pose.";
            });
            feedback = Text(inspectWindow.Body, "Pause and use UI authority to edit a pose.");
            var telemetryWindow = AddWindow("Telemetry"); BuildMotorBars(telemetryWindow.Body);
            telemetry = new WorkspaceReadout(); telemetryWindow.Body.Add(telemetry);
            var connectionWindow = AddWindow("Control Authority"); connections = new WorkspaceReadout(); connectionWindow.Body.Add(connections);
            AddButton(connectionWindow.Body, "Take control in this app", () => backend.SwitchToUI());
            Text(connectionWindow.Body, "UI = this app. Internal = simulation code. External = Python SDK / RPC client.\n\nExternal control requests still require Accept or Deny.");
            var logWindow = AddWindow("Logs");
            AddButton(logWindow.Body, "Clear", () => { logs.Clear(); logsDirty = true; });
            logText = Text(logWindow.Body, "No workspace-session logs.");
            BuildLevelControls(AddWindow("Level Settings").Body);
            BuildFlightControls(AddWindow("Flight controls").Body);
            BuildPidControls(AddWindow("PID tuning").Body);
            BuildCamera(AddWindow("Camera").Body);
            BuildApproval();
            RestoreWorkspace();
            BuildDockResizers();
            BuildSelectionHighlight();
            restoreUi = new Button(() => SetUiHidden(false)) { text = "UI", tooltip = "Show simulation interface" };
            restoreUi.AddToClassList("restore-ui"); root.Add(restoreUi);
            SetUiHidden(PlayerPrefs.GetInt("QuadSim.UI.StartHidden", 0) == 1);
        }

        private WorkspaceWindow AddWindow(string title)
        {
            var window = new WorkspaceWindow(title, title, floating, (w, dock) =>
            {
                (dock == WorkspaceDock.Left ? left : dock == WorkspaceDock.Right ? right : dock == WorkspaceDock.Bottom ? bottom : floating).Add(w);
            }, docking);
            windows.Add(title, window);
            window.LayoutChanged += () =>
            {
                if (window.IsOpen) activeTabs[window.Dock] = window;
                RefreshDocks();
            };
            return window;
        }
        private void OpenPanel(string title)
        {
            workspaceShell.style.display = DisplayStyle.Flex;
            if (!windows.TryGetValue(title, out var window)) return;
            if (!window.IsExpanded) window.ToggleExpanded();
            window.Open();
            nextRefresh = 0;
        }
        private void ToggleHud() => SetUiHidden(!uiHidden);
        private void SetUiHidden(bool hidden)
        {
            uiHidden = hidden;
            taskMenu?.RemoveFromHierarchy(); taskMenu = null;
            foreach (var window in windows.Values) window.Cancel();
            keys.Clear();
            if (lookPointer >= 0) viewport.ReleasePointer(lookPointer);
            lookPointer = -1;
            toolbar.style.display = workspaceShell.style.display = status.style.display = hidden ? DisplayStyle.None : DisplayStyle.Flex;
            RefreshChrome();
            if (restoreUi != null) restoreUi.style.display = hidden ? DisplayStyle.Flex : DisplayStyle.None;
            root.pickingMode = PickingMode.Ignore;
            if (hidden && approval != null) approval.style.display = DisplayStyle.None;
            nextRefresh = 0;
        }
        private VisualElement MakeEntityRow()
        {
            var row = new VisualElement(); row.AddToClassList("entity-row");
            row.Add(new WorkspaceIcon("Drone"));
            row.Add(new Label { name = "entity-name" });
            var idLabel = new Label { name = "entity-id" }; idLabel.AddToClassList("entity-id"); row.Add(idLabel);
            row.RegisterCallback<PointerDownEvent>(e =>
            {
                if (e.button != 0 || !(row.userData is string id)) return;
                HighlightEntity(id);
                if (e.clickCount != 2) return;
                backend.Select(id); backend.FollowSelected(); nextRefresh = 0; e.StopPropagation();
            });
            return row;
        }
        private void AddMenu(string title, params (string Label, Action Action)[] entries)
        {
            var button = new Button { text = title };
            button.AddToClassList("taskbar-menu");
            button.clicked += () =>
            {
                taskMenu?.RemoveFromHierarchy();
                var overlay = taskMenu = new VisualElement(); overlay.AddToClassList("menu-overlay"); root.Add(overlay);
                var menu = new VisualElement(); menu.AddToClassList("task-menu"); overlay.Add(menu);
                var anchor = root.WorldToLocal(button.worldBound.position);
                menu.style.left = Mathf.Max(0, Mathf.Min(anchor.x, root.layout.width - 218));
                menu.style.top = anchor.y + button.layout.height + 3;
                var items = new List<Button>();
                void CloseMenu() { overlay.RemoveFromHierarchy(); if (taskMenu == overlay) taskMenu = null; button.Focus(); }
                foreach (var entry in entries)
                {
                    var action = entry.Action;
                    var item = new Button(() => { CloseMenu(); action(); }) { text = entry.Label };
                    if (title == "Window" && windows.TryGetValue(entry.Label, out var listed))
                    {
                        item.text = (listed.IsOpen ? "✓  " : "    ") + entry.Label;
                    }
                    menu.Add(item); items.Add(item);
                }
                overlay.RegisterCallback<PointerDownEvent>(e => { if (e.target == overlay) { CloseMenu(); e.StopPropagation(); } });
                overlay.RegisterCallback<KeyDownEvent>(e =>
                {
                    if (e.keyCode == KeyCode.Escape) { CloseMenu(); e.StopPropagation(); }
                    else if (e.keyCode == KeyCode.DownArrow || e.keyCode == KeyCode.UpArrow)
                    {
                        int current = items.IndexOf(root.focusController.focusedElement as Button);
                        items[(current + (e.keyCode == KeyCode.DownArrow ? 1 : items.Count - 1)) % items.Count].Focus(); e.StopPropagation();
                    }
                });
                if (items.Count > 0) items[0].Focus();
            };
            menuRegion.Add(button);
        }
        private void RefreshDocks()
        {
            RefreshDock(left, WorkspaceDock.Left);
            RefreshDock(right, WorkspaceDock.Right);
            RefreshDock(bottom, WorkspaceDock.Bottom);
            RefreshChrome();
        }
        private void RefreshChrome()
        {
            bool anyOpen = false;
            foreach (var w in windows.Values) if (w.IsOpen) { anyOpen = true; break; }
            if (viewTools != null) viewTools.style.display = DisplayStyle.Flex;
            if (status != null) status.style.display = anyOpen && !uiHidden ? DisplayStyle.Flex : DisplayStyle.None;
        }
        private void OnApplicationQuit() { quitting = true; SaveWorkspace(); }
        private void RefreshDock(VisualElement region, WorkspaceDock dock)
        {
            activeTabs.TryGetValue(dock, out var active);
            if (active == null || !active.IsOpen || active.Dock != dock)
            {
                active = null;
                foreach (var w in windows.Values) if (w.IsOpen && w.Dock == dock) { active = w; break; }
                activeTabs[dock] = active;
            }
            var tabs = region.Q<VisualElement>("dock-tabs");
            if (tabs == null) { tabs = new VisualElement { name = "dock-tabs" }; tabs.AddToClassList("dock-tabs"); region.Insert(0, tabs); }
            tabs.Clear();
            int count = 0;
            foreach (var w in windows.Values)
            {
                if (w.Dock != dock) continue;
                w.SetPresented(w == active);
                if (!w.IsOpen) continue;
                var tab = AddButton(tabs, w.Id, w.Open); tab.EnableInClassList("is-active", w == active); count++;
            }
            tabs.style.display = count > 1 ? DisplayStyle.Flex : DisplayStyle.None;
            region.style.display = active != null ? DisplayStyle.Flex : DisplayStyle.None;
            region.Q<VisualElement>(className: "dock-resizer")?.BringToFront();
        }
        private void ResetLayout()
        {
            workspaceShell.style.display = DisplayStyle.Flex;
            foreach (var w in windows.Values) { if (!w.IsExpanded) w.ToggleExpanded(); w.Place(WorkspaceDock.Right); w.Close(); }
            windows["Scene"].Place(WorkspaceDock.Left);
            windows["Inspector"].Place(WorkspaceDock.Right);
            windows["Logs"].Place(WorkspaceDock.Bottom);
        }
        private void ShowcaseLayout()
        {
            ResetLayout();
            windows["Scene"].Close(); windows["Inspector"].Close(); windows["Logs"].Close();
            windows["Telemetry"].Place(WorkspaceDock.Right);
            workspaceShell.style.display = DisplayStyle.Flex; nextRefresh = 0;
        }
        private static Button AddButton(VisualElement parent, string title, Action action)
        {
            var button = new Button(action) { text = title, tooltip = title };
            if (title == "Pause" || title == "Play" || title == "Step" || title == "Reset" || title == "Spawn" || title == "Refresh" || title == "Showcase")
            { button.Insert(0, new WorkspaceIcon(title)); button.AddToClassList("icon-button"); button.style.minWidth = title.Length * 6.5f + 27; }
            parent.Add(button); return button;
        }
        private static Label Text(VisualElement parent, string text = "")
        {
            var label = new Label(text); label.AddToClassList("data-text"); parent.Add(label); return label;
        }
        private static void Section(VisualElement parent, string title)
        {
            var label = new Label(title); label.AddToClassList("section-heading"); parent.Add(label);
        }
        private static void GroupSections(VisualElement parent)
        {
            var children = new List<VisualElement>(parent.Children());
            Foldout group = null;
            foreach (var child in children)
            {
                if (child is Label heading && child.ClassListContains("section-heading"))
                {
                    group = new Foldout { text = heading.text, value = false };
                    group.AddToClassList("readout-group"); parent.Insert(parent.IndexOf(child), group); child.RemoveFromHierarchy();
                }
                else if (group != null) group.Add(child);
            }
        }
        private void RefreshConfigs()
        {
            if (!backend.Ready) return;
            var choices = backend.ReadConfigs();
            string previous = backend.SelectedConfig;
            configs.choices = choices;
            configs.SetValueWithoutNotify(choices.Contains(previous) ? previous : choices.Count > 0 ? choices[0] : "");
            configLoaded = true;
        }
        private void ReadPose()
        {
            poseEntity = backend.SelectedId;
            backend.TryReadPose(out var p, out var r);
            position.SetValueWithoutNotify(p); rotation.SetValueWithoutNotify(r);
        }

        private void Update()
        {
            if (backend == null) return;
            Flight?.SampleInput();
            if (!uiHidden && windows["Telemetry"].ShouldRefresh) RefreshMotors();
            if (uiHidden) { backend.Connect(); return; }
            if (Time.unscaledDeltaTime > 0) framesPerSecond = Mathf.Lerp(framesPerSecond, 1f / Time.unscaledDeltaTime, 0.05f);
            if (cameraDirector != null && keys.Count > 0 && cameraDirector.Mode == CameraViewMode.Free)
            {
                var move = new Vector3((keys.Contains(KeyCode.D) ? 1 : 0) - (keys.Contains(KeyCode.A) ? 1 : 0),
                    (keys.Contains(KeyCode.E) ? 1 : 0) - (keys.Contains(KeyCode.Q) ? 1 : 0),
                    (keys.Contains(KeyCode.W) ? 1 : 0) - (keys.Contains(KeyCode.S) ? 1 : 0));
                cameraDirector.AddFreeMovementInput(move.normalized, keys.Contains(KeyCode.LeftShift) || keys.Contains(KeyCode.RightShift));
            }
            if (Time.realtimeSinceStartupAsDouble < nextRefresh) return;
            nextRefresh = Time.realtimeSinceStartupAsDouble + 1.0 / Mathf.Max(1, refreshHz);
            backend.Connect();
            if (backend.ApprovalPending && approval.style.display == DisplayStyle.None) ShowApproval();
            if (!backend.ApprovalPending && approval.style.display != DisplayStyle.None) CloseApproval();
            toolbar.SetEnabled(approval.style.display == DisplayStyle.None);
            transport.SetEnabled(backend.Ready);
            if (!backend.Ready) { status.text = "Waiting for SimCoreApi and DroneManager…"; return; }
            Flight?.PruneDestroyed();
            cameraDirector?.EnsureBoundToFirstDrone(DroneManager.Get());
            if (!configLoaded) RefreshConfigs();
            var s = backend.Status;
            authorityShortcut.text = $"Control: {s.CurrentAuthority}";
            authorityShortcut.EnableInClassList("external-control", s.CurrentAuthority == global::SimCore.Common.InputSource.External);
            play.text = s.IsPaused ? "Play" : "Pause"; transportIcon.Symbol = play.text; play.EnableInClassList("is-running", !s.IsPaused);
            viewLabel.text = $"{cameraDirector?.Mode.ToString().ToUpperInvariant() ?? "SCENE"} VIEW  /  {Flight?.Pawn?.DroneID ?? "No selection"}"; step.SetEnabled(s.IsPaused || s.RunMode == global::SimCore.SimulationManager.RunMode.Manual);
            foreach (var tool in viewTools.Children()) tool.EnableInClassList("is-active", tool.userData is CameraViewMode mode && cameraDirector != null && mode == cameraDirector.Mode);
            var focused = root.focusController.focusedElement as VisualElement;
            if (focused == null || (focused != speed && !speed.Contains(focused)))
                speed.SetValueWithoutNotify((float)s.TimeScale);
            spawn.SetEnabled(configs.choices.Count > 0);
            status.text = $"{(s.IsPaused ? "PAUSED" : "RUNNING")}   |   {s.SimTimeSec:F2} s   |   Requested ×{s.TimeScale:F2}   |   dt {s.FixedDtSec * 1000:F1} ms   |   {s.RunMode}   |   Authority: {s.CurrentAuthority}";
            double wall = Time.realtimeSinceStartupAsDouble;
            double factor = wall > lastStatusWall && s.SimTimeSec >= lastStatusSim ? (s.SimTimeSec - lastStatusSim) / (wall - lastStatusWall) : 0;
            lastStatusWall = wall; lastStatusSim = s.SimTimeSec;
            status.text += $"   |   {framesPerSecond:F0} FPS   |   Measured ×{factor:F2}";
            if (windows["Scene"].ShouldRefresh) { RefreshEntities(); RefreshObjectControls(); }
            if (poseEntity != backend.SelectedId) ReadPose();
            if (windows["Inspector"].ShouldRefresh) inspector.SetReport(backend.ReadInspector());
            applyPose.SetEnabled(backend.CanEditPose && backend.SelectedId != null);
            if (windows["Telemetry"].ShouldRefresh) { telemetry.SetReport(backend.ReadTelemetry()); RefreshMotorLabels(); }
            if (windows["Flight controls"].ShouldRefresh) RefreshFlight();
            if (windows["PID tuning"].ShouldRefresh) RefreshPid();
            if (windows["Level Settings"].ShouldRefresh) RefreshLevel();
            if (windows["Control Authority"].ShouldRefresh)
            {
                string connectionReport = $"CONTROL OWNERSHIP\nActive source   {s.CurrentAuthority}\nThis app   {s.UIStatus}\nSimulation code   {s.InternalStatus}\nPython / RPC   {s.ExternalStatus}";
                var rpc = backend.Level?.Rpc;
                connectionReport += rpc == null ? "\n\nNo RPC adapter in this scene." : $"\n\nRPC CONNECTION\nServer   {(rpc.IsServerRunning ? "Running" : "Stopped")}\nCommand   {rpc.BindAddress}:{rpc.CommandPort}\nTelemetry   {rpc.BindAddress}:{rpc.TelemetryPort}\nClient   {(rpc.IsClientConnected ? rpc.ClientName : "Disconnected")}";
                connections.SetReport(connectionReport);
            }
            if (logsDirty && windows["Logs"].ShouldRefresh)
            {
                logText.text = logs.Count > 0 ? string.Join("\n", logs) : "No workspace-session logs."; logsDirty = false;
            }
        }

        private void RefreshEntities()
        {
            backend.ReadEntities(scratch);
            bool changed = scratch.Count != entities.Count;
            for (int i = 0; !changed && i < scratch.Count; i++)
                changed = scratch[i].Id != entities[i].Id || scratch[i].Name != entities[i].Name;
            if (changed) { entities.Clear(); entities.AddRange(scratch); entityList.RefreshItems(); }
            sceneCount.text = $"DRONES  ·  {entities.Count}";
            sceneEmpty.style.display = entities.Count == 0 ? DisplayStyle.Flex : DisplayStyle.None;
            int selected = -1;
            for (int i = 0; i < entities.Count; i++) if (entities[i].Id == backend.SelectedId) { selected = i; break; }
            if (entityList.selectedIndex != selected)
                entityList.SetSelectionWithoutNotify(selected < 0 ? Array.Empty<int>() : new[] { selected });
        }

        private void OnLog(string message, string stack, LogType type)
        {
            // Main-thread logs only. Bounded count AND message length; no stack-trace retention.
            if (logs.Count == 200) logs.Dequeue();
            logs.Enqueue($"[{type}] {(message.Length > 500 ? message.Substring(0, 500) : message)}"); logsDirty = true;
        }
        private void BuildApproval()
        {
            approval = new VisualElement(); approval.AddToClassList("approval-overlay");
            var card = new VisualElement(); card.AddToClassList("approval-card"); approval.Add(card);
            Text(card, "EXTERNAL CONTROL REQUEST\n\nAn external client is requesting control authority.");
            AddButton(card, "Accept", () => ResolveApproval(true));
            AddButton(card, "Deny", () => ResolveApproval(false));
            approval.style.display = DisplayStyle.None; root.Add(approval);
        }
        private void ShowApproval()
        {
            if (uiHidden) return;
            taskMenu?.RemoveFromHierarchy(); taskMenu = null;
            foreach (var window in windows.Values) window.Cancel();
            keys.Clear();
            if (lookPointer >= 0) viewport.ReleasePointer(lookPointer);
            lookPointer = -1;
            foreach (var child in root.Children()) if (child != approval) child.SetEnabled(false);
            approval.style.display = DisplayStyle.Flex; approval.BringToFront(); approval.Q<Button>().Focus();
        }
        private void ResolveApproval(bool accept)
        {
            backend.ApproveExternal(accept);
            CloseApproval();
        }
        private void CloseApproval()
        {
            approval.style.display = DisplayStyle.None;
            foreach (var child in root.Children()) if (child != approval) child.SetEnabled(true);
            toolbar.SetEnabled(true);
            transport.SetEnabled(backend.Ready);
            viewport.Focus();
        }

        private void BuildCamera(VisualElement body)
        {
            Section(body, "View mode");
            var cameraModes = new VisualElement(); cameraModes.AddToClassList("action-row"); body.Add(cameraModes);
            foreach (CameraViewMode mode in Enum.GetValues(typeof(CameraViewMode)))
            {
                var value = mode;
                AddButton(cameraModes, mode.ToString(), () =>
                {
                    if (cameraDirector == null) return;
                    if (Flight?.Pawn != null && value != CameraViewMode.Free) cameraDirector.Bind(Flight.Pawn, false);
                    cameraDirector.SetMode(value);
                });
            }
            AddButton(body, "Follow selected", () =>
            {
                var drone = DroneManager.Get()?.SelectedDrone;
                if (drone != null) { cameraDirector.Bind(drone); cameraDirector.SetMode(CameraViewMode.Follow); }
            });
            var distance = new Slider("Distance (m)", 0.25f, 10f) { showInputField = true };
            distance.RegisterValueChangedCallback(e => { if (cameraDirector != null) cameraDirector.FollowDistance = e.newValue; }); body.Add(distance);
            var fov = new Slider("FOV (deg)", 30, 120) { showInputField = true };
            fov.RegisterValueChangedCallback(e => { if (cameraDirector != null) cameraDirector.FieldOfView = e.newValue; }); body.Add(fov);
            body.schedule.Execute(() =>
            {
                if (cameraDirector == null || !windows["Camera"].ShouldRefresh) return;
                distance.SetValueWithoutNotify(cameraDirector.FollowDistance); fov.SetValueWithoutNotify(cameraDirector.FieldOfView);
            }).Every(200);
            AddButton(body, "Reset orbit", () => cameraDirector?.ResetOrbit());
            BuildCameraDetails(body);
        }
        private void BuildViewportInput()
        {
            viewport.RegisterCallback<PointerDownEvent>(e =>
            {
                viewport.Focus();
                if (e.button == 1) { lookPointer = e.pointerId; viewport.CapturePointer(e.pointerId); }
                if (e.button == 0 && cameraDirector != null && cameraDirector.ViewCamera != null)
                {
                    HighlightEntity(null);
                    // Panel coordinates include DPI scaling; convert to full-screen camera coordinates.
                    Vector2 p = e.position;
                    var screen = new Vector3(p.x / root.layout.width * Screen.width, (1 - p.y / root.layout.height) * Screen.height, 0);
                    if (Physics.Raycast(cameraDirector.ViewCamera.ScreenPointToRay(screen), out var hit))
                    {
                        var pawn = hit.collider.GetComponentInParent<QuadPawn>();
                        if (pawn != null)
                        {
                            backend.Select(pawn.AgentId);
                            HighlightEntity(pawn.AgentId);
                            if (e.clickCount == 2) backend.FollowSelected();
                            nextRefresh = 0;
                        }
                    }
                }
                e.StopPropagation();
            });
            viewport.RegisterCallback<PointerMoveEvent>(e =>
            {
                if (lookPointer == e.pointerId) cameraDirector?.AddMouseCameraInput(e.deltaPosition, 0);
            });
            viewport.RegisterCallback<PointerUpEvent>(e =>
            {
                if (e.button == 1 && lookPointer == e.pointerId) { viewport.ReleasePointer(e.pointerId); lookPointer = -1; }
            });
            viewport.RegisterCallback<PointerCaptureOutEvent>(_ => lookPointer = -1);
            viewport.RegisterCallback<WheelEvent>(e => { cameraDirector?.AddMouseCameraInput(Vector2.zero, -e.delta.y / 3f); e.StopPropagation(); });
            viewport.RegisterCallback<KeyDownEvent>(e =>
            {
                bool first = keys.Add(e.keyCode);
                if (first && e.keyCode == KeyCode.F) cameraDirector?.ToggleFreeFollow(DroneManager.Get()?.SelectedDrone);
                e.StopPropagation();
            });
            viewport.RegisterCallback<KeyUpEvent>(e => { keys.Remove(e.keyCode); e.StopPropagation(); });
            viewport.RegisterCallback<FocusOutEvent>(_ => keys.Clear());
        }
        private void OnApplicationFocus(bool focused)
        {
            if (!focused) SaveWorkspace();
            if (focused) return;
            foreach (var window in windows.Values) window.Cancel();
            keys.Clear();
            if (viewport != null && lookPointer >= 0) viewport.ReleasePointer(lookPointer);
            lookPointer = -1;
        }
    }
}

