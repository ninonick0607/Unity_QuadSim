using ImGuiNET;
using SimCore;
using SimCore.Common;
using DroneCore;
using DroneCore.Api;
using DroneCore.CameraSystem;
using DroneCore.Core;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    [DefaultExecutionOrder(1000)]
    public sealed class ImGuiRuntimeBootstrap : MonoBehaviour
    {
        private SimCoreApi _sim;
        private DroneManager _dm;
        private UImGui.UImGui _ui;
        private Camera _boundCamera;
        private CameraDirector _cameraDirector;

        private TaskbarUI _taskbar;
        private ControlPanelUI _controlPanel;
        private StateHudUI _stateHud;
        private SettingsUI _settings;
        private JoystickVizUI _joystickViz;

        private bool _appliedStartupPrefs;
        private bool _externalPendingApproval;
        private bool _layoutSeen;
        private bool _subscribed;
        private bool _hudVisible = true;
        private bool _cameraLookDrag;
        private float _noLayoutWarnTimer;

        private void OnEnable()
        {
            if (!Workspace.RuntimeWorkspaceStartup.GraphicsAvailable || !Workspace.RuntimeWorkspaceStartup.DeveloperHud)
            {
                var renderer = GetComponent<UImGui.UImGui>();
                if (renderer != null) renderer.enabled = false;
                enabled = false;
                return;
            }
            _appliedStartupPrefs = false;
            _layoutSeen = false;
            _noLayoutWarnTimer = 0f;
            _taskbar = new TaskbarUI();
            _controlPanel = new ControlPanelUI();
            _stateHud = new StateHudUI();
            _settings = new SettingsUI();
            _joystickViz = new JoystickVizUI();
            _cameraDirector = CameraDirector.GetOrCreate(gameObject);

            _taskbar.OnSettingsClicked = () => _settings.ToggleOpen();
            _taskbar.OnSimReset = () => _controlPanel.OnSimReset();

            _ui = GetComponent<UImGui.UImGui>();
            if (_ui == null)
            {
                Debug.LogError("[ImGuiRuntimeBootstrap] Missing UImGui component on same GameObject.");
                return;
            }

            _ui.Layout += OnLayout;
            _subscribed = true;
        }

        private void OnDisable()
        {
            if (_ui != null && _subscribed)
            {
                _ui.Layout -= OnLayout;
                _subscribed = false;
            }
            UnsubscribeSim();
            // ResolveRefs must re-subscribe after switching back from the native workspace.
            _sim = null;
        }

        private void ResolveRefs()
        {
            if (_sim == null)
            {
                _sim = SimCoreApi.Get();
                if (_sim != null)
                    _sim.OnExternalConnectionRequest += HandleExternalRequest;
            }
            if (_dm == null)
                _dm = DroneManager.Get();
            if (_cameraDirector == null)
                _cameraDirector = CameraDirector.GetOrCreate(gameObject);
            if (_cameraDirector != null)
                _cameraDirector.EnsureBoundToFirstDrone(_dm);
        }

        private void UnsubscribeSim()
        {
            if (_sim != null)
                _sim.OnExternalConnectionRequest -= HandleExternalRequest;
        }

        private void HandleExternalRequest() => _externalPendingApproval = true;

        // ====================================================================
        // Camera binding — keep UImGui pointed at whatever camera is live.
        // Survives drone reload/spawn (new camera object) and drone selection.
        // Switch Camera moves the SAME camera between mounts, so no re-bind.
        // ====================================================================
        private void EnsureUiCamera()
        {
            if (_ui == null) return;

            _cameraDirector?.EnsureBoundToFirstDrone(_dm);
            Camera target = _cameraDirector != null ? _cameraDirector.ViewCamera : null;
            if (target == null && _dm != null && _dm.DroneCount > 0)
                target = _dm.RegisteredDrones[0]?.DroneCamera;
            if (target == null || !target.enabled || !target.gameObject.activeInHierarchy)
                target = Camera.main;

            if (target == null || target == _boundCamera) return;

            _ui.SetCamera(target);
            _boundCamera = target;
            Debug.Log($"[ImGuiRuntimeBootstrap] UImGui camera -> {target.name}");
        }

        private void Update()
        {
            ResolveRefs();

            // Drain deferred UI actions FIRST — spawn/reload/resets happen here,
            // safely outside the render pipeline.
            UiActions.Drain();

            EnsureUiCamera();

            if (_sim == null) return;

            if (!_layoutSeen)
            {
                _noLayoutWarnTimer += Time.unscaledDeltaTime;
                if (_noLayoutWarnTimer > 5f)
                {
                    Debug.LogWarning("[ImGuiRuntimeBootstrap] No ImGui layout callback after 5s. " +
                                     "Check that UImGui is enabled, has a camera, and the RenderImGui pass is active.");
                    _noLayoutWarnTimer = float.NegativeInfinity;
                }
            }

            // ---- Command streaming (independent of HUD visibility) ----
            if (_sim.CurrentAuthority != InputSource.UI) return;
            if (_dm == null || _dm.DroneCount == 0) return;

            QuadPawn streamPawn = _dm.SelectedDrone;
            if (streamPawn == null) return;

            var api = streamPawn.GetComponent<QuadSimApi>();
            if (api == null || !api.IsInitialized) return;

            GoalMode mode = api.CurrentMode;
            if (mode != GoalMode.Position &&
                mode != GoalMode.Velocity &&
                mode != GoalMode.Angle &&
                mode != GoalMode.Rate)
            {
                return;
            }

            var cmd = _controlPanel.GetCommandState(streamPawn);
            if (cmd == null) return;

            api.SendCommand(InputSource.UI, cmd.Build(mode));
        }

        private void OnLayout(UImGui.UImGui _)
        {
            _layoutSeen = true;

            ResolveRefs();
            if (_sim == null) return;

            SimImGuiStyle.Apply();

            if (!_appliedStartupPrefs)
            {
                _appliedStartupPrefs = true;
                if (_settings.PersistControlPanel && !_controlPanel.IsOpen)
                    _controlPanel.ToggleOpen();
                if (_settings.PersistStateHud)
                    _stateHud.SetVisible(true);
            }

            if (_hudVisible)
            {
                _taskbar.Draw(_sim, _dm);
                DrawFloatingButtons();
                _controlPanel.Draw(_sim, _dm);
                _stateHud.Draw(_dm);
                _settings.Draw(_dm, _controlPanel, _stateHud);
                DrawJoystickViz();
            }

            DrawExternalApprovalPopup();   // safety-relevant: always drawn
            DrawHudToggleButton();         // last = topmost, always drawn
            HandleCameraInput();
        }

        private void HandleCameraInput()
        {
            if (_cameraDirector == null) return;

            var io = ImGui.GetIO();

            // Camera look must begin as an intentional right-click in the
            // unobstructed viewport. Do not let a drag that started on an
            // ImGui window leak through when the cursor later leaves it.
            if (ImGui.IsMouseReleased(ImGuiMouseButton.Right))
                _cameraLookDrag = false;
            if (ImGui.IsMouseClicked(ImGuiMouseButton.Right))
                _cameraLookDrag = !io.WantCaptureMouse;

            if (!io.WantCaptureMouse)
            {
                var mouseDelta = _cameraLookDrag && ImGui.IsMouseDown(ImGuiMouseButton.Right)
                    ? new Vector2(io.MouseDelta.X, io.MouseDelta.Y)
                    : Vector2.zero;
                _cameraDirector.AddMouseCameraInput(mouseDelta, io.MouseWheel);

                if (_cameraDirector.Mode == CameraViewMode.Free
                    && ImGui.IsMouseClicked(ImGuiMouseButton.Left))
                {
                    TrySelectDroneUnderMouse(io.MousePos);
                }
            }

            if (_cameraDirector.Mode != CameraViewMode.Free || io.WantCaptureKeyboard)
                return;

            Vector3 movement = Vector3.zero;
            if (ImGui.IsKeyDown(ImGuiKey.W)) movement.z += 1f;
            if (ImGui.IsKeyDown(ImGuiKey.S)) movement.z -= 1f;
            if (ImGui.IsKeyDown(ImGuiKey.D)) movement.x += 1f;
            if (ImGui.IsKeyDown(ImGuiKey.A)) movement.x -= 1f;
            if (ImGui.IsKeyDown(ImGuiKey.E)) movement.y += 1f;
            if (ImGui.IsKeyDown(ImGuiKey.Q)) movement.y -= 1f;

            bool boosted = ImGui.IsKeyDown(ImGuiKey.LeftShift)
                        || ImGui.IsKeyDown(ImGuiKey.RightShift);
            _cameraDirector.AddFreeMovementInput(movement.normalized, boosted);

            if (ImGui.IsKeyPressed(ImGuiKey.F, false))
                _cameraDirector.ToggleFreeFollow(_dm != null ? _dm.SelectedDrone : null);
        }

        private void TrySelectDroneUnderMouse(System.Numerics.Vector2 mousePosition)
        {
            Camera camera = _cameraDirector.ViewCamera;
            if (camera == null || _dm == null) return;

            Rect pixelRect = camera.pixelRect;
            Vector3 screenPoint = new Vector3(
                pixelRect.x + mousePosition.X,
                pixelRect.y + pixelRect.height - mousePosition.Y,
                0f);

            if (!Physics.Raycast(camera.ScreenPointToRay(screenPoint), out RaycastHit hit))
                return;

            QuadPawn drone = hit.collider.GetComponentInParent<QuadPawn>();
            int index = drone != null ? _dm.GetDroneIndex(drone) : -1;
            if (index >= 0)
                _dm.SelectDroneByIndex(index, alsoPossess: false);
        }

        // ====================================================================
        // Master HUD toggle — invisible 26x26 button in the top-left corner.
        // Highlights on hover. Hides/shows the entire HUD.
        // ====================================================================
        private void DrawHudToggleButton()
        {
            const float size = 26f;

            ImGui.SetNextWindowPos(new ImVec2(0f, 0f));
            ImGui.SetNextWindowSize(new ImVec2(size + 4f, size + 4f));
            ImGui.SetNextWindowBgAlpha(0f);

            var flags = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize |
                        ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoScrollbar |
                        ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBackground |
                        ImGuiWindowFlags.NoFocusOnAppearing;

            ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, new ImVec2(2f, 2f));
            if (ImGui.Begin("##QuadSimHudToggle", flags))
            {
                if (ImGui.InvisibleButton("##HudToggleBtn", new ImVec2(size, size)))
                    _hudVisible = !_hudVisible;

                bool hovered = ImGui.IsItemHovered();
                var min = ImGui.GetItemRectMin();
                var max = ImGui.GetItemRectMax();
                var dl = ImGui.GetWindowDrawList();

                // Faint outline always (so it's findable); bright on hover.
                // Set the first alpha to 0 if you want it truly invisible at rest.
                uint col = hovered ? SimImGuiStyle.Col32(120, 200, 255, 230)
                                   : SimImGuiStyle.Col32(255, 255, 255, 25);
                dl.AddRect(min, max, col, 5f, ImDrawFlags.None, hovered ? 2f : 1f);

                if (hovered)
                {
                    // Eye-ish marker + tooltip
                    var c = new ImVec2((min.X + max.X) * 0.5f, (min.Y + max.Y) * 0.5f);
                    dl.AddCircleFilled(c, 4f, col);
                    ImGui.SetTooltip(_hudVisible ? "Hide HUD" : "Show HUD");
                }
            }
            ImGui.End();
            ImGui.PopStyleVar();
        }

        private void DrawFloatingButtons()
        {
            if (_dm == null || _dm.DroneCount == 0) return;

            var io = ImGui.GetIO();
            const float pad = 12f;

            ImGui.SetNextWindowPos(new ImVec2(io.DisplaySize.X - 240f - pad, io.DisplaySize.Y - 120f));
            ImGui.SetNextWindowSize(new ImVec2(240f, 104f));
            ImGui.SetNextWindowBgAlpha(0f);

            var flags = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize |
                        ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoSavedSettings |
                        ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoFocusOnAppearing |
                        ImGuiWindowFlags.NoBackground;

            if (ImGui.Begin("##QuadSimControlPanelBtn", flags))
            {
                ImGui.PushID("FloatingPanelButtons");
                ImGui.PushStyleVar(ImGuiStyleVar.FramePadding, new ImVec2(24f, 14f));

                float cx = (ImGui.GetContentRegionAvail().X
                            - ImGui.CalcTextSize("Control Panel").X
                            - ImGui.GetStyle().FramePadding.X * 2f) * 0.5f;

                ImGui.SetCursorPosX(Mathf.Max(0f, cx));
                if (ImGui.Button("Control Panel##FloatingControlPanel"))
                    _controlPanel.ToggleOpen();

                ImGui.SetCursorPosX(Mathf.Max(0f, cx));
                if (ImGui.Button("State Data##FloatingStateData"))
                    _stateHud.ToggleOpen();

                ImGui.PopStyleVar();
                ImGui.PopID();
            }
            ImGui.End();
        }

        private void DrawJoystickViz()
        {
            if (_dm == null || _dm.DroneCount == 0) return;

            QuadPawn pawn = _dm.SelectedDrone;
            if (pawn == null) return;

            var api = pawn.GetComponent<QuadSimApi>();
            _joystickViz.Draw(pawn, api, _controlPanel.GetCommandState(pawn));
        }

        private void DrawExternalApprovalPopup()
        {
            if (_externalPendingApproval)
            {
                ImGui.OpenPopup("External Connection##QuadSim");
                _externalPendingApproval = false;
            }

            bool open = true;
            if (ImGui.BeginPopupModal("External Connection##QuadSim", ref open, ImGuiWindowFlags.AlwaysAutoResize))
            {
                ImGui.Text("An External client is requesting control authority.");
                ImGui.Separator();

                if (ImGui.Button("Accept##ExternalAccept", new ImVec2(120f, 0f)))
                {
                    _sim.AcceptExternalConnection();
                    ImGui.CloseCurrentPopup();
                }
                ImGui.SameLine();
                if (ImGui.Button("Deny##ExternalDeny", new ImVec2(120f, 0f)))
                {
                    _sim.DenyExternalConnection();
                    ImGui.CloseCurrentPopup();
                }
                ImGui.EndPopup();
            }
        }
    }
}
