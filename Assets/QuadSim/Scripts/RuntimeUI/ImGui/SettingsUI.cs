// Settings##QuadSim window. Config tab is READ-ONLY (YAML is ground truth;
// edit files, not the UI). Reload goes through DroneManager.ReloadCurrentDrone.
using System.Collections.Generic;
using ImGuiNET;
using DroneCore;
using DroneCore.Core;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class SettingsUI
    {
        private const string PrefPersistCP = "QuadSim.UI.PersistControlPanel";
        private const string PrefPersistState = "QuadSim.UI.PersistStateHUD";

        private bool _open;
        private List<string> _configCache;
        private int _configIndex = -1;

        public bool IsOpen => _open;
        public void ToggleOpen() => _open = !_open;

        public bool PersistControlPanel
        {
            get => PlayerPrefs.GetInt(PrefPersistCP, 0) == 1;
            set { PlayerPrefs.SetInt(PrefPersistCP, value ? 1 : 0); PlayerPrefs.Save(); }
        }
        public bool PersistStateHud
        {
            get => PlayerPrefs.GetInt(PrefPersistState, 0) == 1;
            set { PlayerPrefs.SetInt(PrefPersistState, value ? 1 : 0); PlayerPrefs.Save(); }
        }

        public void Draw(DroneManager dm, ControlPanelUI controlPanel, StateHudUI stateHud)
        {
            if (!_open) return;

            ImGui.SetNextWindowSize(new ImVec2(620f, 520f), ImGuiCond.FirstUseEver);
            if (ImGui.Begin("Settings##QuadSim", ref _open))
            {
                if (ImGui.BeginTabBar("##SettingsTabs"))
                {
                    if (ImGui.BeginTabItem("Config")) { DrawConfigTab(dm); ImGui.EndTabItem(); }
                    if (ImGui.BeginTabItem("Toggables")) { DrawTogglablesTab(dm, controlPanel, stateHud); ImGui.EndTabItem(); }
                    ImGui.EndTabBar();
                }
            }
            ImGui.End();
        }

        private void DrawConfigTab(DroneManager dm)
        {
            if (dm == null) { ImGui.TextDisabled("No DroneManager."); return; }

            // ---- Config selection (for NEXT spawn/reload) ----
            if (_configCache == null) RefreshConfigs(dm);
            ImGui.Text("Drone Config (YAML)"); ImGui.Separator();
            string current = dm.SelectedConfigName;
            if (ImGui.BeginCombo("Selected Config", current))
            {
                for (int i = 0; i < _configCache.Count; i++)
                {
                    bool sel = _configCache[i] == current;
                    if (ImGui.Selectable(_configCache[i], sel))
                        dm.SetSelectedConfig(_configCache[i]);
                    if (sel) ImGui.SetItemDefaultFocus();
                }
                ImGui.EndCombo();
            }
            ImGui.SameLine();
            if (ImGui.SmallButton("Refresh")) RefreshConfigs(dm);

            if (ImGui.Button("Reload Drone (re-read YAML)"))
                UiActions.Defer(() => dm.ReloadCurrentDrone(resetPosition:false));
            
            ImGui.SameLine();
            ImGui.TextDisabled("Save: edit YAML directly (StreamingAssets/Configs/Drones)");

            // ---- Read-only view of the SELECTED drone's applied config ----
            QuadPawn pawn = dm.SelectedDrone;
            if (pawn == null || !pawn.HasConfigApplied)
            {
                ImGui.TextDisabled("No drone with applied config.");
                return;
            }
            var cfg = pawn.Config;

            ImGui.Separator();
            ImGui.Text("Model"); ImGui.Separator();
            ImGui.LabelText("Name", cfg.ModelParams.Name ?? "(unnamed)");

            ImGui.Separator();
            ImGui.Text("Drone Parameters"); ImGui.Separator();
            ImGui.LabelText("Mass (kg)", $"{cfg.DroneParams.Mass:F3}");
            ImGui.LabelText("Arm Length (m)", $"{cfg.DroneParams.Arm_Length:F3}");
            ImGui.LabelText("Inertia (kg m2)", $"{cfg.DroneParams.Inertia}");

            ImGui.Separator();
            ImGui.Text("Flight Parameters (limits)"); ImGui.Separator();
            var fp = cfg.FlightParams;
            ImGui.LabelText("Max Velocity XY (m/s)", $"{fp.MaxVelXY:F2}");
            ImGui.LabelText("Max Velocity Z (m/s)", $"{fp.MaxVelZ:F2}");
            ImGui.LabelText("Max Tilt Angle (deg)", $"{fp.MaxAngle:F1}");
            ImGui.LabelText("Max Rate R/P (deg/s)", $"{fp.MaxRateRollPitch:F1}");
            ImGui.LabelText("Max Rate Yaw (deg/s)", $"{fp.MaxRateYaw:F1}");
            ImGui.LabelText("Max Thrust total (N)", $"{fp.MaxThrust:F2}");
            ImGui.LabelText("Max PID Output", $"{fp.MaxPID:F2}");
            ImGui.LabelText("Acceptable Distance (m)", $"{fp.AcceptableDistance:F2}");
        }

        private void RefreshConfigs(DroneManager dm)
        {
            _configCache = dm.GetAvailableConfigs() ?? new List<string>();
        }

        private void DrawTogglablesTab(DroneManager dm, ControlPanelUI cp, StateHudUI hud)
        {
            if (ImGui.Button("Spawn Drone Now") && dm != null)
                UiActions.Defer(() => dm.SpawnDrone(Vector3.zero, Quaternion.identity));
            ImGui.SameLine();
            ImGui.TextDisabled("(DroneManager auto-spawns one on sim start)");

            ImGui.Separator();
            bool pcp = PersistControlPanel;
            if (ImGui.Checkbox("Persistent Control Panel", ref pcp))
            {
                PersistControlPanel = pcp;
                if (cp != null && cp.IsOpen != pcp) cp.ToggleOpen();
            }
            bool psh = PersistStateHud;
            if (ImGui.Checkbox("Persistent State Info", ref psh))
            {
                PersistStateHud = psh;
                hud?.SetVisible(psh);
            }

            ImGui.Separator();
            ImGui.Text("Spawning");
            ImGui.BeginDisabled(true);
            bool obstacles = false;
            ImGui.Checkbox("Spawn Obstacles", ref obstacles);   // TODO: no obstacle system in Unity yet
            ImGui.EndDisabled();
            ImGui.SameLine();
            ImGui.TextDisabled("TODO");
        }
    }
}