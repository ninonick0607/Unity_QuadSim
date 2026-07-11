// Port of ControlPanelUI.cpp. 380px fixed-width, auto-height, right-anchored on
// first show. All command widgets gate on CanUICommand (authority == UI).
//
// Deviations from the old UE panel:
//  - PX4 checkbox -> "FRD" frame toggle.
//  - Angle/Acro W is Throttle01, not Z-velocity.
//  - No manual waypoint queue / no session limit sliders.
//  - Hover = Position mode holding current XY/yaw at HoverAlt.
using System.Collections.Generic;
using ImGuiNET;
using SimCore;
using SimCore.Common;
using DroneCore;
using DroneCore.Api;
using DroneCore.Core;
using MathUtil;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class ControlPanelUI
    {
        private sealed class DronePanelState
        {
            public bool Gamepad;
            public bool HoverActive;
            public float HoverAlt = 3f;
            public readonly UiCommandState Cmd = new();
        }

        private const float PanelW = 380f;
        private const float RightPad = 10f;
        private const float ResetHighAltM = 5f;
        private const float PosRangeM = 50f;
        private const float AltMaxM = 50f;

        private readonly Dictionary<QuadPawn, DronePanelState> _states = new();
        private bool _open;
        private float _panelX = -1f;
        private float _panelY = 80f;

        public bool IsOpen => _open;
        public void ToggleOpen() => _open = !_open;

        public void OnSimReset()
        {
            foreach (var s in _states.Values)
            {
                s.Cmd.ZeroAll();
                s.HoverActive = false;
            }
        }

        public UiCommandState GetCommandState(QuadPawn pawn)
            => pawn != null ? GetState(pawn).Cmd : null;

        private DronePanelState GetState(QuadPawn pawn)
        {

            List<QuadPawn> dead = null;
            foreach (var k in _states.Keys)
                if (k == null) (dead ??= new List<QuadPawn>()).Add(k);
            if (dead != null)
                foreach (var k in dead) _states.Remove(k);

            if (!_states.TryGetValue(pawn, out var s))
            {
                s = new DronePanelState();
                _states[pawn] = s;
            }
            return s;
        }

        public void Draw(SimCoreApi sim, DroneManager dm)
        {
            if (!_open || sim == null || dm == null || dm.DroneCount == 0) return;

            QuadPawn pawn = dm.SelectedDrone;
            if (pawn == null) return;

            var api = pawn.GetComponent<QuadSimApi>();
            var ctrl = pawn.Controller;
            if (api == null || ctrl == null) return;

            var state = GetState(pawn);
            bool canCommand = sim.CurrentAuthority == InputSource.UI;

            var io = ImGui.GetIO();
            ImGui.SetNextWindowPos(
                new ImVec2(Mathf.Max(0f, io.DisplaySize.X - PanelW - RightPad), 80f),
                ImGuiCond.FirstUseEver);

            ImGui.SetNextWindowSizeConstraints(
                new ImVec2(PanelW, 0f),
                new ImVec2(PanelW, float.MaxValue));

            var wFlags = ImGuiWindowFlags.NoResize | ImGuiWindowFlags.AlwaysAutoResize;

            if (ImGui.Begin($"Control Panel - {pawn.DroneID}##CP_{pawn.GetInstanceID()}", ref _open, wFlags))
            {
                ImGui.PushID(pawn.GetInstanceID());

                DrawDroneCombo(dm, pawn);
                DrawAuthorityRow(sim, canCommand);

                ImGui.Separator();

                ImGui.BeginDisabled(!canCommand);
                DrawToggleRow(pawn, api, ctrl, state, canCommand);
                ImGui.Separator();
                DrawModeButtons(pawn, api, state);
                ImGui.EndDisabled();

                ImGui.Separator();

                DrawFlightControls(pawn, api, state, canCommand);

                ImGui.Spacing();
                ImGui.PushStyleVar(ImGuiStyleVar.FramePadding, new ImVec2(16f, 9f));

                if (ImGui.Button("Switch Camera##ControlPanelSwitchCamera"))
                    UiActions.Defer(() => pawn.SwitchCamera());

                ImGui.SameLine();

                ImGui.BeginDisabled(!canCommand);

                if (ImGui.Button("Reset High##ControlPanelResetHigh"))
                {
                    state.HoverActive = false;
                    UiActions.Defer(() => api.ResetPose(dm.SpawnOrigin + Vector3.up * ResetHighAltM, Quaternion.identity));
                }

                ImGui.SameLine();

                if (ImGui.Button("Reset Low##ControlPanelResetLow"))
                {
                    state.HoverActive = false;
                    UiActions.Defer(() => dm.ResetSelectedDrone());
                }

                ImGui.SameLine();

                if (ImGui.Button("Rotation##ControlPanelResetRotation"))
                    UiActions.Defer(() => api.ResetRotation());

                ImGui.EndDisabled();
                ImGui.PopStyleVar();

                ImGui.PopID();
            }

            ImGui.End();
        }

        private void DrawDroneCombo(DroneManager dm, QuadPawn active)
        {
            if (ImGui.BeginCombo("Active Drone##ControlPanelActiveDrone", active.DroneID))
            {
                var drones = dm.RegisteredDrones;
                for (int i = 0; i < drones.Count; i++)
                {
                    if (drones[i] == null) continue;

                    ImGui.PushID(i);

                    bool selected = i == dm.SelectedDroneIndex;
                    int idx = i;
                    if (ImGui.Selectable($"{drones[i].DroneID}##DroneSelect", selected))
                        UiActions.Defer(() => dm.SelectDroneByIndex(idx, alsoPossess: true));

                    if (selected)
                        ImGui.SetItemDefaultFocus();

                    ImGui.PopID();
                }

                ImGui.EndCombo();
            }
        }

        private void DrawAuthorityRow(SimCoreApi sim, bool canCommand)
        {
            var auth = sim.CurrentAuthority;

            if (canCommand)
            {
                ImGui.TextColored(SimImGuiStyle.AccentGreen, $"Authority: {auth}");
            }
            else
            {
                ImGui.TextColored(SimImGuiStyle.WarnOrange, $"{auth} owns authority - UI commands disabled");
                ImGui.SameLine();

                if (ImGui.SmallButton("Switch to UI##ControlPanelSwitchToUI"))
                    sim.SwitchToUI();
            }
        }

        private void DrawToggleRow(
            QuadPawn pawn,
            QuadSimApi api,
            DroneCore.Controllers.CascadedController ctrl,
            DronePanelState state,
            bool canCommand)
        {
            ImGui.Checkbox("Gamepad##ControlPanelGamepad", ref state.Gamepad);
            ImGui.SameLine();

            var sensors = pawn.Sensors;
            // NOTE: this toggles the BODY-axis convention (IMU/mag) only.
            // GPS local position stays world-frame ENU in both modes, so the
            // cascaded PositionController state is convention-independent.
            bool frd = sensors != null && sensors.outputFrame == SimFrame.FRD;

            if (ImGui.Checkbox("FRD##ControlPanelFrameFRD", ref frd) && sensors != null && canCommand)
            {
                sensors.outputFrame = frd ? SimFrame.FRD : SimFrame.FLU;
                ctrl.RebuildAllocator();
                api.ResetController();
                Debug.Log($"[ControlPanelUI] Output frame -> {sensors.outputFrame}");
            }

            if (ImGui.IsItemHovered())
            {
                ImGui.SetTooltip(
                    "Sensor output frame FLU <-> FRD (PX4 convention).\n" +
                    "Rebuilds allocator + resets controller state.");
            }

            ImGui.SameLine();

            bool dbg = ctrl.DebugVisualsEnabled;
            if (ImGui.Checkbox("Debug##ControlPanelDebug", ref dbg))
                ctrl.DebugVisualsEnabled = dbg;
        }

        private void DrawModeButtons(QuadPawn pawn, QuadSimApi api, DronePanelState state)
        {
            void EnterMode(GoalMode mode)
            {
                api.SetMode(mode);
                InitCommandForMode(pawn, state, mode);
            }

            if (!state.Gamepad)
            {
                if (ImGui.Button("Position##ModePosition")) EnterMode(GoalMode.Position);
                ImGui.SameLine();
                if (ImGui.Button("Velocity##ModeVelocity")) EnterMode(GoalMode.Velocity);
                ImGui.SameLine();
                if (ImGui.Button("Angle##ModeAngle")) EnterMode(GoalMode.Angle);
                ImGui.SameLine();
                if (ImGui.Button("Acro##ModeRate")) EnterMode(GoalMode.Rate);
            }
            else
            {
                if (ImGui.Button("Gamepad Angle##ModeGamepadAngle")) EnterMode(GoalMode.Angle);
                ImGui.SameLine();
                if (ImGui.Button("Gamepad Acro##ModeGamepadRate")) EnterMode(GoalMode.Rate);
            }
        }

        private void InitCommandForMode(QuadPawn pawn, DronePanelState state, GoalMode mode)
        {
            var cmd = state.Cmd;
            var latest = pawn.Sensors != null ? pawn.Sensors.Latest : default;
            float hoverThrottle = HoverThrottle01(pawn);

            switch (mode)
            {
                case GoalMode.Position:
                    cmd.PosX = latest.GpsPosition.x;
                    cmd.PosY = latest.GpsPosition.y;
                    cmd.PosAlt = latest.GpsPosition.z;
                    cmd.PosYawDeg = latest.ImuAttitude.z;
                    break;

                case GoalMode.Velocity:
                    cmd.VelX = 0f;
                    cmd.VelY = 0f;
                    cmd.VelZ = 0f;
                    cmd.YawRateDps = 0f;
                    break;

                case GoalMode.Angle:
                    cmd.RollDeg = 0f;
                    cmd.PitchDeg = 0f;
                    cmd.YawRateDps = 0f;
                    cmd.Throttle01 = hoverThrottle;
                    break;

                case GoalMode.Rate:
                    cmd.RollRateDps = 0f;
                    cmd.PitchRateDps = 0f;
                    cmd.YawRateDps = 0f;
                    cmd.Throttle01 = hoverThrottle;
                    break;
            }
        }

        private static float HoverThrottle01(QuadPawn pawn)
        {
            if (pawn.Rigidbody == null || !pawn.HasConfigApplied)
                return 0.4f;

            float maxThrust = pawn.Config.FlightParams.MaxThrust;
            if (maxThrust <= 0f)
                return 0.4f;

            return Mathf.Clamp01(pawn.Rigidbody.mass * 9.81f / maxThrust);
        }
        private static bool SliderFloatZero(
            string label,
            string id,
            ref float value,
            float min,
            float max,
            string format,
            float resetValue = 0f)
        {
            float buttonW = 28f;
            float spacing = ImGui.GetStyle().ItemSpacing.X;
            float sliderW = Mathf.Max(80f, ImGui.GetContentRegionAvail().X - buttonW - spacing);

            ImGui.PushID(id);

            ImGui.SetNextItemWidth(sliderW);
            bool changed = ImGui.SliderFloat(label + "##slider", ref value, min, max, format);

            ImGui.SameLine();

            if (ImGui.SmallButton("0##zero"))
            {
                value = resetValue;
                changed = true;
            }

            ImGui.PopID();
            return changed;
        }
        private void DrawFlightControls(QuadPawn pawn, QuadSimApi api, DronePanelState state, bool canCommand)
        {
            if (!ImGui.CollapsingHeader("Flight Controls##ControlPanelFlightControls", ImGuiTreeNodeFlags.DefaultOpen))
                return;

            var fp = pawn.HasConfigApplied ? pawn.Config.FlightParams : default;

            float maxVelXY = fp.MaxVelXY > 0f ? fp.MaxVelXY : 5f;
            float maxVelZ = fp.MaxVelZ > 0f ? fp.MaxVelZ : 3f;
            float maxAngle = fp.MaxAngle > 0f ? fp.MaxAngle : 30f;
            float maxRateRP = fp.MaxRateRollPitch > 0f ? fp.MaxRateRollPitch : 180f;
            float maxRateYaw = fp.MaxRateYaw > 0f ? fp.MaxRateYaw : 90f;

            var cmd = state.Cmd;
            GoalMode mode = api.CurrentMode;

            ImGui.BeginDisabled(!canCommand);

            if (state.HoverActive)
            {
                ImGui.PushStyleColor(ImGuiCol.Button, SimImGuiStyle.AccentGreen);
                ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new System.Numerics.Vector4(0.25f, 0.80f, 0.35f, 1f));
                ImGui.PushStyleColor(ImGuiCol.ButtonActive, new System.Numerics.Vector4(0.18f, 0.60f, 0.28f, 1f));
            }

            if (ImGui.Button("ACTIVATE HOVER MODE##ControlPanelHoverToggle"))
            {
                state.HoverActive = !state.HoverActive;

                if (state.HoverActive)
                {
                    api.SetMode(GoalMode.Position);

                    var latest = pawn.Sensors != null ? pawn.Sensors.Latest : default;
                    cmd.PosX = latest.GpsPosition.x;
                    cmd.PosY = latest.GpsPosition.y;
                    cmd.PosYawDeg = latest.ImuAttitude.z;
                    cmd.PosAlt = state.HoverAlt;
                }
            }

            if (state.HoverActive)
                ImGui.PopStyleColor(3);

            if (ImGui.SliderFloat("Hover Altitude (m)##ControlPanelHoverAlt",
                    ref state.HoverAlt, 0.5f, AltMaxM, "%.1f") && state.HoverActive)
            {
                cmd.PosAlt = state.HoverAlt;
            }

            string label = mode switch
            {
                GoalMode.Position => "Desired Position (X, Y, Z) and Yaw",
                GoalMode.Velocity => "Desired Velocities (X, Y, Z) and Yaw Rate",
                GoalMode.Angle => "Desired Angles (Roll, Pitch), Yaw Rate, Throttle",
                GoalMode.Rate => "Desired Rates (Roll, Pitch, Yaw), Throttle",
                _ => null,
            };

            if (label != null)
            {
                ImGui.Separator();
                ImGui.TextUnformatted(label);
            }

            bool yawApplicable = mode != GoalMode.Position;

            switch (mode)
            {
                case GoalMode.Position:
                    SliderFloatZero("X (m)", "ControlPanelPosX", ref cmd.PosX, -PosRangeM, PosRangeM, "%.2f");
                    SliderFloatZero("Y (m)", "ControlPanelPosY", ref cmd.PosY, -PosRangeM, PosRangeM, "%.2f");
                    SliderFloatZero("Z (m)", "ControlPanelPosZ", ref cmd.PosAlt, 0f, AltMaxM, "%.2f");
                    SliderFloatZero("Yaw (deg)", "ControlPanelPosYaw", ref cmd.PosYawDeg, -180f, 180f, "%.1f");
                    break;

                case GoalMode.Velocity:
                    SliderFloatZero("Velocity X (m/s)", "ControlPanelVelX", ref cmd.VelX, -maxVelXY, maxVelXY, "%.2f");
                    SliderFloatZero("Velocity Y (m/s)", "ControlPanelVelY", ref cmd.VelY, -maxVelXY, maxVelXY, "%.2f");
                    SliderFloatZero("Velocity Z (m/s)", "ControlPanelVelZ", ref cmd.VelZ, -maxVelZ, maxVelZ, "%.2f");
                    SliderFloatZero("Yaw Rate (deg/s)", "ControlPanelVelYawRate", ref cmd.YawRateDps, -maxRateYaw, maxRateYaw, "%.1f");
                    break;

                case GoalMode.Angle:
                    SliderFloatZero("Roll (deg)", "ControlPanelAngleRoll", ref cmd.RollDeg, -maxAngle, maxAngle, "%.1f");
                    SliderFloatZero("Pitch (deg)", "ControlPanelAnglePitch", ref cmd.PitchDeg, -maxAngle, maxAngle, "%.1f");
                    SliderFloatZero("Yaw Rate (deg/s)", "ControlPanelAngleYawRate", ref cmd.YawRateDps, -maxRateYaw, maxRateYaw, "%.1f");
                    SliderFloatZero("Throttle [0..1]", "ControlPanelAngleThrottle", ref cmd.Throttle01, 0f, 1f, "%.2f");
                    break;

                case GoalMode.Rate:
                    SliderFloatZero("Roll Rate (deg/s)", "ControlPanelRateRoll", ref cmd.RollRateDps, -maxRateRP, maxRateRP, "%.1f");
                    SliderFloatZero("Pitch Rate (deg/s)", "ControlPanelRatePitch", ref cmd.PitchRateDps, -maxRateRP, maxRateRP, "%.1f");
                    SliderFloatZero("Yaw Rate (deg/s)", "ControlPanelRateYaw", ref cmd.YawRateDps, -maxRateYaw, maxRateYaw, "%.1f");
                    SliderFloatZero("Throttle [0..1]", "ControlPanelRateThrottle", ref cmd.Throttle01, 0f, 1f, "%.2f");
                    break;

                default:
                    ImGui.TextDisabled($"Mode {mode} is not UI-driven (external/wrench path).");
                    break;
            }

            if (mode == GoalMode.Position || mode == GoalMode.Velocity ||
                mode == GoalMode.Angle || mode == GoalMode.Rate)
            {
                ImGui.Separator();
                ImGui.TextUnformatted("Reset to 0:");
                ImGui.SameLine();

                if (ImGui.SmallButton("X##ControlPanelResetX"))
                {
                    switch (mode)
                    {
                        case GoalMode.Position: cmd.PosX = 0f; break;
                        case GoalMode.Velocity: cmd.VelX = 0f; break;
                        case GoalMode.Angle: cmd.RollDeg = 0f; break;
                        case GoalMode.Rate: cmd.RollRateDps = 0f; break;
                    }
                }

                ImGui.SameLine();

                if (ImGui.SmallButton("Y##ControlPanelResetY"))
                {
                    switch (mode)
                    {
                        case GoalMode.Position: cmd.PosY = 0f; break;
                        case GoalMode.Velocity: cmd.VelY = 0f; break;
                        case GoalMode.Angle: cmd.PitchDeg = 0f; break;
                        case GoalMode.Rate: cmd.PitchRateDps = 0f; break;
                    }
                }

                ImGui.SameLine();

                if (ImGui.SmallButton("Z##ControlPanelResetZ"))
                {
                    switch (mode)
                    {
                        case GoalMode.Position: cmd.PosAlt = 0f; break;
                        case GoalMode.Velocity: cmd.VelZ = 0f; break;
                        default: cmd.Throttle01 = 0f; break;
                    }
                }

                if (yawApplicable)
                {
                    ImGui.SameLine();

                    if (ImGui.SmallButton("Yaw##ControlPanelResetYaw"))
                        cmd.YawRateDps = 0f;
                }
            }

            ImGui.EndDisabled();
        }
    }
}