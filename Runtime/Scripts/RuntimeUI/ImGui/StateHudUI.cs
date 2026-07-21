// Port of the State Data HUD (ImGuiBootstrapSubsystem.cpp lines 458-566 + PID block).
// Pure read path: SensorManager.Latest + CascadedController.Thrusts/TelemetrySnapshot.
using ImGuiNET;
using DroneCore;
using DroneCore.Core;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class StateHudUI
    {
        private bool _open;
        private bool _showPidSettings;
        private readonly PidSettingsUI _pid = new();

        public bool IsOpen => _open;
        public void ToggleOpen() => _open = !_open;
        public void SetVisible(bool v) => _open = v;

        public void Draw(DroneManager dm)
        {
            if (!_open || dm == null || dm.DroneCount == 0) return;
            QuadPawn pawn = dm.SelectedDrone;
            if (pawn == null || pawn.Controller == null) return;
            var ctrl = pawn.Controller;

            ImGui.SetNextWindowPos(new ImVec2(10f, 100f), ImGuiCond.Always);
            ImGui.SetNextWindowSize(new ImVec2(340f, 460f), ImGuiCond.FirstUseEver);
            if (ImGui.Begin("State Data##HUD", ref _open, ImGuiWindowFlags.NoCollapse))
            {
                DrawThrustBars(pawn, ctrl);

                ImGui.Separator();
                ImGui.TextColored(SimImGuiStyle.HeaderBlue, "State Info");
                ImGui.Separator();

                var s = pawn.Sensors != null ? pawn.Sensors.Latest : default;
                float mass = pawn.Rigidbody != null ? pawn.Rigidbody.mass : 0f;
                ImGui.Text($"Drone Mass: {mass:F2} kg");

                var tel = ctrl.TelemetrySnapshot;

                ImGui.TextColored(SimImGuiStyle.HeaderBlue, "IMU");
                ImGui.Text($"Valid: {s.ImuValid}   t={s.ImuTimestampSec:F3}s");

                if (!s.ImuValid)
                {
                    ImGui.TextDisabled("IMU has not produced a valid sample yet.");
                }
                else
                {
                    Vector3 ratesDeg = s.ImuAngVel * Mathf.Rad2Deg;

                    ImGui.Text($"Attitude R/P/Y: {s.ImuAttitude.x:F2} / {s.ImuAttitude.y:F2} / {s.ImuAttitude.z:F2} deg");
                    ImGui.Text($"Angular Rates R/P/Y: {ratesDeg.x:F2} / {ratesDeg.y:F2} / {ratesDeg.z:F2} deg/s");
                    ImGui.Text($"Body Velocity X/Y/Z: {s.ImuVel.x:F2} / {s.ImuVel.y:F2} / {s.ImuVel.z:F2} m/s");
                    ImGui.Text($"Specific Force X/Y/Z: {s.ImuAccel.x:F2} / {s.ImuAccel.y:F2} / {s.ImuAccel.z:F2} m/s2");
                }

                ImGui.Spacing();
                ImGui.Text($"Desired Roll/Pitch/Yaw: {tel.DesiredAnglesDeg.x:F2} / {tel.DesiredAnglesDeg.y:F2} / {tel.DesiredAnglesDeg.z:F2} deg");
                ImGui.Text($"Desired Rates R/P/Y: {tel.DesiredRatesDeg.x:F2} / {tel.DesiredRatesDeg.y:F2} / {tel.DesiredRatesDeg.z:F2} deg/s");
                ImGui.Text($"Desired Velocity: {tel.DesiredVel.x:F2} / {tel.DesiredVel.y:F2} / {tel.DesiredVel.z:F2} m/s");
                ImGui.Separator();

                ImGui.TextColored(SimImGuiStyle.HeaderYellow, "GPS");
                ImGui.Text($"Position: X {s.GpsPosition.x:F1} Y {s.GpsPosition.y:F1} Z {s.GpsPosition.z:F1}");
                ImGui.Text($"Geo: Lat {s.GpsLatDeg:F5} Lon {s.GpsLonDeg:F5} Alt {s.GpsAltM:F1}");
                ImGui.Separator();

                ImGui.TextColored(SimImGuiStyle.HeaderYellow, "Magnetometer");
                ImGui.Text($"Field: X {s.MagField.x:F3} Y {s.MagField.y:F3} Z {s.MagField.z:F3} Gauss");
                ImGui.Text($"Heading: {s.MagHeadingDeg:F1} deg   Declination: {s.MagDeclinationDeg:F1} deg");
                ImGui.Separator();

                ImGui.TextColored(SimImGuiStyle.HeaderPurple, "Barometer");
                ImGui.Text($"Pressure: {s.BaroPressureHPa:F2} hPa");
                ImGui.Text($"Temperature: {s.BaroTemperatureC:F1} C");
                ImGui.Text($"Altitude: {s.BaroAltitudeMSL:F1} m");

                ImGui.Separator();
                ImGui.Checkbox("Enable PID Settings", ref _showPidSettings);
                if (_showPidSettings)
                    _pid.Draw(pawn, ctrl);
            }
            ImGui.End();

            if (_showPidSettings)
                _pid.DrawHistoryWindow(pawn);
        }

        private void DrawThrustBars(QuadPawn pawn, DroneCore.Controllers.CascadedController ctrl)
        {
            ImGui.Text("Thruster Power");

            var motor01 = ctrl.LastMotor01;
            var thrusts = ctrl.Thrusts;
            string[] labels = { "FL", "FR", "BL", "BR" };

            int count = Mathf.Min(4, Mathf.Min(motor01.Count, thrusts.Count));
            if (count <= 0)
            {
                ImGui.TextDisabled("No motor telemetry yet.");
                return;
            }

            if (ImGui.BeginTable("##ThrustBars", count, ImGuiTableFlags.SizingFixedFit))
            {
                for (int i = 0; i < count; i++)
                {
                    ImGui.TableNextColumn();
                    ImGui.PushID(i);

                    float motor = motor01[i];
                    float thrust = thrusts[i];

                    if (float.IsNaN(motor) || float.IsInfinity(motor)) motor = 0f;
                    if (float.IsNaN(thrust) || float.IsInfinity(thrust)) thrust = 0f;

                    float percent = Mathf.Clamp01(motor);

                    var p = ImGui.GetCursorScreenPos();
                    var barSize = new ImVec2(22f, 100f);
                    var q = new ImVec2(p.X + barSize.X, p.Y + barSize.Y);

                    ImGui.InvisibleButton("##bar", barSize);

                    var dl = ImGui.GetWindowDrawList();
                    dl.AddRect(p, q, SimImGuiStyle.Col32(255, 255, 255, 255));

                    float fillH = barSize.Y * percent;
                    var fillMin = new ImVec2(p.X, q.Y - fillH);
                    dl.AddRectFilled(fillMin, q, SimImGuiStyle.Col32(255, 200, 0, 255));

                    string n = $"{thrust:F1}N";
                    var ts = ImGui.CalcTextSize(n);
                    var textY = fillH > ts.Y ? fillMin.Y + (fillH - ts.Y) * 0.5f : q.Y - ts.Y - 2f;

                    dl.AddText(
                        new ImVec2(p.X + (barSize.X - ts.X) * 0.5f, textY),
                        SimImGuiStyle.Col32(255, 255, 255, 255),
                        n);

                    ImGui.Text(i < labels.Length ? labels[i] : "M");

                    ImGui.PopID();
                }

                ImGui.EndTable();
            }
        }
        
    }
}