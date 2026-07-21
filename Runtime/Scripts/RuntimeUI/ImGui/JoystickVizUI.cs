// Bottom-center stick visualization for Angle/Rate modes.
// Currently visualizes the UI COMMAND STATE (what's actually being streamed).
// TODO(RC injection): swap the data source to real gamepad/RC input when it lands.
using ImGuiNET;
using SimCore.Common;
using DroneCore;
using DroneCore.Api;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class JoystickVizUI
    {
        public void Draw(QuadPawn pawn, QuadSimApi api, UiCommandState cmd)
        {
            if (pawn == null || api == null || cmd == null) return;
            GoalMode mode = api.CurrentMode;
            if (mode != GoalMode.Angle && mode != GoalMode.Rate) return;

            var fp = pawn.HasConfigApplied ? pawn.Config.FlightParams : default;
            float maxYaw = fp.MaxRateYaw > 0f ? fp.MaxRateYaw : 90f;
            float maxRP = mode == GoalMode.Angle
                ? (fp.MaxAngle > 0f ? fp.MaxAngle : 30f)
                : (fp.MaxRateRollPitch > 0f ? fp.MaxRateRollPitch : 180f);

            // Left stick: (yaw, throttle). Right stick: (roll, pitch).
            float lx = Mathf.Clamp(cmd.YawRateDps / maxYaw, -1f, 1f);
            float ly = Mathf.Clamp(cmd.Throttle01 * 2f - 1f, -1f, 1f);
            float rx = Mathf.Clamp((mode == GoalMode.Angle ? cmd.RollDeg : cmd.RollRateDps) / maxRP, -1f, 1f);
            float ry = Mathf.Clamp((mode == GoalMode.Angle ? cmd.PitchDeg : cmd.PitchRateDps) / maxRP, -1f, 1f);

            var io = ImGui.GetIO();
            const float w = 260f, h = 160f;
            ImGui.SetNextWindowPos(new ImVec2((io.DisplaySize.X - w) * 0.5f, io.DisplaySize.Y - h - 10f),
                ImGuiCond.Always);
            ImGui.SetNextWindowSize(new ImVec2(w, h), ImGuiCond.Always);
            var wf = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize |
                     ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoBackground;
            if (ImGui.Begin("Joystick##BottomViz", wf))
            {
                DrawStick("Left Stick", "##leftStick", lx, -ly);
                ImGui.SameLine(0f, 30f);
                DrawStick("Right Stick", "##rightStick", rx, ry);
            }
            ImGui.End();
        }

        private static void DrawStick(string title, string id, float x, float y)
        {
            var boxSize = new ImVec2(100f, 100f);
            var dl = ImGui.GetWindowDrawList();
            ImGui.BeginGroup();
            ImGui.Text(title);
            ImGui.InvisibleButton(id, boxSize);
            var p = ImGui.GetItemRectMin();
            var q = ImGui.GetItemRectMax();
            dl.AddRect(p, q, SimImGuiStyle.Col32(255, 255, 255, 255));
            var center = new ImVec2((p.X + q.X) * 0.5f, (p.Y + q.Y) * 0.5f);
            var pos = new ImVec2(center.X + x * boxSize.X * 0.5f, center.Y + y * boxSize.Y * 0.5f);
            dl.AddCircleFilled(pos, 5f, SimImGuiStyle.Col32(255, 255, 0, 255));
            ImGui.EndGroup();
        }
    }
}