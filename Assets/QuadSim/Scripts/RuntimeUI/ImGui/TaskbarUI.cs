// Port of the old QuadSim Taskbar (ImGuiBootstrapSubsystem.cpp lines 99-391).
// Full-width, 58px, top-anchored. Sim controls via SimCoreApi ONLY —
// never Unity Time.timeScale (ClockFactory owns time).
using ImGuiNET;
using SimCore;
using UnityEngine;
using DroneCore.Core;
using ImVec2 = System.Numerics.Vector2;
using ImVec4 = System.Numerics.Vector4;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class TaskbarUI
    {
        public const float BarHeight = 58f;

        private float _speedScale = -1f;   // init from sim on first draw
        private bool _simMgrActive, _worldMgrActive;
        private float _fps;

        public System.Action OnSettingsClicked;
        public System.Action OnSimReset;   // lets panels clear stale UI command state

        public void Draw(SimCoreApi sim, DroneManager dm)
        {
            var io = ImGui.GetIO();
            if (_speedScale < 0f)
                _speedScale = Mathf.Max(0.05f, (float)sim.TimeScale);

            float dt = Time.unscaledDeltaTime;
            if (dt > 0f) _fps = Mathf.Lerp(_fps <= 0f ? 1f / dt : _fps, 1f / dt, 0.05f);

            bool paused = sim.IsPaused;                    // sim is the truth, not a local flag
            string pauseTxt = paused ? "Play" : "Pause";

            var flags = ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize |
                        ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoScrollbar |
                        ImGuiWindowFlags.NoSavedSettings | ImGuiWindowFlags.NoFocusOnAppearing;

            ImGui.SetNextWindowPos(new ImVec2(0f, 0f));
            ImGui.SetNextWindowSize(new ImVec2(io.DisplaySize.X, BarHeight));
            ImGui.PushStyleVar(ImGuiStyleVar.WindowBorderSize, 1.5f);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowRounding, 0f);
            ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, new ImVec2(10f, 6f));
            ImGui.PushStyleColor(ImGuiCol.Border, new ImVec4(0.06f, 0.07f, 0.09f, 1f));

            if (ImGui.Begin("QuadSim Taskbar##Main", flags))
            {
                const float statusW = 200f;
                if (ImGui.BeginTable("##TaskbarTable", 2,
                        ImGuiTableFlags.SizingFixedFit | ImGuiTableFlags.NoPadInnerX |
                        ImGuiTableFlags.BordersInnerV))
                {
                    ImGui.TableSetupColumn("Controls", ImGuiTableColumnFlags.WidthStretch);
                    ImGui.TableSetupColumn("Status", ImGuiTableColumnFlags.WidthFixed, statusW);
                    ImGui.TableNextRow();

                    // ---- Controls column ----
                    ImGui.TableSetColumnIndex(0);
                    var st = ImGui.GetStyle();
                    float padX = st.FramePadding.X, padY = st.FramePadding.Y, itemX = st.ItemSpacing.X;
                    const float labelBoxW = 180f, wmLabelW = 170f, speedBoxW = 56f;
                    float labelH = BarHeight - 12f;
                    float btnH = ImGui.GetTextLineHeight() + padY * 2f;
                    float labelTopY = Mathf.Max(0f, (BarHeight - labelH) * 0.5f);
                    float btnTopY = labelTopY + Mathf.Max(0f, (labelH - btnH) * 0.5f);

                    float BtnW(string t) => ImGui.CalcTextSize(t).X + padX * 2f;

                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Settings")) OnSettingsClicked?.Invoke();
                    ImGui.SameLine();

                    // Center the main control group (excluding left-anchored Settings)
                    float totalW = labelBoxW + itemX + BtnW("Slower") + itemX + BtnW(pauseTxt) +
                                   itemX + BtnW("Faster>>") + itemX + speedBoxW + itemX +
                                   BtnW("Step") + itemX + BtnW("Reset") + itemX + 40f +
                                   wmLabelW + itemX + BtnW("Spawn Drone");
                    float colStartX = ImGui.GetCursorPosX();
                    float avail = ImGui.GetContentRegionAvail().X;
                    float settingsW = ImGui.CalcTextSize("Settings").X + padX * 2f + itemX;
                    float startXWithin = Mathf.Max(0f, (avail - settingsW - totalW) * 0.5f);
                    ImGui.SetCursorPosY(labelTopY);
                    ImGui.SetCursorPosX(colStartX + startXWithin);

                    if (ImGuiWidgets.RoundedLabel("##SimMgrLabelBtn", "Simulation Manager",
                            labelBoxW, labelH, _simMgrActive))
                        _simMgrActive = !_simMgrActive;
                    ImGui.SameLine();

                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Slower"))
                    {
                        _speedScale = Mathf.Clamp(_speedScale - 0.10f, 0.05f, 100f);
                        sim.SetTimeScale(_speedScale);
                    }
                    ImGui.SameLine();
                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button(pauseTxt)) sim.TogglePause();
                    ImGui.SameLine();
                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Faster>>"))
                    {
                        _speedScale = Mathf.Max(1.0f, _speedScale + 0.25f);
                        sim.SetTimeScale(_speedScale);
                    }
                    ImGui.SameLine();

                    // Speed display box
                    {
                        const float boxH = 28f;
                        ImGui.SetCursorPosY(btnTopY + (btnH - boxH) * 0.5f);
                        ImGuiWidgets.RoundedLabel("##SpeedBox", $"x{_speedScale:F2}", speedBoxW, boxH);
                    }
                    ImGui.SameLine();
                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Step")) sim.StepOnce();
                    ImGui.SameLine();
                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Reset"))
                    {
                        _speedScale = 1f;
                        sim.SetTimeScale(1.0);
                        sim.Resume();
                        sim.ResetSimulation();
                        OnSimReset?.Invoke();
                    }

                    ImGui.SameLine(); ImGui.Dummy(new ImVec2(40f, 0f)); ImGui.SameLine();
                    ImGui.SetCursorPosY(labelTopY);
                    if (ImGuiWidgets.RoundedLabel("##WorldMgrLabelBtn", "World Manager",
                            wmLabelW, labelH, _worldMgrActive))
                        _worldMgrActive = !_worldMgrActive;
                    ImGui.SameLine();
                    ImGui.SetCursorPosY(btnTopY);
                    if (ImGui.Button("Spawn Drone") && dm != null)
                        dm.SpawnDrone(Vector3.zero, Quaternion.identity); // self-selects + FPV cam

                    // ---- Status column ----
                    ImGui.TableSetColumnIndex(1);
                    ImGuiWidgets.RoundedLabel("##FPSBox", $"FPS: {_fps:F1}", statusW, BarHeight - 12f);

                    ImGui.EndTable();
                }
            }
            ImGui.End();
            ImGui.PopStyleColor();
            ImGui.PopStyleVar(3);
        }
    }
}