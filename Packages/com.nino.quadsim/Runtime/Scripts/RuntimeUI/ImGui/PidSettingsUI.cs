// PID tuning UI over the existing CascadedController hooks ONLY:
//   GetPIDGains / SetPIDGainTerm / IsPIDStageActive
// No second PID model. SetPIDGainTerm called only on actual value change.
// Exposes all 4 cascade stages (old UE UI lacked Velocity + Angle-Yaw; the
// current system has them, so they're shown).
// Save/History: RuntimeUI-owned CSV at persistentDataPath — core untouched.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using ImGuiNET;
using DroneCore;
using DroneCore.Api;
using DroneCore.Controllers;
using UnityEngine;
using ImVec2 = System.Numerics.Vector2;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class PidSettingsUI
    {
        // Fixed CSV schema: (stage, axis, shortName) — deterministic column order
        private static readonly (CascadedController.EPIDStage Stage, byte Axis, string Name)[] Schema =
        {
            (CascadedController.EPIDStage.Position, 0, "posX"),
            (CascadedController.EPIDStage.Position, 1, "posY"),
            (CascadedController.EPIDStage.Position, 3, "posZ"),
            (CascadedController.EPIDStage.Velocity, 0, "velX"),
            (CascadedController.EPIDStage.Velocity, 1, "velY"),
            (CascadedController.EPIDStage.Velocity, 3, "velZ"),
            (CascadedController.EPIDStage.Angle,    0, "angRoll"),
            (CascadedController.EPIDStage.Angle,    1, "angPitch"),
            (CascadedController.EPIDStage.Angle,    2, "angYaw"),
            (CascadedController.EPIDStage.Rate,     0, "rateRoll"),
            (CascadedController.EPIDStage.Rate,     1, "ratePitch"),
            (CascadedController.EPIDStage.Rate,     2, "rateYaw"),
        };

        private bool _syncPosXY, _syncVelXY, _syncAngleRP, _syncRateRP;
        private bool _showHistory;
        private List<string[]> _historyRows;   // cached CSV rows
        private string[] _historyHeader;

        private static string CsvPath => Path.Combine(Application.persistentDataPath, "PIDGains.csv");

        public void Draw(QuadPawn pawn, CascadedController ctrl)
        {
            if (!ImGui.CollapsingHeader("PID Settings", ImGuiTreeNodeFlags.DefaultOpen)) return;

            DrawStageGroup(ctrl, "Position PID Gains", CascadedController.EPIDStage.Position,
                new (byte, string)[] { (0, "X"), (1, "Y"), (3, "Z") },
                ref _syncPosXY, "Synchronize X and Y Axis Gains", syncA: 0, syncB: 1,
                min: 0.0001f, max: 30f);

            ImGui.Separator();
            DrawStageGroup(ctrl, "Velocity PID Gains", CascadedController.EPIDStage.Velocity,
                new (byte, string)[] { (0, "X"), (1, "Y"), (3, "Z") },
                ref _syncVelXY, "Synchronize X and Y Velocity Gains", syncA: 0, syncB: 1,
                min: 0.0001f, max: 30f);

            ImGui.Separator();
            DrawStageGroup(ctrl, "Attitude PID Gains", CascadedController.EPIDStage.Angle,
                new (byte, string)[] { (0, "Roll"), (1, "Pitch"), (2, "Yaw") },
                ref _syncAngleRP, "Synchronize Roll and Pitch Gains", syncA: 0, syncB: 1,
                min: 0.0001f, max: 30f);

            ImGui.Separator();
            DrawStageGroup(ctrl, "Angular Rate PID Gains", CascadedController.EPIDStage.Rate,
                new (byte, string)[] { (0, "Roll Rate"), (1, "Pitch Rate"), (2, "Yaw Rate") },
                ref _syncRateRP, "Synchronize Roll and Pitch Rate Gains", syncA: 0, syncB: 1,
                min: 0.0001f, max: 2f);

            ImGui.Separator();
            if (ImGui.Button("Save PID Gains", new ImVec2(200f, 50f)))
                SaveCsv(ctrl);
            ImGui.SameLine();
            if (ImGui.Checkbox("PID Configuration History", ref _showHistory) && _showHistory)
                LoadCsvCache();
        }

        private void DrawStageGroup(CascadedController ctrl, string title,
            CascadedController.EPIDStage stage, (byte Axis, string Label)[] axes,
            ref bool sync, string syncLabel, byte syncA, byte syncB, float min, float max)
        {
            ImGui.Text(title);
            ImGui.Checkbox(syncLabel + "##" + title, ref sync);
            ImGui.Indent();

            foreach (var (axis, label) in axes)
            {
                if (!ctrl.GetPIDGains(stage, axis, out float p, out float i, out float d))
                {
                    ImGui.TextDisabled($"{label} PID Unavailable");
                    continue;
                }

                bool active = ctrl.IsPIDStageActive(stage, axis);
                ImGui.Text(active ? label : $"{label} (inactive in current mode)");
                ImGui.Indent();
                ImGui.PushID(title + label);

                DrawTerm(ctrl, stage, axis, 0, $"{label} P", ref p, min, max, sync, syncA, syncB);
                DrawTerm(ctrl, stage, axis, 1, $"{label} I", ref i, min, max, sync, syncA, syncB);
                DrawTerm(ctrl, stage, axis, 2, $"{label} D", ref d, min, max, sync, syncA, syncB);

                ImGui.PopID();
                ImGui.Unindent();
            }
            ImGui.Unindent();
        }

        private void DrawTerm(CascadedController ctrl, CascadedController.EPIDStage stage,
            byte axis, int term, string label, ref float value, float min, float max,
            bool sync, byte syncA, byte syncB)
        {
            float before = value;
            if (ImGuiWidgets.SliderWithInput(label, ref value, min, max) &&
                !Mathf.Approximately(before, value))
            {
                ctrl.SetPIDGainTerm(stage, axis, term, value, resetIntegral: false);
                if (sync)
                {
                    byte other = axis == syncA ? syncB : (axis == syncB ? syncA : axis);
                    if (other != axis)
                        ctrl.SetPIDGainTerm(stage, other, term, value, resetIntegral: false);
                }
            }
        }

        // ---- CSV save/history ----

        private void SaveCsv(CascadedController ctrl)
        {
            try
            {
                if (!File.Exists(CsvPath))
                {
                    var header = new List<string> { "Timestamp" };
                    foreach (var e in Schema)
                        header.AddRange(new[] { e.Name + "_P", e.Name + "_I", e.Name + "_D" });
                    File.WriteAllText(CsvPath, string.Join(",", header) + "\n");
                }

                var row = new List<string> { DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") };
                foreach (var e in Schema)
                {
                    ctrl.GetPIDGains(e.Stage, e.Axis, out float p, out float i, out float d);
                    row.Add(p.ToString("F6", CultureInfo.InvariantCulture));
                    row.Add(i.ToString("F6", CultureInfo.InvariantCulture));
                    row.Add(d.ToString("F6", CultureInfo.InvariantCulture));
                }
                File.AppendAllText(CsvPath, string.Join(",", row) + "\n");
                LoadCsvCache();
                Debug.Log($"[PidSettingsUI] Gains saved -> {CsvPath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[PidSettingsUI] Save failed: {e.Message}");
            }
        }

        private void LoadCsvCache()
        {
            _historyRows = new List<string[]>();
            _historyHeader = null;
            if (!File.Exists(CsvPath)) return;
            var lines = File.ReadAllLines(CsvPath);
            if (lines.Length < 1) return;
            _historyHeader = lines[0].Split(',');
            for (int i = 1; i < lines.Length; i++)
                if (!string.IsNullOrWhiteSpace(lines[i]))
                    _historyRows.Add(lines[i].Split(','));
        }

        public void DrawHistoryWindow(QuadPawn pawn)
        {
            if (!_showHistory) return;

            ImGui.SetNextWindowPos(new ImVec2(420f, 520f), ImGuiCond.FirstUseEver);
            ImGui.SetNextWindowSize(new ImVec2(800f, 400f), ImGuiCond.FirstUseEver);
            if (ImGui.Begin("PID Configurations History", ref _showHistory))
            {
                if (_historyHeader == null || _historyRows == null || _historyRows.Count == 0)
                {
                    ImGui.TextColored(SimImGuiStyle.AccentRed, $"No PID history at: {CsvPath}");
                }
                else if (ImGui.BeginTable("PIDHistoryTable", _historyHeader.Length,
                             ImGuiTableFlags.Borders | ImGuiTableFlags.RowBg |
                             ImGuiTableFlags.ScrollY | ImGuiTableFlags.ScrollX |
                             ImGuiTableFlags.SizingFixedFit))
                {
                    ImGui.TableSetupScrollFreeze(1, 1);
                    foreach (var h in _historyHeader)
                        ImGui.TableSetupColumn(h, ImGuiTableColumnFlags.WidthFixed);
                    ImGui.TableHeadersRow();

                    for (int r = 0; r < _historyRows.Count; r++)
                    {
                        var row = _historyRows[r];
                        ImGui.TableNextRow();
                        for (int c = 0; c < row.Length && c < _historyHeader.Length; c++)
                        {
                            ImGui.TableSetColumnIndex(c);
                            if (c == 0)
                            {
                                ImGui.TextUnformatted(row[0]);
                                ImGui.SameLine();
                                if (ImGui.SmallButton($"Load##{r}"))
                                    ApplyRow(pawn, row);
                            }
                            else
                            {
                                ImGui.TextUnformatted(row[c]);
                            }
                        }
                    }
                    ImGui.EndTable();
                }
            }
            ImGui.End();
        }

        private void ApplyRow(QuadPawn pawn, string[] row)
        {
            var ctrl = pawn.Controller;
            var api = pawn.GetComponent<QuadSimApi>();
            if (ctrl == null) return;

            int col = 1; // col 0 = timestamp
            foreach (var e in Schema)
            {
                for (int term = 0; term < 3; term++, col++)
                {
                    if (col >= row.Length) return;
                    if (float.TryParse(row[col], NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
                        ctrl.SetPIDGainTerm(e.Stage, e.Axis, term, v, resetIntegral: false);
                }
            }
            api?.ResetController();   // clean integrators once after bulk apply
            Debug.Log($"[PidSettingsUI] Applied gains from history row ({row[0]})");
        }
    }
}