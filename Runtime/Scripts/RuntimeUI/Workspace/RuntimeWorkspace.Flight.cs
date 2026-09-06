using System;
using System.Collections.Generic;
using System.IO;
using DroneCore;
using DroneCore.Controllers;
using MathUtil;
using SimCore.Common;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed partial class RuntimeWorkspace
    {
        private VisualElement flightCommands, pidCommands;
        private readonly System.Collections.Generic.Dictionary<GoalMode, Button> modeButtons = new System.Collections.Generic.Dictionary<GoalMode, Button>();
        private Label flightStatus, pidStatus, inputStatus;
        private readonly Slider[] commandSliders = new Slider[4];
        private readonly FloatField[,] pidFields = new FloatField[12, 3];
        private readonly Label[] pidLabels = new Label[12];
        private readonly Toggle[] syncPid = new Toggle[4];
        private Toggle hoverToggle, gamepadToggle, frdToggle, debugToggle;
        private Slider hoverAltitude;
        private string flightId, pidId;
        private GoalMode flightMode;
        private DropdownField historyChoice;
        private Label historyDetail;
        private Button applyHistory;
        private List<string> historyRows = new List<string>();
        private readonly VisualElement[] motors = new VisualElement[4];
        private readonly Label[] motorLabels = new Label[4];
        private readonly float[] motorLevels = new float[4];
        private QuadPawn motorPawn;
        private WorkspaceStick leftStick, rightStick;
        private WorkspaceFlightSession Flight => backend?.Flight;
        private static string PidPath => Path.Combine(Application.persistentDataPath, "PIDGains.csv");

        private void BuildFlightControls(VisualElement parent)
        {
            flightStatus = Text(parent, "Select a drone to command it.");
            var ownership = new VisualElement(); ownership.AddToClassList("action-row"); parent.Add(ownership);
            AddButton(ownership, "Select drone…", () => OpenPanel("Scene"));
            AddButton(ownership, "Take control in this app", () => backend.SwitchToUI());
            flightCommands = new VisualElement(); parent.Add(flightCommands);
            modeButtons.Clear();
            Section(flightCommands, "Control mode");
            var modes = new VisualElement(); modes.AddToClassList("action-row"); flightCommands.Add(modes);
            foreach (GoalMode mode in new[] { GoalMode.Position, GoalMode.Velocity, GoalMode.Angle, GoalMode.Rate })
            {
                var value = mode;
                modeButtons[mode] = AddButton(modes, mode == GoalMode.Rate ? "Acro" : mode.ToString(), () => { Flight?.EnterMode(flightId, value); RefreshFlight(); });
            }
            var inputOptions = new Foldout { text = "Input & diagnostics", value = false }; inputOptions.AddToClassList("readout-group"); flightCommands.Add(inputOptions);
            frdToggle = new Toggle("FRD body frame (PX4)");
            frdToggle.RegisterValueChangedCallback(e => { Flight?.SetFrame(flightId, e.newValue); RefreshFlight(); }); inputOptions.Add(frdToggle);
            debugToggle = new Toggle("Debug visuals");
            debugToggle.RegisterValueChangedCallback(e => Flight?.SetDebug(flightId, e.newValue)); inputOptions.Add(debugToggle);
            gamepadToggle = new Toggle("Gamepad input (Angle / Acro)");
            gamepadToggle.RegisterValueChangedCallback(e => { Flight?.SetGamepad(flightId, e.newValue); RefreshFlight(); }); inputOptions.Add(gamepadToggle);
            inputStatus = Text(inputOptions);
            Section(flightCommands, "Hover");
            hoverToggle = new Toggle("Hold current XY and yaw");
            hoverToggle.RegisterValueChangedCallback(e => { Flight?.Hover(flightId, e.newValue, hoverAltitude.value); RefreshFlight(); }); flightCommands.Add(hoverToggle);
            hoverAltitude = new Slider("Altitude (m)", 0.5f, 50) { showInputField = true, value = 3 };
            hoverAltitude.RegisterValueChangedCallback(e => Flight?.Hover(flightId, hoverToggle.value, e.newValue)); flightCommands.Add(hoverAltitude);
            Section(flightCommands, "Commands · world ENU / selected body frame");
            for (int i = 0; i < 4; i++)
            {
                int axis = i;
                var row = new VisualElement(); row.AddToClassList("command-row"); flightCommands.Add(row);
                var slider = new Slider("Axis", -1, 1) { showInputField = true }; commandSliders[i] = slider; row.Add(slider);
                slider.RegisterValueChangedCallback(e => Flight?.SetAxis(flightId, flightMode, axis, e.newValue));
                AddButton(row, "0", () => { Flight?.SetAxis(flightId, flightMode, axis, 0); RefreshFlight(); });
            }
            var resets = new VisualElement(); resets.AddToClassList("action-row"); flightCommands.Add(resets);
            AddButton(resets, "Reset high", () => ResetFlight("high"));
            AddButton(resets, "Reset low", () => ResetFlight("low"));
            AddButton(resets, "Level rotation", () => ResetFlight("rotation"));
            AddButton(parent, "Follow / FPV", () =>
            {
                if (Flight?.Pawn == null || cameraDirector == null) return;
                cameraDirector.Bind(Flight.Pawn); cameraDirector.ToggleFollowFpv();
            });
            AddButton(parent, "PID tuning…", () => OpenPanel("PID tuning"));
            var sticks = new VisualElement(); sticks.AddToClassList("action-row"); parent.Add(sticks);
            leftStick = new WorkspaceStick("Yaw / throttle"); rightStick = new WorkspaceStick("Roll / pitch");
            sticks.Add(leftStick); sticks.Add(rightStick);
        }
        private void ResetFlight(string action) { Flight?.ResetDrone(flightId, action); RefreshFlight(); }
        private bool Editing(VisualElement element)
        {
            var focused = root?.focusController?.focusedElement as VisualElement;
            return focused != null && (focused == element || element.Contains(focused));
        }
        private void RefreshFlight()
        {
            if (flightCommands == null) return;
            var session = Flight; var pawn = session?.Pawn; var state = session?.State;
            flightId = pawn != null ? pawn.AgentId : null;
            flightMode = session?.Api != null ? session.Api.CurrentMode : GoalMode.None;
            flightCommands.SetEnabled(session != null && session.CanCommand);
            foreach (var mode in modeButtons) mode.Value.EnableInClassList("is-active", mode.Key == flightMode);
            flightStatus.text = pawn == null ? "Select a drone in Scene objects." : $"{pawn.DroneID}   ·   {flightMode}\nControl: {backend.Status.CurrentAuthority}   ·   {(state != null && state.Engaged ? "Streaming selected drone" : "Choose a mode or edit a command to start")}";
            inputStatus.text = Gamepad.current == null ? "No gamepad connected." : $"Gamepad: {Gamepad.current.displayName}\nLeft: yaw/throttle (center = 50%). Right: roll/pitch.";
            if (state == null) return;
            hoverToggle.SetValueWithoutNotify(state.Hover);
            if (!Editing(hoverAltitude)) hoverAltitude.SetValueWithoutNotify(state.HoverAltitude);
            gamepadToggle.SetValueWithoutNotify(state.Gamepad);
            gamepadToggle.SetEnabled(Gamepad.current != null && (flightMode == GoalMode.Angle || flightMode == GoalMode.Rate));
            frdToggle.SetValueWithoutNotify(pawn.Sensors != null && pawn.Sensors.outputFrame == SimFrame.FRD);
            debugToggle.SetValueWithoutNotify(pawn.Controller != null && pawn.Controller.DebugVisualsEnabled);
            for (int i = 0; i < 4; i++)
            {
                var definition = session.Axis(i); var slider = commandSliders[i];
                slider.label = definition.Label; slider.lowValue = definition.Min; slider.highValue = definition.Max;
                slider.SetEnabled(WorkspaceFlightSession.Supports(flightMode) && !state.Gamepad);
                if (!Editing(slider)) slider.SetValueWithoutNotify(state.Command[i]);
            }
            bool sticksActive = flightMode == GoalMode.Angle || flightMode == GoalMode.Rate;
            leftStick.style.display = rightStick.style.display = sticksActive ? DisplayStyle.Flex : DisplayStyle.None;
            if (sticksActive)
            {
                leftStick.Set(state.Command.Z / session.Axis(2).Max, state.Command.W * 2 - 1);
                rightStick.Set(state.Command.X / session.Axis(0).Max, state.Command.Y / session.Axis(1).Max);
            }
        }
        private void BuildPidControls(VisualElement parent)
        {
            pidStatus = Text(parent);
            pidCommands = new VisualElement(); parent.Add(pidCommands);
            string[] titles = { "Position", "Velocity", "Angle", "Rate" };
            for (int group = 0; group < 4; group++)
            {
                var fold = new Foldout { text = titles[group] + " PID", value = group == 0 }; pidCommands.Add(fold);
                syncPid[group] = new Toggle(group < 2 ? "Link X / Y" : "Link roll / pitch"); fold.Add(syncPid[group]);
                for (int axis = 0; axis < 3; axis++)
                {
                    int index = group * 3 + axis;
                    var schema = WorkspacePidHistory.Schema[index];
                    pidLabels[index] = Text(fold, schema.Name);
                    var row = new VisualElement(); row.AddToClassList("pid-row"); fold.Add(row);
                    for (int term = 0; term < 3; term++)
                    {
                        int t = term, g = group;
                        var field = new FloatField(term == 0 ? "P" : term == 1 ? "I" : "D") { isDelayed = true };
                        pidFields[index, term] = field; row.Add(field);
                        field.RegisterValueChangedCallback(e =>
                        {
                            var session = Flight;
                            if (session == null || !session.Matches(pidId) || !WorkspaceBackend.IsFinite(e.newValue) || e.newValue < 0) { RefreshPid(); return; }
                            var ctrl = session.Pawn.Controller;
                            if (ctrl == null || !ctrl.SetPIDGainTerm(schema.Stage, schema.Axis, t, e.newValue, false)) { RefreshPid(); return; }
                            if (syncPid[g].value && schema.Axis < 2) ctrl.SetPIDGainTerm(schema.Stage, (byte)(1 - schema.Axis), t, e.newValue, false);
                            RefreshPid();
                        });
                    }
                }
            }
            AddButton(pidCommands, "Reset integrators", () => Flight?.ResetDrone(pidId, "controller"));
            AddButton(parent, "Save gains to CSV", () => PidOperation(() => { if (Flight?.Pawn?.Controller == null) throw new InvalidOperationException("Select a drone with a controller."); WorkspacePidHistory.Save(PidPath, Flight.Pawn.Controller); LoadHistory(); pidStatus.text = "Gains saved to " + PidPath; }));
            Section(parent, "PID history · most recent 200 rows");
            historyChoice = new DropdownField(); parent.Add(historyChoice);
            historyChoice.RegisterValueChangedCallback(_ => ShowHistoryRow());
            AddButton(parent, "Refresh history", LoadHistory);
            applyHistory = AddButton(parent, "Apply selected history row", () => PidOperation(() =>
            {
                if (Flight == null || !Flight.Matches(pidId) || historyChoice.index < 0 || historyChoice.index >= historyRows.Count) return;
                if (!WorkspacePidHistory.Apply(Flight.Pawn.Controller, historyRows[historyChoice.index])) throw new InvalidDataException("Invalid or unsupported PID row; gains not applied.");
                Flight.Api.ResetController(); RefreshPid(); pidStatus.text = "History applied; integrators reset.";
            }));
            historyDetail = Text(parent);
            LoadHistory();
        }
        private void PidOperation(Action action) { try { action(); } catch (Exception e) { pidStatus.text = e.Message; Debug.LogWarning("[Workspace PID] " + e.Message); } }
        private void LoadHistory() => PidOperation(() =>
        {
            historyRows = WorkspacePidHistory.Read(PidPath);
            var choices = new List<string>();
            for (int i = 0; i < historyRows.Count; i++) choices.Add($"{i + 1}: {historyRows[i].Split(',')[0]}");
            historyChoice.choices = choices; historyChoice.SetValueWithoutNotify(choices.Count > 0 ? choices[choices.Count - 1] : ""); ShowHistoryRow();
        });
        private void ShowHistoryRow()
        {
            if (historyDetail == null) return;
            int index = historyChoice.index;
            if (index < 0 || index >= historyRows.Count) { historyDetail.text = "No saved PID history.\n" + PidPath; return; }
            if (!WorkspacePidHistory.TryParse(historyRows[index], out var values)) { historyDetail.text = "Invalid CSV row. It cannot be applied."; return; }
            var text = new System.Text.StringBuilder("Stage   P / I / D\n");
            for (int i = 0; i < 12; i++) text.AppendLine($"{WorkspacePidHistory.Schema[i].Name}   {values[i * 3]:G5} / {values[i * 3 + 1]:G5} / {values[i * 3 + 2]:G5}");
            historyDetail.text = text.ToString();
        }
        private void RefreshPid()
        {
            if (pidCommands == null) return;
            var session = Flight; pidId = session?.Pawn != null ? session.Pawn.AgentId : null;
            var ctrl = session?.Pawn != null ? session.Pawn.Controller : null;
            pidCommands.SetEnabled(session != null && session.CanCommand && ctrl != null);
            applyHistory.SetEnabled(session != null && session.CanCommand && ctrl != null && historyChoice.index >= 0);
            for (int i = 0; i < 12; i++)
            {
                var s = WorkspacePidHistory.Schema[i]; float p = 0, integral = 0, d = 0;
                bool available = ctrl != null && ctrl.GetPIDGains(s.Stage, s.Axis, out p, out integral, out d);
                pidLabels[i].text = s.Name + (!available ? " · unavailable" : ctrl.IsPIDStageActive(s.Stage, s.Axis) ? " · active" : " · inactive in this mode");
                for (int t = 0; t < 3; t++)
                {
                    var f = pidFields[i, t]; f.SetEnabled(available);
                    if (!Editing(f)) f.SetValueWithoutNotify(t == 0 ? p : t == 1 ? integral : d);
                }
            }
        }
        private void BuildMotorBars(VisualElement parent)
        {
            var row = new VisualElement(); row.AddToClassList("motor-meters"); parent.Add(row);
            string[] names = { "FL", "FR", "BL", "BR" };
            for (int i = 0; i < 4; i++)
            {
                var column = new VisualElement(); column.AddToClassList("motor-column"); row.Add(column);
                column.Add(new Label(names[i]));
                var track = new VisualElement(); track.AddToClassList("motor-track"); column.Add(track);
                motors[i] = new VisualElement(); motors[i].AddToClassList("motor-fill"); track.Add(motors[i]);
                motorLabels[i] = new Label("0%\n0.00 N"); column.Add(motorLabels[i]);
            }
            row.tooltip = "Motor output: bars smoothed over 80 ms wall time; numeric values are current samples.";
        }
        private void RefreshMotors()
        {
            var pawn = Flight?.Pawn; var ctrl = pawn != null ? pawn.Controller : null;
            bool changed = pawn != motorPawn; motorPawn = pawn;
            float blend = 1 - Mathf.Exp(-Time.unscaledDeltaTime / 0.08f);
            for (int i = 0; i < 4; i++)
            {
                float value = ctrl != null && i < ctrl.LastMotor01.Count ? ctrl.LastMotor01[i] : 0;
                value = WorkspaceBackend.IsFinite(value) ? Mathf.Clamp01(value) : 0;
                motorLevels[i] = changed ? value : Mathf.Lerp(motorLevels[i], value, blend);
                motors[i].style.height = Length.Percent(motorLevels[i] * 100);
            }
        }
        private void RefreshMotorLabels()
        {
            var ctrl = Flight?.Pawn != null ? Flight.Pawn.Controller : null;
            for (int i = 0; i < 4; i++)
            {
                float value = ctrl != null && i < ctrl.LastMotor01.Count ? ctrl.LastMotor01[i] : 0;
                float thrust = ctrl != null && i < ctrl.Thrusts.Count ? ctrl.Thrusts[i] : 0;
                motorLabels[i].text = $"{Mathf.Clamp01(value) * 100:F0}%\n{thrust:F2} N";
            }
        }
    }

    internal sealed class WorkspaceStick : VisualElement
    {
        private readonly VisualElement dot;
        public WorkspaceStick(string title)
        {
            style.width = 120; style.height = 120; style.marginRight = 10;
            Add(new Label(title));
            var box = new VisualElement(); box.style.position = Position.Absolute; box.style.top = 20; box.style.width = 100; box.style.height = 100;
            box.AddToClassList("stick-box"); Add(box);
            dot = new VisualElement(); dot.AddToClassList("stick-dot"); box.Add(dot);
        }
        public void Set(float x, float y) { dot.style.left = 47 + Mathf.Clamp(x, -1, 1) * 47; dot.style.top = 47 - Mathf.Clamp(y, -1, 1) * 47; }
    }
}
