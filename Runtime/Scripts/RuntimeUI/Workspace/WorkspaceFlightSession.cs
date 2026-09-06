using System;
using System.Collections.Generic;
using DroneCore;
using DroneCore.Api;
using DroneCore.Core;
using MathUtil;
using SimCore;
using SimCore.Common;
using UnityEngine;
using UnityEngine.InputSystem;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>One selected-drone command session; no visual elements or per-step allocations.</summary>
    public sealed class WorkspaceFlightSession : ISimulatable, IDisposable
    {
        public sealed class CommandState
        {
            public GoalMode Mode;
            public Axis4 Command;
            public bool Engaged, Hover, Gamepad;
            public float HoverAltitude = 3;
        }
        private readonly SimCoreApi sim;
        private readonly DroneManager drones;
        private readonly Dictionary<QuadPawn, CommandState> states = new Dictionary<QuadPawn, CommandState>();
        private readonly List<QuadPawn> removed = new List<QuadPawn>();
        private QuadPawn cachedPawn;
        private QuadSimApi cachedApi;
        public QuadPawn Pawn => drones != null ? drones.SelectedDrone : null;
        public QuadSimApi Api
        {
            get
            {
                var pawn = Pawn;
                if (pawn != cachedPawn) { cachedPawn = pawn; cachedApi = pawn != null ? pawn.GetComponent<QuadSimApi>() : null; }
                return cachedApi;
            }
        }
        public bool CanCommand => sim != null && sim.IsInitialized && sim.CurrentAuthority == InputSource.UI && Api != null && Api.IsInitialized;
        public CommandState State
        {
            get
            {
                var pawn = Pawn;
                if (pawn == null || Api == null) return null;
                if (!states.TryGetValue(pawn, out var state))
                {
                    state = new CommandState { Mode = Api.CurrentMode };
                    states.Add(pawn, state);
                }
                if (state.Mode != Api.CurrentMode) { state.Mode = Api.CurrentMode; state.Engaged = state.Hover = state.Gamepad = false; state.Command = default; }
                return state;
            }
        }
        public WorkspaceFlightSession(SimCoreApi sim, DroneManager drones)
        {
            this.sim = sim; this.drones = drones;
            sim.OnAuthorityChanged += AuthorityChanged;
            sim.SimManager.RegisterManagers(this);
        }
        private void AuthorityChanged(InputSource previous, InputSource current) => Clear();
        public void Clear() => states.Clear();
        public void Forget(QuadPawn pawn) { if (pawn != null) states.Remove(pawn); }
        public void Dispose()
        {
            if (CanCommand && State != null && State.Engaged) Api.CommandProxy?.ResetToSafe(Api.CurrentMode);
            if (sim != null) { sim.OnAuthorityChanged -= AuthorityChanged; if (sim.SimManager != null) sim.SimManager.UnRegisterManager(this); }
            Clear();
        }
        public int ExecutionOrder => 5; // Before DroneManager (10) consumes the command.
        public string DebugName => "Workspace flight commands";
        public void OnSimulationStart(SimulationManager manager) { }
        public void OnSimulationReset(SimulationManager manager) => Clear();
        public void PostPhysicsStep(double dtSec, long nowNanos) { }
        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            if (!CanCommand) return;
            var state = State;
            if (state == null || !state.Engaged || !Supports(state.Mode)) return;
            // State is sampled from devices in Update; only renewal occurs at simulation rate.
            Api.SendCommand(InputSource.UI, state.Command);
        }
        public static bool Supports(GoalMode mode) => mode == GoalMode.Position || mode == GoalMode.Velocity || mode == GoalMode.Angle || mode == GoalMode.Rate;
        public static Axis4 InitialCommand(GoalMode mode, Vector3 positionEnu, float yaw, float hoverThrottle)
        {
            if (mode == GoalMode.Position) return new Axis4(positionEnu.x, positionEnu.y, yaw, positionEnu.z);
            return new Axis4(0, 0, 0, mode == GoalMode.Angle || mode == GoalMode.Rate ? hoverThrottle : 0);
        }
        public float HoverThrottle => Pawn != null && Pawn.HasConfigApplied && Pawn.Rigidbody != null && Pawn.Config.FlightParams.MaxThrust > 0
            ? Mathf.Clamp01(Pawn.Rigidbody.mass * 9.81f / Pawn.Config.FlightParams.MaxThrust) : 0.4f;
        public bool EnterMode(string expectedId, GoalMode mode)
        {
            if (!Matches(expectedId) || !Supports(mode)) return false;
            if (Api.CurrentMode != mode && !Api.SetMode(mode)) return false;
            var state = State;
            var sensors = Pawn.Sensors != null ? Pawn.Sensors.Latest : default;
            state.Command = InitialCommand(mode, sensors.GpsPosition, sensors.ImuAttitude.z, HoverThrottle);
            state.Mode = mode; state.Engaged = true; state.Hover = false; state.Gamepad = false;
            return true;
        }
        public bool Matches(string id) => CanCommand && Pawn != null && Pawn.AgentId == id;
        public (string Label, float Min, float Max) Axis(int index)
        {
            var mode = Api != null ? Api.CurrentMode : GoalMode.None;
            var f = Pawn != null && Pawn.HasConfigApplied ? Pawn.Config.FlightParams : default;
            float xy = f.MaxVelXY > 0 ? f.MaxVelXY : 5, z = f.MaxVelZ > 0 ? f.MaxVelZ : 3;
            float tilt = f.MaxAngle > 0 ? f.MaxAngle : 30, rate = f.MaxRateRollPitch > 0 ? f.MaxRateRollPitch : 180;
            float yaw = f.MaxRateYaw > 0 ? f.MaxRateYaw : 90;
            if (mode == GoalMode.Position)
                return index == 0 ? ("X (m)", -50, 50) : index == 1 ? ("Y (m)", -50, 50) : index == 2 ? ("Yaw (deg)", -180, 180) : ("Altitude (m)", 0, 50);
            if (index == 2) return ("Yaw rate (deg/s)", -yaw, yaw);
            if (mode == GoalMode.Velocity) return index == 0 ? ("Velocity X (m/s)", -xy, xy) : index == 1 ? ("Velocity Y (m/s)", -xy, xy) : ("Velocity Z (m/s)", -z, z);
            if (index == 3) return ("Throttle (0–1)", 0, 1);
            return mode == GoalMode.Rate ? (index == 0 ? "Roll rate (deg/s)" : "Pitch rate (deg/s)", -rate, rate)
                : (index == 0 ? "Roll (deg)" : "Pitch (deg)", -tilt, tilt);
        }
        public bool SetAxis(string id, GoalMode expectedMode, int axis, float value)
        {
            if (!Matches(id) || Api.CurrentMode != expectedMode || !Supports(expectedMode) || axis < 0 || axis > 3 || !WorkspaceBackend.IsFinite(value)) return false;
            var state = State;
            if (!state.Engaged) EnterMode(id, expectedMode);
            var limits = Axis(axis);
            state.Command[axis] = Mathf.Clamp(value, limits.Min, limits.Max); state.Engaged = true; state.Gamepad = false;
            if (axis != 3) state.Hover = false;
            return true;
        }
        public bool Hover(string id, bool active, float altitude)
        {
            if (!Matches(id) || !WorkspaceBackend.IsFinite(altitude)) return false;
            var state = State;
            if (active && !state.Hover && !EnterMode(id, GoalMode.Position)) return false;
            state.Hover = active; state.HoverAltitude = Mathf.Clamp(altitude, 0.5f, 50);
            if (active) { state.Command.W = state.HoverAltitude; state.Engaged = true; }
            return true;
        }
        public bool SetGamepad(string id, bool enabled)
        {
            if (!Matches(id)) return false;
            if (enabled && (Gamepad.current == null || (Api.CurrentMode != GoalMode.Angle && Api.CurrentMode != GoalMode.Rate))) return false;
            if (enabled && !State.Engaged) EnterMode(id, Api.CurrentMode);
            State.Gamepad = enabled;
            if (!enabled) State.Command = InitialCommand(State.Mode, default, 0, HoverThrottle);
            return true;
        }
        public void SampleInput()
        {
            if (!CanCommand || State == null || !State.Gamepad) return;
            var pad = Gamepad.current;
            if (pad == null || !Application.isFocused) { SetGamepad(Pawn.AgentId, false); return; }
            Vector2 left = pad.leftStick.ReadValue(), right = pad.rightStick.ReadValue();
            State.Command = new Axis4(right.x * Axis(0).Max, right.y * Axis(1).Max, left.x * Axis(2).Max, Mathf.Clamp01((left.y + 1) * 0.5f));
        }
        public void PruneDestroyed()
        {
            removed.Clear(); foreach (var pair in states) if (pair.Key == null) removed.Add(pair.Key);
            foreach (var pawn in removed) states.Remove(pawn);
        }
        public bool SetFrame(string id, bool frd)
        {
            if (!Matches(id) || !Api.SetSensorOutputFrame(frd ? SimFrame.FRD : SimFrame.FLU)) return false;
            Pawn.Controller?.RebuildAllocator(); Api.ResetController(); Forget(Pawn); return true;
        }
        public bool SetDebug(string id, bool enabled)
        {
            if (!Matches(id) || Pawn.Controller == null) return false;
            Pawn.Controller.DebugVisualsEnabled = enabled; return true;
        }
        public bool ResetDrone(string id, string action)
        {
            if (!Matches(id)) return false;
            if (action == "high") Api.ResetPose(drones.SpawnOrigin + Vector3.up * 5, Quaternion.identity);
            else if (action == "low") Api.ResetPose(drones.SpawnOrigin, Quaternion.identity);
            else if (action == "rotation") Api.ResetRotation();
            else if (action == "controller") Api.ResetController();
            else return false;
            Forget(Pawn); return true;
        }
    }
}
