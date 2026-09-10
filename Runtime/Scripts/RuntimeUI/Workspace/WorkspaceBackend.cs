using System;
using System.Collections.Generic;
using DroneCore;
using DroneCore.Api;
using DroneCore.Core;
using SimCore;
using SimCore.Common;
using UnityEngine;

namespace QuadSim.RuntimeUI.Workspace
{
    public readonly struct WorkspaceEntity
    {
        public readonly string Id, Name;
        public WorkspaceEntity(string id, string name) { Id = id; Name = name; }
    }

    /// <summary>Main-thread boundary. Views never own simulation time or physics state.</summary>
    public interface IWorkspaceBackend : IDisposable
    {
        bool Ready { get; }
        bool CanEditPose { get; }
        SimStatus Status { get; }
        string SelectedId { get; }
        WorkspaceFlightSession Flight { get; }
        WorkspaceLevelService Level { get; }
        WorkspaceSceneService Scene { get; }
        string SelectedConfig { get; }
        bool ApprovalPending { get; }
        void SetConfig(string name);
        bool ReloadSelected(string expectedId);
        void SaveDefaultConfig();
        void FollowSelected();
        event Action ExternalApprovalRequested;
        void Connect();
        void ReadEntities(List<WorkspaceEntity> destination);
        List<string> ReadConfigs();
        void Select(string id);
        void TogglePause();
        void Step();
        void Reset();
        void SetSpeed(float speed);
        void Spawn(string config);
        string ReadInspector();
        string ReadTelemetry();
        void ApproveExternal(bool accept);
        void SwitchToUI();
    }

    public sealed class WorkspaceBackend : IWorkspaceBackend
    {
        private SimCoreApi sim;
        private DroneManager drones;
        public WorkspaceFlightSession Flight { get; private set; }
        public WorkspaceLevelService Level { get; private set; }
        public WorkspaceSceneService Scene { get; private set; }
        public string SelectedConfig => drones != null ? drones.SelectedConfigName : "";
        public bool ApprovalPending => sim != null && sim.AuthorityManager != null && sim.AuthorityManager.IsExternalPendingApproval;
        public bool Ready => sim != null && sim.IsInitialized && drones != null;
        public bool CanEditPose => Ready && sim.IsPaused && sim.CurrentAuthority == InputSource.UI;
        public SimStatus Status => Ready ? sim.GetStatus() : default;
        public string SelectedId => Selected != null ? Selected.AgentId : null;
        private QuadPawn Selected => drones != null ? drones.SelectedDrone : null;
        public event Action ExternalApprovalRequested;

        // Called only at the presentation cadence, including when waiting for startup.
        public void Connect()
        {
            if (sim == null)
            {
                sim = SimCoreApi.Get();
                if (sim != null) sim.OnExternalConnectionRequest += RequestApproval;
            }
            if (drones == null) drones = DroneManager.Get();
            if (Ready && Flight == null) { Flight = new WorkspaceFlightSession(sim, drones); Level = new WorkspaceLevelService(sim, drones); Scene = new WorkspaceSceneService(Level, Flight); }
        }
        private void RequestApproval() => ExternalApprovalRequested?.Invoke();
        public void Dispose()
        {
            Flight?.Dispose(); Flight = null;
            if (sim != null) sim.OnExternalConnectionRequest -= RequestApproval;
        }
        public void ReadEntities(List<WorkspaceEntity> destination)
        {
            destination.Clear();
            if (drones == null) return;
            var registered = drones.RegisteredDrones;
            for (int i = 0; i < registered.Count; i++)
                if (registered[i] != null)
                    destination.Add(new WorkspaceEntity(registered[i].AgentId, registered[i].DroneID));
        }
        public List<string> ReadConfigs() => drones != null ? drones.GetAvailableConfigs() : new List<string>();
        public void Select(string id)
        {
            if (!Ready) return;
            var pawn = drones.GetDroneByAgentId(id);
            if (pawn != null) { drones.SelectDroneByIndex(drones.GetDroneIndex(pawn), false); Scene?.Select(pawn.transform); }
        }
        public void TogglePause() { if (Ready) sim.TogglePause(); }
        public void Step() { if (Ready && (sim.IsPaused || sim.RunMode == SimulationManager.RunMode.Manual)) sim.StepOnce(); }
        public void Reset() { if (Ready) { sim.SetTimeScale(1); sim.Resume(); sim.ResetSimulation(); } }
        public void SetSpeed(float speed)
        {
            if (Ready && IsFinite(speed) && speed >= 0.05f && speed <= 100f) sim.SetTimeScale(speed);
        }
        public void Spawn(string config)
        {
            if (Ready && !string.IsNullOrEmpty(config))
            {
                drones.SetSelectedConfig(config);
                var pawn = drones.SpawnDroneWithConfig(config, drones.SpawnOrigin, Quaternion.identity);
                if (pawn != null) Select(pawn.AgentId);
            }
        }
        public void SetConfig(string name) { if (Ready && drones.GetAvailableConfigs().Contains(name)) drones.SetSelectedConfig(name); }
        public void SaveDefaultConfig() { if (Ready) drones.SaveCurrentConfigAsDefault(); }
        public void FollowSelected()
        {
            var camera = DroneCore.CameraSystem.CameraDirector.Get();
            if (Selected == null || camera == null) return;
            camera.Bind(Selected); camera.SetMode(DroneCore.CameraSystem.CameraViewMode.Follow);
        }
        public bool ReloadSelected(string expectedId)
        {
            if (!CanEditPose || SelectedId != expectedId || Selected == null) return false;
            var old = Selected;
            var replacement = drones.SpawnDroneWithConfig(drones.SelectedConfigName, old.transform.position, old.transform.rotation);
            if (replacement == null) return false;
            replacement.Api?.ResetPose(old.transform.position, old.transform.rotation);
            Flight?.Forget(old);
            drones.UnregisterDrone(old);
            Select(replacement.AgentId);
            var camera = DroneCore.CameraSystem.CameraDirector.Get();
            if (camera != null && camera.TargetDrone == old) camera.Bind(replacement);
            UnityEngine.Object.Destroy(old.gameObject);
            return true;
        }
        public static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        public string ReadInspector()
        {
            var pawn = Selected;
            if (pawn == null) return "Select a drone in the Scene panel.";
            var api = pawn.GetComponent<QuadSimApi>();
            string info = $"{pawn.DroneID}\nAgent ID   {pawn.AgentId}\nMode   {(api != null ? api.CurrentMode.ToString() : "Unavailable")}";
            if (!pawn.HasConfigApplied) return info + "\nNo applied configuration.";
            var c = pawn.Config;
            return info + $"\n\nAPPLIED CONFIGURATION\nModel   {c.ModelParams.Name}\nMass   {c.DroneParams.Mass:F3} kg\nArm   {c.DroneParams.Arm_Length:F3} m\nInertia   {c.DroneParams.Inertia}\n\nFLIGHT LIMITS\nXY / Z speed   {c.FlightParams.MaxVelXY:F2} / {c.FlightParams.MaxVelZ:F2} m/s\nTilt   {c.FlightParams.MaxAngle:F1} deg\nRoll/pitch rate   {c.FlightParams.MaxRateRollPitch:F1} deg/s\nYaw rate   {c.FlightParams.MaxRateYaw:F1} deg/s\nThrust   {c.FlightParams.MaxThrust:F2} N\nPID output   {c.FlightParams.MaxPID:F2}\nAcceptance   {c.FlightParams.AcceptableDistance:F2} m";
        }
        public string ReadTelemetry()
        {
            var pawn = Selected;
            if (pawn == null || pawn.Sensors == null) return "No selected drone sensor data.";
            var s = pawn.Sensors.Latest;
            string text = $"INERTIAL MEASUREMENT\nDrone   {pawn.DroneID}\nFrame   {pawn.Sensors.outputFrame}\nIMU   valid: {s.ImuValid}   t: {s.ImuTimestampSec:F3} s\nAttitude   {s.ImuAttitude:F2} deg\nRates   {s.ImuAngVel * Mathf.Rad2Deg:F2} deg/s\nBody velocity   {s.ImuVel:F2} m/s\nSpecific force   {s.ImuAccel:F2} m/s²\n\nGPS / WORLD ENU\nPosition   {s.GpsPosition:F2} m\nLat / lon   {s.GpsLatDeg:F5} / {s.GpsLonDeg:F5}\nAltitude   {s.GpsAltM:F2} m\n\nMAGNETOMETER\nField   {s.MagField:F3} Gauss\nHeading / declination   {s.MagHeadingDeg:F1} / {s.MagDeclinationDeg:F1} deg\n\nBAROMETER\nPressure   {s.BaroPressureHPa:F2} hPa\nTemperature   {s.BaroTemperatureC:F1} °C\nMSL altitude   {s.BaroAltitudeMSL:F2} m";
            if (pawn.Controller != null)
            {
                var c = pawn.Controller;
                var t = c.TelemetrySnapshot;
                text += $"\n\nCONTROLLER TARGETS\nAngles   {t.DesiredAnglesDeg:F2} deg\nRates   {t.DesiredRatesDeg:F2} deg/s\nVelocity   {t.DesiredVel:F2} m/s";
            }
            return text;
        }
        public void ApproveExternal(bool accept)
        {
            if (!Ready) return;
            if (accept) sim.AcceptExternalConnection(); else sim.DenyExternalConnection();
        }
        public void SwitchToUI() { if (Ready) sim.SwitchToUI(); }
    }
}
