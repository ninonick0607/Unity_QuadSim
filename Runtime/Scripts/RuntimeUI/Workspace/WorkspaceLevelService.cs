using DroneCore.Core;
using Experimental;
using SimCore;
using SimCore.Common;
using SimCore.Rpc;
using UnityEngine;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed class WorkspaceLevelService
    {
        public readonly SimCoreApi Sim;
        public readonly DroneManager Drones;
        public readonly ExternalRpcAdapter Rpc;
        public WorkspaceLevelService(SimCoreApi sim, DroneManager drones)
        {
            Sim = sim; Drones = drones; Rpc = Object.FindAnyObjectByType<ExternalRpcAdapter>();
        }
        public bool CanEdit => Sim != null && Sim.IsInitialized && Sim.IsPaused && Sim.CurrentAuthority == InputSource.UI;
        public QuadSim.RL.ScenarioObjectManager Objects => QuadSim.RL.ScenarioObjectManager.GetOrCreate(Sim.gameObject);
        public bool AddObject(string shape, Vector3 positionEnu, Vector3 sizeEnu, bool collision)
        {
            if (!CanEdit) return false;
            return Objects.AddObject(new QuadSim.RL.ScenarioObjectSpec("hud_" + System.Guid.NewGuid().ToString("N").Substring(0, 8),
                shape, positionEnu, 0, sizeEnu, collision, new Color(0.35f, 0.6f, 0.85f)));
        }
        public bool RemoveObject(string id) => CanEdit && Objects.RemoveObject(id);
        public bool Apply(int seed, bool noise, double lat, double lon, double alt, Vector3 spawnOrigin)
        {
            if (!CanEdit || !ValidOrigin(lat, lon, alt) || !WorkspaceBackend.IsFinite(spawnOrigin.x) || !WorkspaceBackend.IsFinite(spawnOrigin.y) || !WorkspaceBackend.IsFinite(spawnOrigin.z)) return false;
            Sim.Geo.SetOrigin(lat, lon, alt);
            Drones.SetSensorEnvironment(seed, noise, lat, lon, alt);
            Drones.SpawnOrigin = spawnOrigin;
            foreach (var pawn in Drones.RegisteredDrones)
            {
                var sensors = pawn != null ? pawn.Sensors : null;
                if (sensors == null) continue;
                sensors.Configure(seed, noise, lat, lon, alt, sensors.outputFrame);
                sensors.ReseedForEpisode(seed, pawn.DroneIndex);
                if (sensors.IMU != null) sensors.OnReset();
            }
            return true;
        }
        public static bool ValidOrigin(double lat, double lon, double alt) =>
            !double.IsNaN(lat) && !double.IsNaN(lon) && !double.IsNaN(alt) && !double.IsInfinity(alt) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
        public bool ApplyWind(WindModule wind, bool on, float speed, float yaw)
        {
            if (!CanEdit || wind == null || !WorkspaceBackend.IsFinite(speed) || speed < 0 || !WorkspaceBackend.IsFinite(yaw)) return false;
            wind.SetEnabled(on); wind.SetWindSpeed(speed);
            var rotation = wind.transform.eulerAngles; rotation.y = yaw; wind.transform.eulerAngles = rotation;
            return true;
        }
        public void SetRunMode(SimulationManager.RunMode mode)
        {
            if (CanEdit) Sim.SetRunMode(mode);
        }
    }
}
