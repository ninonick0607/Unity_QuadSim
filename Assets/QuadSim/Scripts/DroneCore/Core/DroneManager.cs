    using System.Collections.Generic;
    using DroneCore.CameraSystem;
using Experimental;
using MathUtil;
using SimCore;
using SimCore.Common;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Core
{
    public enum StartupSpawnPolicy
    {
        SpawnDefaultIfNone = 0,
        SceneDronesOnly = 1,
    }

    /// <summary>
    /// DroneManager is the central registry and orchestrator for all drones in the simulation.
    /// It handles spawning, selection, configuration, and drives the physics step pipeline.
    ///
    /// Implements ISimulatable so SimulationManager drives the loop:
    ///   PrePhysicsStep  → iterate QuadPawns → pawn.PhysicsStep(dt, now)
    ///   PostPhysicsStep → (reserved for future use)
    ///   Update()        → visual updates (prop spin, effects)
    /// </summary>
    ///
    [DefaultExecutionOrder(-500)]
    public sealed class DroneManager : MonoBehaviour, ISimulatable
    {
        // ============================================================================
        // Serialized Fields
        // ============================================================================

        [Header("Spawning")]
        [Tooltip("Choose whether simulation startup may create a default drone when the scene contains none.")]
        [SerializeField] private StartupSpawnPolicy startupSpawnPolicy = StartupSpawnPolicy.SpawnDefaultIfNone;

        [Tooltip("Prefab to instantiate when spawning drones.")]
        [SerializeField] private QuadPawn quadPawnPrefab;

        [Tooltip("Default spawn location. Set in inspector or override via SpawnOrigin property.")]
        [SerializeField] private Vector3 defaultSpawnOrigin = new Vector3(0f, 1f, 0f);

        [Tooltip("Offset between multiple spawned drones.")]
        [SerializeField] private float spawnOffset = 1.5f;

        [Header("Configuration")]
        [Tooltip("Default config name to load.")]
        [SerializeField] private string selectedConfigName = "StandardDrone_Config";

        [Header("Core References")]
        [Tooltip("SimulationManager reference. Set in inspector, fallback to scene find.")]
        [SerializeField] private SimulationManager simManager;

        [Header("Swarm")]
        [Tooltip("Enable swarm mode (all drones receive same commands).")]
        [SerializeField] private bool swarmMode = false;
        
        [Header("Payload")]
        [SerializeField] private PayloadAttachment payloadPrefab;
        [SerializeField] private bool spawnPayload = false;
        [SerializeField] private string payloadMountPath = "Bottle";
        
        [Header("Scene Env")]
        [SerializeField] private int     masterSeed       = 12345;
        [SerializeField] private bool    enableSensorNoise = false;
        [SerializeField] private double  originLat        = 30.4636;
        [SerializeField] private double  originLon        = -86.5533;
        [SerializeField] private double  originAlt        = 0.0;
        [SerializeField] private SimFrame frame = SimFrame.FLU;
        [SerializeField] private GoalMode startingMode = GoalMode.Rate;
        // ============================================================================
        // Runtime State
        // ============================================================================

        private readonly List<QuadPawn> _registeredDrones = new List<QuadPawn>();
        private int _selectedDroneIndex = 0;
        private DroneConfig _currentConfig;
        private bool _hasCurrentConfig;

        // ============================================================================
        // ISimulatable Implementation
        // ============================================================================

        public int ExecutionOrder => 10;
        public string DebugName => "DroneManager";

        // ============================================================================
        // Public Accessors
        // ============================================================================

        public IReadOnlyList<QuadPawn> RegisteredDrones => _registeredDrones;
        public int DroneCount => _registeredDrones.Count;
        public int SelectedDroneIndex => _selectedDroneIndex;
        public string SelectedConfigName => selectedConfigName;
        public bool SwarmMode { get => swarmMode; set => swarmMode = value; }
        public Vector3 SpawnOrigin { get => defaultSpawnOrigin; set => defaultSpawnOrigin = value; }
        public StartupSpawnPolicy SpawnPolicy => startupSpawnPolicy;

        /// <summary>
        /// Currently selected drone, or null.
        /// </summary>
        public QuadPawn SelectedDrone
        {
            get
            {
                if (_selectedDroneIndex >= 0 && _selectedDroneIndex < _registeredDrones.Count)
                    return _registeredDrones[_selectedDroneIndex];
                return null;
            }
        }

        // ============================================================================
        // Static Accessor
        // ============================================================================

        private static DroneManager _instance;

        /// <summary>
        /// Get cached DroneManager instance. Falls back to scene find with warning.
        /// </summary>
        public static DroneManager Get()
        {
            if (_instance != null) return _instance;

            _instance = FindFirstObjectByType<DroneManager>();
            if (_instance != null)
                Debug.LogWarning("[DroneManager] Instance resolved via FindFirstObjectByType. Prefer direct reference.");

            return _instance;
        }

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            _instance = this;
    
            string savedConfig = PlayerPrefs.GetString("QuadSim.DefaultConfig", "");
            if (!string.IsNullOrEmpty(savedConfig))
                selectedConfigName = savedConfig;

            Debug.Log($"[DroneManager] Awake: selectedConfig={selectedConfigName}");

            // Register with SimManager during Awake so we're in the list 
            // before SimulationManager.Start() fires OnSimulationStart
            if (simManager == null)
            {
                simManager = FindFirstObjectByType<SimulationManager>();
                if (simManager != null)
                    Debug.LogWarning("[DroneManager] SimulationManager resolved via find. Prefer inspector assignment.");
            }
            if (simManager != null)
            {
                simManager.RegisterManagers(this);
                Debug.Log("[DroneManager] Registered with SimulationManager");
            }
        }

        private void Start()
        {
        }

        private void OnDestroy()
        {
            if (simManager != null)
            {
                simManager.UnRegisterManager(this);
            }

            if (_instance == this)
            {
                _instance = null;
            }
        }

        // ============================================================================
        // ISimulatable — Simulation Lifecycle
        // ============================================================================

        public void OnSimulationStart(SimulationManager sim)
        {
            simManager = sim;

            _hasCurrentConfig = DroneConfigLoader.LoadConfig(selectedConfigName, out _currentConfig);
            if (_hasCurrentConfig)
                Debug.Log($"[DroneManager] Loaded config: {selectedConfigName}");
            else
                Debug.LogWarning($"[DroneManager] Failed to load config: {selectedConfigName}");

            if (_registeredDrones.Count == 0 && startupSpawnPolicy == StartupSpawnPolicy.SpawnDefaultIfNone)
            {
                Debug.Log("[DroneManager] Startup policy is SpawnDefaultIfNone; spawning default drone.");
                SpawnDroneWithConfig(selectedConfigName, defaultSpawnOrigin, Quaternion.identity);
            }
            else if (_registeredDrones.Count == 0)
            {
                Debug.Log("[DroneManager] Startup policy is SceneDronesOnly; starting without a drone.");
            }
            else
            {
                foreach (var d in _registeredDrones)
                {
                    if (d == null) continue;

                    if (d.Sensors != null)
                        d.Sensors.Configure(masterSeed, enableSensorNoise, originLat, originLon, originAlt, frame);

                    if (_hasCurrentConfig && !d.HasConfigApplied)
                        d.ApplyConfig(_currentConfig);

                    if (d.Api != null && d.Api.IsInitialized)
                        d.Api.SetMode(startingMode);
                }
            }
        }

        public void OnSimulationReset(SimulationManager sim)
        {
            Debug.Log("[DroneManager] OnSimulationReset");
            ResetAllDrones();
        }

        /// <summary>
        /// Drive the physics step for all drones.
        /// SimulationManager calls this BEFORE Physics.Simulate().
        /// Each QuadPawn runs: Sensors → Controller → Apply Wrench.
        /// </summary>
        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            CleanupInvalidDrones();

            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null)
                {
                    drone.PhysicsStep(dtSec, nowNanos);
                }
            }
        }

        public void PostPhysicsStep(double dtSec, long nowNanos)
        {
            // Reserved for future use (logging, telemetry publish, etc.)
        }

        // ============================================================================
        // Visual Updates — Called by Unity Update (rendering frame)
        // ============================================================================

        private void Update()
        {
            float dt = Time.deltaTime;
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null)
                {
                    drone.UpdateVisuals(dt);
                }
            }
        }

        // ============================================================================
        // Spawning
        // ============================================================================

        /// <summary>
        /// Spawn a drone using the currently selected config.
        /// </summary>
        public QuadPawn SpawnDrone(Vector3 position, Quaternion rotation)
        {
            return SpawnDroneWithConfig(selectedConfigName, position, rotation);
        }

        /// <summary>
        /// Spawn a drone with a specific config name.
        /// </summary>
        public QuadPawn SpawnDroneWithConfig(string configName, Vector3 position, Quaternion rotation)
        {
            if (quadPawnPrefab == null)
            {
                Debug.LogError("[DroneManager] Cannot spawn: QuadPawnPrefab not set!");
                return null;
            }

            // Load config
            if (!DroneConfigLoader.LoadConfig(configName, out DroneConfig config))
            {
                Debug.LogError($"[DroneManager] Failed to load config: {configName}");
                return null;
            }

            // Determine spawn position
            Vector3 spawnPos = DetermineSpawnPosition(position);
            Quaternion spawnRot = rotation == Quaternion.identity ? DetermineSpawnRotation() : rotation;

            int droneNumber = _registeredDrones.Count;

            // Instantiate
            QuadPawn newDrone = Instantiate(quadPawnPrefab, spawnPos, spawnRot);
            if (newDrone == null)
            {
                Debug.LogError("[DroneManager] Failed to instantiate QuadPawn prefab");
                return null;
            }
            
            newDrone.name = $"Drone_{droneNumber:D2}";
            
            if (newDrone.Sensors != null)
                newDrone.Sensors.Configure(masterSeed, enableSensorNoise, originLat, originLon, originAlt,frame);
            
            // Inject manager reference BEFORE Start() runs
            newDrone.SetDroneManager(this);

            // Apply config (triggers QuadPawn initialization)
            newDrone.ApplyConfig(config);

            // The scene config owns the initial cascade entry point. Apply it
            // to the instantiated pawn, after ModeCoordinator/QuadSimApi init.
            if (newDrone.Api != null && newDrone.Api.IsInitialized)
                newDrone.Api.SetMode(startingMode);
            
            if (spawnPayload && payloadPrefab != null)
            {
                Transform mount = newDrone.transform.Find(payloadMountPath);

                if (mount == null)
                {
                    Debug.LogError(
                        $"[DroneManager] Payload mount '{payloadMountPath}' " +
                        $"not found under {newDrone.name}.");
                }
                else
                {
                    PayloadAttachment payload = Instantiate(
                        payloadPrefab,
                        mount.position,
                        mount.rotation);

                    payload.name = $"{newDrone.name}_Payload";
                    payload.AttachTo(newDrone.Rigidbody, mount);
                }
            }
            
            // Select the new drone
            int newIndex = GetDroneIndex(newDrone);
            if (newIndex >= 0)
            {
                SelectDroneByIndex(newIndex, true);
            }

            Debug.Log($"[DroneManager] Spawned drone: {newDrone.name} at {spawnPos}");
            return newDrone;
        }

        private Vector3 DetermineSpawnPosition(Vector3 requestedPosition)
        {
            if (requestedPosition != Vector3.zero)
                return requestedPosition;

            // Stack next to last drone
            if (_registeredDrones.Count > 0)
            {
                QuadPawn last = _registeredDrones[_registeredDrones.Count - 1];
                if (last != null)
                    return last.transform.position + last.transform.right * spawnOffset;
            }

            return defaultSpawnOrigin;
        }

        private Quaternion DetermineSpawnRotation()
        {
            if (_registeredDrones.Count > 0)
            {
                QuadPawn last = _registeredDrones[_registeredDrones.Count - 1];
                if (last != null)
                    return last.transform.rotation;
            }
            return Quaternion.identity;
        }

        // ============================================================================
        // Registration
        // ============================================================================

        public void RegisterDrone(QuadPawn drone)
        {
            if (drone == null) return;

            if (_registeredDrones.Contains(drone))
            {
                Debug.LogWarning($"[DroneManager] Drone already registered: {drone.name}");
                return;
            }

            drone.EnsureAgentIdentity(GetNextAvailableDroneIndex());

            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn existing = _registeredDrones[i];
                if (existing == null) continue;

                if (existing.DroneIndex == drone.DroneIndex)
                {
                    Debug.LogError($"[DroneManager] Duplicate drone_index {drone.DroneIndex}; " +
                                   $"cannot register {drone.name}.");
                    return;
                }

                if (string.Equals(existing.AgentId, drone.AgentId, System.StringComparison.Ordinal))
                {
                    Debug.LogError($"[DroneManager] Duplicate agent_id '{drone.AgentId}'; " +
                                   $"cannot register {drone.name}.");
                    return;
                }
            }

            bool wasEmpty = _registeredDrones.Count == 0;
            QuadPawn selectedBefore = SelectedDrone;
            _registeredDrones.Add(drone);
            _registeredDrones.Sort(CompareDroneIdentity);

            Debug.Log($"[DroneManager] Registered {drone.AgentId} " +
                      $"(index={drone.DroneIndex}, team={drone.TeamId}, total={_registeredDrones.Count})");

            if (wasEmpty)
            {
                SelectDroneByIndex(0, true);
            }
            else if (selectedBefore != null)
            {
                _selectedDroneIndex = _registeredDrones.IndexOf(selectedBefore);
            }
        }

        public void UnregisterDrone(QuadPawn drone)
        {
            if (drone == null) return;

            int index = _registeredDrones.IndexOf(drone);
            if (index < 0) return;

            _registeredDrones.RemoveAt(index);

            if (_selectedDroneIndex >= _registeredDrones.Count)
            {
                _selectedDroneIndex = Mathf.Max(0, _registeredDrones.Count - 1);
            }

            Debug.Log($"[DroneManager] Unregistered drone: {drone.name} (remaining: {_registeredDrones.Count})");
        }

        // ============================================================================
        // Selection
        // ============================================================================

        public int GetDroneIndex(QuadPawn drone) => _registeredDrones.IndexOf(drone);

        public QuadPawn GetDroneByAgentId(string agentId)
        {
            if (string.IsNullOrEmpty(agentId)) return null;

            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null && string.Equals(drone.AgentId, agentId, System.StringComparison.Ordinal))
                    return drone;
            }
            return null;
        }

        public QuadPawn GetDroneByPersistentIndex(int droneIndex)
        {
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null && drone.DroneIndex == droneIndex)
                    return drone;
            }
            return null;
        }

        internal void RefreshIdentityOrder()
        {
            QuadPawn selected = SelectedDrone;
            _registeredDrones.Sort(CompareDroneIdentity);
            if (selected != null)
                _selectedDroneIndex = _registeredDrones.IndexOf(selected);
        }

        public void SelectDroneByIndex(int index, bool alsoPossess = false)
        {
            if (index < 0 || index >= _registeredDrones.Count) return;

            QuadPawn target = _registeredDrones[index];
            if (target == null) return;

            _selectedDroneIndex = index;

            // Activate only the selected drone's camera; keep its current camera mode.
            if (alsoPossess)
            {
                CameraDirector director = CameraDirector.Get();
                if (director != null && director.ViewCamera != null)
                {
                    director.Bind(target);
                }
                else
                {
                    for (int i = 0; i < _registeredDrones.Count; i++)
                    {
                        if (_registeredDrones[i] != null)
                            _registeredDrones[i].SetCameraActive(i == _selectedDroneIndex);
                    }
                }
            }

            Debug.Log($"[DroneManager] Selected drone: {target.name} (index {index})");
        }

        public void SelectNextDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int next = (_selectedDroneIndex + 1) % _registeredDrones.Count;
            SelectDroneByIndex(next, true);
        }

        public void SelectPreviousDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int prev = (_selectedDroneIndex - 1 + _registeredDrones.Count) % _registeredDrones.Count;
            SelectDroneByIndex(prev, true);
        }

        // ============================================================================
        // Configuration
        // ============================================================================

        public void SetSelectedConfig(string configName)
        {
            if (string.IsNullOrEmpty(configName)) return;
            selectedConfigName = configName;
            Debug.Log($"[DroneManager] Selected config: {configName}");
        }

        public void SetSceneEnv(int seed, bool noise, double lat, double lon, double alt,
            StartupSpawnPolicy _spawnPolicy, QuadPawn _quadSimPrefab,
            PayloadAttachment _payloadPrefab, bool spawnPayloads,
            string payloadMount, SimFrame _frame, GoalMode _startingMode)
        {
            masterSeed = seed; 
            enableSensorNoise = noise;
            originLat = lat; 
            originLon = lon; 
            originAlt = alt;
            startupSpawnPolicy = _spawnPolicy;
            quadPawnPrefab = _quadSimPrefab;
            payloadPrefab = _payloadPrefab;
            spawnPayload =  spawnPayloads;
            payloadMountPath = payloadMount;
            frame = _frame;
            startingMode = _startingMode;
        }
        
        public List<string> GetAvailableConfigs()
        {
            return DroneConfigLoader.GetAvailableConfigs();
        }

        public void SaveCurrentConfigAsDefault()
        {
            if (string.IsNullOrEmpty(selectedConfigName)) return;
            PlayerPrefs.SetString("QuadSim.DefaultConfig", selectedConfigName);
            PlayerPrefs.Save();
            Debug.Log($"[DroneManager] Saved default config: {selectedConfigName}");
        }

        public void ReloadCurrentDrone(bool resetPosition = true)
        {
            QuadPawn oldDrone = SelectedDrone;
            if (oldDrone == null)
            {
                Debug.LogWarning("[DroneManager] No drone selected to reload");
                return;
            }

            Vector3 pos = resetPosition ? defaultSpawnOrigin : oldDrone.transform.position;
            Quaternion rot = resetPosition ? Quaternion.identity : oldDrone.transform.rotation;

            Destroy(oldDrone.gameObject);
            CleanupInvalidDrones();

            SpawnDroneWithConfig(selectedConfigName, pos, rot);
        }

        // ============================================================================
        // Reset
        // ============================================================================

        public void ResetAllDrones()
        {
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone == null) continue;

                Vector3 pos = defaultSpawnOrigin + Vector3.right * (i * spawnOffset);
                drone.ResetPosition(pos, Quaternion.identity);
            }

            Debug.Log($"[DroneManager] Reset {_registeredDrones.Count} drones");
        }

        public void ResetSelectedDrone()
        {
            QuadPawn drone = SelectedDrone;
            if (drone == null) return;
            drone.ResetPosition(defaultSpawnOrigin, Quaternion.identity);
        }

        // ============================================================================
        // Utility
        // ============================================================================

        private int GetNextAvailableDroneIndex()
        {
            int candidate = 0;
            while (GetDroneByPersistentIndex(candidate) != null)
                candidate++;
            return candidate;
        }

        private static int CompareDroneIdentity(QuadPawn a, QuadPawn b)
        {
            if (ReferenceEquals(a, b)) return 0;
            if (a == null) return 1;
            if (b == null) return -1;

            int indexOrder = a.DroneIndex.CompareTo(b.DroneIndex);
            return indexOrder != 0
                ? indexOrder
                : string.CompareOrdinal(a.AgentId, b.AgentId);
        }

        private void CleanupInvalidDrones()
        {
            for (int i = _registeredDrones.Count - 1; i >= 0; i--)
            {
                if (_registeredDrones[i] == null)
                    _registeredDrones.RemoveAt(i);
            }
        }

        public string GetStateJson()
        {
            return $"{{\"drone_count\": {_registeredDrones.Count}, \"selected_index\": {_selectedDroneIndex}, \"swarm_mode\": {swarmMode.ToString().ToLower()}, \"config\": \"{selectedConfigName}\"}}";
        }
    }
}
