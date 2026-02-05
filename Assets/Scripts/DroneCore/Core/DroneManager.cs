using System.Collections.Generic;
using SimCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Core
{
    /// <summary>
    /// DroneManager is the central registry and orchestrator for all drones in the simulation.
    /// It handles spawning, selection, configuration, and drives per-frame updates.
    /// 
    /// Implements ISimulatable so SimulationManager can drive physics/rendering updates.
    /// </summary>
    public sealed class DroneManager : MonoBehaviour, ISimulatable
    {
        // ============================================================================
        // Serialized Fields
        // ============================================================================
        
        [Header("Spawning")]
        [Tooltip("Prefab to instantiate when spawning drones.")]
        [SerializeField] private QuadPawn quadPawnPrefab;
        
        [Tooltip("Default spawn location if no PlayerStart found.")]
        [SerializeField] private Vector3 defaultSpawnOrigin = new Vector3(0f, 1f, 0f);
        
        [Tooltip("Offset between multiple spawned drones.")]
        [SerializeField] private float spawnOffset = 1.5f;

        [Header("Configuration")]
        [Tooltip("Default config name to load.")]
        [SerializeField] private string selectedConfigName = "StandardDrone_Config";

        [Header("Swarm")]
        [Tooltip("Enable swarm mode (all drones receive same commands).")]
        [SerializeField] private bool swarmMode = false;

        // ============================================================================
        // Runtime State
        // ============================================================================
        
        private readonly List<QuadPawn> _registeredDrones = new List<QuadPawn>();
        private int _selectedDroneIndex = 0;
        
        private Vector3 _spawnOrigin;
        private Vector3 _lastSpawnLocation;
        
        private DroneConfig _currentConfig;
        private SimulationManager _simManager;

        // ============================================================================
        // ISimulatable Implementation
        // ============================================================================
        
        public int ExecutionOrder => 10; // Run before individual drone controllers (which are ~100)
        public string DebugName => "DroneManager";

        // ============================================================================
        // Public Accessors
        // ============================================================================
        
        public IReadOnlyList<QuadPawn> RegisteredDrones => _registeredDrones;
        public int DroneCount => _registeredDrones.Count;
        public int SelectedDroneIndex => _selectedDroneIndex;
        public string SelectedConfigName => selectedConfigName;
        public bool SwarmMode { get => swarmMode; set => swarmMode = value; }
        public Vector3 SpawnOrigin { get => _spawnOrigin; set => _spawnOrigin = value; }

        /// <summary>
        /// Get the currently selected drone, or null if none.
        /// </summary>
        public QuadPawn SelectedDrone
        {
            get
            {
                if (_selectedDroneIndex >= 0 && _selectedDroneIndex < _registeredDrones.Count)
                {
                    return _registeredDrones[_selectedDroneIndex];
                }
                return null;
            }
        }

        // ============================================================================
        // Static Accessor
        // ============================================================================
        
        private static DroneManager _instance;
        
        /// <summary>
        /// Get the DroneManager instance. Prefers cached instance, falls back to FindFirstObjectByType.
        /// </summary>
        public static DroneManager Get()
        {
            if (_instance != null) return _instance;
            _instance = FindFirstObjectByType<DroneManager>();
            return _instance;
        }
        /// <summary>
        /// Get SimulationManager from scene.
        /// </summary>
        private static SimulationManager FindSimulationManager()
        {
            return FindFirstObjectByType<SimulationManager>();
        }

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            _instance = this;
            
            // Try to load saved default config
            // In Unity, we'd use PlayerPrefs or a settings file
            string savedConfig = PlayerPrefs.GetString("QuadSim.DefaultConfig", "");
            if (!string.IsNullOrEmpty(savedConfig))
            {
                selectedConfigName = savedConfig;
            }
            
            Debug.Log($"[DroneManager] Awake: selectedConfig={selectedConfigName}");
        }
        private void Start()
        {
            // Find spawn origin from scene
            FindSpawnOrigin();
            
            // Register with SimulationManager
            _simManager = FindSimulationManager();
            if (_simManager != null)
            {
                _simManager.RegisterManagers(this);
                Debug.Log("[DroneManager] Registered with SimulationManager");
            }
            else
            {
                Debug.LogWarning("[DroneManager] No SimulationManager found - manual update required");
            }
        }
        private void OnDestroy()
        {
            // Unregister from SimulationManager
            if (_simManager != null)
            {
                _simManager.UnRegisterManager(this);
            }
            
            if (_instance == this)
            {
                _instance = null;
            }
        }

        // ============================================================================
        // ISimulatable - Simulation Lifecycle
        // ============================================================================

        public void OnSimulationStart(SimulationManager sim)
        {
            _simManager = sim;
            
            // Load the selected config
            if (!string.IsNullOrEmpty(selectedConfigName))
            {
                if (DroneConfigLoader.LoadConfig(selectedConfigName, out _currentConfig))
                {
                    Debug.Log($"[DroneManager] Loaded config: {selectedConfigName}");
                }
                else
                {
                    Debug.LogWarning($"[DroneManager] Failed to load config: {selectedConfigName}");
                }
            }

            Debug.Log($"[DroneManager] OnSimulationStart: {_registeredDrones.Count} drones registered");
        }
        public void OnSimulationReset(SimulationManager sim)
        {
            Debug.Log("[DroneManager] OnSimulationReset");
            
            // Reset all drones to spawn origin
            ResetAllDrones();
        }
        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            // Clean up any destroyed drones
            CleanupInvalidDrones();
            
            // Pre-physics is handled by individual drone controllers via ISimulatable
            // DroneManager doesn't need to do anything here for physics
        }
        public void PostPhysicsStep(double dtSec, long nowNanos)
        {
            // Post-physics is handled by individual drone controllers
        }

        // ============================================================================
        // Update - Called by Unity (for rendering updates)
        // ============================================================================

        private void Update()
        {
            // Visual updates happen here (outside physics)
            float dt = Time.deltaTime;
            
            foreach (var drone in _registeredDrones)
            {
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
            DroneConfig config;
            if (!DroneConfigLoader.LoadConfig(configName, out config))
            {
                Debug.LogError($"[DroneManager] Failed to load config: {configName}");
                return null;
            }

            // Determine spawn location
            Vector3 spawnPos = DetermineSpawnPosition(position);
            Quaternion spawnRot = rotation == Quaternion.identity ? DetermineSpawnRotation() : rotation;

            // Instantiate
            QuadPawn newDrone = Instantiate(quadPawnPrefab, spawnPos, spawnRot);
            if (newDrone == null)
            {
                Debug.LogError("[DroneManager] Failed to instantiate QuadPawn prefab");
                return null;
            }

            // Name it
            newDrone.name = $"Drone_{_registeredDrones.Count:D2}";

            // IMPORTANT: Inject DroneManager reference BEFORE Start() runs
            // This ensures the drone registers with us properly
            newDrone.SetDroneManager(this);

            // Apply config (this triggers QuadPawn's initialization)
            newDrone.ApplyConfig(config);

            // Update last spawn location
            _lastSpawnLocation = spawnPos;

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
            // If a specific position was requested and it's not zero, use it
            if (requestedPosition != Vector3.zero)
            {
                return requestedPosition;
            }

            // If we have existing drones, spawn to the right of the last one
            if (_registeredDrones.Count > 0)
            {
                QuadPawn lastDrone = _registeredDrones[_registeredDrones.Count - 1];
                if (lastDrone != null)
                {
                    Vector3 right = lastDrone.transform.right;
                    return lastDrone.transform.position + right * spawnOffset;
                }
            }

            // Fall back to spawn origin
            return _spawnOrigin;
        }
        private Quaternion DetermineSpawnRotation()
        {
            // Match last drone's rotation, or use identity
            if (_registeredDrones.Count > 0)
            {
                QuadPawn lastDrone = _registeredDrones[_registeredDrones.Count - 1];
                if (lastDrone != null)
                {
                    return lastDrone.transform.rotation;
                }
            }
            return Quaternion.identity;
        }
        private void FindSpawnOrigin()
        {
            // Look for a PlayerStart or similar spawn point
            // In Unity, this might be a tagged GameObject or a specific component
            
            GameObject playerStart = GameObject.FindWithTag("PlayerStart");
            if (playerStart != null)
            {
                _spawnOrigin = playerStart.transform.position;
                Debug.Log($"[DroneManager] SpawnOrigin set from PlayerStart: {_spawnOrigin}");
                return;
            }

            // Try finding by name
            GameObject spawnPoint = GameObject.Find("SpawnPoint") ?? GameObject.Find("PlayerStart");
            if (spawnPoint != null)
            {
                _spawnOrigin = spawnPoint.transform.position;
                Debug.Log($"[DroneManager] SpawnOrigin set from SpawnPoint: {_spawnOrigin}");
                return;
            }

            // Use default
            _spawnOrigin = defaultSpawnOrigin;
            Debug.Log($"[DroneManager] SpawnOrigin using default: {_spawnOrigin}");
        }

        // ============================================================================
        // Registration
        // ============================================================================

        /// <summary>
        /// Register a drone with the manager. Called by QuadPawn.Start().
        /// </summary>
        public void RegisterDrone(QuadPawn drone)
        {
            if (drone == null) return;
            
            if (_registeredDrones.Contains(drone))
            {
                Debug.LogWarning($"[DroneManager] Drone already registered: {drone.name}");
                return;
            }

            bool wasEmpty = _registeredDrones.Count == 0;
            _registeredDrones.Add(drone);

            Debug.Log($"[DroneManager] Registered drone: {drone.name} (total: {_registeredDrones.Count})");

            // Auto-select first drone
            if (wasEmpty)
            {
                SelectDroneByIndex(0, true);
            }
        }

        /// <summary>
        /// Unregister a drone from the manager. Called by QuadPawn.OnDestroy().
        /// </summary>
        public void UnregisterDrone(QuadPawn drone)
        {
            if (drone == null) return;

            int index = _registeredDrones.IndexOf(drone);
            if (index < 0)
            {
                Debug.LogWarning($"[DroneManager] Drone not registered: {drone.name}");
                return;
            }

            _registeredDrones.RemoveAt(index);
            
            // Adjust selection if needed
            if (_selectedDroneIndex >= _registeredDrones.Count)
            {
                _selectedDroneIndex = Mathf.Max(0, _registeredDrones.Count - 1);
            }

            Debug.Log($"[DroneManager] Unregistered drone: {drone.name} (remaining: {_registeredDrones.Count})");
        }

        // ============================================================================
        // Selection
        // ============================================================================

        /// <summary>
        /// Get the index of a drone in the registry.
        /// </summary>
        public int GetDroneIndex(QuadPawn drone)
        {
            return _registeredDrones.IndexOf(drone);
        }

        /// <summary>
        /// Select a drone by index.
        /// </summary>
        public void SelectDroneByIndex(int index, bool alsoPossess = false)
        {
            if (index < 0 || index >= _registeredDrones.Count)
            {
                Debug.LogWarning($"[DroneManager] Invalid drone index: {index}");
                return;
            }

            QuadPawn target = _registeredDrones[index];
            if (target == null)
            {
                Debug.LogWarning($"[DroneManager] Drone at index {index} is null");
                return;
            }

            _selectedDroneIndex = index;

            if (alsoPossess)
            {
                // In Unity, "possessing" typically means making this the camera target
                target.ForceFPVCameraActive();
                
                // Disable control on other drones if not in swarm mode
                if (!swarmMode)
                {
                    // Future: enable/disable input routing per drone
                }
            }

            Debug.Log($"[DroneManager] Selected drone: {target.name} (index {index})");
        }

        /// <summary>
        /// Cycle to the next drone.
        /// </summary>
        public void SelectNextDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int next = (_selectedDroneIndex + 1) % _registeredDrones.Count;
            SelectDroneByIndex(next, true);
        }

        /// <summary>
        /// Cycle to the previous drone.
        /// </summary>
        public void SelectPreviousDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int prev = (_selectedDroneIndex - 1 + _registeredDrones.Count) % _registeredDrones.Count;
            SelectDroneByIndex(prev, true);
        }

        // ============================================================================
        // Configuration
        // ============================================================================

        /// <summary>
        /// Set the config to use for future spawns.
        /// </summary>
        public void SetSelectedConfig(string configName)
        {
            if (string.IsNullOrEmpty(configName)) return;
            selectedConfigName = configName;
            Debug.Log($"[DroneManager] Selected config: {configName}");
        }

        /// <summary>
        /// Get list of available config names.
        /// </summary>
        public List<string> GetAvailableConfigs()
        {
            return DroneConfigLoader.GetAvailableConfigs();
        }

        /// <summary>
        /// Save the current config as the default for future sessions.
        /// </summary>
        public void SaveCurrentConfigAsDefault()
        {
            if (string.IsNullOrEmpty(selectedConfigName)) return;
            
            PlayerPrefs.SetString("QuadSim.DefaultConfig", selectedConfigName);
            PlayerPrefs.Save();
            
            Debug.Log($"[DroneManager] Saved default config: {selectedConfigName}");
        }

        /// <summary>
        /// Reload the currently selected drone with fresh config from disk.
        /// </summary>
        public void ReloadCurrentDrone(bool resetPosition = true)
        {
            QuadPawn oldDrone = SelectedDrone;
            if (oldDrone == null)
            {
                Debug.LogWarning("[DroneManager] No drone selected to reload");
                return;
            }

            // Remember transform if not resetting
            Vector3 pos = resetPosition ? _spawnOrigin : oldDrone.transform.position;
            Quaternion rot = resetPosition ? Quaternion.identity : oldDrone.transform.rotation;

            // Destroy old drone
            Destroy(oldDrone.gameObject);
            
            // Clean up immediately
            CleanupInvalidDrones();

            // Spawn fresh
            SpawnDroneWithConfig(selectedConfigName, pos, rot);
        }

        // ============================================================================
        // Reset
        // ============================================================================

        /// <summary>
        /// Reset all drones to spawn positions.
        /// </summary>
        public void ResetAllDrones()
        {
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone == null) continue;

                // Calculate staggered position
                Vector3 pos = _spawnOrigin + Vector3.right * (i * spawnOffset);
                drone.ResetPosition(pos, Quaternion.identity);
            }

            Debug.Log($"[DroneManager] Reset {_registeredDrones.Count} drones");
        }

        /// <summary>
        /// Reset the currently selected drone.
        /// </summary>
        public void ResetSelectedDrone()
        {
            QuadPawn drone = SelectedDrone;
            if (drone == null) return;

            drone.ResetPosition(_spawnOrigin, Quaternion.identity);
        }

        // ============================================================================
        // Utility
        // ============================================================================

        private void CleanupInvalidDrones()
        {
            for (int i = _registeredDrones.Count - 1; i >= 0; i--)
            {
                if (_registeredDrones[i] == null)
                {
                    _registeredDrones.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// Get a JSON representation of the current state (for API/debugging).
        /// </summary>
        public string GetStateJson()
        {
            return $"{{\"drone_count\": {_registeredDrones.Count}, \"selected_index\": {_selectedDroneIndex}, \"swarm_mode\": {swarmMode.ToString().ToLower()}, \"config\": \"{selectedConfigName}\"}}";
        }
    }
    
}