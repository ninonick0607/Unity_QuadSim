using System;
using DroneCore.Common;
using DroneCore.Controllers;
using DroneCore.Core;
using DroneCore.Interfaces;
using RobotCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore
{
    /// <summary>
    /// QuadPawn is the physical drone entity - the "vehicle" that exists in the world.
    /// It owns references to all drone subsystems (controller, sensors, thrusters, command proxy)
    /// and provides the API that DroneManager uses to control the drone lifecycle.
    /// 
    /// This is NOT an ISimulatable - DroneManager iterates over QuadPawns and calls their
    /// update methods to minimize interface overhead.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public sealed class QuadPawn : MonoBehaviour
    {
        // ============================================================================
        // Serialized Fields
        // ============================================================================
        
        [Header("Core Physics")]
        [SerializeField] private Rigidbody rb;

        [Header("Motor Transforms (auto-found if empty)")]
        [SerializeField] private Transform motorFL;
        [SerializeField] private Transform motorFR;
        [SerializeField] private Transform motorBL;
        [SerializeField] private Transform motorBR;

        [Header("Subsystem References (auto-found if empty)")]
        [SerializeField] private ThrusterSet thrusters;
        [SerializeField] private CascadedController controller;
        [SerializeField] private SensorManager sensorManager;
        [SerializeField] private FlightCommandProxy commandProxy;

        [Header("Camera (stub)")]
        [SerializeField] private Transform fpvCameraPoint;
        
        [Header("Visuals")]
        [SerializeField] private Transform[] propellerMeshes = new Transform[4];
        [SerializeField] private float[] propellerRPMs = new float[4];

        // ============================================================================
        // Runtime State
        // ============================================================================
        
        // Canonical motor order: [0]=FL, [1]=FR, [2]=BL, [3]=BR
        private readonly Transform[] _motors = new Transform[4];
        
        // Config applied to this drone
        private DroneConfig _config;
        private bool _configApplied;
        
        // Unique identifier for this drone instance
        public string DroneID { get; private set; }
        
        // Initialization flags
        private bool _sensorsReady;
        private bool _controllerReady;
        private bool _initialized;

        // Cached manager reference (explicitly set, not auto-found)
        private DroneManager _droneManager;

        // ============================================================================
        // Public Accessors
        // ============================================================================
        
        /// <summary>
        /// Set the DroneManager reference. Called by DroneManager after spawning,
        /// or can be set manually for drones placed in scene.
        /// </summary>
        public void SetDroneManager(DroneManager manager)
        {
            _droneManager = manager;
        }
        public Rigidbody Rigidbody => rb;
        public Transform[] Motors => _motors;
        public Transform MotorFL => _motors[0];
        public Transform MotorFR => _motors[1];
        public Transform MotorBL => _motors[2];
        public Transform MotorBR => _motors[3];
        
        public ThrusterSet Thrusters => thrusters;
        public CascadedController Controller => controller;
        public SensorManager Sensors => sensorManager;
        public FlightCommandProxy CommandProxy => commandProxy;
        
        public DroneConfig Config => _config;
        public bool IsInitialized => _initialized;
        public bool IsSensorsReady => _sensorsReady;
        public bool IsControllerReady => _controllerReady;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Reset()
        {
            // Called when component is first added or reset in editor
            rb = GetComponent<Rigidbody>();
        }
        private void Awake()
        {
            DroneID = gameObject.name;
            
            // Wire all references
            AutoWireIfNeeded();
            ValidateOrThrow();
            
            Debug.Log($"[QuadPawn] Awake: DroneID={DroneID}");
        }
        private void Start()
        {
            // Register with DroneManager if one was injected
            if (_droneManager != null)
            {
                _droneManager.RegisterDrone(this);
                Debug.Log($"[QuadPawn] Registered with DroneManager: {DroneID}");
            }
            else
            {
                // For scene-placed drones without explicit manager, try to find one
                // This is a fallback - prefer explicit injection via SetDroneManager()
                _droneManager = DroneManager.Get();
                if (_droneManager != null)
                {
                    _droneManager.RegisterDrone(this);
                    Debug.Log($"[QuadPawn] Registered with DroneManager (fallback find): {DroneID}");
                }
                else
                {
                    Debug.LogWarning($"[QuadPawn] No DroneManager set for {DroneID} - drone will not be managed");
                }
            }

            // Initialize subsystems if config already applied
            if (_configApplied)
            {
                InitializeSubsystems();
            }
        }
        private void OnDestroy()
        {
            // Unregister from DroneManager
            if (_droneManager != null)
            {
                _droneManager.UnregisterDrone(this);
                Debug.Log($"[QuadPawn] Unregistered from DroneManager: {DroneID}");
            }
        }
        // ============================================================================
        // Wiring & Validation
        // ============================================================================

        /// <summary>
        /// Auto-wire all component references. Safe to call multiple times.
        /// </summary>
        public void AutoWireIfNeeded()
        {
            // Core physics
            if (rb == null) rb = GetComponent<Rigidbody>();

            // Motor transforms - prefer explicit, fallback to search by name
            if (motorFL == null) motorFL = FindDeepChild(transform, "MotorFL");
            if (motorFR == null) motorFR = FindDeepChild(transform, "MotorFR");
            if (motorBL == null) motorBL = FindDeepChild(transform, "MotorBL");
            if (motorBR == null) motorBR = FindDeepChild(transform, "MotorBR");

            _motors[0] = motorFL;
            _motors[1] = motorFR;
            _motors[2] = motorBL;
            _motors[3] = motorBR;

            // Subsystems - check self first, then children
            if (thrusters == null) thrusters = GetComponent<ThrusterSet>();
            if (controller == null) controller = GetComponent<CascadedController>();
            if (sensorManager == null) sensorManager = GetComponent<SensorManager>();
            if (commandProxy == null) commandProxy = GetComponent<FlightCommandProxy>();
            
            // FPV camera point - search for common names
            if (fpvCameraPoint == null)
            {
                fpvCameraPoint = FindDeepChild(transform, "FPVCamera") 
                              ?? FindDeepChild(transform, "CameraPoint")
                              ?? FindDeepChild(transform, "FPVCam");
            }
        }
        /// <summary>
        /// Validate that all required references are present. Throws if not.
        /// </summary>
        public void ValidateOrThrow()
        {
            if (rb == null)
                throw new Exception($"{name}: QuadPawn requires a Rigidbody reference.");

            for (int i = 0; i < 4; i++)
            {
                if (_motors[i] == null)
                    throw new Exception($"{name}: Missing motor transform for index {i} (FL,FR,BL,BR).");
            }

            // Subsystems are warnings, not errors - they might be added later
            if (thrusters == null)
                Debug.LogWarning($"[QuadPawn] {name}: ThrusterSet not found.");
            if (controller == null)
                Debug.LogWarning($"[QuadPawn] {name}: CascadedController not found.");
            if (sensorManager == null)
                Debug.LogWarning($"[QuadPawn] {name}: SensorManager not found.");
            if (commandProxy == null)
                Debug.LogWarning($"[QuadPawn] {name}: FlightCommandProxy not found.");
        }
        // ============================================================================
        // Configuration
        // ============================================================================

        /// <summary>
        /// Apply a drone configuration. Called by DroneManager after spawning.
        /// </summary>
        public void ApplyConfig(DroneConfig config)
        {
            _config = config;
            _configApplied = true;
            
            Debug.Log($"[QuadPawn] {DroneID}: Applying config...");

            // Apply physical properties
            ApplyPhysicalConfig();

            // Apply controller gains (if controller exists)
            ApplyControllerConfig();
            
            // If Start() already ran, initialize now
            if (gameObject.activeInHierarchy && Application.isPlaying)
            {
                InitializeSubsystems();
            }
        }
        private void ApplyPhysicalConfig()
        {
            if (rb == null) return;

            // Apply mass from config
            if (_config.DroneParams.Mass > 0.01f)
            {
                rb.mass = _config.DroneParams.Mass;
            }
            else
            {
                rb.mass = 1.28f; // Default quad mass in kg
            }

            // Apply center of mass offset if specified
            if (_config.DroneParams.CenterOfMass != Vector3.zero)
            {
                rb.centerOfMass = _config.DroneParams.CenterOfMass;
            }

            // Apply inertia tensor if specified
            if (_config.DroneParams.Inertia != Vector3.zero)
            {
                rb.inertiaTensor = _config.DroneParams.Inertia;
            }

            // Apply drag coefficients
            if (_config.DroneParams.LinearDrag_Coeff >= 0)
            {
                rb.linearDamping = _config.DroneParams.LinearDrag_Coeff;
            }
            if (_config.DroneParams.AngularDrag_Coeff >= 0)
            {
                rb.angularDamping = _config.DroneParams.AngularDrag_Coeff;
            }

            // Configure rigidbody for simulation
            rb.useGravity = true;
            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.None;
            rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            
            // Prevent Unity's angular velocity clamping from wrecking rate control
            rb.maxAngularVelocity = 100f;

            Debug.Log($"[QuadPawn] {DroneID}: Physical config applied (mass={rb.mass}kg, drag={rb.linearDamping}/{rb.angularDamping})");
        }
        private void ApplyControllerConfig()
        {
            if (controller == null) return;
            // Pass config to controller for PID gain application
            controller.ApplyConfig(_config);
            
            Debug.Log($"[QuadPawn] {DroneID}: Controller config applied");
        }


        // ============================================================================
        // Initialization (called after config applied and Start() runs)
        // ============================================================================

        private void InitializeSubsystems()
        {
            if (_initialized)
            {
                Debug.LogWarning($"[QuadPawn] {DroneID}: Already initialized, skipping.");
                return;
            }

            Debug.Log($"[QuadPawn] {DroneID}: Initializing subsystems...");

            // 1. Initialize sensors
            if (sensorManager != null)
            {
                // SensorManager initializes via ISimulatable.OnSimulationStart
                // but we can mark ready here if it exists
                _sensorsReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: SensorManager ready");
            }

            // 2. Initialize controller
            if (controller != null && _sensorsReady)
            {
                // Controller initializes via ISimulatable.OnSimulationStart
                // but we can mark ready here if it exists
                _controllerReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: Controller ready");
            }

            // 3. Initialize command proxy
            if (commandProxy != null)
            {
                commandProxy.ResetToSafe();
                Debug.Log($"[QuadPawn] {DroneID}: CommandProxy reset to safe");
            }

            _initialized = true;
            Debug.Log($"[QuadPawn] {DroneID}: Initialization complete");
        }

        // ============================================================================
        // Reset Functions (called by DroneManager)
        // ============================================================================

        /// <summary>
        /// Full reset - position, rotation, velocities, and controller state.
        /// </summary>
        public void ResetDrone()
        {
            ResetPhysics();
            ResetControllerState();
            
            Debug.Log($"[QuadPawn] {DroneID}: Full reset complete");
        }

        /// <summary>
        /// Reset position to origin or specified location.
        /// </summary>
        public void ResetPosition(Vector3? targetPosition = null, Quaternion? targetRotation = null)
        {
            Vector3 pos = targetPosition ?? Vector3.up; // Default 1m above origin
            Quaternion rot = targetRotation ?? Quaternion.identity;

            // Teleport with physics reset
            transform.SetPositionAndRotation(pos, rot);
            
            ResetPhysics();
            ResetControllerState();

            Debug.Log($"[QuadPawn] {DroneID}: Position reset to {pos}");
        }

        /// <summary>
        /// Reset rotation only, keeping position.
        /// </summary>
        public void ResetRotation()
        {
            if (rb == null) return;

            // Keep yaw, zero roll/pitch
            Vector3 currentEuler = transform.eulerAngles;
            Quaternion uprightRotation = Quaternion.Euler(0f, currentEuler.y, 0f);
            
            transform.rotation = uprightRotation;
            rb.angularVelocity = Vector3.zero;
            rb.linearVelocity = Vector3.zero;

            Debug.Log($"[QuadPawn] {DroneID}: Rotation reset (kept yaw={currentEuler.y:F1}°)");
        }

        /// <summary>
        /// Reset physics state (velocities) without moving position.
        /// </summary>
        public void ResetPhysics()
        {
            if (rb == null) return;

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.WakeUp();
        }

        /// <summary>
        /// Reset all controller/PID state.
        /// </summary>
        public void ResetControllerState()
        {
            // Reset controller PIDs
            if (controller != null)
            {
                // CascadedController has internal reset via OnSimulationReset
                // but we want a direct call for runtime resets
                controller.acro?.Reset();
                controller.zPid?.Reset();
            }

            // Reset command proxy to safe state
            if (commandProxy != null)
            {
                commandProxy.ResetToSafe();
            }

            // Reset thruster commands
            if (thrusters != null)
            {
                thrusters.SetAllMotorCommands01(0f, 0f, 0f, 0f);
            }
        }

        // ============================================================================
        // Update Functions (called by DroneManager each frame)
        // ============================================================================

        /// <summary>
        /// Update visual elements (prop spin, effects). Called from DroneManager.RenderingUpdate.
        /// </summary>
        public void UpdateVisuals(float deltaTime)
        {
            UpdatePropellerVisuals(deltaTime);
            // Future: debug draw, effects, etc.
        }

        private void UpdatePropellerVisuals(float deltaTime)
        {
            if (propellerMeshes == null || propellerMeshes.Length != 4) return;
            if (controller == null) return;

            // Get motor outputs from controller's last allocation
            var motorOutputs = controller.LastMotor01;
            if (motorOutputs == null || motorOutputs.Count != 4) return;

            // Get thrust model params from config
            float thrustCoef = 1e-5f;  // Default Ct
            float propDiameter = 0.127f; // Default 5" prop in meters
            float airDensity = 1.225f;
            float maxThrustPerMotor = 4.12f; // Default max thrust N

            // Read from config if available
            if (_configApplied)
            {
                if (_config.RotorParams.Model.ThrustCoeff > 0)
                    thrustCoef = _config.RotorParams.Model.ThrustCoeff;
                if (_config.RotorParams.Model.Prop_Diameter > 0)
                    propDiameter = _config.RotorParams.Model.Prop_Diameter;
                if (_config.FlightParams.MaxThrust > 0)
                    maxThrustPerMotor = _config.FlightParams.MaxThrust / 4f; // Divide total by 4 motors
            }

            float denom = thrustCoef * airDensity * Mathf.Pow(propDiameter, 4);
            if (denom < 1e-10f) return;

            // Spin direction: get from config rotors if available, else use default X-config
            // FL=CW(+1), FR=CCW(-1), BL=CCW(-1), BR=CW(+1) typical X-config
            int[] spinDir = { 1, -1, -1, 1 };
            if (_configApplied && _config.RotorParams.Rotors != null && _config.RotorParams.Rotors.Length == 4)
            {
                for (int i = 0; i < 4; i++)
                {
                    spinDir[i] = _config.RotorParams.Rotors[i].Spin_Dir;
                }
            }

            for (int i = 0; i < 4; i++)
            {
                if (propellerMeshes[i] == null) continue;

                // Calculate RPM from thrust (simplified model)
                float thrust01 = Mathf.Max(0f, motorOutputs[i]);
                float thrustN = thrust01 * maxThrustPerMotor;
                float rpm = 60f * Mathf.Sqrt(thrustN / denom);
                propellerRPMs[i] = rpm;

                // Rotate prop mesh
                float direction = spinDir[i];
                float frameRotation = (rpm / 60f) * 360f * deltaTime * direction;
                propellerMeshes[i].Rotate(0f, frameRotation, 0f, Space.Self);
            }
        }

        // ============================================================================
        // Camera Functions (stubs)
        // ============================================================================

        /// <summary>
        /// Force FPV camera active. Stub for now.
        /// </summary>
        public void ForceFPVCameraActive()
        {
            // TODO: Implement camera system
            Debug.Log($"[QuadPawn] {DroneID}: ForceFPVCameraActive (stub)");
        }

        /// <summary>
        /// Cycle to next camera mode. Stub for now.
        /// </summary>
        public void SwitchCamera()
        {
            // TODO: Implement camera system
            Debug.Log($"[QuadPawn] {DroneID}: SwitchCamera (stub)");
        }

        /// <summary>
        /// Get the FPV camera attachment point.
        /// </summary>
        public Transform GetFPVCameraPoint()
        {
            return fpvCameraPoint != null ? fpvCameraPoint : transform;
        }

        // ============================================================================
        // Physics Helpers
        // ============================================================================

        /// <summary>
        /// Get body inertia tensor in SI units (kg·m²).
        /// </summary>
        public Vector3 GetBodyInertiaSI()
        {
            if (rb == null) return Vector3.zero;
            return rb.inertiaTensor; // Unity uses kg·m² already
        }

        /// <summary>
        /// Compute arm length from motor positions (average distance from CoM).
        /// </summary>
        public float ComputeArmLength()
        {
            if (rb == null) return 0f;

            Vector3 com = rb.worldCenterOfMass;
            float sum = 0f;
            int count = 0;

            for (int i = 0; i < 4; i++)
            {
                if (_motors[i] == null) continue;
                
                Vector3 motorPos = _motors[i].position;
                Vector3 delta = motorPos - com;
                
                // Horizontal distance only (X-Z plane in Unity)
                float dist = new Vector2(delta.x, delta.z).magnitude;
                sum += dist;
                count++;
            }

            return count > 0 ? sum / count : 0f;
        }

        // ============================================================================
        // Utility
        // ============================================================================

        private static Transform FindDeepChild(Transform root, string childName)
        {
            // BFS search for stable behavior
            var queue = new System.Collections.Generic.Queue<Transform>();
            queue.Enqueue(root);
            
            while (queue.Count > 0)
            {
                var t = queue.Dequeue();
                if (t.name == childName) return t;
                
                for (int i = 0; i < t.childCount; i++)
                {
                    queue.Enqueue(t.GetChild(i));
                }
            }
            
            return null;
        }
    }
}