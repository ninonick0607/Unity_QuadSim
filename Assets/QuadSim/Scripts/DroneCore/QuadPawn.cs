using System;
using System.Collections.Generic;
using DroneCore.Controllers;
using DroneCore.CameraSystem;
using DroneCore.Core;
using DroneCore.Interfaces;
using RobotCore;
using SimCore;
using SimCore.Common;
using UnityEngine;
using Yaml.Drone;
using DroneCore.Api;
using QuadSim.RL;

namespace DroneCore
{
    /// <summary>
    /// QuadPawn is the physical drone entity — the "vehicle" that exists in the world.
    /// It owns references to all drone subsystems and orchestrates the physics step:
    ///   Sensors → Controller → Apply Wrench to Rigidbody
    ///
    /// DroneManager iterates QuadPawns in PrePhysicsStep and calls PhysicsStep().
    /// QuadPawn is NOT an ISimulatable — DroneManager drives it.
    ///
    /// Mirrors UE's AQuadPawn: owns the CustomPhysicsTick sequence,
    /// buffered force application, and controller routing.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(DroneCameraRig))]
    [DisallowMultipleComponent]
    public sealed class QuadPawn : MonoBehaviour
    {
        // ============================================================================
        // Serialized Fields — Set in Inspector / Prefab
        // ============================================================================

        [Header("Agent Identity")]
        [Tooltip("Stable RPC identity. Leave empty to assign drone_<index> when registered.")]
        [SerializeField] private string agentId = "";

        [Tooltip("Team identity used by multi-agent tasks.")]
        [SerializeField] private string teamId = "neutral";

        [Tooltip("Persistent RPC index. Leave at -1 to assign the next available index.")]
        [SerializeField] private int droneIndex = -1;

        [Header("Core Physics")]
        [SerializeField] private Rigidbody rb;

        [Header("Motor Transforms")]
        [SerializeField] private Transform motorFL;
        [SerializeField] private Transform motorFR;
        [SerializeField] private Transform motorBL;
        [SerializeField] private Transform motorBR;

        [Header("Subsystem References")]
        [SerializeField] private CascadedController controller;
        [SerializeField] private SensorManager sensorManager;
        [SerializeField] private FlightCommandProxy commandProxy;
        [SerializeField] private ModeCoordinator modeCoordinator;
        [SerializeField] private QuadSimApi quadSimApi;
        
        [Header("Camera")]
        [SerializeField] private Transform fpvCameraPoint;
        [SerializeField] private DroneCameraRig cameraRig;
        [SerializeField] private DroneImageSensor imageSensor;

        [Header("Visuals")]
        [SerializeField] private Transform[] propellerMeshes = new Transform[4];

        // ============================================================================
        // Runtime State
        // ============================================================================

        // Canonical motor order: [0]=FL, [1]=FR, [2]=BL, [3]=BR
        private readonly Transform[] _motors = new Transform[4];
        private readonly float[] _propellerRPMs = new float[4];

        // Config
        private DroneConfig _config;
        private bool _configApplied;

        // Rotor physics derived from config (matches UE's RotorPhysics member)
        private RotorPhysicsDerived _rotorPhysics;

        public string DroneID => gameObject.name;

        // Initialization tracking
        private bool _sensorsReady;
        private bool _controllerReady;
        private bool _initialized;
        public bool HasConfigApplied => _configApplied;
        /// <summary>Read-only view of the applied YAML config. UI reads limits/ranges from here.</summary>
        public Yaml.Drone.DroneConfig Config => _config;
        // Active controller type (matches UE's currentControllerType)
        [Header("Controller Selection")]
        [SerializeField] private ControllerKind currentControllerKind = ControllerKind.Cascade;

        // Buffered wrench (matches UE's PhysicsState pattern)
        // In Unity we don't need atomics since PrePhysicsStep and Physics.Simulate
        // run on the same thread, but we keep the generation pattern for clarity.
        private Vector3 _bufferedForce;
        private Vector3 _bufferedTorque;
        private uint _wrenchGeneration;
        private uint _lastAppliedGeneration;
        private readonly HashSet<Collider> _activeColliders = new HashSet<Collider>();
        private readonly HashSet<Collider> _groundColliders = new HashSet<Collider>();
        private readonly List<RLAgentEvent> _pendingRLEvents = new List<RLAgentEvent>();

        // Manager reference (set explicitly by DroneManager, not searched)
        private DroneManager _droneManager;

        // ============================================================================
        // Public Accessors
        // ============================================================================

        public void SetDroneManager(DroneManager manager) => _droneManager = manager;

        public Rigidbody Rigidbody => rb;
        public Transform[] Motors => _motors;
        public Transform MotorFL => _motors[0];
        public Transform MotorFR => _motors[1];
        public Transform MotorBL => _motors[2];
        public Transform MotorBR => _motors[3];

        public CascadedController Controller => controller;
        public SensorManager Sensors => sensorManager;
        public FlightCommandProxy CommandProxy => commandProxy;
        public ModeCoordinator ModeCoordinator => modeCoordinator;
        public QuadSimApi Api => quadSimApi;
        public RotorPhysicsDerived RotorPhysics => _rotorPhysics;
        public bool IsInitialized => _initialized;
        public ControllerKind CurrentControllerKind => currentControllerKind;
        public string AgentId => agentId;
        public string TeamId => teamId;
        public int DroneIndex => droneIndex;
        public DroneImageSensor ImageSensor { get { EnsureCameraRig(); return imageSensor; } }
        public bool HasCollision => _activeColliders.Count > 0;
        public bool IsGrounded => _groundColliders.Count > 0;

        internal void EnsureAgentIdentity(int fallbackIndex)
        {
            if (droneIndex < 0)
                droneIndex = fallbackIndex;
            if (string.IsNullOrWhiteSpace(agentId))
                agentId = $"drone_{droneIndex}";
            if (string.IsNullOrWhiteSpace(teamId))
                teamId = "neutral";
        }

        internal void SetEpisodeIdentity(string newAgentId, string newTeamId, int newDroneIndex)
        {
            agentId = newAgentId;
            teamId = newTeamId;
            droneIndex = newDroneIndex;
        }

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Reset()
        {
            // Editor helper: auto-grab rigidbody when component is added
            rb = GetComponent<Rigidbody>();
        }

        private void Awake()
        {
            // Auto-bind subsystem references if not set in prefab
            if (rb == null) rb = GetComponent<Rigidbody>();
            if (controller == null) controller = GetComponent<CascadedController>();
            if (sensorManager == null) sensorManager = GetComponent<SensorManager>();
            if (commandProxy == null) commandProxy = GetComponent<FlightCommandProxy>();
            if (modeCoordinator == null) modeCoordinator = GetComponent<ModeCoordinator>();
            if (quadSimApi == null) quadSimApi = GetComponent<QuadSimApi>();
            EnsureCameraRig();
            
            // Build motor array from serialized references
            _motors[0] = motorFL;
            _motors[1] = motorFR;
            _motors[2] = motorBL;
            _motors[3] = motorBR;

            // Safety fallback: if motor refs weren’t assigned, try find by name
            if (_motors[0] == null) _motors[0] = transform.Find("MotorFL");
            if (_motors[1] == null) _motors[1] = transform.Find("MotorFR");
            if (_motors[2] == null) _motors[2] = transform.Find("MotorBL");
            if (_motors[3] == null) _motors[3] = transform.Find("MotorBR");

            if (controller == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing CascadedController!");
            if (sensorManager == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing SensorManager!");
            if (commandProxy == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing FlightCommandProxy!");

            // Register during Awake so scene-placed drones are known before
            // SimulationManager evaluates DroneManager's startup spawn policy.
            if (_droneManager == null)
                _droneManager = DroneManager.Get();

            if (_droneManager != null)
            {
                _droneManager.RegisterDrone(this);
                Debug.Log($"[QuadPawn] Registered with DroneManager: {DroneID}");
            }
            else
            {
                Debug.LogWarning($"[QuadPawn] {DroneID}: No DroneManager — drone will not be managed.");
            }
        }

        private void Start()
        {
            // If config was applied before Start(), initialize now
            if (_configApplied)
            {
                InitializeSubsystems();
            }
        }

        private void OnDestroy()
        {
            if (_droneManager != null)
            {
                _droneManager.UnregisterDrone(this);
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            bool wasGrounded = IsGrounded;
            bool isGroundContact = TrackCollision(collision);

            QuadPawn otherPawn = collision.collider.GetComponentInParent<QuadPawn>();
            if (otherPawn != null && otherPawn != this)
                AddRLEvent("agent_collision", otherPawn.AgentId);
            RLSceneObject sceneObject = collision.collider.GetComponentInParent<RLSceneObject>();
            if (sceneObject != null)
                AddRLEvent("object_collision", objectId: sceneObject.ObjectId);
            if (isGroundContact && !wasGrounded)
                AddRLEvent("ground_contact");
        }

        private void OnCollisionStay(Collision collision)
        {
            bool wasGrounded = IsGrounded;
            if (TrackCollision(collision) && !wasGrounded)
                AddRLEvent("ground_contact");
        }

        private void OnCollisionExit(Collision collision)
        {
            _activeColliders.Remove(collision.collider);
            _groundColliders.Remove(collision.collider);
        }

        private bool TrackCollision(Collision collision)
        {
            Collider other = collision.collider;
            _activeColliders.Add(other);

            bool isGroundContact = false;
            for (int i = 0; i < collision.contactCount; i++)
            {
                if (Vector3.Dot(collision.GetContact(i).normal, Vector3.up) >= 0.5f)
                {
                    isGroundContact = true;
                    break;
                }
            }

            if (isGroundContact)
                _groundColliders.Add(other);
            else
                _groundColliders.Remove(other);

            return isGroundContact;
        }

        private void AddRLEvent(
            string type,
            string otherAgentId = null,
            string objectId = null)
        {
            double simTime = ClockFactory.IsInitialized
                ? ClockFactory.Clock.NowNanos / 1_000_000_000.0
                : 0.0;
            _pendingRLEvents.Add(new RLAgentEvent(
                type, AgentId, otherAgentId, simTime, objectId));
        }

        public void DrainRLEvents(List<RLAgentEvent> destination)
        {
            if (destination == null || _pendingRLEvents.Count == 0) return;
            destination.AddRange(_pendingRLEvents);
            _pendingRLEvents.Clear();
        }

        // ============================================================================
        // Configuration (called by DroneManager after spawning)
        // ============================================================================

        /// <summary>
        /// Apply a drone configuration. Mirrors UE's AQuadPawn::ApplyConfig.
        /// </summary>
        public void ApplyConfig(DroneConfig config)
        {
            _config = config;
            _configApplied = true;

            EnsureCameraRig();
            CameraDirector director = CameraDirector.Get();
            if (director != null && director.TargetDrone == this)
                director.ApplyCameraSettings(_config.CamParams);

            Debug.Log($"[QuadPawn] {DroneID}: Applying config...");

            ApplyPhysicalConfig();

            if (_config.DroneParams.Arm_Length > 0f)
            {
                _rotorPhysics = new RotorPhysicsDerived();
                Debug.Log($"[QuadPawn] {DroneID}: RotorPhysics computed (arm={_config.DroneParams.Arm_Length}m)");
            }
            InitializeSubsystems();


            ApplyControllerConfig();

            Debug.Log("==== Rotor Geometry Check ====");

            for (int i = 0; i < _motors.Length; i++)
            {
                if (_motors[i] == null) continue;

                Vector3 local = _motors[i].localPosition;
                Vector3 world = _motors[i].position;

                float armLocalXZ = new Vector2(local.x, local.z).magnitude;
                float armWorldXZ = new Vector2(
                    world.x - transform.position.x,
                    world.z - transform.position.z
                ).magnitude;

                Debug.Log(
                    $"Motor {i}: " +
                    $"local={local} " +
                    $"world={world} " +
                    $"armLocal={armLocalXZ:F3}m " +
                    $"armWorld={armWorldXZ:F3}m");
            }
        }

        private void ApplyPhysicalConfig()
        {
            if (rb == null) return;

            // Mass
            if (_config.DroneParams.Mass > 0.01f)
                rb.mass = _config.DroneParams.Mass;
            else
                rb.mass = 1.28f; // Default

            if (_config.DroneParams.CenterOfMass != Vector3.zero)
                rb.centerOfMass = _config.DroneParams.CenterOfMass;
            if (_config.DroneParams.Inertia != Vector3.zero)
                 rb.inertiaTensor = _config.DroneParams.Inertia;
            if (_config.DroneParams.LinearDrag_Coeff >= 0)
                rb.linearDamping = _config.DroneParams.LinearDrag_Coeff;
            if (_config.DroneParams.AngularDrag_Coeff >= 0)
                rb.angularDamping = _config.DroneParams.AngularDrag_Coeff;

            // Rigidbody setup for simulation
            rb.useGravity = true;
            rb.isKinematic = false;
           // rb.interpolation = RigidbodyInterpolation.None;
           // rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            rb.maxAngularVelocity = 100f;
            Debug.Log($"[QuadPawn] {DroneID}: Inertia tensor = {rb.inertiaTensor}");
            Debug.Log($"[QuadPawn] {DroneID}: Physical config applied (mass={rb.mass}kg)");
        }

        private void ApplyControllerConfig()
        {
            // Cascaded controller
            if (controller != null)
            {
                
                _rotorPhysics.ComputeRotor(_config.RotorParams.Model, _config.DroneParams.Arm_Length);
                controller.SetRotorPhysics(_rotorPhysics, _config);
                Debug.Log($"[QuadPawn] {DroneID}: CascadedController config applied");
            }

            // Future: geometric controller
            // if (geoController != null)
            //     geoController.ApplyConfig(_config, _rotorPhysics);
        }

        // ============================================================================
        // Initialization
        // ============================================================================

        private void InitializeSubsystems()
        {
            if (_initialized) return;

            Debug.Log($"[QuadPawn] {DroneID}: Initializing subsystems...");

            // 1. Sensors
            if (sensorManager != null)
            {
                sensorManager.OnStart();
                _sensorsReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: SensorManager initialized");
            }

            // 2. Controller — only ready if sensors are
            if (controller != null && _sensorsReady)
            {
                // Controller handles its own internal init (allocator, gains, etc.)
                // QuadPawn just needs to know it exists and sensors are available.
                controller.Initialize(this, _config, sensorManager);
                _controllerReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: Controller initialized");
            }
            
            if (modeCoordinator != null && commandProxy != null)
            {
                modeCoordinator.Initialize(this, commandProxy);
                Debug.Log($"[QuadPawn] {DroneID}: ModeCoordinator initialized");
            }
            
            if (quadSimApi != null)
            {
                quadSimApi.Initialize(this, commandProxy, modeCoordinator, sensorManager);
                Debug.Log($"[QuadPawn] {DroneID}: QuadSimApi initialized");
            }
            _initialized = true;
            Debug.Log($"[QuadPawn] {DroneID}: Initialization complete");
        }
        
        public void PhysicsStep(double dtSec, long nowNanos)
        {
            if (!_initialized) return;

            float dt = (float)dtSec;

            // --- Step 1: Update Sensors ---
            if (sensorManager != null && _sensorsReady)
            {
                sensorManager.UpdateSensors(dtSec, nowNanos);
            }

            // --- Step 2: Run Active Controller ---
            if (_controllerReady)
            {
                RunController(dt);
            }

            // --- Step 3: Apply Buffered Wrench to Rigidbody ---
            ApplyBufferedForces();
        }

        private void RunController(float dt)
        {
            // Use ModeCoordinator as source of truth if available, else fall back to local field
            ControllerKind kind = modeCoordinator != null 
                ? modeCoordinator.ActiveController 
                : currentControllerKind;

            switch (kind)
            {
                case ControllerKind.Cascade:
                    if (controller != null)
                        controller.StepController(dt);
                    break;

                case ControllerKind.Geometric:
                    // Future: geoController.Update(sensorManager.Latest, dt);
                    break;
            }
        }

        // ============================================================================
        // Buffered Force Application (mirrors UE's SetBodyWrench / ApplyBufferedForces)
        // ============================================================================

        /// <summary>
        /// Set the net wrench to apply at next physics step.
        /// Called by controllers after computing control output.
        /// Force is body-frame Z thrust, Torque is body-frame (X,Y,Z).
        /// </summary>
        public void SetBodyWrench(Vector3 netForce, Vector3 netTorque)
        {
            _bufferedForce = netForce;
            _bufferedTorque = netTorque;
            _wrenchGeneration++;

            // Wake body if significant force
            if (rb != null && (netForce.sqrMagnitude > 1f || netTorque.sqrMagnitude > 0.1f))
            {
                rb.WakeUp();
            }
        }

        /// <summary>
        /// Apply buffered wrench to rigidbody. Body-local to world transform.
        /// Called at end of PhysicsStep, before Physics.Simulate().
        /// </summary>
        private void ApplyBufferedForces()
        {
            if (rb == null) return;
            if (_wrenchGeneration == _lastAppliedGeneration) return;

            Vector3 bodyForce = new Vector3(0, _bufferedForce.y, 0);            
            
            // Transform body-local force/torque to world space
            Vector3 worldForce = transform.TransformDirection(_bufferedForce);
            Vector3 worldTorque = transform.TransformDirection(_bufferedTorque);

            rb.AddForce(worldForce, ForceMode.Force);
            rb.AddTorque(worldTorque, ForceMode.Force);

            _lastAppliedGeneration = _wrenchGeneration;
        }

        // ============================================================================
        // Reset Functions
        // ============================================================================

        /// <summary>
        /// Full reset — position, physics, controller state.
        /// </summary>
        public void ResetDrone()
        {
            ResetPhysics();
            ResetControllerState();
            Debug.Log($"[QuadPawn] {DroneID}: Full reset complete");
        }

        /// <summary>
        /// Reset to specific position and rotation.
        /// </summary>
        public void ResetPosition(Vector3? targetPosition = null, Quaternion? targetRotation = null)
        {
            Vector3 pos = targetPosition ?? Vector3.up;
            Quaternion rot = targetRotation ?? Quaternion.identity;

            if (rb == null)
                rb = GetComponent<Rigidbody>();

            if (rb != null)
            {
                // Rigidbody is authoritative for physics + sensors.
                rb.position = pos;
                rb.rotation = rot;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;

                // Clear sleeping/contact weirdness after teleport.
                rb.Sleep();
                rb.WakeUp();
            }

            // Keep Transform synchronized too.
            transform.SetPositionAndRotation(pos, rot);
            Physics.SyncTransforms();

            // Clear buffered wrench/controller state.
            ResetPhysics();
            ResetControllerState();

            if (sensorManager != null)
                sensorManager.OnReset();

            Debug.Log($"[QuadPawn] {DroneID}: Position reset to {pos}, rb.position={(rb != null ? rb.position.ToString() : "NULL")}");
        }
        /// <summary>
        /// Level the drone (zero roll/pitch, keep yaw), zero velocities.
        /// </summary>
        public void ResetRotation()
        {
            if (rb == null) return;

            Vector3 euler = transform.eulerAngles;
            transform.rotation = Quaternion.Euler(0f, euler.y, 0f);

            rb.angularVelocity = Vector3.zero;
            rb.linearVelocity = Vector3.zero;
        }

        /// <summary>
        /// Zero all velocities without moving.
        /// </summary>
        public void ResetPhysics()
        {
            _activeColliders.Clear();
            _groundColliders.Clear();
            _pendingRLEvents.Clear();

            if (rb == null) return;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // Clear buffered wrench
            _bufferedForce = Vector3.zero;
            _bufferedTorque = Vector3.zero;
            _wrenchGeneration = 0;
            _lastAppliedGeneration = 0;

            rb.WakeUp();
        }

        /// <summary>
        /// Reset all controller and command state.
        /// </summary>
        public void ResetControllerState()
        {
            // Opaque reset — controller handles its own internals (PIDs, allocator, etc.)
            if (controller != null)
            {
                controller.ResetControllerState();
            }

            // if (commandProxy != null)
            // {
            //     commandProxy.ResetToSafe();
            // }

            // Clear any pending wrench
            _bufferedForce = Vector3.zero;
            _bufferedTorque = Vector3.zero;
        }

        // ============================================================================
        // Config Reload (mirrors UE)
        // ============================================================================

        public void ReapplyConfig()
        {
            if (_configApplied)
                ApplyConfig(_config);
        }

        public void ReloadYaml()
        {
            if (_droneManager != null)
                _droneManager.ReloadCurrentDrone(false);
        }

        // ============================================================================
        // Visual Updates (called by DroneManager from Update)
        // ============================================================================

        public void UpdateVisuals(float deltaTime)
        {
            UpdatePropellerVisuals(deltaTime);
            // Future: debug draw, effects
        }

        private void UpdatePropellerVisuals(float deltaTime)
        {
            if (propellerMeshes == null || propellerMeshes.Length != 4) return;
            if (controller == null) return;

            var motorOutputs = controller.LastMotor01;
            if (motorOutputs == null || motorOutputs.Count != 4) return;

            // Thrust model params from config or defaults
            float thrustCoef = 1e-5f;
            float propDiameter = 0.127f;
            float airDensity = 1.225f;
            float maxThrustPerMotor = 4.12f;

            if (_configApplied)
            {
                if (_config.RotorParams.Model.ThrustCoeff > 0)
                    thrustCoef = _config.RotorParams.Model.ThrustCoeff;
                if (_config.RotorParams.Model.Prop_Diameter > 0)
                    propDiameter = _config.RotorParams.Model.Prop_Diameter;
                if (_config.FlightParams.MaxThrust > 0)
                    maxThrustPerMotor = _config.FlightParams.MaxThrust / 4f;
            }

            float denom = thrustCoef * airDensity * Mathf.Pow(propDiameter, 4);
            if (denom < 1e-10f) return;

            // Spin direction from config or default X-config
            int[] spinDir = { 1, -1, -1, 1 };
            if (_configApplied && _config.RotorParams.Rotors != null && _config.RotorParams.Rotors.Length == 4)
            {
                for (int i = 0; i < 4; i++)
                    spinDir[i] = _config.RotorParams.Rotors[i].Spin_Dir;
            }

            for (int i = 0; i < 4; i++)
            {
                if (propellerMeshes[i] == null) continue;

                float thrust01 = Mathf.Max(0f, motorOutputs[i]);
                float thrustN = thrust01 * maxThrustPerMotor;
                float rpm = 60f * Mathf.Sqrt(thrustN / denom);
                _propellerRPMs[i] = rpm;

                float direction = spinDir[i];
                float frameRotation = (rpm / 60f) * 360f * deltaTime * direction;

                propellerMeshes[i].Rotate(0f, 0f, frameRotation, Space.Self);            }
        }

        // ============================================================================
        // Camera
        // ============================================================================

        public enum CameraMode { ThirdPerson, FPV }

        [Header("Camera")]
        [Tooltip("Drone view camera. Auto-found in children if unset.")]
        [SerializeField] private Camera droneCamera;

        public CameraMode CamMode { get; private set; } = CameraMode.ThirdPerson;

        private Transform _tpParent;
        private Vector3 _tpLocalPos;
        private Quaternion _tpLocalRot;
        private bool _camInit;

        private void EnsureCameraRig()
        {
            if (cameraRig == null)
                cameraRig = GetComponent<DroneCameraRig>();
            if (cameraRig == null)
                cameraRig = gameObject.AddComponent<DroneCameraRig>();
            cameraRig.Initialize(this, fpvCameraPoint);

            if (imageSensor == null)
                imageSensor = GetComponent<DroneImageSensor>();
            if (imageSensor == null)
                imageSensor = gameObject.AddComponent<DroneImageSensor>();
            imageSensor.Initialize(this, cameraRig);
        }

        private void InitCameraRefs()
        {
            if (_camInit) return;

            if (droneCamera == null)
            {
                // Prefer the canonical ThirdPerson camera by name; fall back to any child camera.
                Transform tp = transform.Find("ThirdPerson") ?? FindDeepChild(transform, "ThirdPerson");
                if (tp != null) droneCamera = tp.GetComponent<Camera>();
                if (droneCamera == null) droneCamera = GetComponentInChildren<Camera>(true);
            }
            if (droneCamera == null) return;

            _tpParent   = droneCamera.transform.parent;
            _tpLocalPos = droneCamera.transform.localPosition;
            _tpLocalRot = droneCamera.transform.localRotation;
            _camInit = true;
        }

        private static Transform FindDeepChild(Transform root, string name)
        {
            foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
                if (t.name == name) return t;
            return null;
        }

        /// <summary>This drone's camera (may be null if prefab has none).</summary>
        public Camera DroneCamera { get { InitCameraRefs(); return droneCamera; } }

        /// <summary>Mount-only camera data for the central CameraDirector.</summary>
        public DroneCameraRig CameraRig { get { EnsureCameraRig(); return cameraRig; } }

        /// <summary>Enable/disable this drone's camera (+AudioListener). DroneManager
        /// calls this on selection so only the selected drone renders.</summary>
        public void SetCameraActive(bool active)
        {
            CameraDirector director = CameraDirector.Get();
            if (director != null)
            {
                if (active) director.Bind(this);
                return;
            }

            InitCameraRefs();
            if (droneCamera == null) return;
            droneCamera.enabled = active;
            var listener = droneCamera.GetComponent<AudioListener>();
            if (listener != null) listener.enabled = active;
        }

        public void SetCameraMode(CameraMode mode)
        {
            CameraDirector director = CameraDirector.Get();
            if (director != null)
            {
                director.Bind(this, snap: false);
                director.SetMode(mode == CameraMode.FPV ? CameraViewMode.FPV : CameraViewMode.Follow);
                CamMode = mode;
                return;
            }

            InitCameraRefs();
            if (droneCamera == null) return;

            var camT = droneCamera.transform;
            if (mode == CameraMode.FPV)
            {
                Transform mount = fpvCameraPoint != null ? fpvCameraPoint : transform;
                camT.SetParent(mount, worldPositionStays: false);
                camT.localPosition = Vector3.zero;
                camT.localRotation = Quaternion.identity;
            }
            else
            {
                camT.SetParent(_tpParent, worldPositionStays: false);
                camT.localPosition = _tpLocalPos;
                camT.localRotation = _tpLocalRot;
            }
            CamMode = mode;
        }

        public void SwitchCamera()
            => SetCameraMode(CamMode == CameraMode.ThirdPerson ? CameraMode.FPV : CameraMode.ThirdPerson);

        public void ForceFPVCameraActive()
        {
            SetCameraActive(true);
            SetCameraMode(CameraMode.FPV);
        }

        public Transform GetFPVCameraPoint()
        {
            return CameraRig.FpvMount;
        }

        // ============================================================================
        // Physics Helpers
        // ============================================================================

        /// <summary>
        /// Body inertia tensor in SI (kg·m²). Unity already uses SI.
        /// </summary>
        public Vector3 GetBodyInertiaSI()
        {
            return rb != null ? rb.inertiaTensor : Vector3.zero;
        }
    }
}
