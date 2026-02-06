using System.Collections.Generic;
using DroneCore.Common;
using DroneCore.Controllers.Cascades;
using DroneCore.Interfaces;
using RobotCore;
using SimCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers
{
    /// <summary>
    /// Cascaded flight controller implementing Rate (Acro) mode.
    /// Runs the control loop: Command -> Rate PID -> Allocator -> Motor outputs.
    /// 
    /// Implements ISimulatable to receive physics-synchronized updates from SimulationManager.
    /// </summary>
    [RequireComponent(typeof(QuadPawn))]
    [DisallowMultipleComponent]
    public sealed class CascadedController : MonoBehaviour, ISimulatable
    {
        // ============================================================================
        // Serialized References
        // ============================================================================
        
        [SerializeField] private QuadPawn body;
        [SerializeField] private ThrusterSet thrusters;
        [SerializeField] private SensorManager sensorManager;
        [SerializeField] private FlightCommandProxy commandProxy;

        // ============================================================================
        // Altitude Hold
        // ============================================================================
        
        [Header("Altitude Hold (world Y meters)")]
        public bool altitudeHoldEnabled = true;
        public float desiredAltitudeM = 1.0f;

        [Header("Baseline hover throttle")]
        [Range(0, 1)] public float hoverThrottle01 = 0.75f;
        public float maxThrottleDelta = 0.25f;

        [Header("Z PID")]
        public QuadPIDController zPid = new QuadPIDController
        {
            Kp = 1.5f, Ki = 0.2f, Kd = 0.4f,
            MinOutput = -0.25f, MaxOutput = 0.25f,
            IntegralLimit = 2.0f, DecayFactor = 0.99f
        };

        // ============================================================================
        // Rate (Acro) Controller
        // ============================================================================
        
        [Header("Acro (Rate) Controller")]
        public AcroController acro = new AcroController();

        [Header("Desired body rates (deg/s) - for UI/debug input")]
        public Vector3 desiredRatesDeg = Vector3.zero; // (roll, pitch, yaw)

        // ============================================================================
        // Effectiveness Allocator
        // ============================================================================
        
        [Header("Effectiveness Allocator")]
        public bool useEffectivenessAllocator = true;
        public EffectivenessAllocator4 allocator = new EffectivenessAllocator4();

        [Tooltip("Max roll/pitch rate used to normalize PID output into [-1..1] torque demand.")]
        public float maxAngleRateRad = 6.0f; // ~343 deg/s

        [Tooltip("Max yaw rate used to normalize PID output into [-1..1] torque demand.")]
        public float maxYawRateRad = 3.0f; // ~172 deg/s

        [Tooltip("Scale yaw authority (start small).")]
        public float yawAuthorityScale = 0.03f;

        // ============================================================================
        // Runtime State
        // ============================================================================
        
        private readonly float[] _motor = new float[4];
        private bool _initialized;
        
        /// <summary>Last computed motor outputs [0..1].</summary>
        public IReadOnlyList<float> LastMotor01 => _motor;
        
        /// <summary>Desired body rates in radians/sec.</summary>
        public Vector3 DesiredRatesRad => desiredRatesDeg * Mathf.Deg2Rad;
        
        /// <summary>Measured body rates in radians/sec from sensors.</summary>
        public Vector3 MeasuredRatesRad => sensorManager != null && sensorManager.Latest.ImuValid 
            ? sensorManager.Latest.ImuAngVel 
            : Vector3.zero;

        // ============================================================================
        // ISimulatable Implementation
        // ============================================================================
        
        public int ExecutionOrder => 100; // Run after sensors (~50), before rendering
        public string DebugName => name;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            AutoWireReferences();
            InitializeGains();
        }

        private void AutoWireReferences()
        {
            if (body == null) body = GetComponent<QuadPawn>();
            if (thrusters == null) thrusters = GetComponent<ThrusterSet>();
            if (sensorManager == null) sensorManager = GetComponent<SensorManager>() ?? GetComponentInParent<SensorManager>();
            if (commandProxy == null) commandProxy = GetComponent<FlightCommandProxy>();
        }

        private void InitializeGains()
        {
            // Default gains matching typical quad dynamics
            // These should eventually come from DroneConfig
            
            acro.roll.SetGains(0.35f, 0.02f, 0.0001f);
            acro.pitch.SetGains(0.35f, 0.02f, 0.0001f);
            acro.yaw.SetGains(1.0f, 0.0f, 0.0f);

            acro.roll.SetLimits(-0.35f, 0.35f);
            acro.pitch.SetLimits(-0.5f, 0.5f);
            acro.yaw.SetLimits(-0.5f, 0.5f);

            acro.roll.SetIntegralLimits(2.0f);
            acro.pitch.SetIntegralLimits(2.0f);
            acro.yaw.SetIntegralLimits(2.0f);
        }

        // ============================================================================
        // ISimulatable - Simulation Lifecycle
        // ============================================================================

        public void OnSimulationStart(SimulationManager sim)
        {
            AutoWireReferences();
            
            if (body == null)
            {
                Debug.LogError($"[CascadedController] {name}: QuadPawn reference missing!");
                return;
            }

            // Reset all PIDs
            ResetControllerState();

            // Prevent Unity angular velocity clamping
            if (body.Rigidbody != null)
            {
                body.Rigidbody.maxAngularVelocity = 100f;
            }

            // Build effectiveness allocator from motor geometry
            if (useEffectivenessAllocator && thrusters != null)
            {
                allocator.Build(body.transform, body.Motors, thrusters.SpinDirection);
                Debug.Log($"[CascadedController] {name}: Allocator built");
            }

            _initialized = true;
            Debug.Log($"[CascadedController] {name}: OnSimulationStart complete");
        }

        public void OnSimulationReset(SimulationManager sim)
        {
            ResetControllerState();
            Debug.Log($"[CascadedController] {name}: OnSimulationReset");
        }

        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            if (!_initialized) return;
            
            float dt = (float)dtSec;
            FlightController(dt);
        }

        public void PostPhysicsStep(double dtSec, long nowNanos)
        {
            // Nothing needed post-physics for the controller
        }

        // ============================================================================
        // Reset (called by QuadPawn or DroneManager)
        // ============================================================================

        /// <summary>
        /// Reset all controller state (PIDs, motor outputs).
        /// </summary>
        public void ResetControllerState()
        {
            zPid.Reset();
            acro.Reset();
            
            // Zero motor outputs
            for (int i = 0; i < 4; i++)
            {
                _motor[i] = 0f;
            }

            // Zero desired rates
            desiredRatesDeg = Vector3.zero;

            Debug.Log($"[CascadedController] {name}: Controller state reset");
        }

        // ============================================================================
        // Flight Controller Core
        // ============================================================================

        /// <summary>
        /// Main control loop. Called from PrePhysicsStep.
        /// </summary>
        public void FlightController(float deltaTime)
        {
            if (commandProxy == null)
            {
                Debug.LogError("[CascadedController] commandProxy is null");
                return;
            }

            // Latch command inputs for this step
            commandProxy.LatchForStep();
            var snap = commandProxy.GetSnapshot();

            // --- Altitude Hold (Z axis) ---
            float throttle = hoverThrottle01;
            if (altitudeHoldEnabled && body != null && body.Rigidbody != null)
            {
                float y = body.Rigidbody.position.y;
                float delta = zPid.Calculate(desiredAltitudeM, y, deltaTime);
                delta = Mathf.Clamp(delta, -maxThrottleDelta, maxThrottleDelta);
                throttle = Mathf.Clamp01(hoverThrottle01 + delta);
            }

            // --- Rate Controller (Roll/Pitch/Yaw) ---
            Vector3 measuredRates = Vector3.zero;
            if (sensorManager != null)
            {
                var s = sensorManager.Latest;
                if (s.ImuValid)
                {
                    measuredRates = s.ImuAngVel; // rad/s
                }
            }

            // Get desired rates from command proxy (assumes deg/s, convert to rad/s)
            Vector3 desiredRates = snap.Command.XYZ * Mathf.Deg2Rad;

            // Run rate PIDs
            Vector3 rateOut = acro.Update(desiredRates, measuredRates, deltaTime);

            // Normalize to torque demands [-1..1]
            float rollTorque  = Mathf.Clamp(rateOut.x / maxAngleRateRad, -1f, 1f);
            float pitchTorque = Mathf.Clamp(rateOut.y / maxAngleRateRad, -1f, 1f);
            float yawTorque   = Mathf.Clamp(rateOut.z / maxYawRateRad, -1f, 1f) * yawAuthorityScale;

            // --- Motor Allocation ---
            if (useEffectivenessAllocator)
            {
                allocator.Allocate(rollTorque, pitchTorque, yawTorque, throttle, _motor);
            }
            else
            {
                // Fallback: simple + mixer (for testing)
                SimpleMixer(rollTorque, pitchTorque, yawTorque, throttle);
            }

            // --- Apply to Thrusters ---
            if (thrusters != null)
            {
                thrusters.SetAllMotorCommands01(_motor[0], _motor[1], _motor[2], _motor[3]);
                thrusters.ApplyForces();
            }
        }

        /// <summary>
        /// Simple + mixer fallback (X-config quad).
        /// </summary>
        private void SimpleMixer(float roll, float pitch, float yaw, float throttle)
        {
            // Motor order: FL, FR, BL, BR
            // X-config mixing:
            //   FL = throttle + pitch + roll - yaw
            //   FR = throttle + pitch - roll + yaw
            //   BL = throttle - pitch + roll + yaw
            //   BR = throttle - pitch - roll - yaw
            
            _motor[0] = Mathf.Clamp01(throttle + pitch + roll - yaw); // FL
            _motor[1] = Mathf.Clamp01(throttle + pitch - roll + yaw); // FR
            _motor[2] = Mathf.Clamp01(throttle - pitch + roll + yaw); // BL
            _motor[3] = Mathf.Clamp01(throttle - pitch - roll - yaw); // BR
        }

        // ============================================================================
        // Configuration (called by QuadPawn.ApplyConfig)
        // ============================================================================

        /// <summary>
        /// Apply gains from a DroneConfig. Called when config is loaded.
        /// </summary>
        public void ApplyConfig(DroneConfig config)
        {
            // Apply Acro (Rate) PID gains
            if (config.Acro.AcroRoll.P != 0 || config.Acro.AcroRoll.I != 0 || config.Acro.AcroRoll.D != 0)
            {
                acro.roll.SetGains(config.Acro.AcroRoll.P, config.Acro.AcroRoll.I, config.Acro.AcroRoll.D);
            }
            if (config.Acro.AcroPitch.P != 0 || config.Acro.AcroPitch.I != 0 || config.Acro.AcroPitch.D != 0)
            {
                acro.pitch.SetGains(config.Acro.AcroPitch.P, config.Acro.AcroPitch.I, config.Acro.AcroPitch.D);
            }
            if (config.Acro.AcroYaw.P != 0 || config.Acro.AcroYaw.I != 0 || config.Acro.AcroYaw.D != 0)
            {
                acro.yaw.SetGains(config.Acro.AcroYaw.P, config.Acro.AcroYaw.I, config.Acro.AcroYaw.D);
            }

            // Apply flight limits from config
            if (config.FlightParams.MaxRateRollPitch > 0)
            {
                maxAngleRateRad = config.FlightParams.MaxRateRollPitch * Mathf.Deg2Rad;
            }
            if (config.FlightParams.MaxRateYaw > 0)
            {
                maxYawRateRad = config.FlightParams.MaxRateYaw * Mathf.Deg2Rad;
            }

            // Apply Z PID from Position config (using PosZ for altitude hold)
            if (config.Position.PosZ.P != 0 || config.Position.PosZ.I != 0 || config.Position.PosZ.D != 0)
            {
                zPid.SetGains(config.Position.PosZ.P, config.Position.PosZ.I, config.Position.PosZ.D);
            }

            Debug.Log($"[CascadedController] {name}: Config applied - Acro gains: R({acro.roll.Kp:F3},{acro.roll.Ki:F3},{acro.roll.Kd:F4}) P({acro.pitch.Kp:F3},{acro.pitch.Ki:F3},{acro.pitch.Kd:F4}) Y({acro.yaw.Kp:F3},{acro.yaw.Ki:F3},{acro.yaw.Kd:F4})");
        }
    }
}