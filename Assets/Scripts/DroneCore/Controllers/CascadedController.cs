using System.Collections.Generic;
using DroneCore.Common;
using DroneCore.Controllers.Cascades;
using RobotCore;
using SimCore;
using UnityEngine;

namespace DroneCore.Controllers
{
    [RequireComponent(typeof(DroneBody))]

    public sealed class CascadedController : MonoBehaviour, ISimulatable
    {
        [SerializeField] private DroneBody body;
        [SerializeField] private ThrusterSet thrusters;
        [SerializeField] private SensorManager sensorManager;

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

        [Header("Acro (Rate) Controller")]
        public AcroController acro = new AcroController();

        [Header("Desired body rates (deg/s) (temporary test input)")]
        public Vector3 desiredRatesDeg = Vector3.zero; // (roll,pitch,yaw)

        [Header("Effectiveness Allocator (UE-style)")]
        public bool useEffectivenessAllocator = true;

        public EffectivenessAllocator4 allocator = new EffectivenessAllocator4();

        [Tooltip("Max roll/pitch rate used to normalize PID output into [-1..1] torque demand.")]
        public float maxAngleRateRad = 6.0f; // example: ~343 deg/s

        [Tooltip("Max yaw rate used to normalize PID output into [-1..1] torque demand.")]
        public float maxYawRateRad = 3.0f;   // example: ~172 deg/s

        [Tooltip("Scale yaw authority like your UE yawAuthorityScale (start small).")]
        public float yawAuthorityScale = 0.03f;

        private readonly float[] _motor = new float[4];
        public IReadOnlyList<float> LastMotor01 => _motor;
        public Vector3 DesiredRatesRad => desiredRatesDeg * Mathf.Deg2Rad;
        public Vector3 MeasuredRatesRad => sensorManager != null && sensorManager.Latest.ImuValid ? sensorManager.Latest.ImuAngVel : Vector3.zero;

        public int ExecutionOrder => 100;
        public string DebugName => name;

        private void Awake()
        {
            if (body == null) body = GetComponent<DroneBody>();
            if (thrusters == null) thrusters = GetComponent<ThrusterSet>();
            if (sensorManager == null) sensorManager = GetComponent<SensorManager>() ?? GetComponentInParent<SensorManager>();

            // Initialize gains from your YAML defaults (rate_gains)
            // Outputs are mix terms; limits are important.
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

        public void OnSimulationStart(SimulationManager sim)
        {
            if (body == null) body = GetComponent<DroneBody>();
            if (thrusters == null) thrusters = GetComponent<ThrusterSet>();

            zPid.Reset();
            acro.Reset();

            // Prevent Unity angular velocity clamping from wrecking rate control.
            body.Rigidbody.maxAngularVelocity = 100f;

            if (useEffectivenessAllocator)
            {
                allocator.Build(body.transform, body.Motors, thrusters.SpinDirection);
            }
        }

        public void OnSimulationReset(SimulationManager sim)
        {
            zPid.Reset();
            acro.Reset();
        }

        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            float dt = (float)dtSec;

            float throttle = hoverThrottle01;
            if (altitudeHoldEnabled && body != null)
            {
                float y = body.Rigidbody.position.y;
                float delta = zPid.Calculate(desiredAltitudeM, y, dt);
                delta = Mathf.Clamp(delta, -maxThrottleDelta, maxThrottleDelta);
                throttle = Mathf.Clamp01(hoverThrottle01 + delta);
            }

            Vector3 measuredRates = Vector3.zero;
            var s = sensorManager.Latest;
            if (s.ImuValid)
                measuredRates = s.ImuAngVel;

            Vector3 desiredRates = desiredRatesDeg * Mathf.Deg2Rad;

            Vector3 rateOut = acro.Update(desiredRates, measuredRates, dt);

            float rollTorque  = Mathf.Clamp(rateOut.x / maxAngleRateRad, -1f, 1f);
            float pitchTorque = Mathf.Clamp(rateOut.y / maxAngleRateRad, -1f, 1f);
            float yawTorque   = Mathf.Clamp(rateOut.z / maxYawRateRad, -1f, 1f) * yawAuthorityScale;


            allocator.Allocate(rollTorque, pitchTorque, yawTorque, throttle, _motor);

            thrusters.SetAllMotorCommands01(_motor[0], _motor[1], _motor[2], _motor[3]);
            thrusters.ApplyForces();
            
        }

        public void PostPhysicsStep(double dtSec, long nowNanos) { }
    }
}
