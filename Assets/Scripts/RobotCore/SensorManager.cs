using UnityEngine;
using QuadSim.SimCore;
using QuadSim.MathUtil;

namespace QuadSim.RobotCore
{
    public sealed class SensorManager : MonoBehaviour, ISimulatable
    {
        [Header("Refs")]
        [SerializeField] private Rigidbody rb;
        [SerializeField] private Transform bodyTransform;

        [Header("Output Frame")]
        public SimFrame outputFrame = SimFrame.FLU;

        [Header("Rates (Hz)")]
        public double imuHz = 250.0;
        public double gpsHz = 10.0;

        [Header("Debug Logging")]
        public bool logSensors = true;
        public float logHz = 2f;

        private double _logAccum;

        private FrequencyLimiter _imuLimiter;
        private FrequencyLimiter _gpsLimiter;
        private FrequencyLimiter _logLimiter;

        public readonly IMUSensor IMU = new IMUSensor();
        public readonly GPSSensor GPS = new GPSSensor();

        public SensorData Latest { get; private set; }

        public int ExecutionOrder => 50;
        public string DebugName => name;

        private void ResolveRefsOrThrow()
        {
            if (rb == null) rb = GetComponentInParent<Rigidbody>();
            if (rb == null)
                throw new System.Exception($"SensorManager on '{name}' cannot find Rigidbody in parents. Put it under DroneRoot.");

            if (bodyTransform == null) bodyTransform = rb.transform;
        }

        private void Awake()
        {
            ResolveRefsOrThrow();
            IMU.OutputFrame = outputFrame;
        }

        public void OnSimulationStart(SimulationManager sim)
        {
            ResolveRefsOrThrow();

            // Create clock-driven limiters using sim time.
            // Start them at t=now so first ShouldRunAndConsume will fire immediately.
            long now = 0; // safe because your limiter runs immediately on first call anyway
            _imuLimiter = new FrequencyLimiter(imuHz, now);
            _gpsLimiter = new FrequencyLimiter(gpsHz, now);
            _logLimiter = new FrequencyLimiter(logHz <= 0 ? 1e-6 : logHz, now);

            IMU.OutputFrame = outputFrame;
            IMU.Reset();
            GPS.Reset();

            IMU.Initialize(rb);
            GPS.Initialize();

            Latest = default;
        }

        public void OnSimulationReset(SimulationManager sim)
        {
            IMU.Reset();
            GPS.Reset();

            IMU.Initialize(rb);
            GPS.Initialize();

            Latest = default;
        }

        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            double nowSec = nowNanos * 1e-9;

            // keep output frame synced
            IMU.OutputFrame = outputFrame;

            var latest = Latest;

            // IMU
            if (_imuLimiter.ShouldRunAndConsume(nowNanos))
            {
                IMU.Sample(rb, bodyTransform, nowSec, (float)dtSec);

                latest.imuAngVelRad = IMU.LastAngVel;
                latest.imuLinAccMS2 = IMU.LastLinAcc;
                latest.imuTimestampSec = IMU.LastTimestampSec;
                latest.imuValid = IMU.IsValid;
            }

            // GPS
            if (_gpsLimiter.ShouldRunAndConsume(nowNanos))
            {
                GPS.Sample(rb, nowSec);

                latest.gpsPositionM = GPS.LastPositionWorldM;
                latest.gpsTimestampSec = GPS.LastTimestampSec;
                latest.gpsValid = GPS.HasFix;
            }

            Latest = latest;

            // debug log at logHz using same limiter style
            if (logSensors && _logLimiter.ShouldRunAndConsume(nowNanos))
            {
                var s = Latest;
                Vector3 gyroDeg = s.imuAngVelRad * Mathf.Rad2Deg;
                Vector3 eulerWorldDeg = bodyTransform.rotation.eulerAngles;

                Debug.Log(
                    $"[Sensors {outputFrame}] " +
                    $"gyro(deg/s)={gyroDeg} acc(m/s^2)={s.imuLinAccMS2} " +
                    $"rotWorld(deg)=(R={eulerWorldDeg.x:F1}, P={eulerWorldDeg.y:F1}, Y={eulerWorldDeg.z:F1}) " +
                    $"GPS posW(m)={s.gpsPositionM}"
                );



            }
        }

        public void PostPhysicsStep(double dtSec, long nowNanos) { }
    }
}
