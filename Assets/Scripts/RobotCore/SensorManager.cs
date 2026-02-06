using DroneCore;
using MathUtil;
using RobotCore.Sensors;
using SimCore;
using UnityEngine;

namespace RobotCore
{
    [RequireComponent(typeof(QuadPawn))]
    [DisallowMultipleComponent]
    public sealed class SensorManager : MonoBehaviour
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
        
        public SimFrame TargetFrame = SimFrame.FLU; 

        private FrequencyLimiter _imuLimiter;
        private FrequencyLimiter _gpsLimiter;
        private FrequencyLimiter _logLimiter;

        public IMUSensor IMU { get; private set; }
        public GPSSensor GPS { get; private set; }

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
        }

        public void OnStart()
        {
            ResolveRefsOrThrow();
            
            IMU = new IMUSensor(rb, bodyTransform);
            GPS = new GPSSensor(rb);
            IMU.OutputFrame = outputFrame;
            IMU.Reset();
            IMU.Initialize(rb);

            
            GPS.Reset();
            GPS.Initialize();

            long now = 0; 
            _imuLimiter = new FrequencyLimiter(imuHz, now);
            _gpsLimiter = new FrequencyLimiter(gpsHz, now);
            _logLimiter = new FrequencyLimiter(logHz <= 0 ? 1e-6 : logHz, now);
            Latest = default;
        }

        public void OnReset()
        {
            IMU.Reset();
            GPS.Reset();

            IMU.Initialize(rb);
            GPS.Initialize();

            Latest = default;
        }

        public void UpdateSensors(double dtSec, long nowNanos)
        {
            double nowSec = nowNanos * 1e-9;

            // keep output frame synced
            IMU.OutputFrame = outputFrame;

            var latest = Latest;

            // IMU
            if (_imuLimiter.ShouldRunAndConsume(nowNanos))
            {
                IMU.Sample(nowSec, (float)dtSec);

                latest.ImuAngVel = IMU.LastAngVel;
                latest.ImuAttitude = IMU.LastAtt;
                latest.ImuOrientation = IMU.LastOrientation; 
                latest.ImuAccel = IMU.LastLinAcc;
                latest.ImuVel = IMU.LastVel;
                latest.ImuTimestampSec = IMU.LastTimestampSec;
                latest.ImuValid = IMU.IsValid;

            }

            // GPS
            if (_gpsLimiter.ShouldRunAndConsume(nowNanos))
            {
                GPS.Sample(nowSec);

                latest.GpsPosition = GPS.LastPositionWorldM;
                latest.GpsTimestampSec = GPS.LastTimestampSec;
                latest.GpsValid = GPS.HasFix;
            }

            Latest = latest;
        }
        
    }
}
