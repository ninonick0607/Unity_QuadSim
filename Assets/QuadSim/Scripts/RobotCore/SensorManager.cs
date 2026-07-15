using DroneCore;
using MathUtil;
using RobotCore.Sensors;
using SimCore;
using UnityEngine;
using RobotCore.Core_Utils;

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
        public double baroHz = 50.0;  
        public double magHz = 50.0;
        
        [Header("Noise")]
        // Keep enabled for PX4 SITL/HIL: PX4's sensor validator flags more
        // than 100 bit-identical samples as STALE even when timestamps advance.
        public bool enableNoise = false;

        [Header("Georeferencing (PX4 origin)")]
        public double originLatDeg  = 47.397742;  // PX4 SITL default home (Zurich)
        public double originLonDeg  = 8.545594;
        public double originHeightM = 488.0;
        
        [Header("Debug Logging")]
        public bool logSensors = true;
        public float logHz = 2f;

        private double _logAccum;
        
        public SimFrame TargetFrame = SimFrame.FLU; 
        private FrequencyLimiter _imuLimiter;
        private FrequencyLimiter _gpsLimiter;
        private FrequencyLimiter _baroLimiter;
        private FrequencyLimiter _magLimiter;
        private FrequencyLimiter _logLimiter;

        public GeoReferencingSystem Geo { get; private set; }
        public IMUSensor IMU { get; private set; }
        public GPSSensor GPS { get; private set; }
        public BaroSensor Baro { get; private set; }
        public MagSensor Mag { get; private set; }
        

        public SensorData Latest { get; private set; }
        public int noiseSeed = 12345; 
        public int ExecutionOrder => 50;
        public string DebugName => name;
        
        [ContextMenu("Toggle Output Frame  FLU <-> FRD")]
        public void ToggleOutputFrame()
        {
            SetOutputFrame((outputFrame == SimFrame.FRD) ? SimFrame.FLU : SimFrame.FRD);
        }

        /// <summary>
        /// Select the body-axis convention exposed to external clients.
        /// World-frame fields remain ENU. UnityBody is intentionally not exposed
        /// because vectors and pseudovectors require different chirality maps.
        /// </summary>
        public bool SetOutputFrame(SimFrame frame)
        {
            if (frame != SimFrame.FLU && frame != SimFrame.FRD)
                return false;

            outputFrame = frame;
            if (IMU != null) IMU.OutputFrame = frame;
            if (Mag != null) Mag.OutputFrame = frame;
            Debug.Log($"[SensorManager] outputFrame → {outputFrame}");
            return true;
        }
        
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
        
        /// <summary>Apply scene-level env BEFORE OnStart(). Single writer for seed/noise/origin.
        /// The +1/+2/+3/+4 per-sensor offsets are still applied from noiseSeed inside OnStart().</summary>
        public void Configure(int seed, bool noise, double latDeg, double lonDeg, double heightM,SimFrame frame)
        {
            noiseSeed     = seed;
            enableNoise   = noise;
            originLatDeg  = latDeg;
            originLonDeg  = lonDeg;
            originHeightM = heightM;
            outputFrame = frame;
            
        }

        public void ReseedForEpisode(int episodeSeed, int agentIndex)
        {
            noiseSeed = DeriveSeed(episodeSeed, agentIndex, 0);
            IMU?.Reseed(DeriveSeed(episodeSeed, agentIndex, 1));
            GPS?.Reseed(DeriveSeed(episodeSeed, agentIndex, 2));
            Baro?.Reseed(DeriveSeed(episodeSeed, agentIndex, 3));
            Mag?.Reseed(DeriveSeed(episodeSeed, agentIndex, 4));
        }

        private static int DeriveSeed(int episodeSeed, int agentIndex, int streamId)
        {
            unchecked
            {
                uint value = (uint)episodeSeed;
                value ^= 0x9E3779B9u * (uint)(agentIndex + 1);
                value ^= 0x85EBCA6Bu * (uint)(streamId + 1);
                value ^= value >> 16;
                value *= 0x7FEB352Du;
                value ^= value >> 15;
                value *= 0x846CA68Bu;
                value ^= value >> 16;
                return (int)value;
            }
        }
        public void OnStart()
        {
            ResolveRefsOrThrow();

            IMU  = new IMUSensor(rb, bodyTransform, noiseSeed + 1);
            GPS  = new GPSSensor(rb, noiseSeed + 2);

            Geo = SimCoreApi.Get().Geo;
            Geo.SetOrigin(originLatDeg, originLonDeg,originHeightM);
            
            Baro = new BaroSensor(rb, bodyTransform, noiseSeed + 3);
            
            Mag = new MagSensor(rb, bodyTransform, noiseSeed + 4);

            
            // outputFrame selects the BODY-axis convention only (FLU/FRD).
            // World-frame data (GPS local position) is always ENU.
            IMU.OutputFrame = outputFrame;
            Mag.OutputFrame = outputFrame;

            IMU.Reset();
            IMU.Initialize(rb);

            GPS.Reset();
            GPS.Initialize(Geo);

            Baro.Reset();           
            Baro.Initialize(Geo);  

            Mag.Reset();
            Mag.Initialize(Geo);
            
            long now = ClockFactory.Clock.NowNanos;
            _imuLimiter  = new FrequencyLimiter(imuHz, now);
            _gpsLimiter  = new FrequencyLimiter(gpsHz, now);
            _baroLimiter = new FrequencyLimiter(baroHz, now);   
            _magLimiter = new FrequencyLimiter(magHz, now);

            _logLimiter  = new FrequencyLimiter(logHz <= 0 ? 1e-6 : logHz, now);

            Latest = default;
            PrimeSnapshot(now);
        }

        public void OnReset()
        {
            IMU.Reset();
            IMU.Initialize(rb);
            
            GPS.Reset();
            GPS.Initialize(Geo);

            Baro.Reset();
            Baro.Initialize(Geo);
            
            Mag.Reset();
            Mag.Initialize(Geo);
            
            long now = ClockFactory.Clock.NowNanos;

            _imuLimiter?.Reset(now);
            _gpsLimiter?.Reset(now);
            _baroLimiter?.Reset(now);
            _magLimiter?.Reset(now);
            _logLimiter?.Reset(now);

            Latest = default;
            PrimeSnapshot(now);
        }

        /// <summary>
        /// Populate the cached snapshot immediately without advancing physics.
        /// This guarantees that the first SDK get_sensors() call is valid even
        /// when the simulation starts or remains paused.
        /// </summary>
        private void PrimeSnapshot(long nowNanos)
        {
            double dtSec = ClockFactory.BaseDtSec;
            if (dtSec <= 0.0)
                dtSec = 1.0 / 250.0;

            UpdateSensors(dtSec, nowNanos);
        }

        public void UpdateSensors(double dtSec, long nowNanos)
        {
            double nowSec = nowNanos * 1e-9;

            // Keep BODY output frame synced (IMU + mag). GPS position is ENU always.
            IMU.OutputFrame = outputFrame;
            Mag.OutputFrame = outputFrame;

            var latest = Latest;
            latest.BodyFrame = outputFrame;   // self-description for the RPC payload

            // IMU
            if (_imuLimiter.ShouldRunAndConsume(nowNanos))
            {
                IMU.Sample(nowSec, (float)dtSec, enableNoise);   // was: IMU.Sample(nowSec, (float)dtSec);

                latest.ImuAngVel = IMU.LastAngVel;
                latest.ImuAttitude = IMU.LastAtt;
                latest.ImuOrientation = IMU.LastOrientation; 
                latest.ImuAccel = IMU.LastLinAcc;
                latest.ImuVel = IMU.LastVel;
                latest.ImuTimestampSec = IMU.LastTimestampSec;
                latest.ImuValid = IMU.IsValid;
            }

            // GPS — local position is always world-frame ENU from GPSSensor
            if (_gpsLimiter.ShouldRunAndConsume(nowNanos))
            {
                GPS.Sample(nowSec, enableNoise);

                latest.GpsPosition     = GPS.LastPosition;
                latest.GpsTimestampSec = GPS.LastTimestampSec;
                latest.GpsValid        = GPS.HasFix;

                latest.GpsLatDeg = GPS.LastLatDeg;
                latest.GpsLonDeg = GPS.LastLonDeg;
                latest.GpsAltM   = GPS.LastAltM;
                latest.GpsVelNED = GPS.LastVelNED;
            }

            // Baro
            if (_baroLimiter.ShouldRunAndConsume(nowNanos))
            {
                Baro.Sample(nowSec, enableNoise);

                latest.BaroPressurePa       = Baro.LastPressure;
                latest.BaroPressureHPa      = Baro.LastPressureHPa;
                latest.BaroTemperatureC     = Baro.LastTemperature;
                latest.BaroAltitudeMSL      = Baro.EstimatedAltitude;
                latest.BaroPressureAltitude = Baro.PressureAltitude;
                latest.BaroTimestampSec     = Baro.LastTimestampSec;
                latest.BaroValid            = Baro.IsValid;
            }
            // Mag
            if (_magLimiter.ShouldRunAndConsume(nowNanos))
            {
                Mag.Sample(nowSec, (float)dtSec, enableNoise);

                latest.MagField          = Mag.LastMagField;
                latest.MagHeadingDeg     = Mag.MagneticHeading;
                latest.MagDeclinationDeg = Mag.MagneticDeclination;
                latest.MagTimestampSec   = Mag.LastTimestampSec;
                latest.MagValid          = Mag.IsValid;
            }
            
            Latest = latest;
        }
    }
}
