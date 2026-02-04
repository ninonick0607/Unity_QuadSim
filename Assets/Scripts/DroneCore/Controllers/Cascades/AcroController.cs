using DroneCore.Common;
<<<<<<< HEAD
=======
using DroneCore.Interfaces;
using RobotCore;
>>>>>>> 714525f (Lots of additions since last build, trying to match unreals model)
using UnityEngine;

namespace DroneCore.Controllers.Cascades
{
<<<<<<< HEAD
    /// <summary>
    /// Body-rate controller: desired rates -> (rollMix, pitchMix, yawMix).
    /// Uses QuadPidController math.
    /// </summary>
    [System.Serializable]
    public sealed class AcroController
    {
        public QuadPIDController roll = new QuadPIDController();
        public QuadPIDController pitch = new QuadPIDController();
        public QuadPIDController yaw = new QuadPIDController();

        public void Reset()
        {
            roll.Reset();
            pitch.Reset();
            yaw.Reset();
        }

        /// <summary>
        /// All rates in rad/s. Output are mix terms (dimensionless) for the motor mixer.
        /// </summary>
        public Vector3 Update(Vector3 desiredRatesRad, Vector3 measuredRatesRad, float dt)
        {
            float rollMix  = roll.Calculate(desiredRatesRad.x, measuredRatesRad.x, dt);
            float pitchMix = pitch.Calculate(desiredRatesRad.y, measuredRatesRad.y, dt);
            float yawMix   = yaw.Calculate(desiredRatesRad.z, measuredRatesRad.z, dt);
            return new Vector3(rollMix, pitchMix, yawMix);
        }
    }
}
=======
    [System.Serializable]
    public sealed class AcroController
    {
        [SerializeField] private QuadPIDController pid = new QuadPIDController();

        // Axis index: 0=roll, 1=pitch, 2=yaw, 3=throttle (pass-through)
        public int Axis { get; private set; }
        private ICommandSource _cmd;
        private SensorManager _sensors;
        
        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors, float minOut, float maxOut, float kp, float ki, float kd, float iLimit = 0f)
        {
            Axis = axis;
            _cmd = cmd;
            _sensors = sensors;
            
            pid.SetLimits(minOut, maxOut);
            pid.SetGains(kp, ki, kd);
            if (iLimit > 0f) pid.SetIntegralLimits(iLimit);
        }

        public void Reset() => pid.Reset();

        /// <summary>
        /// desiredRateDegPerSec and measuredRateDegPerSec are deg/s (match your UE semantics).
        /// For throttle axis (3), returns desired directly.
        /// </summary>
        public float Update(float deltaTime)
        {
            SensorData stateData = _sensors.Latest;
            float desiredRate = _cmd.Command[Axis];
            float currentRate = stateData.ImuAngVel[Axis];
            
            if (Axis == 3) return desiredRate;
            return pid.Calculate(desiredRate, currentRate, deltaTime);
        }

        public QuadPIDController Pid => pid;
    }
}
>>>>>>> 714525f (Lots of additions since last build, trying to match unreals model)
