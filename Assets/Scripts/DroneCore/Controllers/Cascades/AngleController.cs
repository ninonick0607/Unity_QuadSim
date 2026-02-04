using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore; // SensorManager, SensorData
using UnityEngine;

namespace DroneCore.Controllers.Cascades
{
    [System.Serializable]
    public sealed class AngleController // per-axis, UE-style
    {
        private int _axis;
        private ICommandSource _cmd;
        private SensorManager _sensors;

        [SerializeField] private QuadPIDController anglePid = new QuadPIDController();

        // Child per-axis rate controller (single-axis version)
        [SerializeField] private AcroController childRateController = new AcroController();

        private bool _useExternal;
        private float _externalGoal;
        private float _output;

        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors,
            // angle pid params
            float angleMinOut, float angleMaxOut, float kp, float ki, float kd, float iLimit,
            // rate pid params
            float rateMinOut, float rateMaxOut, float rKp, float rKi, float rKd, float rILimit)
        {
            _axis = axis;
            _cmd = cmd;
            _sensors = sensors;

            anglePid.SetLimits(angleMinOut, angleMaxOut);
            anglePid.SetGains(kp, ki, kd);
            if (iLimit > 0f) anglePid.SetIntegralLimits(iLimit);

            childRateController.Initialize(axis,cmd,sensors, rateMinOut, rateMaxOut, rKp, rKi, rKd, rILimit);
        }

        public void Reset()
        {
            anglePid.Reset();
            childRateController.Reset();
            _useExternal = false;
            _externalGoal = 0f;
            _output = 0f;
        }

        public void SetExternalGoal(float v) { _externalGoal = v; _useExternal = true; }
        public void ClearExternalGoal() { _useExternal = false; }

        public void Update(float dt)
        {
            // 1) Read command for this axis
            var mode = _cmd.Mode;
            var currentState = _cmd.Command[_axis];

            float goal = _useExternal ? _externalGoal : currentState;

            // 2) Read sensors (stable for step if SensorManager runs earlier)
            SensorData s = _sensors.Latest; // struct copy
            if (!s.ImuValid) { _output = 0f; return; } // or hold last output

            // 3) UE-matching special cases:
            // yaw axis treated as rate goal
            if (_axis == 2)
            {
                _output = childRateController.Update(dt);
                return;
            }

            // throttle passthrough (usually handled outside, but parity is fine)
            if (_axis == 3)
            {
                _output = goal;
                return;
            }

            // roll/pitch: angle -> rate -> output
            float currentAngle = s.ImuAttitude[_axis];
            float desiredRate = anglePid.Calculate(goal, currentAngle, dt);

            float measuredRate = s.ImuAngVel[_axis];
            _output = childRateController.Update(dt);
        }

        public float GetOutput() => _output;
        public QuadPIDController GetPID() => anglePid;
    }
}
