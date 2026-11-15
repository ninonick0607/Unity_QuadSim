using System;
using DroneCore.Common;
using DroneCore.Interfaces;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    public class VelocityController
    {
        private const float Gravity = 9.81f;

        // Keep half of hover lift available while braking an ascent. A full
        // -g request turns the motors effectively off until the vehicle slows,
        // which presents as a stall after a large vertical velocity step.
        public const float ClosedLoopMinLiftFraction = 0.5f;

        [SerializeField] private QuadPIDController _pidSet = new QuadPIDController();
        public QuadPIDController Pid => _pidSet;

        private int _axis;
        private ICommandSource _cmdGoal;
        private VehicleStateHolder _state;
        private DroneConfig _config;
        private float _output;
        private float _externalGoal;
        private bool _bInitialized;
        private bool _bUseExternalGoal;
        private float maxAcceleration;
        public VelocityController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, VehicleStateHolder state)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _state = state;
            VelocityPID PID = _config.Velocity;

            float maxAccelLimit = Mathf.Tan(_config.FlightParams.MaxAngle * Mathf.Deg2Rad) * Gravity;
            if (axis == 3)
            {
                float maxDownwardAccel = Gravity * (1f - ClosedLoopMinLiftFraction);
                _pidSet.SetLimits(-maxDownwardAccel, Gravity);
            }
            else
            {
                _pidSet.SetLimits(-maxAccelLimit, maxAccelLimit);
            }
            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);

            _bInitialized = true;
        }
        
        public void Update(float deltaTime)
        {
            if (!_bInitialized || _state == null || _pidSet == null || (_cmdGoal == null && !_bUseExternalGoal))
            {
                _output = 0.0f;
                return;
            }

            float Goal = _bUseExternalGoal ? _externalGoal :  _cmdGoal.GetCommandValue()[_axis];

            if (_axis == 2)
            {
                _output = Goal;
                return;
            }

            // Velocity in the level heading frame (forward, left, up): the frame
            // the setpoints and this loop's acceleration output are expressed in.
            // Roll and pitch do not rotate horizontal motion into the vertical axis.
            Vector3 v = _state.Current.HeadingVelocity;
            float StateVel = _axis == 0 ? v.x : _axis == 1 ? v.y : v.z;
            _output = _pidSet.Calculate(Goal, StateVel, deltaTime);
            
        }
        
        public void Reset()
        {
            if(_pidSet!=null) _pidSet.Reset();
            _output = 0.0f;
            _bUseExternalGoal = false;
        }
        public void SetExternalGoal(float Value)
        {
            _externalGoal = Value;
            _bUseExternalGoal = true;
        }
        public void ClearExternalGoal()
        {
            // TODO: Double check using this is actually needed or if its updated during runtime
            _externalGoal = 0.0f;
            _bUseExternalGoal = false;
        }
        public float GetOutput() { return _output; }
        public float GetExternalGoal() { return _externalGoal; }
        public QuadPIDController GetPID()
        {
            return _pidSet;
        }
    }
}
