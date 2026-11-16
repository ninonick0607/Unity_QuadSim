using DroneCore.Common;
using DroneCore.Interfaces;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    [System.Serializable]
    public sealed class AngleController // per-axis, UE-style
    {
        [SerializeField] private QuadPIDController _pidSet = new QuadPIDController();
        public QuadPIDController Pid => _pidSet;

        private int _axis;
        private ICommandSource _cmdGoal;
        private VehicleDataStore _store;   // reads Estimate
        private DroneConfig _config;
        private float _output;
        private float _externalGoal;
        private bool _bInitialized;
        private bool _bUseExternalGoal;
        private float _prevYawDeg;
        private bool _hasPrevYaw;

        public AngleController(DroneConfig inConfig)
        {
            _config = inConfig;
        }

        public void Initialize(int axis, ICommandSource cmd, VehicleDataStore store)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _store = store;
            AnglePID PID = _config.Angle;

            // Angle PID output is a rate setpoint (deg/s)
            if (axis == 2)
                _pidSet.SetLimits(-_config.FlightParams.MaxRateYaw, _config.FlightParams.MaxRateYaw);
            else
                _pidSet.SetLimits(-_config.FlightParams.MaxRateRollPitch, _config.FlightParams.MaxRateRollPitch);

            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);

            _bInitialized = true;
        }
        
        public void Update(float deltaTime)
        {
            if (!_bInitialized || _store == null || _pidSet == null || (_cmdGoal == null && !_bUseExternalGoal))
            {
                _output = 0.0f;
                return;
            }

            float StateAngle = _store.Estimate.AttitudeDeg[_axis];
            float Goal = _bUseExternalGoal ? _externalGoal :  _cmdGoal.GetCommandValue()[_axis];
            
            if (_axis == 3)
            {
                _output = Goal;
                return;
            }

            // Euler yaw wraps at +/-180 degrees. Unwrap the measurement against
            // the previous one so the PID's derivative (taken on the
            // measurement) sees a continuous signal, then move the goal to the
            // equivalent angle nearest the measurement, matching AirSim's
            // shortest-arc yaw handling.
            if (_axis == 2)
            {
                if (_hasPrevYaw)
                    StateAngle = NearestEquivalentAngle(_prevYawDeg, StateAngle);
                _prevYawDeg = StateAngle;
                _hasPrevYaw = true;
                Goal = NearestEquivalentAngle(StateAngle, Goal);
            }

            _output = _pidSet.Calculate(Goal, StateAngle, deltaTime);
        }

        public static float NearestEquivalentAngle(float measuredDeg, float goalDeg)
        {
            return measuredDeg + Mathf.DeltaAngle(measuredDeg, goalDeg);
        }
        public void Reset()
        {
            if(_pidSet!=null) _pidSet.Reset();
            _output = 0.0f;
            _bUseExternalGoal = false;
            _hasPrevYaw = false;
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
