using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore;
using Unity.VisualScripting; // SensorManager, SensorData
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    public class PositionController
    {
        [SerializeField] private QuadPIDController _pidSet = new QuadPIDController();
        public QuadPIDController Pid => _pidSet;

        private int _axis;
        private ICommandSource _cmdGoal;
        private SensorManager _sensorManager;
        private DroneConfig _config;
        private float _output;
        private float _externalGoal;
        private bool _bInitialized;
        public PositionController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _sensorManager = sensors;
            PositionPID PID = _config.Position;
            
            _pidSet.SetLimits(-_config.FlightParams.MaxPID, _config.FlightParams.MaxPID);
            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);

            _bInitialized = true;
            Debug.Log("[PositionController] Initialized");

        }
        
        public void Update(float deltaTime)
        {
            if (!_bInitialized || _cmdGoal == null || _sensorManager == null || _pidSet == null)
            {
                _output = 0f;
                return;
            }

            SensorData StateData = _sensorManager.Latest;
            float Goal = _cmdGoal.GetCommandValue()[_axis];
            Debug.Log("[PositionController] Goal: " + Goal);
            Debug.Log("[PositionController] StateRate: " + StateData);
            if (_axis == 2)
            {
                _output = Goal;
                return;
            }

            if (_axis == 3)
            {
                float StateZ = StateData.GpsPosition.z;
                _output = _pidSet.Calculate(Goal, StateZ,deltaTime);
                return;
            }
            float State = StateData.GpsPosition[_axis];
            _output = _pidSet.Calculate(Goal, State, deltaTime);

        }
   
        public void Reset()
        {
            if(_pidSet!=null) _pidSet.Reset();
            _output = 0.0f;
        }
        public void SetExternalGoal(float Value)
        {
            _externalGoal = Value;
        }
        public void ClearExternalGoal()
        {
            // TODO: Double check using this is actually needed or if its updated during runtime
            _externalGoal = 0.0f; 
        }
        public float GetOutput() { return _output; }
        public float GetExternalGoal() { return _externalGoal; }
        public QuadPIDController GetPID()
        {
            return _pidSet;
        }
    }
}