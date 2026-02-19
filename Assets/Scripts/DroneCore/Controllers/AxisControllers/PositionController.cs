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
    
            if (axis == 3)
                _pidSet.SetLimits(-_config.FlightParams.MaxVelZ, _config.FlightParams.MaxVelZ);
            else
                _pidSet.SetLimits(-_config.FlightParams.MaxVelXY, _config.FlightParams.MaxVelXY);
    
            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);
            _bInitialized = true;
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

            if (_axis == 2)
            {
                _output = Goal;
                return;
            }
            
            float State;
            switch (_axis)
            {
                case 0: State = StateData.GpsPosition.x; break;  
                case 1: State = StateData.GpsPosition.z; break;  
                case 3: State = StateData.GpsPosition.y; break;  
                default: State = 0f; break;
            }

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