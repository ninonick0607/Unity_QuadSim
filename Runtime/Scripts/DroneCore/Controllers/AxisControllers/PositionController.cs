using DroneCore.Common;
using DroneCore.Interfaces;
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
        private VehicleDataStore _store;   // reads Estimate
        private DroneConfig _config;
        private float _output;
        private float _externalGoal;
        private bool _bInitialized;

        public PositionController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, VehicleDataStore store)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _store = store;
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
            if (!_bInitialized || _cmdGoal == null || _store == null || _pidSet == null)
            {
                _output = 0f;
                return;
            }

            float Goal = _cmdGoal.GetCommandValue()[_axis];

            // Axis 2 (yaw) passes through — no position PID for yaw
            if (_axis == 2)
            {
                _output = Goal;
                return;
            }
            
            // Position and position setpoints are both world ENU.
            // Cascade axis mapping:
            //   Axis 0 → East (.x)
            //   Axis 1 → North (.y)
            //   Axis 3 → Up / altitude (.z)
            // The resulting ENU velocity setpoint is rotated into the heading
            // frame by CascadedController before the velocity loop.
            float State;
            switch (_axis)
            {
                case 0: State = _store.Estimate.PositionEnu.x; break;  // forward
                case 1: State = _store.Estimate.PositionEnu.y; break;  // lateral
                case 3: State = _store.Estimate.PositionEnu.z; break;  // altitude
                default: State = 0f; break;
            }

            _output = _pidSet.Calculate(Goal, State, deltaTime);
        }
   
        public void Reset()
        {
            if (_pidSet != null) _pidSet.Reset();
            _output = 0.0f;
        }

        public void SetExternalGoal(float Value)
        {
            _externalGoal = Value;
        }

        public void ClearExternalGoal()
        {
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