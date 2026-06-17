using MathUtil;
using UnityEngine;


namespace RobotCore.Sensors
{
    public class BaroSensor
    {
        // Parameters
        
        // PX4 is 20hz 
        private float UpdateRate = 50.0f;
        private bool isInitialized = false;
        
        // Drift sim
        private float BaroDriftPa = 0.0f;
        private float PressureNoiseStdDev = 0.5f;
        private float DriftPaPerSec = 0.005f;
        
        // Box Muller Noise
        private bool bUseLastGaussian = false;
        private float LastGaussianValue = 0.0f;

        // Sensor Offset (should be configurable) 
        private float PressureOffset = 0.0f;
        private float TemperatureOffset = 0.0f;
        
        // ISA Model

        private const float ISA_LAPSE_RATE = 0.0065f;
        private const float ISA_TEMPERATURE_MSL = 288.15f;
        private const float ISA_PRESSURE_MSL = 101325.0f;
        private const float ISA_GAS_CONSTANT = 287.05f;
        private const float ISA_GRAVITY = 9.80665f;

        
        private Rigidbody _rb;
        private Transform _bodyTransform;

        public float LastPressure { get; private set; }
        public float LastPressureHPa => LastPressure / 100.0f;
        public float LastTemperature { get; private set; }
        public float EstimatedAltitude { get; private set; }
        
        
        public double LastTimestampSec { get; private set; }
        public bool IsValid { get; private set; }
        
        public BaroSensor(Rigidbody rb, Transform bodyTransform)
        {
            _rb = rb;
            _bodyTransform = bodyTransform;
        }
        
        public void Reset()
        
    }
}