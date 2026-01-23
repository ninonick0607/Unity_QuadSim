using UnityEngine;

namespace RobotCore
{
    public struct SensorData
    {
        // IMU (frame: selected output frame)
        public Vector3 ImuAngVel;  
        public Vector3 ImuAttitude;
        public Vector3 ImuAccel;     
        public Vector3 ImuVel;
        public Quaternion ImuOrientation;
        public double ImuTimestampSec;
        public bool ImuValid;

        // GPS
        public Vector3 GpsPosition;    
        public double GpsTimestampSec;
        public bool GpsValid;
    }
}