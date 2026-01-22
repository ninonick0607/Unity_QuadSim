using UnityEngine;

namespace QuadSim.RobotCore
{
    // Keep it simple for now; add validity + timestamps like UE.
    public struct SensorData
    {
        // IMU (frame: selected output frame)
        public Vector3 imuAngVelRad;     // rad/s
        public Vector3 imuLinAccMS2;     // m/s^2 (see note below on gravity)
        public Quaternion imuOrientation;// orientation in chosen frame (optional for now)
        public double imuTimestampSec;
        public bool imuValid;

        // GPS
        public Vector3 gpsPositionM;     // meters (world or frame-defined; we will pick one)
        public double gpsTimestampSec;
        public bool gpsValid;
    }
}