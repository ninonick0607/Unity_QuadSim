using MathUtil;
using UnityEngine;

namespace RobotCore
{
    // ========================================================================
    // FRAME CONVENTIONS (see MathUtil/Frames.cs header for the full policy)
    //
    //   WORLD data  → always ENU (x=East, y=North, z=Up).
    //                 NED appears only in explicitly named fields (GpsVelNED).
    //   BODY data   → BodyFrame convention: FLU (default) or FRD (PX4 mode).
    // ========================================================================
    public struct SensorData
    {
        /// <summary>Body-axis convention of all body-frame fields below
        /// (ImuAngVel/ImuAttitude/ImuAccel/ImuVel/ImuOrientation/MagField).
        /// FLU by default; FRD when the sim is in PX4 mode. World-frame
        /// fields are unaffected — always ENU.</summary>
        public SimFrame BodyFrame;

        // IMU — BODY frame (BodyFrame convention)
        public Vector3 ImuAngVel;        // rad/s, body
        public Vector3 ImuAttitude;      // RPY deg in BodyFrame convention
        public Vector3 ImuAccel;         // m/s², body specific force (gravity included)
        public Vector3 ImuVel;           // m/s, BODY-frame velocity (not world!)
        public Quaternion ImuOrientation; // body(BodyFrame) → world(ENU)
        public double ImuTimestampSec;
        public bool ImuValid;

        // GPS — WORLD frame, always ENU (x=E, y=N, z=Up/altitude)
        public Vector3 GpsPosition;
        public double GpsTimestampSec;
        public bool GpsValid;
        
        // Baro (pressure in Pa, altitudes in m, temp in degC) — scalars
        public float  BaroPressurePa;
        public float  BaroPressureHPa;       // HIL_SENSOR.abs_pressure
        public float  BaroTemperatureC;      // HIL_SENSOR.temperature
        public float  BaroAltitudeMSL;       // true input altitude (ground truth)
        public float  BaroPressureAltitude;  // HIL_SENSOR.pressure_alt (derived from pressure)
        public double BaroTimestampSec;
        public bool   BaroValid;
        
        // Mag — BODY frame (BodyFrame convention), Gauss
        public Vector3 MagField;
        public float   MagHeadingDeg;
        public float   MagDeclinationDeg;
        public double  MagTimestampSec;
        public bool    MagValid;
        
        // GPS geodetic (WGS84) + WORLD-frame NED velocity (PX4 HIL_GPS wire quantity)
        public double  GpsLatDeg;
        public double  GpsLonDeg;
        public double  GpsAltM;        // ellipsoidal
        public Vector3 GpsVelNED;      // (vn, ve, vd) m/s — world NED, explicitly named
    }
}
