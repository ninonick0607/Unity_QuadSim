using DroneCore.Common;
using SimCore;
using MathUtil;
using UnityEngine;
using RobotCore.Core_Utils;

namespace RobotCore.Sensors
{
    public sealed class MagSensor
    {
        public float MagNoiseStdDev = 0.005f;                 // Gauss
        public float MagOffsetX = 0f, MagOffsetY = 0f, MagOffsetZ = 0f;  // Gauss bias

        public const float CALIBRATION_DURATION = 1.0f;       // vestigial; mirrors Unreal, gates nothing

        private readonly GaussianRng _rng;
        private GeoReferencingSystem _geo;   // samples the published physics state (VehicleDataStore.Truth)
        private Quaternion _rotationUnity = Quaternion.identity;   // body to world at the last sample

        public SimFrame OutputFrame = SimFrame.FLU;

        public Vector3 LastMagField     { get; private set; } // body, OutputFrame, Gauss  (PX4 wants FRD)
        public Vector3 EarthMagFieldNED { get; private set; } // geographic NED, Gauss (debug)
        public float   MagneticHeading      { get; private set; } // deg, debug (≈ declination — see note)
        public float   MagneticDeclination  { get; private set; } // deg, from WMM
        public double  LastTimestampSec { get; private set; }
        public bool    IsValid          { get; private set; }

        public bool  IsCalibrating { get; private set; }
        private float _calibrationTime;
        private bool  _earthFieldValid;
        private bool  _declinationLogged;

        public MagSensor(int noiseSeed = 4)
        {
            _rng = new GaussianRng(noiseSeed);
        }

        public void Reseed(int seed) => _rng.Reset(seed);

        public void Reset()
        {
            LastMagField = Vector3.zero;
            EarthMagFieldNED = Vector3.zero;
            MagneticHeading = 0f;
            MagneticDeclination = 0f;
            LastTimestampSec = 0.0;
            IsValid = false;
            IsCalibrating = true;
            _calibrationTime = 0f;
            _earthFieldValid = false;
            _declinationLogged = false;
        }

        public void Initialize(GeoReferencingSystem geo)
        {
            _geo = geo;
            IsValid = _geo != null;
        }

        public void Sample(in PhysicsState truth, double nowSec, float dtSec, bool addNoise)
        {
            if (_geo == null)
            {
                IsValid = false;
                return;
            }

            if (IsCalibrating)
            {
                _calibrationTime += dtSec;
                if (_calibrationTime >= CALIBRATION_DURATION) IsCalibrating = false;
            }

            _rotationUnity = truth.RotationUnity;
            UpdateEarthMagField(truth);
            if (!_earthFieldValid) { IsValid = false; return; }

            // Earth field NED (Gauss) -> Unity world (X=East, Y=Up, Z=North):
            //   N -> +Z, E -> +X, D -> -Y
            float n = EarthMagFieldNED.x, e = EarthMagFieldNED.y, d = EarthMagFieldNED.z;
            Vector3 fieldWorldUnity = new Vector3(e, -d, n);

            // World -> body (Unity), then -> OutputFrame. Same machinery as the IMU.
            Vector3 fieldBodyUnity = Quaternion.Inverse(_rotationUnity) * fieldWorldUnity;
            Vector3 field = Frames.TransformLinear(fieldBodyUnity, OutputFrame);

            if (addNoise)
                field += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * MagNoiseStdDev;

            field.x += MagOffsetX;
            field.y += MagOffsetY;
            field.z += MagOffsetZ;

            LastMagField = field;
            CalculateMagneticHeading();

            LastTimestampSec = nowSec;
            IsValid = true;
        }

        private void UpdateEarthMagField(in PhysicsState truth)
        {
            // World position in ENU (East, North, Up) — TransformWorldPosition(FLU) is the ENU map.
            Vector3 flu = Frames.TransformWorldPosition(truth.PositionUnity, SimFrame.FLU);
            _geo.EnuToGeodetic(flu.x, flu.y, flu.z, out double lat, out double lon, out _);

            EarthMagFieldNED    = GeoMagneticTables.GetMagFieldVectorNED((float)lat, (float)lon);
            MagneticDeclination = GeoMagneticTables.GetMagDeclinationDegrees((float)lat, (float)lon);

            if (!_declinationLogged)
            {
                Debug.Log($"[Mag] WMM declination={MagneticDeclination:F2}deg  lat={lat:F5} lon={lon:F5}");
                _declinationLogged = true;
            }
            _earthFieldValid = true;
        }

        private void CalculateMagneticHeading()
        {
            // Rotate measured body field back to Unity world and read the horizontal bearing.
            // This points at *magnetic* North regardless of vehicle yaw, so the value tracks
            // declination — it's a transform self-consistency check, not vehicle heading.
            Vector3 bodyUnity  = Frames.InverseTransformLinear(LastMagField, OutputFrame);
            Vector3 worldUnity = _rotationUnity * bodyUnity;

            float headingRad = Mathf.Atan2(worldUnity.x /*East*/, worldUnity.z /*North*/);
            float headingDeg = Mathf.Rad2Deg * headingRad;
            if (headingDeg < 0f) headingDeg += 360f;
            MagneticHeading = headingDeg;
        }
    }
}
