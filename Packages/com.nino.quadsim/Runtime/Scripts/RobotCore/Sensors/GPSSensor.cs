using MathUtil;
using RobotCore.Core_Utils;
using UnityEngine;

namespace RobotCore.Sensors
{
    public sealed class GPSSensor
    {
        private Rigidbody _rb;

        /// <summary>
        /// Last sampled local position, ALWAYS world-frame ENU
        /// (x=East, y=North, z=Up/altitude), independent of the body-frame
        /// output convention (FLU/FRD) used by the IMU and magnetometer.
        /// NED is a client-side relabel: (n, e, d) = (y, x, -z).
        /// Policy: world I/O is ENU everywhere inside QuadSim; NED exists
        /// only at the MAVLink wire boundary and in LastVelNED.
        /// </summary>
        public Vector3 LastPosition { get; private set; }

        public double LastTimestampSec { get; private set; }
        public bool HasFix { get; private set; }

        public float LatLonNoiseStdDev = 0.5f;  // m, horizontal (X,Y)
        public float AltNoiseStdDev    = 1.0f;  // m, vertical (Z)
        private readonly GaussianRng _rng;
        GeoReferencingSystem _geo;
        
        public double  LastLatDeg { get; private set; }
        public double  LastLonDeg { get; private set; }
        public double  LastAltM   { get; private set; }   // ellipsoidal
        public Vector3 LastVelNED { get; private set; } 
        public bool    IsValid          { get; private set; }

        
        public GPSSensor(Rigidbody inRb, int noiseSeed = 2)
        {
            _rb = inRb;
            _rng = new GaussianRng(noiseSeed);
        }

        public void Reseed(int seed) => _rng.Reset(seed);
        
        public void Reset()
        {
            LastPosition = Vector3.zero;
            LastTimestampSec = 0;
            HasFix = false;
        }

        public void Initialize(GeoReferencingSystem inGeo )
        {
            _geo = inGeo;
            IsValid = (_geo != null && _rb != null);
            HasFix = true;
        }

        public void Sample(double nowSec, bool addNoise = false)
        {
            if (_rb == null)
            {
                HasFix = false;
                return;
            }

            // World-frame local position is always ENU (see LastPosition doc).
            Vector3 pos = Frames.TransformWorldPosition(_rb.position, SimFrame.FLU);

            if (addNoise)
            {
                pos.x += _rng.Next() * LatLonNoiseStdDev;  // East  (horizontal)
                pos.y += _rng.Next() * LatLonNoiseStdDev;  // North (horizontal)
                pos.z += _rng.Next() * AltNoiseStdDev;     // Up    (vertical)
            }
            
            if (_geo != null)
            {
                // Noise-free ENU for the geodetic conversion (ground truth).
                Vector3 enu = Frames.TransformWorldPosition(_rb.position, SimFrame.FLU);
                _geo.EnuToGeodetic(enu.x, enu.y, enu.z,
                    out double lat, out double lon, out double h);
                LastLatDeg = lat;
                LastLonDeg = lon;
                LastAltM   = h;

                // Unity world is X=East, Y=Up, Z=North (Mag's own convention comment),
                // so NED = (N, E, D) = (vz, vx, -vy). No Frames call needed, and this
                // avoids any origin-offset ambiguity in transforming a velocity vector.
                Vector3 v = _rb.linearVelocity;          // _rb.linearVelocity on Unity 6+
                LastVelNED = new Vector3(v.z, v.x, -v.y);
            }
            
            LastPosition     = pos;
            LastTimestampSec = nowSec;
            HasFix = true;
        }
    }
}
