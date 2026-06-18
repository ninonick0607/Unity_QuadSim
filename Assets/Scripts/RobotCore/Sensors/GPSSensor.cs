using MathUtil;
using RobotCore.Core_Utils;
using UnityEngine;

namespace RobotCore.Sensors
{
    public sealed class GPSSensor
    {
        private Rigidbody _rb;

        /// <summary>
        /// Output frame for position data. Set by SensorManager to match
        /// the global outputFrame setting. Defaults to FLU.
        /// </summary>
        public SimFrame OutputFrame = SimFrame.FLU;

        /// <summary>
        /// Last sampled position in the OutputFrame coordinate convention.
        /// When OutputFrame is FLU: (x=fwd, y=left, z=up).
        /// When OutputFrame is FRD: (x=fwd, y=right, z=down).
        /// When OutputFrame is UnityBody: raw Unity world (x, y=up, z).
        /// </summary>
        public Vector3 LastPosition { get; private set; }

        public double LastTimestampSec { get; private set; }
        public bool HasFix { get; private set; }

        public float LatLonNoiseStdDev = 0.5f;  // m, horizontal (X,Y)
        public float AltNoiseStdDev    = 1.0f;  // m, vertical (Z)
        private readonly GaussianRng _rng;
        
        public GPSSensor(Rigidbody inRb, int noiseSeed = 2)
        {
            _rb = inRb;
            _rng = new GaussianRng(noiseSeed);
        }
        
        public void Reset()
        {
            LastPosition = Vector3.zero;
            LastTimestampSec = 0;
            HasFix = false;
        }

        public void Initialize()
        {
            HasFix = true;
        }

        public void Sample(double nowSec, bool addNoise = false)
        {
            if (_rb == null)
            {
                HasFix = false;
                return;
            }

            Vector3 pos = Frames.TransformWorldPosition(_rb.position, OutputFrame);

            if (addNoise)
            {
                pos.x += _rng.Next() * LatLonNoiseStdDev;  // horizontal
                pos.y += _rng.Next() * LatLonNoiseStdDev;  // horizontal
                pos.z += _rng.Next() * AltNoiseStdDev;     // vertical (Z is the up/down axis in FLU and FRD)
            }

            LastPosition     = pos;
            LastTimestampSec = nowSec;
            HasFix = true;
        }
    }
}