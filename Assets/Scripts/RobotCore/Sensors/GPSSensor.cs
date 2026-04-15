using MathUtil;
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

        public GPSSensor(Rigidbody inRb)
        {
            _rb = inRb;
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

        public void Sample(double nowSec)
        {
            if (_rb == null)
            {
                HasFix = false;
                return;
            }

            // Raw Unity world position (Y-up) → convert to output frame
            Vector3 rawUnity = _rb.position;
            LastPosition = Frames.TransformWorldPosition(rawUnity, OutputFrame);

            LastTimestampSec = nowSec;
            HasFix = true;
        }
    }
}