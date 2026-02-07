using UnityEngine;

namespace RobotCore.Sensors
{
    public sealed class GPSSensor
    {
        private Rigidbody _rb;

        public Vector3 LastPositionWorldM { get; private set; }
        public double LastTimestampSec { get; private set; }
        public bool HasFix { get; private set; }

        public GPSSensor(Rigidbody inRb)
        {
            _rb = inRb;
        }
        public void Reset()
        {
            LastPositionWorldM = Vector3.zero;
            LastTimestampSec = 0;
            HasFix = false;
        }

        public void Initialize()
        {
            // For now always "has fix" once initialized.
            HasFix = true;
        }

        public void Sample(double nowSec)
        {
            if (_rb == null)
            {
                HasFix = false;
                return;
            }

            LastPositionWorldM = _rb.position; // Unity units are meters
            LastTimestampSec = nowSec;
            HasFix = true;
        }
    }
}