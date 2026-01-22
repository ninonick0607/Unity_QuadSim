using UnityEngine;

namespace QuadSim.RobotCore
{
    public sealed class GPSSensor
    {
        public Vector3 LastPositionWorldM { get; private set; }
        public double LastTimestampSec { get; private set; }
        public bool HasFix { get; private set; }

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

        public void Sample(Rigidbody rb, double nowSec)
        {
            if (rb == null)
            {
                HasFix = false;
                return;
            }

            LastPositionWorldM = rb.position; // Unity units are meters
            LastTimestampSec = nowSec;
            HasFix = true;
        }
    }
}