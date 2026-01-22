using UnityEngine;
using QuadSim.MathUtil;

namespace QuadSim.RobotCore
{
    public sealed class IMUSensor
    {
        public SimFrame OutputFrame = SimFrame.FLU;

        // State
        private Vector3 _prevVelWorld;
        private bool _hasPrev;

        // Latest
        public Vector3 LastAngVel { get; private set; }   // rad/s in OutputFrame
        public Vector3 LastLinAcc { get; private set; }   // m/s^2 in OutputFrame
        public double LastTimestampSec { get; private set; }
        public bool IsValid { get; private set; }

        // If true, accelerometer reports "specific force" (what real IMUs measure):
        // a_specific = a_body - g_body
        public bool RemoveGravity = true;

        public void Reset()
        {
            _hasPrev = false;
            _prevVelWorld = Vector3.zero;
            LastAngVel = Vector3.zero;
            LastLinAcc = Vector3.zero;
            LastTimestampSec = 0;
            IsValid = false;
        }

        public void Initialize(Rigidbody rb)
        {
            if (rb == null)
            {
                IsValid = false;
                return;
            }

            _prevVelWorld = rb.linearVelocity; // world m/s
            _hasPrev = true;
            IsValid = true;
        }

        // Call from SensorManager.PrePhysicsStep() using deterministic dt.
        public void Sample(Rigidbody rb, Transform bodyTransform, double nowSec, float dt)
        {
            if (rb == null || bodyTransform == null || dt <= 0f)
            {
                IsValid = false;
                return;
            }

            if (!_hasPrev)
            {
                _prevVelWorld = rb.linearVelocity;
                _hasPrev = true;
            }

            // World velocities are in m/s
            Vector3 vWorld = rb.linearVelocity;
            Vector3 aWorld = (vWorld - _prevVelWorld) / dt;
            _prevVelWorld = vWorld;

            // Convert world accel to BODY coordinates
            Vector3 aBody = bodyTransform.InverseTransformDirection(aWorld);

            // Convert world angular velocity to BODY coordinates
            // Rigidbody.angularVelocity is world rad/s
            Vector3 wWorld = rb.angularVelocity;
            Vector3 wBody = bodyTransform.InverseTransformDirection(wWorld);

            // If you want "specific force" (accelerometer output), subtract gravity in body frame
            if (RemoveGravity)
            {
                Vector3 gWorld = Physics.gravity;              // (0,-9.81,0)
                Vector3 gBody = bodyTransform.InverseTransformDirection(gWorld);
                aBody = aBody - gBody;
            }

            // Now convert from your UnityBody axes (+X fwd, +Y up, +Z left) to output frame (FLU/FRD)
            LastLinAcc = Frames.TransformAcceleration(aBody, OutputFrame);
            LastAngVel = Frames.TransformAngularVelocity(wBody, OutputFrame);

            LastTimestampSec = nowSec;
            IsValid = true;
        }
    }
}
