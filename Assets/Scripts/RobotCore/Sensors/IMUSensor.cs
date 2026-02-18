using MathUtil;
using UnityEngine;

namespace RobotCore.Sensors
{
    // ReSharper disable once InconsistentNaming
    public sealed class IMUSensor
    {
        private Rigidbody _rb;
        private Transform _bodyTransform;
        
        // State
        private Vector3 _prevVelWorld;
        private bool _hasPrev;
        
        public SimFrame OutputFrame = SimFrame.FLU;
        public bool RemoveGravity = true;

        public Vector3 LastAngVel { get; private set; }      // rad/s in OutputFrame
        public Vector3 LastAtt { get; private set; }         // degrees (Euler) in OutputFrame
        public Quaternion LastOrientation { get; private set; } // orientation in OutputFrame
        public Vector3 LastLinAcc { get; private set; }      // m/s^2 in OutputFrame
        public Vector3 LastVel { get; private set; }

        public double LastTimestampSec { get; private set; }
        public bool IsValid { get; private set; }

        public IMUSensor(Rigidbody rb, Transform bodyTransform)
        {
            _rb = rb;
            _bodyTransform = bodyTransform;
        }
        
        public void Reset()
        {
            _hasPrev = false;
            _prevVelWorld = Vector3.zero;
            LastAngVel = Vector3.zero;
            LastAtt = Vector3.zero;
            LastOrientation = Quaternion.identity;
            LastLinAcc = Vector3.zero;
            LastVel = Vector3.zero;
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


        public void SampleAngularVelocity(double nowSec, float deltaTime)
        {
            Vector3 wWorld = _rb.angularVelocity;
            Vector3 wBody = _bodyTransform.InverseTransformDirection(wWorld);
            
            LastAngVel = Frames.TransformAngularVelocity(wBody, OutputFrame); // Output is in Rad/s
        }

        public void SampleAttitude(double nowSec, float deltaTime)
        {
            Vector3 euler = _bodyTransform.rotation.eulerAngles;
    
            // Reorder to (roll, pitch, yaw) — NO sign changes here, 
            // TransformAttitude handles LH→RH
            Vector3 rpyDeg = new Vector3(euler.x, euler.z, euler.y);
    
            Vector3 att = Frames.TransformAttitude(rpyDeg, OutputFrame);
    
            // Wrap to ±180 (Unity eulerAngles are [0,360), UE FRotator is [-180,180))
            att.x = Wrap180(att.x);
            att.y = Wrap180(att.y);
            att.z = Wrap180(att.z);
    
            LastAtt = att;
            LastOrientation = Frames.TransformQuaternion(_bodyTransform.rotation, OutputFrame);
        }

        private static float Wrap180(float a)
        {
            a %= 360f;
            if (a > 180f) a -= 360f;
            if (a < -180f) a += 360f;
            return a;
        }


        
        public void SampleAcceleration(double nowSec, float deltaTime)
        {
            Vector3 vWorld = _rb.linearVelocity;
            Vector3 aWorld = (vWorld - _prevVelWorld) / deltaTime;
            _prevVelWorld = vWorld;

            Vector3 aBody = _bodyTransform.InverseTransformDirection(aWorld);
            
            if (RemoveGravity)
            {
                Vector3 gWorld = Physics.gravity;              
                Vector3 gBody = _bodyTransform.InverseTransformDirection(gWorld);
                aBody = aBody - gBody;
            }
            
            LastLinAcc = Frames.TransformAcceleration(aBody, OutputFrame);
        }

        public void SampleVelocity(double nowSec, float deltaTime)
        {
            Vector3 vWorld = _rb.linearVelocity;
            Vector3 vBody = _bodyTransform.InverseTransformDirection(vWorld);
            
            LastVel = Frames.TransformLinear(vBody, OutputFrame);
        }
        // Call from SensorManager.PrePhysicsStep() using deterministic dt.
        public void Sample(double nowSec, float deltaTime)
        {
            if (_rb == null || _bodyTransform == null || deltaTime <= 0f)
            {
                IsValid = false;
                return;
            }

            if (!_hasPrev)
            {
                _prevVelWorld = _rb.linearVelocity;
                _hasPrev = true;
            }

            SampleAcceleration(nowSec, deltaTime);
            SampleAngularVelocity(nowSec, deltaTime);
            SampleAttitude(nowSec, deltaTime);
            SampleVelocity(nowSec, deltaTime);

            LastTimestampSec = nowSec;
            IsValid = true;
        }
    }
}
