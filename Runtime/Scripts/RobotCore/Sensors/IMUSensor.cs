using DroneCore.Common;
using MathUtil;
using RobotCore.Core_Utils;
using UnityEngine;

namespace RobotCore.Sensors
{
    // ReSharper disable once InconsistentNaming
    public sealed class IMUSensor
    {
        // Samples the vehicle's published physics state (VehicleDataStore.Truth);
        // it never reads the Rigidbody or transform itself.

        // State
        private Vector3 _prevVelWorld;
        private bool _hasPrev;
        
        public SimFrame OutputFrame = SimFrame.FLU;
        public bool RemoveGravity = false;

        public Vector3 LastAngVel { get; private set; }      // rad/s in OutputFrame
        public Vector3 LastAtt { get; private set; }         // degrees (Euler) in OutputFrame
        public Quaternion LastOrientation { get; private set; } // orientation in OutputFrame
        public Quaternion LastRotationUnity { get; private set; } = Quaternion.identity; // body to world, Unity axes
        public Vector3 LastLinAcc { get; private set; }      // m/s^2 in OutputFrame
        public Vector3 LastVel { get; private set; }

        public double LastTimestampSec { get; private set; }
        public bool IsValid { get; private set; }

        public float AccelNoiseStdDev = 0.05f;   // m/s^2
        public float GyroNoiseStdDev  = 0.002f;  // rad/s (also drives attitude noise, converted to deg)
        public float VelNoiseStdDev   = 0.02f;   // m/s
        private readonly GaussianRng _rng;
        
        public IMUSensor(int noiseSeed = 1)
        {
            _rng = new GaussianRng(noiseSeed);
        }

        public void Reseed(int seed) => _rng.Reset(seed);
        
        public void Reset()
        {
            _hasPrev = false;
            _prevVelWorld = Vector3.zero;
            LastAngVel = Vector3.zero;
            LastAtt = Vector3.zero;
            LastOrientation = Quaternion.identity;
            LastRotationUnity = Quaternion.identity;
            LastLinAcc = Vector3.zero;
            LastVel = Vector3.zero;
            LastTimestampSec = 0;
            IsValid = false;
        }
        public void Initialize(in PhysicsState truth)
        {
            _prevVelWorld = truth.VelocityUnity;
            _hasPrev = true;
            IsValid = true;
        }
        private void SampleAngularVelocity(in PhysicsState truth)
        {
            Vector3 wBody = Quaternion.Inverse(truth.RotationUnity) * truth.AngularVelocityUnity;
            LastAngVel = Frames.TransformAngularVelocity(wBody, OutputFrame); // Output is in Rad/s
        }
        private void SampleAttitude(in PhysicsState truth)
        {
            LastAtt = AttitudeDeg(truth.RotationUnity, OutputFrame);
            LastOrientation = Frames.TransformQuaternion(truth.RotationUnity, OutputFrame);
            LastRotationUnity = truth.RotationUnity;
        }

        /// <summary>Roll, pitch, yaw (deg, each in +-180) of a Unity body rotation, in the given frame.</summary>
        public static Vector3 AttitudeDeg(Quaternion rotationUnity, SimFrame frame)
        {
            Vector3 euler = rotationUnity.eulerAngles;

            // Reorder to (roll, pitch, yaw) — NO sign changes here,
            // TransformAttitude handles LH→RH
            Vector3 rpyDeg = new Vector3(euler.x, euler.z, euler.y);

            Vector3 att = Frames.TransformAttitude(rpyDeg, frame);

            // Wrap to ±180 (Unity eulerAngles are [0,360), UE FRotator is [-180,180))
            att.x = Wrap180(att.x);
            att.y = Wrap180(att.y);
            att.z = Wrap180(att.z);
            return att;
        }
        private static float Wrap180(float a)
        {
            a %= 360f;
            if (a > 180f) a -= 360f;
            if (a < -180f) a += 360f;
            return a;
        }
        private void SampleAcceleration(in PhysicsState truth, float deltaTime)
        {
            Vector3 vWorld = truth.VelocityUnity;
            Vector3 aWorld = (vWorld - _prevVelWorld) / deltaTime;
            _prevVelWorld = vWorld;

            // Specific force (what a real accelerometer / PX4 HIL_SENSOR wants):
            //   f = a_coordinate - g    (g = Physics.gravity = (0,-9.81,0))
            // At rest a=0  -> f = -g = (0,+9.81,0) world (up) -> FRD (0,0,-9.81)
            // Free-fall    -> f = 0
            if (!RemoveGravity)
                aWorld -= Physics.gravity;

            Vector3 aBody = Quaternion.Inverse(truth.RotationUnity) * aWorld;
            LastLinAcc = Frames.TransformAcceleration(aBody, OutputFrame);
        }
        private void SampleVelocity(in PhysicsState truth)
        {
            Vector3 vBody = Quaternion.Inverse(truth.RotationUnity) * truth.VelocityUnity;
            LastVel = Frames.TransformLinear(vBody, OutputFrame);
        }
        // Call from SensorManager's post-physics sample phase using deterministic dt.
        public void Sample(in PhysicsState truth, double nowSec, float deltaTime, bool addNoise = false)
        {
            if (deltaTime <= 0f)
            {
                IsValid = false;
                return;
            }

            if (!_hasPrev)
            {
                _prevVelWorld = truth.VelocityUnity;
                _hasPrev = true;
            }

            SampleAcceleration(truth, deltaTime);
            SampleAngularVelocity(truth);
            SampleAttitude(truth);
            SampleVelocity(truth);

            if (addNoise)
            {
                LastLinAcc += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * AccelNoiseStdDev;
                LastAngVel += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * GyroNoiseStdDev;
                LastVel    += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * VelNoiseStdDev;

                float attStdDeg = GyroNoiseStdDev * Mathf.Rad2Deg;
                LastAtt += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * attStdDeg;
            }

            LastTimestampSec = nowSec;
            IsValid = true;
        }
    }
}
