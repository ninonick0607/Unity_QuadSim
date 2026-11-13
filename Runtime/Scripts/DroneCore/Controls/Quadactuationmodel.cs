using UnityEngine;
using MathUtil;
using Yaml.Drone;

namespace DroneCore.Controls
{
    
    /// <summary>
    /// Rotor-speed plant. Every motor command ends as a speed target
    /// w = omega/omegaMax in [0, 1]. The speed follows its target through a
    /// first-order lag when enabled (separate spin-up and spin-down time
    /// constants, as in Gazebo's motor model), and thrust scales with speed
    /// squared, so the thrust fraction produced is w^2.
    /// </summary>
    public sealed class RotorSpeedModel
    {
        public bool  Enabled        = false;   // lag on; off means speed = target
        public float TauUpSeconds   = 0.020f;
        public float TauDownSeconds = 0.020f;

        private float[] _speed01;    // actual omega / omegaMax per rotor
        private float[] _target01;   // commanded omega / omegaMax per rotor

        /// <summary>Speed fraction that produces thrust fraction f.</summary>
        public static float SpeedForThrust01(float thrust01) => Mathf.Sqrt(Mathf.Clamp01(thrust01));

        public float Speed01(int i) => (_speed01 != null && i < _speed01.Length) ? _speed01[i] : 0f;
        public float Target01(int i) => (_target01 != null && i < _target01.Length) ? _target01[i] : 0f;

        public void Reset(int n)   // rotors at rest
        {
            _speed01 = new float[n];
            _target01 = new float[n];
        }

        /// <summary>
        /// Advance the rotor speeds one step toward targetSpeed01 and write the
        /// thrust fractions they produce into outThrust01.
        /// </summary>
        public void Step(float[] targetSpeed01, float dt, float[] outThrust01)
        {
            int n = targetSpeed01.Length;
            if (_speed01 == null || _speed01.Length != n)
            {
                Reset(n);
                for (int i = 0; i < n; i++)   // first step: start at the command
                    _speed01[i] = Mathf.Clamp01(targetSpeed01[i]);
            }

            for (int i = 0; i < n; i++)
            {
                _target01[i] = Mathf.Clamp01(targetSpeed01[i]);
                _speed01[i] = Enabled
                    ? RotorMath.FirstOrderStep(_speed01[i], _target01[i], dt, TauUpSeconds, TauDownSeconds)
                    : _target01[i];
                outThrust01[i] = _speed01[i] * _speed01[i];
            }
        }
    }
    
    public static class QuadActuationModel
    {
        private const int SanityCap = 64;

        public static void ComputeBodyWrench(float[] motor01, RotorInstance[] rotors, RotorPhysicsDerived rotorPhysics, out Vector3 outForceBody, out Vector3 outTorqueBody)
        {
            outForceBody  = Vector3.zero;
            outTorqueBody = Vector3.zero;

            if (motor01 == null || rotors == null) return;

            int n = Mathf.Min(motor01.Length, rotors.Length); 
            n = Mathf.Min(n, SanityCap);

            for (int i = 0; i < n; i++)
            {
                float cs = Mathf.Clamp01(motor01[i]);

                float thrustN  = rotorPhysics.ControlToThrust(cs, 1f);
                float torqueNm = rotorPhysics.ControlToYawTorque(cs, rotors[i].Spin_Dir, 1f);

                Vector3 rotorLoc = rotors[i].Location_Body;

                // Body thrust points +Y (up) — equivalent to UE's +Z body thrust
                Vector3 forceVec = new Vector3(0f, thrustN, 0f);

                Vector3 thrustMoment = Vector3.Cross(rotorLoc, forceVec);
                Vector3 dragTorque   = new Vector3(0f, torqueNm, 0f);
                outForceBody  += forceVec;
                outTorqueBody += thrustMoment + dragTorque;
            }
        }

        public static void BodyToWorldWrench(
            Quaternion bodyToWorld, Vector3 forceBody, Vector3 torqueBody,
            out Vector3 forceWorld, out Vector3 torqueWorld)
        {
            forceWorld = bodyToWorld * forceBody;
            torqueWorld = bodyToWorld * torqueBody;
        }
    }
}
