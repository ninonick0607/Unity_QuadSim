using UnityEngine;
using MathUtil;
using Yaml.Drone;

namespace DroneCore.Controls
{
    
    /// <summary>
    /// Rotor-speed lag. Thrust scales with rotor speed squared, so a thrust
    /// fraction f corresponds to a speed fraction w = omega/omegaMax = sqrt(f).
    /// The speed follows its target through a first-order lag (separate
    /// spin-up and spin-down time constants, as in Gazebo's motor model) and
    /// the thrust actually produced is w^2.
    /// </summary>
    public sealed class MotorLagFilter
    {
        public bool  Enabled        = false;
        public float TauUpSeconds   = 0.020f;
        public float TauDownSeconds = 0.020f;

        private float[] _speed01;   // omega / omegaMax per rotor

        public void Reset(int n) => _speed01 = new float[n];   // rotors at rest

        /// In-place: motor01 holds commanded thrust fractions on entry and the
        /// thrust fractions the lagged rotor speeds produce on return.
        public void Apply(float[] motor01, float dt)
        {
            if (!Enabled || motor01 == null) return;
            if (_speed01 == null || _speed01.Length != motor01.Length)
            {
                _speed01 = new float[motor01.Length];
                for (int i = 0; i < motor01.Length; i++)
                    _speed01[i] = Mathf.Sqrt(Mathf.Clamp01(motor01[i]));
            }

            for (int i = 0; i < motor01.Length; i++)
            {
                float targetSpeed01 = Mathf.Sqrt(Mathf.Clamp01(motor01[i]));
                _speed01[i] = RotorMath.FirstOrderStep(
                    _speed01[i], targetSpeed01, dt, TauUpSeconds, TauDownSeconds);
                motor01[i] = _speed01[i] * _speed01[i];
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
