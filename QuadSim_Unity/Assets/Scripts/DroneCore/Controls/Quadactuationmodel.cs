using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controls
{
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
    }
}