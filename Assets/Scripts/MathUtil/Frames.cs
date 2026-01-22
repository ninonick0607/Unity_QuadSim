using UnityEngine;

namespace QuadSim.MathUtil
{
    public enum SimFrame
    {
        UnityBody, // drone local: +X forward, +Y up, +Z left
        FLU,       // +X forward, +Y left, +Z up  (ROS)
        FRD        // +X forward, +Y right, +Z down (PX4)
    }

    public static class Frames
    {
        // -------- Public API (UE style) --------

        public static Vector3 TransformLinear(Vector3 vBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_Linear(vBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_Linear(vBodyUnity),
                _ => vBodyUnity
            };
        }

        public static Vector3 TransformAcceleration(Vector3 aBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_Accel(aBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_Accel(aBodyUnity),
                _ => aBodyUnity
            };
        }

        public static Vector3 TransformAngularVelocity(Vector3 wBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_AngularRate(wBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_AngularRate(wBodyUnity),
                _ => wBodyUnity
            };
        }

        // -------- Implementation (UnityBody -> FLU/FRD) --------
        //
        // Your UnityBody basis:
        //   X = forward
        //   Y = up
        //   Z = left
        //
        // Target FLU:
        //   X = forward
        //   Y = left
        //   Z = up
        //
        // Therefore for pure vectors (linear/accel):
        //   (x,y,z)_body -> (x, z, y)_flu
        //
        // Target FRD:
        //   X = forward
        //   Y = right = -left
        //   Z = down  = -up
        //
        // Therefore:
        //   (x,y,z)_body -> (x, -z, -y)_frd

        private static Vector3 UnityBodyToFLU_Linear(Vector3 v)
            => new Vector3(v.x, v.z, v.y);

        private static Vector3 UnityBodyToFRD_Linear(Vector3 v)
            => new Vector3(v.x, -v.z, -v.y);

        private static Vector3 UnityBodyToFLU_Accel(Vector3 a)
            => new Vector3(a.x, a.z, a.y);

        private static Vector3 UnityBodyToFRD_Accel(Vector3 a)
            => new Vector3(a.x, -a.z, -a.y);

        // Angular velocity: THIS is where we encode your controller convention.
        //
        // Your controller expects: (rollRate, pitchRate, yawRate)
        // with:
        //   roll  about +X (forward)
        //   pitch about +RIGHT axis
        //   yaw   about +UP axis (or +Z in the chosen frame)
        //
        // In FLU, +Y is LEFT, so +RIGHT is -Y.
        // If we want the returned vector's Y component to be "pitch about right",
        // we must negate the FLU Y component.
        //
        // So we return "FLU_RPY" angular rates:
        //   w_flu_vec = (wx, wy_left, wz_up)  from pure transform
        //   w_rpy     = (wx, -wy_left, wz_up)
        //
        // For FRD, +Y is RIGHT already, so no extra sign flip is needed for pitch.

        private static Vector3 UnityBodyToFLU_AngularRate(Vector3 w)
        {
            // pure vector transform body->FLU:
            // (x,y,z)_body -> (x, z, y)_fluVec  where y component is LEFT-rate
            Vector3 wFluVec = new Vector3(w.x, w.z, w.y);

            // Convert to controller semantics: pitch about RIGHT = -LEFT
            return new Vector3(wFluVec.x, -wFluVec.y, wFluVec.z);
        }

        private static Vector3 UnityBodyToFRD_AngularRate(Vector3 w)
        {
            // pure vector transform body->FRD:
            // (x,y,z)_body -> (x, -z, -y)
            // Here +Y is RIGHT and +Z is DOWN already, matching PX4-ish semantics.
            return new Vector3(w.x, -w.z, -w.y);
        }
    }
}
