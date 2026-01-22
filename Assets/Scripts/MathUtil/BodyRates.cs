using UnityEngine;

namespace QuadSim.MathUtil
{
    /// <summary>
    /// Adapts Unity body-local angular velocity (rad/s) into controller RPY rates.
    /// Assumes: drone forward is Unity +X, up is +Y.
    ///
    /// roll  = about +X (forward)
    /// pitch = about +Z (right)  <-- sign chosen to match your previously-stable behavior
    /// yaw   = about +Y (up)
    /// </summary>
    public static class BodyRates
    {
        public static Vector3 ToRPY(Vector3 wBodyRad)
        {
            float roll  = wBodyRad.x;
            float pitch = wBodyRad.z;  // IMPORTANT: keep this sign for stability (was stable before)
            float yaw   = wBodyRad.y;
            return new Vector3(roll, pitch, yaw);
        }
    }
}