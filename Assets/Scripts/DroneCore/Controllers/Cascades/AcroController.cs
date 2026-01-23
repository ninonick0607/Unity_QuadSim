using DroneCore.Common;
using UnityEngine;

namespace DroneCore.Controllers.Cascades
{
    /// <summary>
    /// Body-rate controller: desired rates -> (rollMix, pitchMix, yawMix).
    /// Uses QuadPidController math.
    /// </summary>
    [System.Serializable]
    public sealed class AcroController
    {
        public QuadPIDController roll = new QuadPIDController();
        public QuadPIDController pitch = new QuadPIDController();
        public QuadPIDController yaw = new QuadPIDController();

        public void Reset()
        {
            roll.Reset();
            pitch.Reset();
            yaw.Reset();
        }

        /// <summary>
        /// All rates in rad/s. Output are mix terms (dimensionless) for the motor mixer.
        /// </summary>
        public Vector3 Update(Vector3 desiredRatesRad, Vector3 measuredRatesRad, float dt)
        {
            float rollMix  = roll.Calculate(desiredRatesRad.x, measuredRatesRad.x, dt);
            float pitchMix = pitch.Calculate(desiredRatesRad.y, measuredRatesRad.y, dt);
            float yawMix   = yaw.Calculate(desiredRatesRad.z, measuredRatesRad.z, dt);
            return new Vector3(rollMix, pitchMix, yawMix);
        }
    }
}