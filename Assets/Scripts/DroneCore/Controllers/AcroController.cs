using UnityEngine;

namespace QuadSim.Controllers
{
    /// <summary>
    /// Body-rate controller: desired rates -> (rollMix, pitchMix, yawMix).
    /// Uses QuadPidController math.
    /// </summary>
    [System.Serializable]
    public sealed class AcroController
    {
        public QuadPidController Roll = new QuadPidController();
        public QuadPidController Pitch = new QuadPidController();
        public QuadPidController Yaw = new QuadPidController();

        public void Reset()
        {
            Roll.Reset();
            Pitch.Reset();
            Yaw.Reset();
        }

        /// <summary>
        /// All rates in rad/s. Output are mix terms (dimensionless) for the motor mixer.
        /// </summary>
        public Vector3 Update(Vector3 desiredRatesRad, Vector3 measuredRatesRad, float dt)
        {
            float rollMix  = Roll.Calculate(desiredRatesRad.x, measuredRatesRad.x, dt);
            float pitchMix = Pitch.Calculate(desiredRatesRad.y, measuredRatesRad.y, dt);
            float yawMix   = Yaw.Calculate(desiredRatesRad.z, measuredRatesRad.z, dt);
            return new Vector3(rollMix, pitchMix, yawMix);
        }
    }
}