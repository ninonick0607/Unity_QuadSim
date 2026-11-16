using UnityEngine;

namespace DroneCore.Common
{
    /// <summary>
    /// The state the controller loops read: one snapshot per step, filled by
    /// the configured state source (sensors by default, Rigidbody truth when
    /// opted in, an estimator later). No loop reads sensors or the transform
    /// directly.
    /// </summary>
    public struct VehicleState
    {
        /// <summary>World position, ENU (m). Position loop.</summary>
        public Vector3 PositionEnu;

        /// <summary>Velocity in the level heading frame: forward, left, up (m/s). Velocity loop.</summary>
        public Vector3 HeadingVelocity;

        /// <summary>Roll, pitch, yaw in the sensor output frame (deg). Angle loop.</summary>
        public Vector3 AttitudeDeg;

        /// <summary>Body angular rates in the sensor output frame (rad/s). Rate loop.</summary>
        public Vector3 BodyRatesRadS;

        /// <summary>Body-to-world rotation, Unity axes. Heading and collective.</summary>
        public Quaternion RotationUnity;
    }
}
