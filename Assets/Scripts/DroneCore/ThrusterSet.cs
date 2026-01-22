using System;
using UnityEngine;

namespace QuadSim.DroneCore
{
    /// <summary>
    /// Applies per-motor thrust using AddForceAtPosition (physical rotor wrench).
    /// Commands are normalized [0..1] and mapped to Newtons.
    /// </summary>
    public sealed class ThrusterSet : MonoBehaviour
    {
        [Header("Refs")]
        [SerializeField] private DroneBody body;

        [Header("Thrust Model")]
        [Tooltip("Max thrust per motor in Newtons at command=1.")]
        [SerializeField] private float maxThrustPerMotorN = 12.0f;

        [Tooltip("Clamp commands to [0..1].")]
        [SerializeField] private bool clamp01 = true;

        [Header("Optional Yaw Reaction Torque (very rough)")]
        [Tooltip("If >0, applies a body yaw torque proportional to thrust. Units: Nm per Newton.")]
        [SerializeField] private float yawTorquePerNewton = 0.02f; // start small

        [Tooltip("Motor spin direction: +1 or -1 per motor. Typical X quad: [-1,+1,+1,-1] depending on layout.")]
        [SerializeField] private int[] spinDirection = new[] { +1, -1, -1, +1 }; // FL FR BL BR
        public int[] SpinDirection => spinDirection;

        private readonly float[] _cmd01 = new float[4];

        private void Reset()
        {
            body = GetComponent<DroneBody>();
        }

        private void Awake()
        {
            if (body == null) body = GetComponent<DroneBody>();
            body.AutoWireIfNeeded(); 
            body.ValidateOrThrow();

            if (spinDirection == null || spinDirection.Length != 4)
                throw new Exception($"{name}: spinDirection must be length 4.");
        }

        /// <summary>Set normalized motor command [0..1]. Stored until next ApplyForces call.</summary>
        public void SetMotorCommand01(int motorIndex, float cmd01)
        {
            if ((uint)motorIndex >= 4) throw new ArgumentOutOfRangeException(nameof(motorIndex));
            if (clamp01) cmd01 = Mathf.Clamp01(cmd01);
            _cmd01[motorIndex] = cmd01;
        }

        public void SetAllMotorCommands01(float c0, float c1, float c2, float c3)
        {
            if (clamp01)
            {
                c0 = Mathf.Clamp01(c0); c1 = Mathf.Clamp01(c1);
                c2 = Mathf.Clamp01(c2); c3 = Mathf.Clamp01(c3);
            }
            _cmd01[0] = c0; _cmd01[1] = c1; _cmd01[2] = c2; _cmd01[3] = c3;
        }

        public float GetMotorCommand01(int motorIndex) => _cmd01[motorIndex];

        /// <summary>
        /// Call during PrePhysicsStep to apply thrust for the upcoming Physics.Simulate(dt).
        /// </summary>
        public void ApplyForces()
        {
            var rb = body.Rigidbody;
            var motors = body.Motors;

            // Per-motor thrust
            for (int i = 0; i < 4; i++)
            {
                float thrustN = _cmd01[i] * maxThrustPerMotorN;
                if (thrustN <= 0) continue;

                Transform m = motors[i];

                // Thrust direction: motor's local up in world-space.
                Vector3 dir = m.up;
                Vector3 force = dir * thrustN;
                Vector3 pos = m.position;

                rb.AddForceAtPosition(force, pos, ForceMode.Force);
            }

            if (yawTorquePerNewton > 0f)
            {
                float tau = 0f;
                for (int i = 0; i < 4; i++)
                {
                    float thrustN = _cmd01[i] * maxThrustPerMotorN;
                    tau += spinDirection[i] * thrustN * yawTorquePerNewton;
                }

                // Apply about body up axis in world space.
                rb.AddTorque(rb.transform.up * tau, ForceMode.Force);
            }

        }
    }
}
