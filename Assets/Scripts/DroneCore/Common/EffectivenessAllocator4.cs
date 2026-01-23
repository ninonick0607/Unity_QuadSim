using System;
using UnityEngine;

namespace DroneCore.Common
{
    /// <summary>
    /// UE-style effectiveness allocator for a 4-motor quad.
    ///
    /// Motor order: [0]=FL, [1]=FR, [2]=BL, [3]=BR
    ///
    /// Body axes per your model:
    /// +X forward (nose)  = transform.right
    /// +Y up (lift)       = transform.up
    /// +Z left            = transform.forward
    /// Therefore:
    /// Roll torque axis  = +X
    /// Pitch torque axis = +RIGHT = -Z
    /// Yaw torque axis   = +Y
    ///
    /// This mirrors your Unreal:
    /// - Build effectiveness matrix from r x thrustDir + reactive yaw torque
    /// - Compute pseudo-inverse using rowNormSq with authority clamp
    /// - Allocate control setpoint and do idle/shift/scale saturation handling
    /// </summary>
    [Serializable]
    public sealed class EffectivenessAllocator4
    {
        [Header("Allocation / Authority Clamp")]
        [Tooltip("Yaw can be at most 1/minAuthorityRatio weaker than strongest axis.")]
        [Range(1e-4f, 0.5f)]
        public float minAuthorityRatio = 0.01f;

        [Header("Motor Output Constraints")]
        [Tooltip("Small idle to prevent motor stall (UE used 0.05).")]
        [Range(0f, 0.2f)]
        public float minMotorOutput = 0.05f;

        [Header("Yaw Reaction Torque Model")]
        [Tooltip("Nm per Newton of thrust (same concept as torque_coef / Kq).")]
        public float yawTorquePerNewton = 0.02f;

        // Effectiveness: axes x motors (4 x 4)
        // axes: 0=roll, 1=pitch, 2=yaw, 3=thrust
        private readonly float[,] _E = new float[4, 4];

        // Pseudo-inverse: motors x axes (4 x 4)
        private readonly float[,] _Pinv = new float[4, 4];

        private bool _isBuilt;

        public void Invalidate() => _isBuilt = false;

        /// <summary>
        /// Build E and pseudo-inverse from the current motor transforms.
        /// Must be called after DroneBody has wired motors.
        /// </summary>
        public void Build(Transform body, Transform[] motors, int[] spinDir)
        {
            if (body == null) throw new ArgumentNullException(nameof(body));
            if (motors == null || motors.Length != 4) throw new ArgumentException("motors must be length 4.");
            if (spinDir == null || spinDir.Length != 4) throw new ArgumentException("spinDir must be length 4.");

            // Build effectiveness matrix exactly like UE:
            // arm = r x thrustDir (in body frame)
            // reactive yaw torque = spinDir * yawTorquePerNewton
            // thrust row = thrustDir along lift axis / N
            for (int m = 0; m < 4; m++)
            {
                Transform mot = motors[m];

                // r in BODY coords
                Vector3 rBody = body.InverseTransformPoint(mot.position);

                // thrust direction in BODY coords (unit)
                Vector3 thrustDirBody = body.InverseTransformDirection(mot.up).normalized;

                // torque arm per Newton (Nm / N)
                Vector3 arm = Vector3.Cross(rBody, thrustDirBody);

                // Map to our control axes:
                // roll axis  = +X (body)
                // pitch axis = +RIGHT = -Z (body)  <-- critical for your +Z-left model
                // yaw axis   = +Y (body)
                float rollEff  = arm.x;
                
                // TODO: Make this implicit, hide it in robotcore
                float pitchEff = -arm.z;
                float yawEff   = arm.y + (spinDir[m] * yawTorquePerNewton);

                // Thrust effectiveness: along +Y (up) divided by N (matches UE /NumMotors)
                // TODO: Def use NumMotors when we implement yaml
                float thrustEff = thrustDirBody.y / 4f;

                _E[0, m] = rollEff;
                _E[1, m] = pitchEff;
                _E[2, m] = yawEff;
                _E[3, m] = thrustEff;
            }

            ComputePseudoInverse();
            _isBuilt = true;
        }

        private void ComputePseudoInverse()
        {
            // Mirrors your UE ComputePseudoInverse():
            // rowNormSq per axis
            // maxAuthority reference
            // clamp weak axes via minAuthorityRatio
            // Pinv[m, axis] = E[axis, m] / effectiveAuthority

            // Clear
            for (int m = 0; m < 4; m++)
                for (int a = 0; a < 4; a++)
                    _Pinv[m, a] = 0f;

            float[] rowNormSq = new float[4];

            for (int a = 0; a < 4; a++)
            {
                float s = 0f;
                for (int m = 0; m < 4; m++)
                    s += _E[a, m] * _E[a, m];
                rowNormSq[a] = s;
            }

            float maxAuthority = 0f;
            for (int a = 0; a < 4; a++)
                maxAuthority = Mathf.Max(maxAuthority, rowNormSq[a]);

            for (int m = 0; m < 4; m++)
            {
                for (int a = 0; a < 4; a++)
                {
                    float rn = rowNormSq[a];
                    if (rn > 1e-10f)
                    {
                        float effAuth = Mathf.Max(rn, maxAuthority * minAuthorityRatio);
                        _Pinv[m, a] = _E[a, m] / effAuth;
                    }
                }
            }
        }

        /// <summary>
        /// Allocate motor commands from (rollTorque, pitchTorque, yawTorque, thrust01).
        /// roll/pitch/yaw inputs should be normalized torque demands [-1..1], thrust is [0..1].
        ///
        /// Output motor01 must be length 4.
        /// </summary>
        public void Allocate(
            float rollTorque,
            float pitchTorque,
            float yawTorque,
            float thrust01,
            float[] motor01)
        {
            if (!_isBuilt) throw new InvalidOperationException("Allocator not built. Call Build(...) first.");
            if (motor01 == null || motor01.Length != 4) throw new ArgumentException("motor01 must be length 4.");

            // Control setpoint vector (4 axes) like UE's [roll, pitch, yaw, thrust]
            float c0 = rollTorque;
            float c1 = pitchTorque;
            float c2 = yawTorque;
            float c3 = thrust01;

            // Step 1: raw motor outputs = Pinv * controlSetpoint
            for (int m = 0; m < 4; m++)
            {
                float outm =
                    _Pinv[m, 0] * c0 +
                    _Pinv[m, 1] * c1 +
                    _Pinv[m, 2] * c2 +
                    _Pinv[m, 3] * c3;
                motor01[m] = outm;
            }

            // Step 2: UE-style preserve authority with idle + shift/scale
            float minOut = motor01[0];
            float maxOut = motor01[0];
            for (int m = 1; m < 4; m++)
            {
                minOut = Mathf.Min(minOut, motor01[m]);
                maxOut = Mathf.Max(maxOut, motor01[m]);
            }

            float controlRange = maxOut - minOut;

            if (minOut < minMotorOutput)
            {
                float shift = minMotorOutput - minOut;
                for (int m = 0; m < 4; m++) motor01[m] += shift;
                maxOut += shift;
                minOut += shift;
            }

            if (maxOut > 1f)
            {
                float availableRange = 1f - minMotorOutput;

                if (controlRange > availableRange)
                {
                    float scale = availableRange / controlRange;

                    float currentMin = motor01[0];
                    for (int m = 1; m < 4; m++)
                        currentMin = Mathf.Min(currentMin, motor01[m]);

                    for (int m = 0; m < 4; m++)
                        motor01[m] = minMotorOutput + (motor01[m] - currentMin) * scale;
                }
                else
                {
                    float shift = maxOut - 1f;
                    for (int m = 0; m < 4; m++) motor01[m] -= shift;
                }
            }

            // Final clamp [0..1]
            for (int m = 0; m < 4; m++)
                motor01[m] = Mathf.Clamp01(motor01[m]);
        }
    }
}
