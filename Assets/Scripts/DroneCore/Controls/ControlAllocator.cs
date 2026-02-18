using System;
using UnityEngine;
using MathUtil;
using Yaml.Drone;

namespace DroneCore.Controls
{
    // ────────────────────────────────────────────────────────────────
    //  Effectiveness matrix  (mirrors UE FEffectivenessMatrix)
    // ────────────────────────────────────────────────────────────────

    /// <summary>
    /// 6-axis effectiveness matrix and its pseudo-inverse.
    /// Axes: [0]=Roll, [1]=Pitch, [2]=Yaw, [3]=Fx, [4]=Fy, [5]=Fz(thrust).
    /// Supports up to 8 motors.
    /// </summary>
    [Serializable]
    public sealed class EffectivenessMatrix
    {
        public const int MaxMotors = 8;
        public const int NumAxes   = 6;

        public int NumMotors;

        /// <summary>Axes × Motors  [NumAxes, MaxMotors]</summary>
        public readonly float[,] Data         = new float[NumAxes, MaxMotors];

        /// <summary>Motors × Axes  [MaxMotors, NumAxes]</summary>
        public readonly float[,] PseudoInverse = new float[MaxMotors, NumAxes];

        public void Clear()
        {
            NumMotors = 0;
            Array.Clear(Data,          0, Data.Length);
            Array.Clear(PseudoInverse, 0, PseudoInverse.Length);
        }
    }

    // ────────────────────────────────────────────────────────────────
    //  Allocation constraints  (mirrors UE FAllocationConstraints)
    // ────────────────────────────────────────────────────────────────

    [Serializable]
    public struct AllocationConstraints
    {
        /// <summary>Reserve headroom at top so torque axes still work at high thrust.</summary>
        public float ThrottleReserve01;

        /// <summary>Enable yaw limiting based on remaining headroom.</summary>
        public bool LimitYaw;

        /// <summary>Optional safety margin for saturation checks.</summary>
        public float SafetyMargin01;

        public static AllocationConstraints Default => new AllocationConstraints
        {
            ThrottleReserve01 = 0f,
            LimitYaw          = true,
            SafetyMargin01    = 0f,
        };
    }

    // ────────────────────────────────────────────────────────────────
    //  Control allocator  (mirrors UE FControlAllocator)
    // ────────────────────────────────────────────────────────────────

    /// <summary>
    /// Allocates (roll, pitch, yaw, thrustZ) control demands into motor outputs [0..1]
    /// using an effectiveness-matrix-based approach.
    ///
    /// Note: current "pseudo inverse" is a normalised transpose with authority clamping
    /// (not a true SVD pseudo-inverse). That's fine; encapsulating here makes it easy
    /// to evolve later.
    ///
    /// Unity body frame convention:
    ///   +X forward (nose)   = transform.right
    ///   +Y up      (lift)   = transform.up
    ///   +Z left             = transform.forward
    ///
    /// Control axes:
    ///   Roll  = torque about +X (forward)
    ///   Pitch = torque about -Z (right, since +Z is left)
    ///   Yaw   = torque about +Y (up)
    /// </summary>
    public sealed class ControlAllocator
    {

        public float MinAuthorityRatio = 0.01f;

        // ── Internal state ─────────────────────────────────────────
        private readonly EffectivenessMatrix _effectiveness = new EffectivenessMatrix();

        public ControlAllocator()
        {
            Reset();
        }

        // ── Public accessors ───────────────────────────────────────
        public int                      NumMotors       => _effectiveness.NumMotors;
        public EffectivenessMatrix      Effectiveness   => _effectiveness;

        // ── Reset ──────────────────────────────────────────────────
        public void Reset()
        {
            _effectiveness.Clear();
        }
        
        public void RebuildEffectiveness(RotorInstance[] rotors,RotorPhysicsDerived  rotorPhysics, SimFrame frame)
        {
            if (rotors == null)
            {
                Reset();
                return;
            }

            int numMotors = Mathf.Min(rotors.Length, EffectivenessMatrix.MaxMotors);
            _effectiveness.NumMotors = numMotors;

            if (numMotors <= 0)
            {
                Reset();
                return;
            }

            // Torque-to-thrust ratio  (mirrors UE Kq)
            float kq = (rotorPhysics.MaxThrust > 1e-6f)? (rotorPhysics.MaxTorque / rotorPhysics.MaxThrust): 0f;

            // Clear data rows
            Array.Clear(_effectiveness.Data, 0, _effectiveness.Data.Length);

            for (int m = 0; m < numMotors; m++)
            {
                ref readonly RotorInstance rotor = ref rotors[m];
                
                Vector3 pos       = Frames.TransformLinear(rotor.Location_Body,frame);
                float   spinDir   = Frames.TransformBinary(rotor.Spin_Dir,frame);
                Vector3 thrustDir = Frames.TransformLinear(new Vector3(0f, 1f, 0f), frame);
                
                // Reactive yaw torque per unit motor command
                float reactiveTorque = spinDir * kq;

                Vector3 torqueArm = Vector3.Cross(pos, thrustDir);

                // Direct FLU axis mapping — no manual negations needed
                _effectiveness.Data[0, m] = torqueArm.x;                    // Roll  about FLU +X
                _effectiveness.Data[1, m] = torqueArm.y;                    // Pitch about FLU +Y (was: -torqueArm.z)
                _effectiveness.Data[2, m] = torqueArm.z + reactiveTorque;   // Yaw   about FLU +Z (was: torqueArm.y + reactive)
                _effectiveness.Data[3, m] = 0f;
                _effectiveness.Data[4, m] = 0f;
                _effectiveness.Data[5, m] = thrustDir.z / numMotors;        // Thrust along FLU +Z (was: thrustDir.y)

            }

            ComputePseudoInverse();
        }
        private void ComputePseudoInverse()
        {
            int numMotors = _effectiveness.NumMotors;
            int numAxes = EffectivenessMatrix.NumAxes;
            Array.Clear(_effectiveness.PseudoInverse, 0, _effectiveness.PseudoInverse.Length);

            if (numMotors <= 0) return;

            // Row norm squared for each axis
            float[] rowNormSq = new float[numAxes];
            for (int axis = 0; axis < numAxes; axis++)
            {
                for (int m = 0; m < numMotors; m++)
                    rowNormSq[axis] += _effectiveness.Data[axis, m] * _effectiveness.Data[axis, m];
            }

            // Max authority reference
            float maxAuthority = 0f;
            for (int axis = 0; axis < numAxes; axis++)
                maxAuthority = Mathf.Max(maxAuthority, rowNormSq[axis]);

            // Build pseudo-inverse with authority clamping
            for (int m = 0; m < numMotors; m++)
            {
                for (int axis = 0; axis < numAxes; axis++)
                {
                    if (rowNormSq[axis] > 1e-10f && maxAuthority > 1e-10f)
                    {
                        float effectiveAuthority = Mathf.Max(rowNormSq[axis],maxAuthority * MinAuthorityRatio);
                        _effectiveness.PseudoInverse[m, axis] = _effectiveness.Data[axis, m] / effectiveAuthority;
                    }
                    else
                    {
                        _effectiveness.PseudoInverse[m, axis] = 0f;
                    }
                }
            }
        }
        
        public float AllocateYawLimited(float rollTorque01,float pitchTorque01,float yawTorque01,float thrustZ01,float minMotorOutput, float[] outMotor01)
        {
            int numMotors = _effectiveness.NumMotors;

            if (outMotor01 == null || outMotor01.Length < numMotors)
                throw new ArgumentException(
                    $"outMotor01 must be at least length {numMotors}.");

            if (numMotors <= 0)
            {
                if (outMotor01 != null) Array.Clear(outMotor01, 0, outMotor01.Length);
                return 0f;
            }

            // ── 1) Base motor outputs (roll + pitch + thrust, no yaw) ──

            // Control setpoint: [roll, pitch, 0, 0, 0, thrust]
            float[] uBase = { rollTorque01, pitchTorque01, 0f, 0f, 0f, thrustZ01 };

            float[] baseMotor = new float[numMotors];
            for (int m = 0; m < numMotors; m++)
            {
                float o = 0f;
                for (int axis = 0; axis < EffectivenessMatrix.NumAxes; axis++)
                    o += _effectiveness.PseudoInverse[m, axis] * uBase[axis];
                baseMotor[m] = o;
            }

            // ── 2) Per-motor direction for +1.0 yaw command ──
            //   (column for yaw axis = axis 2 of pseudo-inverse)
            
            // TODO: Missing Num 2 from ControlAllocator.cpp
            float yawSign = (yawTorque01 >= 0f) ? 1f : -1f;
            float[] yawDirSigned = new float[numMotors];
            for (int m = 0; m < numMotors; m++)
                yawDirSigned[m] = _effectiveness.PseudoInverse[m, 2] * yawSign; // TODO: Change here

            // ── 3) Compute max yaw scale based on headroom ──

            float sMax = ComputeYawScaleFromHeadroom(baseMotor,yawDirSigned,numMotors,minMotorOutput, 1f, 0f);       // safetyMargin — match UE default
            float yawApplied01 = yawTorque01 * sMax;

            // ── 4) Final allocation with applied yaw ──

            float[] uFinal = { rollTorque01, pitchTorque01, yawApplied01, 0f, 0f, thrustZ01 };

            for (int m = 0; m < numMotors; m++)
            {
                float o = 0f;
                for (int axis = 0; axis < EffectivenessMatrix.NumAxes; axis++)
                    o += _effectiveness.PseudoInverse[m, axis] * uFinal[axis];
                outMotor01[m] = o;
            }

            // ── 5) Post-process: shift / clamp (mirrors UE PostProcessMotorOutputs) ──

            PostProcessMotorOutputs(outMotor01, numMotors, minMotorOutput);

            return sMax;
        }

        // ────────────────────────────────────────────────────────────
        //  PostProcessMotorOutputs  (mirrors UE static function)
        // ────────────────────────────────────────────────────────────
        private static void PostProcessMotorOutputs(float[] motor01,int count,float minMotorOutput)
        {
            if (count <= 0) return;

            float minOut = motor01[0];
            float maxOut = motor01[0];
            for (int i = 1; i < count; i++)
            {
                minOut = Mathf.Min(minOut, motor01[i]);
                maxOut = Mathf.Max(maxOut, motor01[i]);
            }

            // Shift up if below minimum
            if (minOut < minMotorOutput)
            {
                float shift = minMotorOutput - minOut;
                for (int i = 0; i < count; i++) motor01[i] += shift;
                maxOut += shift;
            }

            // Shift down if above 1
            if (maxOut > 1f)
            {
                float shift = maxOut - 1f;
                for (int i = 0; i < count; i++) motor01[i] -= shift;
            }

            // Final clamp
            for (int i = 0; i < count; i++)
                motor01[i] = Mathf.Clamp(motor01[i], minMotorOutput, 1f);
        }

        // ────────────────────────────────────────────────────────────
        //  ComputeYawScaleFromHeadroom  (mirrors UE static function)
        // ────────────────────────────────────────────────────────────
        private static float ComputeYawScaleFromHeadroom(float[] baseMotor, float[] yawDirPerUnit,int numMotors,float minMotorOutput,float maxMotorOutput = 1f,float safetyMargin   = 0f)
        {
            float sMax = 1f;

            for (int m = 0; m < numMotors; m++)
            {
                float b = baseMotor[m];
                float d = yawDirPerUnit[m];

                if (Mathf.Abs(d) < 1e-8f)
                    continue;

                // We want: min <= b + s*d <= max
                // Solve for s bounds depending on sign of d.

                if (d > 0f)
                {
                    float sUp = (maxMotorOutput - safetyMargin - b) / d;
                    sMax = Mathf.Min(sMax, sUp);
                }
                else // d < 0
                {
                    float sDown = (b - (minMotorOutput + safetyMargin)) / (-d);
                    sMax = Mathf.Min(sMax, sDown);
                }
            }

            return Mathf.Clamp(sMax, 0f, 1f);
        }
    }
}