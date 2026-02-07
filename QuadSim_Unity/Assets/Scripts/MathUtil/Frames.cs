using UnityEngine;

namespace MathUtil
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

        public static float TransformBinary(float Unity, SimFrame target)
        {
            if (target == SimFrame.FRD)
            {
                return -Unity;
            }
            return Unity;
        }

        public static Vector3 TransformFlip(Vector3 Unity, SimFrame target)
        {
            if (target == SimFrame.FRD)
            {
                return new Vector3(0,0,Unity.z);
            }
            return Unity;
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
        
        public static Vector3 TransformAttitude(Vector3 rpyDegUnityBody, SimFrame target)
        {
            // rpyDegUnityBody is expected to be (roll, pitch, yaw) in DEGREES,
            // in YOUR controller convention, same as Unreal FRotator usage.

            return target switch
            {
                // Match your Unreal UEToFLU_Attitude:
                // roll stays, pitch flips, yaw flips
                SimFrame.FLU => new Vector3(
                    rpyDegUnityBody.x,
                    -rpyDegUnityBody.y,
                    -rpyDegUnityBody.z
                ),

                // Match your Unreal UEToFRD_Attitude:
                // roll stays, pitch stays, yaw flips
                SimFrame.FRD => new Vector3(
                    rpyDegUnityBody.x,
                    rpyDegUnityBody.y,
                    -rpyDegUnityBody.z
                ),

                _ => rpyDegUnityBody
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
        
        public static Quaternion TransformQuaternion(Quaternion qBodyUnity, SimFrame target)
        {
            // True basis-change for rotations.
            // qBodyUnity is the body orientation expressed in UnityBody basis.
            // Returns the same physical orientation expressed in the target frame basis.
            if (target == SimFrame.UnityBody) return qBodyUnity;

            Quaternion unityFromTarget = GetUnityFromTargetBasis(target);
            return Quaternion.Inverse(unityFromTarget) * qBodyUnity * unityFromTarget;
        }

        private static Quaternion GetUnityFromTargetBasis(SimFrame target)
        {
            // Target frame axes expressed in UnityBody coordinates (X fwd, Y up, Z left).
            // FLU: X=fwd, Y=left, Z=up  => (x,y,z) = (X, Z, Y)
            // FRD: X=fwd, Y=right=-left, Z=down=-up => (x,y,z) = (X, -Z, -Y)
            return target switch
            {
                SimFrame.FLU => QuaternionFromBasis(
                    xAxisUnityBody: new Vector3(1f, 0f, 0f),
                    yAxisUnityBody: new Vector3(0f, 0f, 1f),
                    zAxisUnityBody: new Vector3(0f, 1f, 0f)
                ),
                SimFrame.FRD => QuaternionFromBasis(
                    xAxisUnityBody: new Vector3(1f, 0f, 0f),
                    yAxisUnityBody: new Vector3(0f, 0f, -1f),
                    zAxisUnityBody: new Vector3(0f, -1f, 0f)
                ),
                _ => Quaternion.identity
            };
        }

        private static Quaternion QuaternionFromBasis(Vector3 xAxisUnityBody, Vector3 yAxisUnityBody, Vector3 zAxisUnityBody)
        {
            var m = new Matrix4x4();
            m.SetColumn(0, new Vector4(xAxisUnityBody.x, xAxisUnityBody.y, xAxisUnityBody.z, 0f));
            m.SetColumn(1, new Vector4(yAxisUnityBody.x, yAxisUnityBody.y, yAxisUnityBody.z, 0f));
            m.SetColumn(2, new Vector4(zAxisUnityBody.x, zAxisUnityBody.y, zAxisUnityBody.z, 0f));
            m.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
            return QuaternionFromMatrix(m);
        }

        private static Quaternion QuaternionFromMatrix(Matrix4x4 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            if (trace > 0f)
            {
                float s = Mathf.Sqrt(trace + 1f) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    (m.m21 - m.m12) * invS,
                    (m.m02 - m.m20) * invS,
                    (m.m10 - m.m01) * invS,
                    0.25f * s
                );
            }

            if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    0.25f * s,
                    (m.m01 + m.m10) * invS,
                    (m.m02 + m.m20) * invS,
                    (m.m21 - m.m12) * invS
                );
            }

            if (m.m11 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    (m.m01 + m.m10) * invS,
                    0.25f * s,
                    (m.m12 + m.m21) * invS,
                    (m.m02 - m.m20) * invS
                );
            }

            float sZ = Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
            float invSZ = 1f / sZ;
            return new Quaternion(
                (m.m02 + m.m20) * invSZ,
                (m.m12 + m.m21) * invSZ,
                0.25f * sZ,
                (m.m10 - m.m01) * invSZ
            );
        }

    }
}
