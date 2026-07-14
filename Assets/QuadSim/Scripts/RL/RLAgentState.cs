using DroneCore;
using MathUtil;
using UnityEngine;

namespace QuadSim.RL
{
    public struct RLAgentState
    {
        public string AgentId;
        public string TeamId;
        public int DroneIndex;
        public Vector3 PositionEnu;
        public Vector3 VelocityEnu;
        public Quaternion OrientationFluToEnu;
        public Vector3 BodyVelocityFlu;
        public Vector3 BodyAngularVelocityFlu;
        public Vector3 ForwardEnu;
        public Vector3 UpEnu;
        public bool IsValid;
        public bool HasCollision;
        public bool IsGrounded;

        public static RLAgentState Capture(QuadPawn pawn)
        {
            if (pawn == null || pawn.Rigidbody == null)
                return default;

            Rigidbody body = pawn.Rigidbody;
            Transform transform = pawn.transform;
            Vector3 bodyVelocity = transform.InverseTransformDirection(body.linearVelocity);
            Vector3 bodyAngularVelocity = transform.InverseTransformDirection(body.angularVelocity);

            var state = new RLAgentState
            {
                AgentId = pawn.AgentId,
                TeamId = pawn.TeamId,
                DroneIndex = pawn.DroneIndex,
                PositionEnu = Frames.TransformWorldPosition(body.position, SimFrame.FLU),
                VelocityEnu = Frames.TransformWorldVector(body.linearVelocity, SimFrame.FLU),
                OrientationFluToEnu = Frames.TransformQuaternion(body.rotation, SimFrame.FLU),
                BodyVelocityFlu = Frames.TransformLinear(bodyVelocity, SimFrame.FLU),
                BodyAngularVelocityFlu = Frames.TransformAngularVelocity(bodyAngularVelocity, SimFrame.FLU),
                ForwardEnu = Frames.TransformWorldVector(transform.forward, SimFrame.FLU),
                UpEnu = Frames.TransformWorldVector(transform.up, SimFrame.FLU),
                HasCollision = pawn.HasCollision,
                IsGrounded = pawn.IsGrounded,
            };

            state.IsValid = IsFinite(state.PositionEnu)
                && IsFinite(state.VelocityEnu)
                && IsFinite(state.OrientationFluToEnu)
                && IsFinite(state.BodyVelocityFlu)
                && IsFinite(state.BodyAngularVelocityFlu);
            return state;
        }

        private static bool IsFinite(Vector3 value) =>
            IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);

        private static bool IsFinite(Quaternion value) =>
            IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);

        private static bool IsFinite(float value) =>
            !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
