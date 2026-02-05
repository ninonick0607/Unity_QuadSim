using MathUtil;
using UnityEngine;

namespace DroneCore
{
    public sealed class DroneDebugDraw : MonoBehaviour
    {
        [SerializeField] private QuadPawn body;

        [Header("Draw")]
        public bool drawUnityBodyAxes = true;
        public bool drawSelectedFrameAxes = true;
        public SimFrame frameToDraw = SimFrame.FLU;

        public bool drawMotorAxes = false;
        public float axisLength = 0.25f;

        [Header("Style")]
        [Tooltip("Dim the reference (UnityBody) axes a bit so the selected frame stands out.")]
        [Range(0.1f, 1f)] public float unityBodyBrightness = 0.5f;

        private void Awake()
        {
            if (body == null) body = GetComponent<QuadPawn>();
        }

        private void Update()
        {
            if (body == null || body.Rigidbody == null) return;

            Vector3 origin = body.Rigidbody.worldCenterOfMass;

            // Drone "UnityBody" basis (per your model):
            // +X = transform.right (forward)
            // +Y = transform.up    (up)
            // +Z = transform.forward (left)
            Vector3 Xb = body.transform.right;
            Vector3 Yb = body.transform.up;
            Vector3 Zb = body.transform.forward;

            if (drawUnityBodyAxes)
            {
                DrawAxisTriplet(origin, Xb, Yb, Zb, axisLength, unityBodyBrightness);
            }

            if (drawSelectedFrameAxes)
            {
                GetFrameAxesWorld(frameToDraw, Xb, Yb, Zb, out Vector3 Xf, out Vector3 Yf, out Vector3 Zf);
                DrawAxisTriplet(origin, Xf, Yf, Zf, axisLength, 1.0f);
            }

            if (drawMotorAxes && body.Motors != null)
            {
                for (int i = 0; i < body.Motors.Length; i++)
                {
                    var m = body.Motors[i];
                    if (m == null) continue;

                    Vector3 p = m.position;
                    Debug.DrawRay(p, m.right * (axisLength * 0.5f), Color.red);
                    Debug.DrawRay(p, m.up * (axisLength * 0.5f), Color.green);
                    Debug.DrawRay(p, m.forward * (axisLength * 0.5f), Color.blue);
                }
            }
        }

        private static void DrawAxisTriplet(Vector3 origin, Vector3 x, Vector3 y, Vector3 z, float len, float brightness)
        {
            // Convention: X=red, Y=green, Z=blue
            Debug.DrawRay(origin, x.normalized * len, new Color(1f * brightness, 0f, 0f));
            Debug.DrawRay(origin, y.normalized * len, new Color(0f, 1f * brightness, 0f));
            Debug.DrawRay(origin, z.normalized * len, new Color(0f, 0f, 1f * brightness));
        }

        private static void GetFrameAxesWorld(SimFrame frame, Vector3 Xb, Vector3 Yb, Vector3 Zb,
                                              out Vector3 Xf, out Vector3 Yf, out Vector3 Zf)
        {
            // Inputs are UnityBody axes in world space:
            // Xb = forward, Yb = up, Zb = left

            switch (frame)
            {
                case SimFrame.FLU:
                    // FLU: X forward, Y left, Z up
                    Xf = Xb;
                    Yf = Zb;
                    Zf = Yb;
                    break;

                case SimFrame.FRD:
                    // FRD: X forward, Y right, Z down
                    Xf = Xb;
                    Yf = -Zb;
                    Zf = -Yb;
                    break;

                case SimFrame.UnityBody:
                default:
                    Xf = Xb;
                    Yf = Yb;
                    Zf = Zb;
                    break;
            }
        }
    }
}
