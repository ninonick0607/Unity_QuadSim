using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using SimCore;

namespace Experimental
{
    /// <summary>
    /// A wind/fan volume. Applies velocity-relative drag toward the zone's air
    /// velocity to any non-kinematic Rigidbody inside it, exactly ONCE per
    /// rigidbody per physics step (OnTriggerStay fires once per collider, so
    /// multi-collider rigs would otherwise multiply the force).
    ///
    /// All timing is sim-time via ClockFactory — never UnityEngine.Time.
    /// Can optionally log applied force per step to CSV for ground-truth overlay
    /// against the Python-side d_est / phi. This is disabled by default because
    /// per-step string formatting and disk I/O are expensive at simulation rates.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
    public class WindModule : MonoBehaviour
    {
        [Header("Base Wind Settings")]
        [Tooltip("Fan jet air speed at the source face, m/s. Blow direction is local forward (blue Z axis).")]
        [SerializeField] private float _windSpeed = 10f;

        [Tooltip("Linear drag, N per (m/s) of relative air speed.")]
        [SerializeField] private float _dragCoefficient = 0.5f;

        [Header("Spreading & Cone Settings")]
        [Range(0f, 2f)] [SerializeField] private float _spreadFactor = 0f;

        [Header("Turbulence Settings")]
        [SerializeField] private float _turbulenceStrength = 2f;
        [SerializeField] private float _turbulenceFrequency = 5f;

        [SerializeField] private bool _toggle = false;

        [Header("Diagnostics")]
        [Tooltip("Write per-step applied-force CSV to <project>/runs/wind_log_*.csv. Expensive; enable only for diagnostics.")]
        [SerializeField] private bool _logForces = false;

        private BoxCollider _boxCollider;

        // --- per-step dedupe + logging state ---
        private ulong _lastStepCount = ulong.MaxValue;
        private readonly Dictionary<Rigidbody, PendingRow> _pendingRows = new();
        private StreamWriter _log;

        private struct PendingRow
        {
            public double SimTime;
            public ulong Step;
            public string Name;
            public Vector3 Pos;
            public float TravelRatio;
            public float WindScale;
            public Vector3 AirVel;
            public Vector3 RelVel;
            public Vector3 Force;
            public int CallbackCount;   // how many trigger callbacks hit this rb this step
        }

        private void Awake()
        {
            _boxCollider = GetComponent<BoxCollider>();

            // ---- Debug logging for exact dimensions and center/origin ----
            Vector3 worldSize = Vector3.Scale(_boxCollider.size, transform.lossyScale);
            Vector3 worldCenter = transform.TransformPoint(_boxCollider.center);
            
            Debug.Log($"[WindModule] Awake - Windzone Initialized.\n" +
                      $"Dimensions (World Size): X:{worldSize.x:F3}, Y:{worldSize.y:F3}, Z:{worldSize.z:F3}\n" +
                      $"Origin/Center (World Position): {worldCenter}\n" +
                      $"Collider Center (Local Offset): {_boxCollider.center}");

            if (_logForces)
            {
                string dir = Path.Combine(Application.dataPath, "../runs");
                Directory.CreateDirectory(dir);
                string path = Path.Combine(dir,
                    $"wind_log_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
                _log = new StreamWriter(path, false, Encoding.ASCII);
                _log.WriteLine("step,sim_t,rb,px,py,pz,travel_ratio,wind_scale," +
                               "air_vx,air_vy,air_vz,rel_vx,rel_vy,rel_vz," +
                               "fx,fy,fz,f_mag,callbacks");
                Debug.Log($"[WindModule] Logging applied forces -> {path}");
            }
        }

        private void Reset()
        {
            var col = GetComponent<BoxCollider>();
            if (col != null) col.isTrigger = true;
        }

        private void OnDestroy()
        {
            FlushPendingRows();
            _log?.Flush();
            _log?.Dispose();
        }
        // ============================================================================
        // Runtime registry + control API
        //   Self-registering so the RPC layer never scans the scene. _toggle is the
        //   force gate (see OnTriggerStay); SetEnabled flips it live so one build can
        //   serve the wind-OFF gain study and the wind-ON net study without a rebuild.
        // ============================================================================

        private static readonly List<WindModule> _active = new();
        public static IReadOnlyList<WindModule> Active => _active;

        private void OnEnable()  { if (!_active.Contains(this)) _active.Add(this); }
        private void OnDisable() { _active.Remove(this); }

        public bool  IsEnabled => _toggle;
        public float WindSpeed => _windSpeed;

        /// <summary>Turn the disturbance force on/off at runtime.</summary>
        public void SetEnabled(bool on) => _toggle = on;

        /// <summary>Override fan jet speed (m/s) at runtime. Clamped non-negative.</summary>
        public void SetWindSpeed(float metersPerSecond) => _windSpeed = Mathf.Max(0f, metersPerSecond);
        private void OnTriggerStay(Collider other)
        {
            var rb = other.attachedRigidbody;
            if (rb == null || rb.isKinematic) return;

            // ---- per-step dedupe ----
            ulong step = ClockFactory.Clock.StepCount;
            if (step != _lastStepCount)
            {
                FlushPendingRows();          // write last step's rows (with final callback counts)
                _lastStepCount = step;
            }

            // With both force and diagnostics disabled there is nothing to do.
            // Avoid geometry, turbulence, allocations, and dictionary work for
            // every trigger callback while the wind zone is inactive.
            if (!_toggle && _log == null) return;

            if (_pendingRows.TryGetValue(rb, out var existing))
            {
                // Force already applied to this rb this step — just count the duplicate.
                existing.CallbackCount++;
                _pendingRows[rb] = existing;
                return;
            }

            // ---- geometry (world-scale aware, collider-center aware) ----
            Vector3 worldSize = Vector3.Scale(_boxCollider.size, transform.lossyScale);
            float maxRange = Mathf.Max(worldSize.z, 1e-4f);

            Vector3 boxCenter = transform.TransformPoint(_boxCollider.center);
            Vector3 sourceFacePosition = boxCenter - transform.forward * (maxRange * 0.5f);

            Vector3 toTarget = rb.position - sourceFacePosition;
            float distanceAlongFlow = Mathf.Clamp(
                Vector3.Dot(toTarget, transform.forward), 0f, maxRange);
            float travelRatio = distanceAlongFlow / maxRange;

            float distanceFalloff = 1f - travelRatio;

            Vector3 centralProjectedPoint =
                sourceFacePosition + transform.forward * distanceAlongFlow;
            float lateralDistance = Vector3.Distance(rb.position, centralProjectedPoint);
            float spreadEdgeLoss =
                Mathf.Clamp01(1f - lateralDistance * _spreadFactor * travelRatio);

            float finalWindScale = distanceFalloff * spreadEdgeLoss;
            Vector3 airVelocity = transform.forward * (_windSpeed * finalWindScale);

            // ---- turbulence: SIM time, not wall time ----
            if (_turbulenceStrength > 0f && travelRatio > 0.05f)
            {
                float simT = (float)(ClockFactory.Clock.NowNanos / 1e9);
                float timeOffset = simT * _turbulenceFrequency;

                float noiseX = Mathf.PerlinNoise(rb.position.x + timeOffset, rb.position.y);
                float noiseY = Mathf.PerlinNoise(rb.position.y + timeOffset, rb.position.z);
                float noiseZ = Mathf.PerlinNoise(rb.position.z + timeOffset, rb.position.x);

                Vector3 turbDir = new Vector3(
                    noiseX * 2f - 1f, noiseY * 2f - 1f, noiseZ * 2f - 1f).normalized;

                airVelocity += turbDir * (_turbulenceStrength * travelRatio * spreadEdgeLoss);
            }

            // ---- apply (once per rb per step) ----
            Vector3 relVel = airVelocity - rb.linearVelocity;
            Vector3 force = _toggle ? _dragCoefficient * relVel : Vector3.zero;

            if (_toggle)
                rb.AddForce(force, ForceMode.Force);

            // ---- record row ----
            if (_log != null)
            {
                _pendingRows[rb] = new PendingRow
                {
                    SimTime = ClockFactory.Clock.NowNanos / 1e9,
                    Step = step,
                    Name = rb.name,
                    Pos = rb.position,
                    TravelRatio = travelRatio,
                    WindScale = finalWindScale,
                    AirVel = airVelocity,
                    RelVel = relVel,
                    Force = force,
                    CallbackCount = 1,
                };
            }
            else
            {
                // still need the dedupe key even when not logging
                _pendingRows[rb] = default;
            }
        }

        private void FlushPendingRows()
        {
            if (_log != null)
            {
                foreach (var kv in _pendingRows)
                {
                    var r = kv.Value;
                    _log.WriteLine(
                        $"{r.Step},{r.SimTime:F4},{r.Name}," +
                        $"{r.Pos.x:F4},{r.Pos.y:F4},{r.Pos.z:F4}," +
                        $"{r.TravelRatio:F4},{r.WindScale:F4}," +
                        $"{r.AirVel.x:F4},{r.AirVel.y:F4},{r.AirVel.z:F4}," +
                        $"{r.RelVel.x:F4},{r.RelVel.y:F4},{r.RelVel.z:F4}," +
                        $"{r.Force.x:F4},{r.Force.y:F4},{r.Force.z:F4}," +
                        $"{r.Force.magnitude:F4},{r.CallbackCount}");
                }
            }
            _pendingRows.Clear();
        }

        // ---- Always draw the windzone outline ----
        private void OnDrawGizmos()
        {
            if (_boxCollider == null) _boxCollider = GetComponent<BoxCollider>();
            if (_boxCollider == null) return;

            // Save the original matrix and color to restore later
            Matrix4x4 oldMatrix = Gizmos.matrix;
            Color oldColor = Gizmos.color;

            // Align the Gizmo drawing space with the Transform's local space (handles rotation/scale)
            Gizmos.matrix = transform.localToWorldMatrix;
            
            // Draw the box outline (semi-transparent green)
            Gizmos.color = new Color(0f, 1f, 0f, 0.4f);
            Gizmos.DrawWireCube(_boxCollider.center, _boxCollider.size);

            // Restore previous Gizmo states
            Gizmos.matrix = oldMatrix;
            Gizmos.color = oldColor;
        }

        private void OnDrawGizmosSelected()
        {
            if (_boxCollider == null) _boxCollider = GetComponent<BoxCollider>();
            if (_boxCollider == null) return;

            Vector3 scale = transform.lossyScale;
            Vector3 worldSize = Vector3.Scale(
                _boxCollider.size,
                new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z))
            );

            float length = Mathf.Max(worldSize.z, 0.01f);
            float width  = Mathf.Max(worldSize.x, 0.01f);
            float height = Mathf.Max(worldSize.y, 0.01f);

            Vector3 forward = transform.forward.normalized;
            Vector3 right   = transform.right.normalized;
            Vector3 up      = transform.up.normalized;

            Vector3 center = transform.TransformPoint(_boxCollider.center);
            Vector3 sourceFace = center - forward * (length * 0.5f);
            Vector3 exitFace   = center + forward * (length * 0.5f);

            Color activeColor = _toggle ? Color.cyan : new Color(0.55f, 0.55f, 0.55f, 1f);
            Color softColor   = _toggle ? new Color(0f, 1f, 1f, 0.30f) : new Color(0.55f, 0.55f, 0.55f, 0.25f);

            // Source and exit faces, so you can immediately see where wind starts/ends.
            Gizmos.color = new Color(0f, 0.7f, 1f, 0.75f);
            DrawFaceRect(sourceFace, right, up, width, height);

            Gizmos.color = softColor;
            DrawFaceRect(exitFace, right, up, width, height);

            // Big center arrow: actual wind direction.
            Gizmos.color = activeColor;
            DrawArrowGizmo(
                sourceFace,
                forward,
                length,
                Mathf.Clamp(length * 0.18f, 0.25f, 1.25f),
                Mathf.Clamp(Mathf.Min(width, height) * 0.18f, 0.15f, 0.75f)
            );

            // Smaller streamlines across the fan face. These make it read as flow, not a single axis.
            float xOffset = width * 0.25f;
            float yOffset = height * 0.25f;
            float streamlineLength = length * 0.72f;
            float streamlineHeadLen = Mathf.Clamp(length * 0.10f, 0.15f, 0.65f);
            float streamlineHeadWidth = Mathf.Clamp(Mathf.Min(width, height) * 0.08f, 0.08f, 0.30f);

            Vector3[] offsets =
            {
                right * xOffset,
                -right * xOffset,
                up * yOffset,
                -up * yOffset,
                right * xOffset + up * yOffset,
                right * xOffset - up * yOffset,
                -right * xOffset + up * yOffset,
                -right * xOffset - up * yOffset,
            };

            Gizmos.color = softColor;
            foreach (Vector3 offset in offsets)
            {
                DrawArrowGizmo(
                    sourceFace + offset,
                    forward,
                    streamlineLength,
                    streamlineHeadLen,
                    streamlineHeadWidth
                );
            }

            // Show the spread cone/edge loss preview at the exit face.
            float spreadRadius = Mathf.Max(0.05f, _spreadFactor * length * 0.5f);
            Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
            Gizmos.DrawWireSphere(exitFace, spreadRadius);

        #if UNITY_EDITOR
            UnityEditor.Handles.color = activeColor;
            UnityEditor.Handles.Label(
                exitFace + up * (height * 0.35f),
                $"Wind → {forward}\nSpeed: {_windSpeed:F1} m/s\n{(_toggle ? "ENABLED" : "DISABLED")}"
            );
        #endif
        }

        private static void DrawArrowGizmo(
            Vector3 start,
            Vector3 direction,
            float length,
            float headLength,
            float headWidth)
        {
            if (direction.sqrMagnitude < 1e-6f) return;

            direction.Normalize();

            Vector3 end = start + direction * length;
            Gizmos.DrawLine(start, end);

            // Build arrowhead basis.
            Vector3 side = Vector3.Cross(direction, Vector3.up);
            if (side.sqrMagnitude < 1e-6f)
                side = Vector3.Cross(direction, Vector3.right);

            side.Normalize();
            Vector3 up = Vector3.Cross(side, direction).normalized;

            Vector3 baseCenter = end - direction * headLength;

            // Four-sided pyramid arrowhead.
            Vector3 p1 = baseCenter + side * headWidth;
            Vector3 p2 = baseCenter - side * headWidth;
            Vector3 p3 = baseCenter + up * headWidth;
            Vector3 p4 = baseCenter - up * headWidth;

            Gizmos.DrawLine(end, p1);
            Gizmos.DrawLine(end, p2);
            Gizmos.DrawLine(end, p3);
            Gizmos.DrawLine(end, p4);

            Gizmos.DrawLine(p1, p3);
            Gizmos.DrawLine(p3, p2);
            Gizmos.DrawLine(p2, p4);
            Gizmos.DrawLine(p4, p1);
        }

        private static void DrawFaceRect(
            Vector3 center,
            Vector3 right,
            Vector3 up,
            float width,
            float height)
        {
            Vector3 r = right.normalized * (width * 0.5f);
            Vector3 u = up.normalized * (height * 0.5f);

            Vector3 a = center - r - u;
            Vector3 b = center + r - u;
            Vector3 c = center + r + u;
            Vector3 d = center - r + u;

            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);
        }
    }
}
