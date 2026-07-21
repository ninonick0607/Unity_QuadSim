using UnityEngine;
using Yaml.Drone;

namespace DroneCore
{
    /// <summary>
    /// Edit-time visualization of the YAML rotor geometry that actually drives
    /// the allocator. Draws rotor positions, spin directions, and the declared
    /// center of mass so mesh/config mismatches are visible in the Scene view.
    /// Runtime no-op.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RotorGeometryGizmo : MonoBehaviour
    {
        [Tooltip("Drone YAML to preview (no extension). Editor visualization only — " +
                 "the runtime config comes from QuadSimSceneConfig.droneConfigName.")]
        public string previewConfigName = "flightmare_quad";

        public bool drawRotors = true;
        public bool drawCenterOfMass = true;
        public bool drawMountDelta = true;
        [Range(0.005f, 0.05f)] public float markerRadius = 0.018f;

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            if (string.IsNullOrWhiteSpace(previewConfigName)) return;
            if (!DroneConfigLoader.LoadConfig(previewConfigName, out DroneConfig cfg)) return;

            var rotors = cfg.RotorParams.Rotors;
            if (rotors == null) return;

            Matrix4x4 prev = Gizmos.matrix;
            Gizmos.matrix = transform.localToWorldMatrix;

            Vector3 com = cfg.DroneParams.CenterOfMass;

            if (drawRotors)
            {
                for (int i = 0; i < rotors.Length; i++)
                {
                    Vector3 p = rotors[i].Location_Body;

                    // CW (+1) warm, CCW (-1) cool.
                    Gizmos.color = rotors[i].Spin_Dir > 0
                        ? new Color(1f, 0.55f, 0.1f)
                        : new Color(0.2f, 0.7f, 1f);

                    Gizmos.DrawSphere(p, markerRadius);
                    Gizmos.DrawLine(com, p);                  // arm, measured from CoM
                    Gizmos.DrawLine(p, p + Vector3.up * 0.06f); // thrust axis
                    Gizmos.color = Color.green;
                    Gizmos.DrawRay(Vector3.zero, Vector3.right * 0.25f);   // +X = nose
                    UnityEditor.Handles.Label(Vector3.right * 0.27f, "FWD (+X)");
                    
                    UnityEditor.Handles.matrix = transform.localToWorldMatrix;
                    UnityEditor.Handles.color = Gizmos.color;
                    UnityEditor.Handles.Label(
                        p + Vector3.up * 0.07f,
                        $"id{rotors[i].ID} {(rotors[i].Spin_Dir > 0 ? "CW" : "CCW")} " +
                        $"{Quadrant(p)}  r={(p - com).magnitude:F3}m");
                }
            }

            if (drawCenterOfMass)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawWireSphere(com, markerRadius * 1.4f);

                // Residual centroid about the CoM — the coupling term.
                Vector3 resid = Vector3.zero;
                for (int i = 0; i < rotors.Length; i++) resid += rotors[i].Location_Body - com;
                if (rotors.Length > 0) resid /= rotors.Length;

                Gizmos.color = Color.red;
                Gizmos.DrawLine(com, com + resid * 10f);   // 10x for visibility
                UnityEditor.Handles.Label(com + resid * 10f,
                    $"residual x10  |perp|={new Vector2(resid.x, resid.z).magnitude:F4} m");
            }

            if (drawMountDelta)
            {
                var pawn = GetComponent<QuadPawn>();
                if (pawn != null)
                {
                    Gizmos.color = Color.yellow;
                    foreach (var t in new[] { "MotorFL", "MotorFR", "MotorBL", "MotorBR" })
                    {
                        Transform mt = transform.Find(t);
                        if (mt == null) continue;
                        Vector3 mp = mt.localPosition;
                        Vector3 yp = NearestRotor(rotors, mp);
                        if ((mp - yp).sqrMagnitude > 1e-6f) Gizmos.DrawLine(mp, yp);
                    }
                }
            }

            Gizmos.matrix = prev;
        }

        private static string Quadrant(Vector3 p) =>
            (p.x >= 0f ? "F" : "B") + (p.z >= 0f ? "L" : "R");
        private static Vector3 NearestRotor(RotorInstance[] r, Vector3 p)
        {
            Vector3 best = p; float bd = float.MaxValue;
            for (int i = 0; i < r.Length; i++)
            {
                float d = (r[i].Location_Body - p).sqrMagnitude;
                if (d < bd) { bd = d; best = r[i].Location_Body; }
            }
            return best;
        }
#endif
    }
}