using DroneCore;
using UnityEditor;
using UnityEngine;
using Yaml.Drone;

namespace QuadSim.Scripts.Editor
{
    [CustomEditor(typeof(RotorGeometryGizmo))]
    public sealed class RotorGeometryGizmoEditor : UnityEditor.Editor
    {
        private static readonly string[] MountFields = { "motorFL", "motorFR", "motorBL", "motorBR" };
        private static readonly string[] MountNames  = { "MotorFL", "MotorFR", "MotorBL", "MotorBR" };

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var gz = (RotorGeometryGizmo)target;

            EditorGUILayout.Space();

            if (GUILayout.Button("Align Motor Mounts to Config"))
                Align(gz, apply: true);

            if (GUILayout.Button("Report Only (no changes)"))
                Align(gz, apply: false);
        }

        private static void Align(RotorGeometryGizmo gz, bool apply)
        {
            if (!DroneConfigLoader.LoadConfig(gz.previewConfigName, out DroneConfig cfg))
            {
                Debug.LogError($"[Aligner] Could not load '{gz.previewConfigName}'.");
                return;
            }

            var rotors = cfg.RotorParams.Rotors;
            if (rotors == null || rotors.Length != 4)
            {
                Debug.LogError("[Aligner] Expected exactly 4 rotors.");
                return;
            }

            Transform root = gz.transform;

            // Classify by quadrant, NOT by array index — YAML id order and the
            // FL/FR/BL/BR mount order are not guaranteed to agree.
            int[] slot = new int[4]; for (int i = 0; i < 4; i++) slot[i] = -1;
            for (int i = 0; i < rotors.Length; i++)
            {
                Vector3 p = rotors[i].Location_Body;
                int idx = (p.x >= 0f)
                    ? (p.z >= 0f ? 0 : 1)     // FL, FR
                    : (p.z >= 0f ? 2 : 3);    // BL, BR


                if (slot[idx] != -1)
                {
                    Debug.LogError($"[Aligner] Two rotors in the same quadrant " +
                                   $"({MountNames[idx]}): ids {rotors[slot[idx]].ID} and {rotors[i].ID}. " +
                                   $"Fix the config before aligning.");
                    return;
                }
                slot[idx] = i;

                if (rotors[i].ID != idx)
                    Debug.LogWarning($"[Aligner] YAML id {rotors[i].ID} sits in the " +
                                     $"{MountNames[idx]} quadrant. Index order and mount order " +
                                     $"disagree — harmless for the allocator, confusing for humans.");
            }

            
            var pawn = gz.GetComponent<QuadPawn>();
            if (pawn == null) { Debug.LogError("[Aligner] No QuadPawn on this object."); return; }
            var so = new SerializedObject(pawn);

            for (int m = 0; m < 4; m++)
            {
                var prop = so.FindProperty(MountFields[m]);
                var t = prop?.objectReferenceValue as Transform;

                if (t == null)
                {
                    Debug.LogWarning($"[Aligner] QuadPawn.{MountFields[m]} is unassigned.");
                    continue;
                }

                if (t.GetComponentInChildren<Renderer>() != null)
                {
                    Debug.LogError($"[Aligner] {MountFields[m]} points at '{t.name}', which renders. " +
                                   "Mounts must be empty markers — aligning would move the mesh. Skipped.");
                    continue;
                }

                Vector3 target = rotors[slot[m]].Location_Body;
                float delta = (t.localPosition - target).magnitude;

                Debug.Log($"[Aligner] {MountNames[m]} <- rotor id{rotors[slot[m]].ID} " +
                          $"{target:F4} (was {t.localPosition:F4}, delta {delta * 1000f:F1} mm)");

                if (apply && delta > 1e-5f)
                {
                    Undo.RecordObject(t, "Align motor mount");
                    t.localPosition = target;
                    EditorUtility.SetDirty(t);
                }
            }
            
            
        }
    }
}