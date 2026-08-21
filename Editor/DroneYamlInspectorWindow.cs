using DroneCore.Controls;
using MathUtil;
using UnityEditor;
using UnityEngine;
using Yaml.Drone;

namespace Nino.QuadSim.Editor
{
    public sealed class DroneYamlInspectorWindow : EditorWindow
    {
        private TextAsset yamlAsset;
        private ModelType model;
        private DroneParameters physical;
        private RotorModelInstance propulsion;
        private RotorPhysicsDerived rotorPhysics;
        private ControlAllocator allocator;
        private string sourcePath;
        private string error;

        [MenuItem("Tools/QuadSim/Drone YAML Inspector")]
        public static void Open() => GetWindow<DroneYamlInspectorWindow>("Drone YAML");

        private void OnGUI()
        {
            EditorGUI.BeginChangeCheck();
            yamlAsset = (TextAsset)EditorGUILayout.ObjectField(
                "Drone YAML", yamlAsset, typeof(TextAsset), false);
            if (EditorGUI.EndChangeCheck()) Load();

            if (yamlAsset == null)
            {
                EditorGUILayout.HelpBox(
                    "Select a drone YAML from StreamingAssets or the package Resources folder.",
                    MessageType.Info);
                return;
            }
            if (!string.IsNullOrEmpty(error))
            {
                EditorGUILayout.HelpBox(error, MessageType.Error);
                return;
            }

            EditorGUILayout.LabelField("Source", sourcePath);
            EditorGUILayout.LabelField("Schema", model.SchemaVersion.ToString());
            EditorGUILayout.LabelField("Name", model.Name);
            EditorGUILayout.LabelField("Mass", $"{physical.Mass:F4} kg");
            EditorGUILayout.LabelField("Inertia", $"{physical.Inertia} kg m²");
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Derived plant", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("Max thrust / rotor", $"{rotorPhysics.MaxThrust:F5} N");
            EditorGUILayout.LabelField("Max yaw torque / rotor", $"{rotorPhysics.MaxTorque:F6} N m");
            float hover = rotorPhysics.MaxThrust > 0f
                ? physical.Mass * 9.81f / (propulsion.Rotors.Length * rotorPhysics.MaxThrust)
                : 0f;
            EditorGUILayout.LabelField("Hover command", hover.ToString("F4"));
            EditorGUILayout.LabelField("Thrust-to-weight", hover > 0f ? (1f / hover).ToString("F3") : "n/a");
            EditorGUILayout.LabelField("Effectiveness rank", allocator.Rank.ToString());
            EditorGUILayout.LabelField("Condition estimate", allocator.ConditionNumber.ToString("G6"));

            DrawLayout();
            DrawEffectiveness();
        }

        private void Load()
        {
            error = null;
            if (yamlAsset == null) return;
            sourcePath = AssetDatabase.GetAssetPath(yamlAsset);
            if (!YamlParser.LoadDroneConfigFromString(
                    yamlAsset.text, sourcePath, out model, out physical,
                    out propulsion, out _, out _, out _, out _, out _, out _))
            {
                error = "The selected YAML did not pass the Unity runtime parser.";
                return;
            }
            if (propulsion.Rotors == null || propulsion.Rotors.Length != 4)
            {
                error = "The selected YAML must define exactly four rotors.";
                return;
            }
            rotorPhysics = new RotorPhysicsDerived();
            rotorPhysics.ComputeRotor(propulsion.Model, physical.Arm_Length);
            var rebased = new RotorInstance[propulsion.Rotors.Length];
            for (int index = 0; index < propulsion.Rotors.Length; index++)
            {
                rebased[index] = propulsion.Rotors[index];
                rebased[index].Location_Body -= physical.CenterOfMass;
            }
            allocator = new ControlAllocator();
            allocator.RebuildEffectiveness(rebased, rotorPhysics, SimFrame.FLU);
        }

        private void DrawLayout()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Rotor layout (+x forward, +z left)", EditorStyles.boldLabel);
            Rect rect = GUILayoutUtility.GetRect(240f, 240f, GUILayout.ExpandWidth(true));
            EditorGUI.DrawRect(rect, new Color(0.12f, 0.12f, 0.12f));
            Vector2 center = rect.center;
            float radius = 0f;
            foreach (RotorInstance rotor in propulsion.Rotors)
            {
                Vector3 relative = rotor.Location_Body - physical.CenterOfMass;
                radius = Mathf.Max(radius, Mathf.Abs(relative.x), Mathf.Abs(relative.z));
            }
            float scale = radius > 1e-6f ? 0.4f * Mathf.Min(rect.width, rect.height) / radius : 1f;
            EditorGUI.DrawRect(new Rect(center.x - 1f, rect.y, 2f, rect.height), Color.gray);
            EditorGUI.DrawRect(new Rect(rect.x, center.y - 1f, rect.width, 2f), Color.gray);
            foreach (RotorInstance rotor in propulsion.Rotors)
            {
                Vector3 relative = rotor.Location_Body - physical.CenterOfMass;
                Vector2 point = center + new Vector2(relative.z, -relative.x) * scale;
                Rect marker = new Rect(point.x - 12f, point.y - 12f, 24f, 24f);
                EditorGUI.DrawRect(marker, rotor.Spin_Dir > 0
                    ? new Color(0.2f, 0.65f, 1f) : new Color(1f, 0.45f, 0.2f));
                GUI.Label(marker, rotor.ID.ToString(), EditorStyles.centeredGreyMiniLabel);
            }
        }

        private void DrawEffectiveness()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Effectiveness [roll, pitch, yaw, thrust]", EditorStyles.boldLabel);
            int[] rows = { 0, 1, 2, 5 };
            foreach (int row in rows)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    for (int motor = 0; motor < allocator.NumMotors; motor++)
                        EditorGUILayout.SelectableLabel(
                            allocator.Effectiveness.Data[row, motor].ToString("G5"),
                            GUILayout.Height(EditorGUIUtility.singleLineHeight));
                }
            }
        }
    }
}
