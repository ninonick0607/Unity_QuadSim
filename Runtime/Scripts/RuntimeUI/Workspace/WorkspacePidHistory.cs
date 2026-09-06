using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using DroneCore.Controllers;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Compatible with ImGui's PIDGains.csv; validates the entire row before any mutation.</summary>
    public static class WorkspacePidHistory
    {
        public static readonly (CascadedController.EPIDStage Stage, byte Axis, string Name)[] Schema =
        {
            (CascadedController.EPIDStage.Position, 0, "posX"), (CascadedController.EPIDStage.Position, 1, "posY"), (CascadedController.EPIDStage.Position, 3, "posZ"),
            (CascadedController.EPIDStage.Velocity, 0, "velX"), (CascadedController.EPIDStage.Velocity, 1, "velY"), (CascadedController.EPIDStage.Velocity, 3, "velZ"),
            (CascadedController.EPIDStage.Angle, 0, "angRoll"), (CascadedController.EPIDStage.Angle, 1, "angPitch"), (CascadedController.EPIDStage.Angle, 2, "angYaw"),
            (CascadedController.EPIDStage.Rate, 0, "rateRoll"), (CascadedController.EPIDStage.Rate, 1, "ratePitch"), (CascadedController.EPIDStage.Rate, 2, "rateYaw")
        };
        public static string Header
        {
            get { var columns = new List<string> { "Timestamp" }; foreach (var s in Schema) { columns.Add(s.Name + "_P"); columns.Add(s.Name + "_I"); columns.Add(s.Name + "_D"); } return string.Join(",", columns); }
        }
        public static bool TryParse(string line, out float[] gains)
        {
            gains = null; if (line == null) return false;
            var cells = line.Split(','); if (cells.Length != 37) return false;
            var result = new float[36];
            for (int i = 0; i < 36; i++)
                if (!float.TryParse(cells[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out result[i]) || !WorkspaceBackend.IsFinite(result[i]) || result[i] < 0) return false;
            gains = result; return true;
        }
        public static float[] Capture(CascadedController controller)
        {
            var values = new float[36];
            for (int i = 0; i < Schema.Length; i++)
            {
                var s = Schema[i];
                if (controller == null || !controller.GetPIDGains(s.Stage, s.Axis, out values[i * 3], out values[i * 3 + 1], out values[i * 3 + 2]))
                    throw new InvalidOperationException("Selected controller does not expose all PID stages.");
            }
            return values;
        }
        public static void Save(string path, CascadedController controller)
        {
            var gains = Capture(controller);
            if (File.Exists(path)) { using var reader = File.OpenText(path); if (reader.ReadLine() != Header) throw new InvalidDataException("PID CSV header is incompatible."); }
            else File.WriteAllText(path, Header + "\n");
            var row = new List<string> { DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture) };
            foreach (float value in gains)
            {
                if (!WorkspaceBackend.IsFinite(value) || value < 0) throw new InvalidDataException("Controller returned invalid PID gains.");
                row.Add(value.ToString("F6", CultureInfo.InvariantCulture));
            }
            File.AppendAllText(path, string.Join(",", row) + "\n");
        }
        public static List<string> Read(string path)
        {
            var rows = new Queue<string>();
            if (!File.Exists(path)) return new List<string>();
            using var reader = File.OpenText(path);
            if (reader.ReadLine() != Header) throw new InvalidDataException("PID CSV header is incompatible.");
            string row;
            while ((row = reader.ReadLine()) != null)
            {
                if (string.IsNullOrWhiteSpace(row)) continue;
                if (rows.Count == 200) rows.Dequeue(); rows.Enqueue(row);
            }
            return new List<string>(rows);
        }
        public static bool Apply(CascadedController controller, string line)
        {
            if (controller == null || !TryParse(line, out var values)) return false;
            var before = Capture(controller);
            for (int i = 0; i < values.Length; i++)
            {
                var s = Schema[i / 3];
                if (controller.SetPIDGainTerm(s.Stage, s.Axis, i % 3, values[i], false)) continue;
                for (int j = 0; j < i; j++) { var prior = Schema[j / 3]; controller.SetPIDGainTerm(prior.Stage, prior.Axis, j % 3, before[j], false); }
                return false;
            }
            return true;
        }
    }
}
