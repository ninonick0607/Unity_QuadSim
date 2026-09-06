using System.Collections.Generic;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Retained, aligned rows for the backend's grouped reports. Values refresh without rebuilding the tree.</summary>
    public sealed class WorkspaceReadout : VisualElement
    {
        private readonly List<Label> values = new List<Label>();
        private string structure;
        public void SetReport(string report)
        {
            var lines = report.Split('\n');
            var signature = new System.Text.StringBuilder();
            foreach (var line in lines)
            {
                int split = line.IndexOf("   ", System.StringComparison.Ordinal);
                signature.Append(split >= 0 ? line.Substring(0, split) : line).Append('\n');
            }
            string next = signature.ToString();
            if (structure != next)
            {
                structure = next; Clear(); values.Clear(); VisualElement group = this;
                foreach (var line in lines)
                {
                    if (string.IsNullOrWhiteSpace(line)) continue;
                    int split = line.IndexOf("   ", System.StringComparison.Ordinal);
                    if (split < 0)
                    {
                        var fold = new Foldout { text = line, value = true }; fold.AddToClassList("readout-group"); Add(fold); group = fold;
                    }
                    else
                    {
                        var row = new VisualElement(); row.AddToClassList("property-row");
                        var key = new Label(line.Substring(0, split)); key.AddToClassList("property-key"); row.Add(key);
                        var value = new Label(); value.AddToClassList("property-value"); row.Add(value); group.Add(row); values.Add(value);
                    }
                }
            }
            int index = 0;
            foreach (var line in lines)
            {
                int split = line.IndexOf("   ", System.StringComparison.Ordinal);
                if (split >= 0) { var value = line.Substring(split + 3); values[index].text = value; values[index++].tooltip = value; }
            }
        }
    }
}
