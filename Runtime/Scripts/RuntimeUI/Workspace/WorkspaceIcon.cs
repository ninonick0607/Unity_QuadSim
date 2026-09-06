using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Original 16-unit line icons. Retained vector geometry, no textures or per-frame work.</summary>
    public sealed class WorkspaceIcon : VisualElement
    {
        private string symbol;
        public string Symbol { get => symbol; set { if (symbol == value) return; symbol = value; MarkDirtyRepaint(); } }
        public WorkspaceIcon(string symbol)
        {
            this.symbol = symbol; pickingMode = PickingMode.Ignore; AddToClassList("workspace-icon");
            generateVisualContent += Draw;
        }
        private void Draw(MeshGenerationContext context)
        {
            var p = context.painter2D; p.strokeColor = resolvedStyle.color; p.lineWidth = 1.4f;
            void Line(params Vector2[] points) { p.BeginPath(); p.MoveTo(points[0]); for (int i = 1; i < points.Length; i++) p.LineTo(points[i]); p.Stroke(); }
            void Box(float x, float y, float w, float h) => Line(new Vector2(x,y), new Vector2(x+w,y), new Vector2(x+w,y+h), new Vector2(x,y+h), new Vector2(x,y));
            void Circle(float x, float y, float r) { p.BeginPath(); p.Arc(new Vector2(x,y), r, 0, 360); p.Stroke(); }
            switch (symbol)
            {
                case "Follow": Circle(8,8,4); Line(new Vector2(8,1),new Vector2(8,5)); Line(new Vector2(8,11),new Vector2(8,15)); Line(new Vector2(1,8),new Vector2(5,8)); Line(new Vector2(11,8),new Vector2(15,8)); break;
                case "Orbit": Circle(8,8,5); Line(new Vector2(9,1),new Vector2(13,3),new Vector2(11,6)); break;
                case "Free": Line(new Vector2(8,1),new Vector2(8,15)); Line(new Vector2(1,8),new Vector2(15,8)); Line(new Vector2(5,4),new Vector2(8,1),new Vector2(11,4)); Line(new Vector2(4,5),new Vector2(1,8),new Vector2(4,11)); break;
                case "Play": Line(new Vector2(5,3),new Vector2(12,8),new Vector2(5,13),new Vector2(5,3)); break;
                case "Pause": Line(new Vector2(5,3),new Vector2(5,13)); Line(new Vector2(11,3),new Vector2(11,13)); break;
                case "Step": Line(new Vector2(3,3),new Vector2(10,8),new Vector2(3,13),new Vector2(3,3)); Line(new Vector2(13,3),new Vector2(13,13)); break;
                case "Reset": case "Refresh": p.BeginPath(); p.Arc(new Vector2(8,8),5,40,310); p.Stroke(); Line(new Vector2(8,3),new Vector2(12,3),new Vector2(12,7)); break;
                case "Drone": case "Scene": Line(new Vector2(4,4),new Vector2(12,12)); Line(new Vector2(12,4),new Vector2(4,12)); Circle(4,4,2.5f); Circle(12,4,2.5f); Circle(4,12,2.5f); Circle(12,12,2.5f); break;
                case "Camera": Box(2,5,9,7); Line(new Vector2(11,7),new Vector2(15,5),new Vector2(15,12),new Vector2(11,10)); break;
                case "Telemetry": Line(new Vector2(1,9),new Vector2(4,9),new Vector2(6,3),new Vector2(9,13),new Vector2(11,7),new Vector2(15,7)); break;
                case "Logs": Box(3,2,10,12); Line(new Vector2(5,5),new Vector2(11,5)); Line(new Vector2(5,8),new Vector2(11,8)); Line(new Vector2(5,11),new Vector2(9,11)); break;
                case "Control Authority": Circle(4,8,2); Circle(12,4,2); Circle(12,12,2); Line(new Vector2(6,7),new Vector2(10,5)); Line(new Vector2(6,9),new Vector2(10,11)); break;
                case "Showcase": Box(1,3,14,10); Line(new Vector2(11,3),new Vector2(11,13)); break;
                case "Spawn": Line(new Vector2(8,2),new Vector2(8,14)); Line(new Vector2(2,8),new Vector2(14,8)); break;
                default: Line(new Vector2(3,2),new Vector2(3,14)); Line(new Vector2(8,2),new Vector2(8,14)); Line(new Vector2(13,2),new Vector2(13,14)); Box(1,5,4,3); Box(6,9,4,3); Box(11,4,4,3); break;
            }
        }
    }
}
