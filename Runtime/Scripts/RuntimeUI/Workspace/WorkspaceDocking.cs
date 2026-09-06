using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Shared drop geometry, expressed in panel coordinates (independent of screen DPI).</summary>
    public sealed class WorkspaceDocking
    {
        private readonly VisualElement surface;
        private readonly VisualElement preview;
        private readonly VisualElement center;
        public System.Func<Vector2, WorkspaceDock> RegionTarget;
        public WorkspaceDocking(VisualElement surface, VisualElement center = null)
        {
            this.surface = surface;
            this.center = center;
            preview = new VisualElement { pickingMode = PickingMode.Ignore };
            preview.AddToClassList("dock-preview");
            surface.Add(preview);
            Hide();
        }
        public static WorkspaceDock Resolve(Vector2 point, Vector2 size)
        {
            if (size.x <= 0 || size.y <= 0 || point.x < 0 || point.y < 0 || point.x > size.x || point.y > size.y)
                return WorkspaceDock.Floating;
            float edge = Mathf.Min(64, size.x * 0.15f);
            if (point.x <= edge) return WorkspaceDock.Left;
            if (point.x >= size.x - edge) return WorkspaceDock.Right;
            if (point.y >= size.y - Mathf.Min(64, size.y * 0.2f)) return WorkspaceDock.Bottom;
            return WorkspaceDock.Floating;
        }
        public WorkspaceDock Target(Vector2 world)
        {
            var region = RegionTarget?.Invoke(world) ?? WorkspaceDock.Floating;
            return region != WorkspaceDock.Floating ? region : Resolve(surface.WorldToLocal(world), surface.contentRect.size);
        }
        public void Show(Vector2 world)
        {
            var dock = Target(world);
            if (dock == WorkspaceDock.Floating) { Hide(); return; }
            var size = surface.contentRect.size;
            float width = dock == WorkspaceDock.Left ? Mathf.Min(256, size.x * 0.28f) : Mathf.Min(336, size.x * 0.34f);
            float height = Mathf.Min(220, size.y * 0.35f);
            preview.style.left = dock == WorkspaceDock.Bottom && center != null ? surface.WorldToLocal(center.worldBound.position).x : dock == WorkspaceDock.Right ? size.x - width : 0;
            preview.style.top = dock == WorkspaceDock.Bottom ? size.y - height : 0;
            preview.style.width = dock == WorkspaceDock.Bottom ? (center != null ? center.layout.width : size.x) : width;
            preview.style.height = dock == WorkspaceDock.Bottom ? height : size.y;
            preview.style.display = DisplayStyle.Flex;
            preview.BringToFront();
        }
        public void Hide() => preview.style.display = DisplayStyle.None;
    }
}
