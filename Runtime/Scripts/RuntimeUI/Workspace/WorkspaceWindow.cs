using System;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public enum WorkspaceDock { Left, Right, Bottom, Floating }

    /// <summary>Runtime chrome and pointer-driven docking. No editor APIs or position menus.</summary>
    public sealed class WorkspaceWindow : VisualElement
    {
        public readonly ScrollView Body = new ScrollView();
        public readonly string Id;
        public WorkspaceDock Dock { get; private set; }
        public bool IsExpanded { get; private set; } = true;
        public bool IsOpen { get; private set; } = true;
        public bool IsPresented { get; private set; } = true;
        public Vector2 FloatingPosition => new Vector2(style.left.value.value, style.top.value.value);
        public Vector2 FloatingSize => floatSize;
        public void Restore(WorkspaceDock dock, Vector2 position, Vector2 size)
        {
            if (WorkspaceBackend.IsFinite(size.x) && WorkspaceBackend.IsFinite(size.y)) floatSize = new Vector2(Mathf.Clamp(size.x, 160, 1200), Mathf.Clamp(size.y, 100, 1200));
            Place(dock);
            if (dock == WorkspaceDock.Floating && WorkspaceBackend.IsFinite(position.x) && WorkspaceBackend.IsFinite(position.y)) { style.left = position.x; style.top = position.y; schedule.Execute(() => ClampPosition(false)); }
        }
        public bool ShouldRefresh => IsOpen && IsExpanded && IsPresented;
        public event Action LayoutChanged;
        private readonly VisualElement surface, resizeHandle;
        private readonly Action<WorkspaceWindow, WorkspaceDock> relocate;
        private readonly WorkspaceDocking docking;
        private readonly Button collapse;
        private Vector2 startPointer, grabOffset, originalPosition, floatSize = new Vector2(360, 430);
        private WorkspaceDock originalDock;
        private int pointer = -1;
        private bool dragging, resizing;

        public WorkspaceWindow(string id, string title, VisualElement floating,
            Action<WorkspaceWindow, WorkspaceDock> relocate, WorkspaceDocking docking = null)
        {
            Id = id; name = id; surface = floating; this.relocate = relocate; this.docking = docking;
            AddToClassList("workspace-window");
            var header = new VisualElement(); header.AddToClassList("window-header"); Add(header);
            header.Add(new WorkspaceIcon(title));
            var dragTitle = new Label(title) { tooltip = "Drag to a workspace edge to dock. Escape cancels." };
            dragTitle.AddToClassList("window-title"); header.Add(dragTitle);
            collapse = new Button(ToggleExpanded) { text = "−", tooltip = "Collapse / expand" }; header.Add(collapse);
            header.Add(new Button(Close) { text = "×", tooltip = "Close panel" });
            Body.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            Body.AddToClassList("window-body"); Add(Body);
            resizeHandle = new VisualElement(); resizeHandle.AddToClassList("resize-handle"); Add(resizeHandle);
            dragTitle.RegisterCallback<PointerDownEvent>(e => Begin(e, false));
            resizeHandle.RegisterCallback<PointerDownEvent>(e => Begin(e, true));
            // Capture on the stable overlay: moving the window between parents cannot lose the drag.
            surface.RegisterCallback<PointerMoveEvent>(Move);
            surface.RegisterCallback<PointerUpEvent>(End);
            surface.RegisterCallback<PointerCaptureOutEvent>(e => { if (e.pointerId == pointer) Cancel(); });
            surface.RegisterCallback<KeyDownEvent>(e =>
            {
                if (pointer < 0 || e.keyCode != KeyCode.Escape) return;
                Cancel(); e.StopPropagation();
            });
            surface.RegisterCallback<GeometryChangedEvent>(_ => { if (Dock == WorkspaceDock.Floating) ClampPosition(original: false); });
        }
        private void Begin(PointerDownEvent e, bool resize)
        {
            if (e.button != 0 || pointer >= 0 || (resize && (Dock != WorkspaceDock.Floating || !IsExpanded))) return;
            pointer = e.pointerId; resizing = resize; dragging = false;
            startPointer = e.position; originalDock = Dock;
            originalPosition = surface.WorldToLocal(worldBound.position);
            grabOffset = (Vector2)e.position - worldBound.position;
            if (resize) floatSize = layout.size;
            surface.focusable = true; surface.Focus(); surface.CapturePointer(pointer);
            e.StopPropagation();
        }
        private void Move(PointerMoveEvent e)
        {
            if (e.pointerId != pointer) return;
            Vector2 delta = (Vector2)e.position - startPointer;
            if (resizing)
            {
                style.width = Mathf.Clamp(floatSize.x + delta.x, 250, Mathf.Max(250, surface.contentRect.width - layout.x));
                style.height = Mathf.Clamp(floatSize.y + delta.y, 120, Mathf.Max(120, surface.contentRect.height - layout.y));
            }
            else
            {
                if (!dragging && delta.sqrMagnitude < 25) return;
                if (!dragging)
                {
                    dragging = true;
                    if (Dock != WorkspaceDock.Floating) Place(WorkspaceDock.Floating);
                    grabOffset.x = Mathf.Min(grabOffset.x, floatSize.x - 24);
                }
                Vector2 pos = surface.WorldToLocal(e.position) - grabOffset;
                foreach (var child in surface.Children())
                {
                    if (!(child is WorkspaceWindow other) || other == this || !other.IsOpen) continue;
                    var otherPos = surface.WorldToLocal(other.worldBound.position);
                    if (Mathf.Abs(pos.x - (otherPos.x + other.layout.width)) < 12) pos.x = otherPos.x + other.layout.width;
                    if (Mathf.Abs(pos.x + layout.width - otherPos.x) < 12) pos.x = otherPos.x - layout.width;
                    if (Mathf.Abs(pos.y - (otherPos.y + other.layout.height)) < 12) pos.y = otherPos.y + other.layout.height;
                    if (Mathf.Abs(pos.y + layout.height - otherPos.y) < 12) pos.y = otherPos.y - layout.height;
                }
                style.left = pos.x; style.top = pos.y; ClampPosition(original: false);
                docking?.Show(e.position);
            }
            e.StopPropagation();
        }
        private void End(PointerUpEvent e)
        {
            if (e.pointerId != pointer || e.button != 0) return;
            var target = docking != null ? docking.Target(e.position) : WorkspaceDock.Floating;
            bool drop = dragging;
            if (resizing) floatSize = new Vector2(style.width.value.value, style.height.value.value);
            Release();
            if (drop && target != WorkspaceDock.Floating) Place(target);
            LayoutChanged?.Invoke();
            e.StopPropagation();
        }
        public void Cancel()
        {
            if (pointer < 0) return;
            bool restore = dragging;
            if (resizing) { style.width = floatSize.x; style.height = floatSize.y; }
            Release();
            if (restore) { Place(originalDock); if (originalDock == WorkspaceDock.Floating) ClampPosition(original: true); }
        }
        private void Release()
        {
            int captured = pointer; pointer = -1; dragging = resizing = false;
            docking?.Hide();
            if (captured >= 0) surface.ReleasePointer(captured);
        }
        public void SetPresented(bool presented)
        {
            IsPresented = presented;
            style.display = IsOpen && presented ? DisplayStyle.Flex : DisplayStyle.None;
        }
        public void Open() { IsOpen = true; SetPresented(true); if (Dock == WorkspaceDock.Floating) BringToFront(); LayoutChanged?.Invoke(); }
        public void Close() { Cancel(); IsOpen = false; SetPresented(false); LayoutChanged?.Invoke(); }
        public void ToggleExpanded()
        {
            IsExpanded = !IsExpanded; collapse.text = IsExpanded ? "−" : "+";
            Body.style.display = IsExpanded ? DisplayStyle.Flex : DisplayStyle.None;
            ApplyGeometry(); LayoutChanged?.Invoke();
        }
        public void Place(WorkspaceDock dock)
        {
            Dock = dock; EnableInClassList("is-floating", dock == WorkspaceDock.Floating);
            ApplyGeometry();
            style.left = dock == WorkspaceDock.Floating ? new Length(40) : new StyleLength(StyleKeyword.Auto);
            style.top = dock == WorkspaceDock.Floating ? new Length(30) : new StyleLength(StyleKeyword.Auto);
            relocate(this, dock); Open();
            if (dock == WorkspaceDock.Floating) schedule.Execute(() => ClampPosition(false));
        }
        private void ApplyGeometry()
        {
            bool isFloat = Dock == WorkspaceDock.Floating;
            resizeHandle.style.display = IsExpanded && isFloat ? DisplayStyle.Flex : DisplayStyle.None;
            style.position = isFloat ? Position.Absolute : Position.Relative;
            style.width = isFloat ? new Length(floatSize.x) : new StyleLength(StyleKeyword.Auto);
            style.height = isFloat ? new Length(IsExpanded ? floatSize.y : 32) : new Length(0);
            style.flexGrow = isFloat ? 0 : 1;
        }
        private void ClampPosition(bool original)
        {
            Vector2 pos = original ? originalPosition : new Vector2(style.left.value.value, style.top.value.value);
            if (surface.contentRect.width > 0 && surface.contentRect.height > 0)
            {
                style.width = Mathf.Min(floatSize.x, surface.contentRect.width);
                style.height = Mathf.Min(IsExpanded ? floatSize.y : 32, surface.contentRect.height);
            }
            float width = style.width.value.value;
            style.left = Mathf.Clamp(pos.x, 0, Mathf.Max(0, surface.contentRect.width - width));
            style.top = Mathf.Clamp(pos.y, 0, Mathf.Max(0, surface.contentRect.height - (IsExpanded ? style.height.value.value : 32)));
        }
    }
}
