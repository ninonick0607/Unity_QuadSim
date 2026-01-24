using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.UI
{
    public enum DockSide { Floating, Left, Right, Bottom, Top }

    public sealed class DockableWindowController
    {
        private readonly VisualElement _hudRoot;
        private readonly VisualElement _window;
        private readonly VisualElement _titleBar;

        private readonly VisualElement _tl;
        private readonly VisualElement _tr;
        private readonly VisualElement _bl;
        private readonly VisualElement _br;
        
        private readonly VisualElement _l;
        private readonly VisualElement _r;
        private readonly VisualElement _t;
        private readonly VisualElement _b;

        public DockSide Dock { get; private set; } = DockSide.Bottom;

        // Floating rect
        private float _x = 40f;
        private float _y = 80f;
        private float _w = 620f;
        private float _h = 380f;

        // Remembered floating rect (used when undocking)
        private float _lastFloatX = 40f, _lastFloatY = 80f, _lastFloatW = 620f, _lastFloatH = 380f;

        // Dock thickness: for Bottom/Top it's height; for Left/Right it's width
        private float _dockThickness = 360f;

        // Constraints
        private float _minW = 360f;
        private float _minH = 220f;

        // Safe area (TopBar height)
        private float _topSafePx = 44f;

        // Snap threshold (only used for title-bar drag release)
        private float _snapPx = 16f;

        // Drag
        private bool _dragging;
        private Vector2 _dragOffsetInHud;

        // Resize
        private bool _resizing;
        private Vector2 _resizeStartMouseHud;
        private float _resizeStartX, _resizeStartY, _resizeStartW, _resizeStartH;
        private int _resizeDirX; // -1 left, +1 right
        private int _resizeDirY; // -1 top, +1 bottom

        public DockableWindowController(
            VisualElement hudRoot,
            VisualElement window,
            VisualElement titleBar,
            VisualElement resizeTL,
            VisualElement resizeTR,
            VisualElement resizeBL,
            VisualElement resizeBR,
            VisualElement resizeL,
            VisualElement resizeR,
            VisualElement resizeT,
            VisualElement resizeB)
        {
            _hudRoot = hudRoot;
            _window = window;
            _titleBar = titleBar;

            _tl = resizeTL; _tr = resizeTR; _bl = resizeBL; _br = resizeBR;
            _l = resizeL; _r = resizeR; _t = resizeT; _b = resizeB;

            _window.style.position = Position.Absolute;

            HookDrag();

            HookResizeCorner(_tl, -1, -1);
            HookResizeCorner(_tr, +1, -1);
            HookResizeCorner(_bl, -1, +1);
            HookResizeCorner(_br, +1, +1);

            HookResizeEdge(_l, -1,  0);
            HookResizeEdge(_r, +1,  0);
            HookResizeEdge(_t,  0, -1);
            HookResizeEdge(_b,  0, +1);
            
            // Keep placement stable on window resize / resolution changes
            _hudRoot.RegisterCallback<GeometryChangedEvent>(_ => ApplyLayout());
        }

        // ---- Public API ----

        public void SetConstraints(float minW, float minH)
        {
            _minW = Mathf.Max(1f, minW);
            _minH = Mathf.Max(1f, minH);
            ApplyLayout();
        }

        public void SetTopSafeArea(float px)
        {
            _topSafePx = Mathf.Max(0f, px);
            ApplyLayout();
        }

        public void SetSnapPixels(float px)
        {
            _snapPx = Mathf.Max(0f, px);
        }

        public void SetInitialFloating(float x, float y, float w, float h)
        {
            _x = x; _y = y; _w = w; _h = h;
            _lastFloatX = x; _lastFloatY = y; _lastFloatW = w; _lastFloatH = h;
            ApplyLayout();
        }

        public void SetDockThickness(float px)
        {
            _dockThickness = Mathf.Max(1f, px);
            ApplyLayout();
        }

        public void SetDock(DockSide dock)
        {
            Dock = dock;
            ApplyLayout();
        }

        public void ApplyLayout()
        {
            float hudW = _hudRoot.resolvedStyle.width;
            float hudH = _hudRoot.resolvedStyle.height;
            if (hudW <= 1f || hudH <= 1f) return;

            float safeTop = _topSafePx;
            float usableH = Mathf.Max(0f, hudH - safeTop);

            // Clamp dock thickness to usable dimension
            if (Dock == DockSide.Left || Dock == DockSide.Right)
                _dockThickness = Mathf.Clamp(_dockThickness, _minW, hudW);
            else if (Dock == DockSide.Top || Dock == DockSide.Bottom)
                _dockThickness = Mathf.Clamp(_dockThickness, _minH, usableH);

            // Clamp floating rect against safe area
            _w = Mathf.Clamp(_w, _minW, hudW);
            _h = Mathf.Clamp(_h, _minH, usableH);

            _x = Mathf.Clamp(_x, 0f, Mathf.Max(0f, hudW - _w));
            _y = Mathf.Clamp(_y, safeTop, Mathf.Max(safeTop, hudH - _h));

            switch (Dock)
            {
                case DockSide.Floating:
                    SetRect(_x, _y, _w, _h);
                    break;

                case DockSide.Bottom:
                    SetRect(0f, hudH - _dockThickness, hudW, _dockThickness);
                    break;

                case DockSide.Top:
                    SetRect(0f, safeTop, hudW, _dockThickness);
                    break;

                case DockSide.Left:
                    SetRect(0f, safeTop, _dockThickness, hudH - safeTop);
                    break;

                case DockSide.Right:
                    SetRect(hudW - _dockThickness, safeTop, _dockThickness, hudH - safeTop);
                    break;
            }
        }

        // ---- Internals ----

        private void SetRect(float x, float y, float w, float h)
        {
            _window.style.left = x;
            _window.style.top = y;
            _window.style.width = w;
            _window.style.height = h;
        }

        private void EnsureFloatingFromDock()
        {
            if (Dock == DockSide.Floating) return;

            // Switch to last remembered floating rect (Unity-like behavior)
            Dock = DockSide.Floating;
            _x = _lastFloatX;
            _y = _lastFloatY;
            _w = _lastFloatW;
            _h = _lastFloatH;
        }

        private void SaveFloatingRect()
        {
            if (Dock != DockSide.Floating) return;
            _lastFloatX = _x; _lastFloatY = _y; _lastFloatW = _w; _lastFloatH = _h;
        }

        private void HookDrag()
        {
            if (_titleBar == null) return;

            _titleBar.RegisterCallback<PointerDownEvent>(evt =>
            {
                _dragging = true;

                // Pointer in HUD coordinates
                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);

                // If docked, undock to last floating
                EnsureFloatingFromDock();
                ApplyLayout();

                // Offset inside window so it doesn't jump
                _dragOffsetInHud = mouseHud - new Vector2(_x, _y);

                _titleBar.CapturePointer(evt.pointerId);
                evt.StopPropagation();
            });

            _titleBar.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if (!_dragging) return;

                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);

                _x = mouseHud.x - _dragOffsetInHud.x;
                _y = mouseHud.y - _dragOffsetInHud.y;

                ApplyLayout();
                evt.StopPropagation();
            });

            _titleBar.RegisterCallback<PointerUpEvent>(evt =>
            {
                if (!_dragging) return;
                _dragging = false;

                if (_titleBar.HasPointerCapture(evt.pointerId))
                    _titleBar.ReleasePointer(evt.pointerId);

                SaveFloatingRect();

                // Snap ONLY on titlebar drag release (desktop behavior)
                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);
                SnapIfNearEdge(mouseHud);

                ApplyLayout();
                evt.StopPropagation();
            });
        }

        private void HookResizeEdge(VisualElement handle, int dirX, int dirY)
        {
            if (handle == null) return;

            handle.RegisterCallback<PointerDownEvent>(evt =>
            {
                _resizing = true;

                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);
                _resizeStartMouseHud = mouseHud;

                _resizeDirX = dirX;
                _resizeDirY = dirY;

                if (Dock == DockSide.Floating)
                {
                    _resizeStartX = _x;
                    _resizeStartY = _y;
                    _resizeStartW = _w;
                    _resizeStartH = _h;
                }

                handle.CapturePointer(evt.pointerId);
                evt.StopPropagation();
            });

            handle.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if (!_resizing) return;

                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);
                Vector2 d = mouseHud - _resizeStartMouseHud;

                float hudW = _hudRoot.resolvedStyle.width;
                float hudH = _hudRoot.resolvedStyle.height;
                float safeTop = _topSafePx;
                float usableH = Mathf.Max(0f, hudH - safeTop);

                if (Dock == DockSide.Floating)
                {
                    // Floating edge resize
                    if (_resizeDirX != 0)
                    {
                        if (_resizeDirX < 0)
                        {
                            _x = _resizeStartX + d.x;
                            _w = _resizeStartW - d.x;
                        }
                        else
                        {
                            _w = _resizeStartW + d.x;
                        }
                    }

                    if (_resizeDirY != 0)
                    {
                        if (_resizeDirY < 0)
                        {
                            _y = _resizeStartY + d.y;
                            _h = _resizeStartH - d.y;
                        }
                        else
                        {
                            _h = _resizeStartH + d.y;
                        }
                    }

                    _w = Mathf.Clamp(_w, _minW, hudW);
                    _h = Mathf.Clamp(_h, _minH, usableH);

                    ApplyLayout();
                }
                else
                {
                    // Docked edge resize adjusts thickness, stays docked
                    if (Dock == DockSide.Left || Dock == DockSide.Right)
                    {
                        // Only respond to X edges when docked left/right
                        float delta = d.x;

                        // For right dock, moving left increases thickness (invert)
                        if (Dock == DockSide.Right) delta = -delta;

                        _dockThickness = Mathf.Clamp(_dockThickness + delta, _minW, hudW);
                    }
                    else if (Dock == DockSide.Top || Dock == DockSide.Bottom)
                    {
                        // Only respond to Y edges when docked top/bottom
                        float delta = d.y;

                        // For bottom dock, moving up increases thickness (invert)
                        if (Dock == DockSide.Bottom) delta = -delta;

                        _dockThickness = Mathf.Clamp(_dockThickness + delta, _minH, usableH);
                    }

                    ApplyLayout();
                }

                evt.StopPropagation();
            });

            handle.RegisterCallback<PointerUpEvent>(evt =>
            {
                if (!_resizing) return;
                _resizing = false;

                if (handle.HasPointerCapture(evt.pointerId))
                    handle.ReleasePointer(evt.pointerId);

                // No snapping on resize end
                if (Dock == DockSide.Floating) SaveFloatingRect();

                ApplyLayout();
                evt.StopPropagation();
            });
        }

        
        private void HookResizeCorner(VisualElement handle, int dirX, int dirY)
        {
            if (handle == null) return;

            handle.RegisterCallback<PointerDownEvent>(evt =>
            {
                _resizing = true;

                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);
                _resizeStartMouseHud = mouseHud;

                // If docked, resizing should stay docked and change thickness (Unity-like)
                // If floating, we resize the floating rect.
                if (Dock == DockSide.Floating)
                {
                    _resizeStartX = _x;
                    _resizeStartY = _y;
                    _resizeStartW = _w;
                    _resizeStartH = _h;

                    _resizeDirX = dirX;
                    _resizeDirY = dirY;
                }
                else
                {
                    // When docked, we interpret "resize" as changing dock thickness.
                    // Direction chooses whether we resize width (left/right dock) or height (top/bottom dock).
                    _resizeDirX = dirX;
                    _resizeDirY = dirY;
                }

                handle.CapturePointer(evt.pointerId);
                evt.StopPropagation();
            });

            handle.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if (!_resizing) return;

                Vector2 mouseHud = _hudRoot.WorldToLocal(evt.position);
                Vector2 d = mouseHud - _resizeStartMouseHud;

                float hudW = _hudRoot.resolvedStyle.width;
                float hudH = _hudRoot.resolvedStyle.height;
                float safeTop = _topSafePx;
                float usableH = Mathf.Max(0f, hudH - safeTop);

                if (Dock == DockSide.Floating)
                {
                    // Floating corner resize
                    if (_resizeDirX < 0)
                    {
                        _x = _resizeStartX + d.x;
                        _w = _resizeStartW - d.x;
                    }
                    else
                    {
                        _w = _resizeStartW + d.x;
                    }

                    if (_resizeDirY < 0)
                    {
                        _y = _resizeStartY + d.y;
                        _h = _resizeStartH - d.y;
                    }
                    else
                    {
                        _h = _resizeStartH + d.y;
                    }

                    // Apply constraints/safe area
                    _w = Mathf.Clamp(_w, _minW, hudW);
                    _h = Mathf.Clamp(_h, _minH, usableH);

                    ApplyLayout();
                }
                else
                {
                    // Docked resize changes thickness only, stays docked
                    if (Dock == DockSide.Left || Dock == DockSide.Right)
                    {
                        // Horizontal panels: thickness is width
                        float delta = d.x;
                        // If docked on left, dragging right increases; on right, dragging left increases
                        if (Dock == DockSide.Right) delta = -delta;

                        _dockThickness = Mathf.Clamp(_dockThickness + delta, _minW, hudW);
                    }
                    else
                    {
                        // Top/Bottom panels: thickness is height
                        float delta = d.y;
                        // Bottom: dragging up increases height; Top: dragging down increases height
                        if (Dock == DockSide.Bottom) delta = -delta;

                        _dockThickness = Mathf.Clamp(_dockThickness + delta, _minH, usableH);
                    }

                    ApplyLayout();
                }

                evt.StopPropagation();
            });

            handle.RegisterCallback<PointerUpEvent>(evt =>
            {
                if (!_resizing) return;
                _resizing = false;

                if (handle.HasPointerCapture(evt.pointerId))
                    handle.ReleasePointer(evt.pointerId);

                // No snapping on resize end (desktop behavior)
                if (Dock == DockSide.Floating)
                    SaveFloatingRect();

                ApplyLayout();
                evt.StopPropagation();
            });
        }

        private void SnapIfNearEdge(Vector2 mouseHud)
        {
            float hudW = _hudRoot.resolvedStyle.width;
            float hudH = _hudRoot.resolvedStyle.height;
            if (hudW <= 1f || hudH <= 1f) return;

            float safeTop = _topSafePx;

            // Distances to usable edges (top edge is safeTop, not 0)
            float distLeft = mouseHud.x;
            float distRight = hudW - mouseHud.x;
            float distTop = Mathf.Max(0f, mouseHud.y - safeTop);
            float distBottom = hudH - mouseHud.y;

            DockSide target = DockSide.Floating;
            float best = _snapPx;

            if (distLeft < best) { best = distLeft; target = DockSide.Left; }
            if (distRight < best) { best = distRight; target = DockSide.Right; }
            if (distBottom < best) { best = distBottom; target = DockSide.Bottom; }
            if (distTop < best) { best = distTop; target = DockSide.Top; }

            if (target != DockSide.Floating)
            {
                Dock = target;

                // Use current floating size as thickness seed (Unity-like)
                if (Dock == DockSide.Left || Dock == DockSide.Right)
                    _dockThickness = _w;
                else
                    _dockThickness = _h;
            }
        }
        
    }
}
