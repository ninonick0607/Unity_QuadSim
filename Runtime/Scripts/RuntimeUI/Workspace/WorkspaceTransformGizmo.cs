using System;
using MathUtil;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public static class WorkspaceAxes
    {
        public static Vector3 Direction(int axis, SimFrame frame, bool body, Quaternion rotation)
        {
            var unit = axis == 0 ? Vector3.right : axis == 1 ? Vector3.up : Vector3.forward;
            return body ? rotation * Frames.InverseTransformLinear(unit, frame) : Frames.InverseTransformWorldPosition(unit, frame);
        }
        public static string Caption(SimFrame frame, bool body) => frame == SimFrame.UnityBody ? "Unity XYZ" :
            body ? frame == SimFrame.FRD ? "FRD · BODY" : "FLU · BODY" : frame == SimFrame.FRD ? "NED · WORLD" : "ENU · WORLD";
        public static string Label(int axis, SimFrame frame, bool body) => axis == 0 ? "X" : axis == 1 ? "Y" : "Z";
        public static readonly Color[] Colors = { new Color(1, .32f, .35f), new Color(.35f, .9f, .45f), new Color(.3f, .65f, 1) };
        public static float DistanceToSegment(Vector2 point, Vector2 start, Vector2 end)
        {
            var delta = end - start;
            float t = delta.sqrMagnitude > .01f ? Mathf.Clamp01(Vector2.Dot(point - start, delta) / delta.sqrMagnitude) : 0;
            return Vector2.Distance(point, start + delta * t);
        }
        public static void Arrow(Painter2D painter, Vector2 start, Vector2 end, Color color, float width = 3)
        {
            var direction = end - start;
            painter.strokeColor = painter.fillColor = color; painter.lineWidth = width;
            if (direction.magnitude < 8)
            {
                painter.BeginPath(); painter.Arc(end, 5, Angle.Degrees(0), Angle.Degrees(360)); painter.Stroke(); return;
            }
            direction.Normalize();
            var side = new Vector2(-direction.y, direction.x) * 5;
            painter.BeginPath(); painter.MoveTo(start); painter.LineTo(end - direction * 8); painter.Stroke();
            painter.BeginPath(); painter.MoveTo(end); painter.LineTo(end - direction * 12 + side);
            painter.LineTo(end - direction * 12 - side); painter.ClosePath(); painter.Fill();
        }
    }

    /// <summary>Screen-sized runtime translate handles, projected from the actual frame basis.</summary>
    public sealed class WorkspaceTransformGizmo : VisualElement
    {
        private readonly WorkspaceSceneService scene;
        private readonly Func<Camera> getCamera;
        private readonly Func<SimFrame> getFrame;
        private readonly Func<bool> getBody;
        private readonly Action changed;
        private readonly Vector2[] ends = new Vector2[3];
        private readonly Label[] labels = new Label[3];
        private Vector2 center, dragStart, dragVector;
        private Vector3 startPosition, startScale, dragAxis;
        private Quaternion startRotation;
        private Transform dragged;
        private int axis = -1, pointer = -1;
        private float worldLength;
        private bool visible;
        public bool IsDragging => pointer >= 0;

        public WorkspaceTransformGizmo(WorkspaceSceneService scene, Func<Camera> camera, Func<SimFrame> frame, Func<bool> body, Action changed)
        {
            this.scene = scene; getCamera = camera; getFrame = frame; getBody = body; this.changed = changed;
            AddToClassList("transform-gizmo"); focusable = true;
            for (int i = 0; i < 3; i++)
            {
                labels[i] = new Label { pickingMode = PickingMode.Ignore };
                labels[i].AddToClassList("axis-label"); labels[i].style.color = WorkspaceAxes.Colors[i]; Add(labels[i]);
            }
            generateVisualContent += Draw;
            RegisterCallback<PointerDownEvent>(Begin);
            RegisterCallback<PointerMoveEvent>(Move);
            RegisterCallback<PointerUpEvent>(e => { if (e.pointerId == pointer && e.button == 0) { End(); e.StopPropagation(); } });
            RegisterCallback<PointerCaptureOutEvent>(_ => { pointer = -1; axis = -1; dragged = null; });
            RegisterCallback<KeyDownEvent>(e => { if (e.keyCode == KeyCode.Escape && IsDragging) { Cancel(); e.StopPropagation(); } });
        }
        private Vector2 Project(Camera camera, Vector3 point)
        {
            var screen = camera.WorldToScreenPoint(point);
            var panelPoint = RuntimePanelUtils.ScreenToPanel(panel, new Vector2(screen.x, Screen.height - screen.y));
            return this.WorldToLocal(panelPoint);
        }
        public void Refresh()
        {
            if (IsDragging && (!scene.CanEdit || scene.Selected != dragged)) End();
            var camera = getCamera();
            visible = camera != null && scene.HasSelection && panel != null &&
                camera.WorldToScreenPoint(scene.Selected.position).z > camera.nearClipPlane;
            if (visible)
            {
                var selected = scene.Selected;
                center = Project(camera, selected.position);
                worldLength = 84 / Mathf.Max(.001f, Vector2.Distance(center, Project(camera, selected.position + camera.transform.up)));
                for (int i = 0; i < 3; i++)
                {
                    var direction = WorkspaceAxes.Direction(i, getFrame(), getBody(), selected.rotation);
                    ends[i] = Project(camera, selected.position + direction * worldLength);
                    labels[i].text = WorkspaceAxes.Label(i, getFrame(), getBody());
                    labels[i].style.translate = new Translate(ends[i].x + 6, ends[i].y - 18);
                }
            }
            foreach (var label in labels) label.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
            MarkDirtyRepaint();
        }
        public override bool ContainsPoint(Vector2 localPoint) => Hit(localPoint) >= 0;
        private int Hit(Vector2 point)
        {
            if (!visible || !scene.CanEdit) return -1;
            int found = -1; float best = 10;
            for (int i = 0; i < 3; i++)
            {
                // Looking straight down an axis cannot define a stable drag.
                if ((ends[i] - center).sqrMagnitude < 64) continue;
                var start = Vector2.Lerp(center, ends[i], .18f);
                float distance = WorkspaceAxes.DistanceToSegment(point, start, ends[i]);
                if (distance < best) { found = i; best = distance; }
            }
            return found;
        }
        private void Begin(PointerDownEvent e)
        {
            if (e.button != 0 || IsDragging) return;
            Refresh(); axis = Hit(e.localPosition);
            if (axis < 0) return;
            dragged = scene.Selected; startPosition = dragged.position; startRotation = dragged.rotation; startScale = dragged.localScale;
            dragAxis = WorkspaceAxes.Direction(axis, getFrame(), getBody(), startRotation);
            dragStart = e.localPosition; dragVector = (ends[axis] - center) / worldLength;
            pointer = e.pointerId; Focus(); this.CapturePointer(pointer); e.StopPropagation();
        }
        private void Move(PointerMoveEvent e)
        {
            if (e.pointerId != pointer) return;
            float distance = Vector2.Dot((Vector2)e.localPosition - dragStart, dragVector) / dragVector.sqrMagnitude;
            if (e.shiftKey) distance *= .1f;
            if (e.ctrlKey) distance = Mathf.Round(distance / .25f) * .25f;
            if (!scene.ApplyTransform(dragged, startPosition + dragAxis * distance, startRotation, startScale)) End();
            else changed();
            e.StopPropagation();
        }
        public void Cancel()
        {
            if (!IsDragging) return;
            if (scene.ApplyTransform(dragged, startPosition, startRotation, startScale)) changed();
            End();
        }
        public void End()
        {
            int captured = pointer; pointer = -1; axis = -1; dragged = null;
            if (captured >= 0 && this.HasPointerCapture(captured)) this.ReleasePointer(captured);
        }
        private void Draw(MeshGenerationContext context)
        {
            if (!visible) return;
            for (int i = 0; i < 3; i++)
            {
                var color = i == axis ? Color.yellow : WorkspaceAxes.Colors[i];
                if (!scene.CanEdit) color.a = .4f;
                WorkspaceAxes.Arrow(context.painter2D, center, ends[i], color, i == axis ? 5 : 3);
            }
        }
    }

    public sealed class WorkspaceAxisCompass : VisualElement
    {
        private Camera camera;
        private SimFrame frame;
        private readonly Label[] labels = new Label[3];
        private readonly Vector2[] ends = new Vector2[3];
        public WorkspaceAxisCompass()
        {
            AddToClassList("axis-compass"); pickingMode = PickingMode.Ignore;
            for (int i = 0; i < 3; i++)
            {
                labels[i] = new Label { pickingMode = PickingMode.Ignore }; labels[i].AddToClassList("axis-label");
                labels[i].style.color = WorkspaceAxes.Colors[i]; Add(labels[i]);
            }
            generateVisualContent += ctx =>
            {
                if (camera == null) return;
                for (int i = 0; i < 3; i++) WorkspaceAxes.Arrow(ctx.painter2D, new Vector2(64, 65), ends[i], WorkspaceAxes.Colors[i], 2);
            };
        }
        public void Refresh(Camera camera, SimFrame frame)
        {
            this.camera = camera; this.frame = frame;
            if (camera == null) return;
            for (int i = 0; i < 3; i++)
            {
                var direction = camera.transform.InverseTransformDirection(WorkspaceAxes.Direction(i, frame, false, Quaternion.identity));
                ends[i] = new Vector2(64 + direction.x * 32, 65 - direction.y * 32);
                labels[i].text = WorkspaceAxes.Label(i, frame, false);
                labels[i].style.translate = new Translate(ends[i].x + 4, ends[i].y - 14);
            }
            MarkDirtyRepaint();
        }
    }
}
