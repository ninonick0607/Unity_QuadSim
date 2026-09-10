using System.Collections.Generic;
using DroneCore;
using MathUtil;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed partial class RuntimeWorkspace
    {
        private WorkspaceTransformGizmo transformGizmo;
        private WorkspaceAxisCompass compass;
        private DropdownField gizmoSpace, gizmoFrame;
        private Label editHint;
        private string lastAxes;
        private bool lastEditState;
        private SimFrame poseFrame;
        private readonly List<Transform> sceneObjects = new List<Transform>();
        private SimFrame DisplayFrame
        {
            get
            {
                if (gizmoFrame == null || gizmoFrame.index == 0) return highlightedPawn?.Sensors != null ? highlightedPawn.Sensors.outputFrame : Flight?.Pawn?.Sensors != null ? Flight.Pawn.Sensors.outputFrame : SimFrame.FLU;
                return gizmoFrame.index == 1 ? SimFrame.FLU : gizmoFrame.index == 2 ? SimFrame.FRD : SimFrame.UnityBody;
            }
        }
        private void BuildHierarchy(VisualElement parent)
        {
            Section(parent, "Hierarchy");
            hierarchy = new WorkspaceHierarchy();
            hierarchy.Selected += target => SelectSceneObject(target, highlight: true);
            hierarchy.FrameRequested += target => cameraDirector?.FrameTarget(target);
            parent.Add(hierarchy);
            parent.contentContainer.style.flexGrow = 1;
            AddButton(parent, "Clear selection", ClearSceneSelection).tooltip = "Remove selection, highlight and handles (Escape)";
            Text(parent, "Select a row to highlight · Double-click to frame").AddToClassList("panel-hint");
            root.RegisterCallback<KeyDownEvent>(e =>
            {
                if (e.keyCode != KeyCode.Escape || transformGizmo?.IsDragging == true ||
                    Editing(position) || Editing(rotation) || Editing(scale) || taskMenu != null ||
                    (approval != null && approval.style.display != DisplayStyle.None)) return;
                ClearSceneSelection(); e.StopPropagation();
            }, TrickleDown.TrickleDown);
        }
        private void ClearSceneSelection()
        {
            SelectSceneObject(null);
            cameraInputActive = false;
            keys.Clear();
        }
        private void QueueInspectorTransform()
        {
            poseDirty = true;
            var expected = poseTarget;
            // The composite vector updates during field dispatch. Commit after that dispatch finishes.
            root.schedule.Execute(() =>
            {
                if (!poseDirty || poseTarget != expected) return;
                bool applied = ApplySelectedTransform();
                feedback.text = applied ? "Transform applied." : "Not applied. Enable scene editing and use finite values with positive scale.";
                if (!applied) ReadPose();
            });
        }

        private void BuildSceneTools()
        {
            lastAxes = null;
            var tools = new VisualElement(); tools.AddToClassList("scene-tools"); viewport.Add(tools);
            var edit = AddButton(tools, "Edit scene", EnableSceneEditing);
            edit.tooltip = "Pause simulation and take UI control to spawn or transform objects";
            gizmoSpace = new DropdownField(new List<string> { "World", "Body" }, 0);
            gizmoSpace.tooltip = "Translate along world axes or the selected object's body axes"; tools.Add(gizmoSpace);
            gizmoFrame = new DropdownField(new List<string> { "Sensor frame", "FLU / ENU", "FRD / NED", "Unity XYZ" }, 0);
            gizmoFrame.tooltip = "Sensor frame follows the selected drone's FLU / FRD setting"; tools.Add(gizmoFrame);
            tools.RegisterCallback<PointerDownEvent>(e => e.StopPropagation());
            tools.RegisterCallback<WheelEvent>(e => e.StopPropagation());
            gizmoSpace.RegisterValueChangedCallback(_ => transformGizmo?.End());
            gizmoFrame.RegisterValueChangedCallback(_ => transformGizmo?.End());
            compass = new WorkspaceAxisCompass(); viewport.Add(compass);
            editHint = new Label { pickingMode = PickingMode.Ignore }; editHint.AddToClassList("scene-edit-hint"); viewport.Add(editHint);
        }
        private void EnableSceneEditing()
        {
            backend.SwitchToUI();
            if (!backend.Status.IsPaused) backend.TogglePause();
            nextRefresh = 0; RefreshObjectControls();
        }
        private void SelectSceneObject(Transform target, bool highlight = false)
        {
            transformGizmo?.End();
            backend.Scene?.Select(target);
            cameraSelection = target;
            highlightedPawn = target != null ? target.GetComponent<QuadPawn>() : null;
            if (highlightedPawn != null) backend.Select(highlightedPawn.AgentId);
            hierarchyHighlight = highlight && target != null;
            selectionPass?.Select(hierarchyHighlight ? target : null);
            if (position != null) { ReadPose(); RefreshSceneInspector(); }
            if (hierarchy != null && backend.Scene != null) RefreshObjects();
            nextRefresh = 0;
        }
        private void PrepareTransformEdit(Transform target)
        {
            if (cameraDirector != null && cameraDirector.Target != null &&
                (cameraDirector.Target == target || cameraDirector.Target.IsChildOf(target))) cameraDirector.ReleaseTarget();
        }
        private bool ApplySelectedTransform()
        {
            bool applied = backend.Scene != null && backend.Scene.ApplyTransform(poseTarget,
                Frames.InverseTransformWorldPosition(position.value, poseFrame), Quaternion.Euler(rotation.value), scale.value);
            if (applied) poseDirty = false;
            return applied;
        }
        private void RefreshSceneInspector()
        {
            var scene = backend.Scene;
            bool selected = scene?.HasSelection == true;
            if (poseTarget != (selected ? scene.Selected : null)) ReadPose();
            if (windows["Inspector"].ShouldRefresh)
            {
                if (!poseDirty && !Editing(position) && !Editing(rotation) && !Editing(scale)) ReadPose();
                if (!selected) inspector.SetReport("Select a drone or scene object to inspect its transform.");
                else if (scene.Selected.GetComponent<QuadPawn>() != null) inspector.SetReport(backend.ReadInspector());
                else inspector.SetReport($"SCENE OBJECT\nName   {scene.Selected.name}\nFrame   World position · Unity rotation\nPhysics   {(scene.Selected.GetComponent<Rigidbody>() != null ? "Rigidbody" : "Static / marker")}");
            }
            bool canEdit = scene?.CanEdit == true;
            position.SetEnabled(canEdit); rotation.SetEnabled(canEdit);
            scale.SetEnabled(scene?.CanScale == true);
        }
        private void RefreshViewportTools()
        {
            if (backend.Scene == null || cameraDirector == null) return;
            if (transformGizmo == null)
            {
                backend.Scene.TransformChanging += PrepareTransformEdit;
                transformGizmo = new WorkspaceTransformGizmo(backend.Scene, () => cameraDirector.ViewCamera,
                    () => DisplayFrame, () => gizmoSpace.index == 1, ReadPose);
                viewport.Insert(0, transformGizmo);
            }
            transformGizmo.Refresh();
            compass.Refresh(cameraDirector.ViewCamera, DisplayFrame);
            string axes = WorkspaceAxes.Caption(DisplayFrame, gizmoSpace.index == 1);
            if (axes == lastAxes && lastEditState == backend.Level.CanEdit) return;
            lastAxes = axes; lastEditState = backend.Level.CanEdit;
            editHint.text = lastEditState
                ? $"EDIT · {axes} · Drag axis to move · Ctrl snap 0.25 m · Shift precision · Esc cancel"
                : $"VIEW · {axes} · Edit scene to enable transforms";
        }
    }
}
