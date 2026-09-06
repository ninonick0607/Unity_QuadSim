using DroneCore;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.Rendering;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed partial class RuntimeWorkspace
    {
        private WorkspaceSelectionPass selectionPass;
        private UnityEngine.Rendering.HighDefinition.CustomPassVolume selectionVolume;
        private QuadPawn highlightedPawn;
        private Camera framedCamera;
        private void BuildSelectionHighlight()
        {
            highlightedPawn = null;
            selectionPass = new WorkspaceSelectionPass { name = "QuadSim explicit selection" };
            selectionVolume = document.gameObject.AddComponent<UnityEngine.Rendering.HighDefinition.CustomPassVolume>();
            selectionVolume.isGlobal = true;
            selectionVolume.injectionPoint = UnityEngine.Rendering.HighDefinition.CustomPassInjectionPoint.AfterPostProcess;
            selectionVolume.customPasses.Add(selectionPass);
            viewport.style.overflow = Overflow.Hidden;
        }
        private void HighlightEntity(string id)
        {
            highlightedPawn = string.IsNullOrEmpty(id) ? null : DroneCore.Core.DroneManager.Get()?.GetDroneByAgentId(id);
            selectionPass?.Select(highlightedPawn != null ? highlightedPawn.transform : null);
        }
        private void ReleaseSelectionHighlight()
        {
            if (selectionVolume != null) selectionVolume.enabled = false;
            selectionPass = null; highlightedPawn = null;
        }
        private void LateUpdate()
        {
            FrameViewport();
            UpdateSelectionHighlight();
        }
        private void RestoreViewportCamera()
        {
            if (framedCamera != null) framedCamera.ResetProjectionMatrix();
            framedCamera = null;
        }
        private void FrameViewport()
        {
            if (cameraDirector == null || root == null) return;
            var camera = cameraDirector.ViewCamera;
            if (camera != framedCamera) { RestoreViewportCamera(); framedCamera = camera; }
            if (camera == null) return;
            camera.ResetProjectionMatrix();
            // Clear the entire frame, then shift only the display-camera projection to
            // keep the followed drone inside the unobscured canvas. Sensor views are untouched.
            var pawn = Flight?.Pawn;
            if (uiHidden || pawn == null || cameraDirector.Mode != DroneCore.CameraSystem.CameraViewMode.Follow || root.layout.width <= 0) return;
            var current = camera.WorldToViewportPoint(pawn.transform.position);
            if (current.z <= camera.nearClipPlane) return;
            var center = root.WorldToLocal(viewport.worldBound.center);
            float x = center.x / root.layout.width, y = 1 - (center.y + 14) / root.layout.height;
            var projection = camera.projectionMatrix;
            projection.m02 -= (x - current.x) * 2;
            projection.m12 -= (y - current.y) * 2;
            camera.projectionMatrix = projection;
        }
        private void UpdateSelectionHighlight()
        {
            if (selectionPass == null) return;
            selectionPass.DisplayCamera = cameraDirector != null ? cameraDirector.ViewCamera : null;
            selectionPass.Visible = !uiHidden && highlightedPawn != null &&
                !(cameraDirector != null && cameraDirector.Mode == DroneCore.CameraSystem.CameraViewMode.FPV && cameraDirector.TargetDrone == highlightedPawn);
        }
    }
}
