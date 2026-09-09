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
        private Transform cameraSelection;
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
            BuildSceneTools();
        }
        private void HighlightEntity(string id)
        {
            var pawn = string.IsNullOrEmpty(id) ? null : DroneCore.Core.DroneManager.Get()?.GetDroneByAgentId(id);
            SelectSceneObject(pawn != null ? pawn.transform : null);
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
            if (root != null && !uiHidden) RefreshViewportTools();
        }
        private void RestoreViewportCamera()
        {
            if (framedCamera != null) { framedCamera.ResetProjectionMatrix(); framedCamera.ResetAspect(); }
            framedCamera = null;
        }
        private void FrameViewport()
        {
            if (cameraDirector == null || root == null) return;
            var camera = cameraDirector.ViewCamera;
            if (camera != framedCamera) { RestoreViewportCamera(); framedCamera = camera; }
            if (camera == null) return;
            camera.ResetAspect();
            camera.ResetProjectionMatrix();
            if (uiHidden || root.layout.width <= 0 || viewport.layout.width <= 0 || viewport.layout.height <= 0) return;
            var center = root.WorldToLocal(viewport.worldBound.center);
            float width = viewport.layout.width / root.layout.width, height = viewport.layout.height / root.layout.height;
            float x = center.x / root.layout.width, y = 1 - center.y / root.layout.height;
            // Fit the frustum to the unobscured viewport while clearing the full backbuffer.
            // This preserves HDRP compositing and leaves every sensor camera untouched.
            camera.aspect = Screen.width * width / (Screen.height * height);
            var projection = camera.projectionMatrix;
            projection.m00 *= width; projection.m11 *= height;
            if (camera.orthographic) { projection.m03 += 2 * x - 1; projection.m13 += 2 * y - 1; }
            else { projection.m02 += 1 - 2 * x; projection.m12 += 1 - 2 * y; }
            camera.projectionMatrix = projection;
        }
        private void UpdateSelectionHighlight()
        {
            if (selectionPass == null) return;
            selectionPass.DisplayCamera = cameraDirector != null ? cameraDirector.ViewCamera : null;
            selectionPass.Visible = !uiHidden && backend.Scene?.HasSelection == true &&
                !(cameraDirector != null && cameraDirector.Mode == DroneCore.CameraSystem.CameraViewMode.FPV && cameraDirector.TargetDrone == highlightedPawn);
        }
    }
}
