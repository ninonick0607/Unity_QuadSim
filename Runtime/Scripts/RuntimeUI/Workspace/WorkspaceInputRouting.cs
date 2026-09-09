using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public static class WorkspaceInputRouting
    {
        public static void SampleCamera(DroneCore.CameraSystem.CameraDirector camera, VisualElement viewport, bool active)
        {
            if (!active || camera == null || camera.Mode != DroneCore.CameraSystem.CameraViewMode.Free ||
                viewport == null || viewport.focusController?.focusedElement != viewport) return;
            var keyboard = Keyboard.current;
            if (keyboard == null) return;
            var move = new Vector3((keyboard.dKey.isPressed ? 1 : 0) - (keyboard.aKey.isPressed ? 1 : 0),
                (keyboard.eKey.isPressed ? 1 : 0) - (keyboard.qKey.isPressed ? 1 : 0),
                (keyboard.wKey.isPressed ? 1 : 0) - (keyboard.sKey.isPressed ? 1 : 0));
            camera.AddFreeMovementInput(move.normalized, keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed);
        }
        public static void Install(VisualElement root, VisualElement viewport, System.Action<bool> activation = null)
        {
            // Popups are siblings of UIDocument roots. Guard the panel so navigation cannot
            // reach either a HUD control or an open dropdown from the camera's WASD keys.
            EventCallback<NavigationMoveEvent> move = e =>
            {
                var keyboard = Keyboard.current;
                bool wasd = keyboard != null && (keyboard.wKey.isPressed || keyboard.aKey.isPressed ||
                    keyboard.sKey.isPressed || keyboard.dKey.isPressed);
                if (root.focusController?.focusedElement == viewport || wasd) e.StopImmediatePropagation();
            };
            EventCallback<NavigationSubmitEvent> submit = e =>
            {
                if (root.focusController?.focusedElement == viewport) e.StopImmediatePropagation();
            };
            EventCallback<PointerDownEvent> pointer = e =>
            {
                activation?.Invoke(e.target == viewport);
                if (e.target != viewport && e.target is not WorkspaceTransformGizmo) viewport.Blur();
            };
            VisualElement panelRoot = null;
            void Detach()
            {
                if (panelRoot == null) return;
                panelRoot.UnregisterCallback(move, TrickleDown.TrickleDown);
                panelRoot.UnregisterCallback(submit, TrickleDown.TrickleDown);
                panelRoot.UnregisterCallback(pointer, TrickleDown.TrickleDown);
                panelRoot = null;
            }
            void Attach(IPanel panel)
            {
                Detach();
                panelRoot = panel.visualTree;
                panelRoot.RegisterCallback(move, TrickleDown.TrickleDown);
                panelRoot.RegisterCallback(submit, TrickleDown.TrickleDown);
                panelRoot.RegisterCallback(pointer, TrickleDown.TrickleDown);
            }
            root.RegisterCallback<AttachToPanelEvent>(e => Attach(e.destinationPanel));
            root.RegisterCallback<DetachFromPanelEvent>(_ => Detach());
            if (root.panel != null) Attach(root.panel);
        }
    }
}
