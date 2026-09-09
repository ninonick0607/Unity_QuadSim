using System.Collections;
using DroneCore.CameraSystem;
using NUnit.Framework;
using QuadSim.RuntimeUI.Workspace;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;
using UnityEngine.TestTools;
using UnityEngine.UIElements;

namespace Nino.QuadSim.PlayModeTests
{
    public sealed class WorkspaceInputTests
    {
        [Test]
        public void UiNavigationExcludesWasdAndRestoresOriginalBindings()
        {
            var keyboard = InputSystem.AddDevice<Keyboard>();
            var map = new InputActionMap("UI");
            var navigate = map.AddAction("Navigate", InputActionType.Value);
            navigate.AddCompositeBinding("2DVector").With("Up", "<Keyboard>/w").With("Down", "<Keyboard>/s")
                .With("Left", "<Keyboard>/a").With("Right", "<Keyboard>/d");
            navigate.AddCompositeBinding("2DVector").With("Up", "<Keyboard>/upArrow").With("Down", "<Keyboard>/downArrow")
                .With("Left", "<Keyboard>/leftArrow").With("Right", "<Keyboard>/rightArrow");
            navigate.AddBinding("<Gamepad>/dpad");
            map.Enable();
            string original = map.SaveBindingOverridesAsJson();
            try
            {
                using (var bindings = new WorkspaceNavigationBindings())
                {
                    InputState.Change(keyboard, new KeyboardState(Key.W));
                    Assert.That(navigate.ReadValue<Vector2>(), Is.EqualTo(Vector2.zero), "W cannot generate UI navigation");
                    InputState.Change(keyboard, new KeyboardState(Key.UpArrow));
                    Assert.That(navigate.ReadValue<Vector2>(), Is.EqualTo(Vector2.up), "Arrow navigation still works");
                    Assert.That(navigate.bindings[navigate.bindings.Count - 1].effectivePath, Is.EqualTo("<Gamepad>/dpad"));
                }
                Assert.That(map.SaveBindingOverridesAsJson(), Is.EqualTo(original), "Closing the workspace restores all overrides");
                InputState.Change(keyboard, new KeyboardState(Key.W));
                Assert.That(navigate.ReadValue<Vector2>(), Is.EqualTo(Vector2.up));
            }
            finally { map.Dispose(); InputSystem.RemoveDevice(keyboard); }
        }

        [UnityTest]
        public IEnumerator ViewportActivationAndHudClickSeparateCameraInput()
        {
            var host = new GameObject("Input boundary test");
            var settings = ScriptableObject.CreateInstance<PanelSettings>();
            var keyboard = InputSystem.AddDevice<Keyboard>();
            var bindings = new WorkspaceNavigationBindings();
            try
            {
                settings.themeStyleSheet = Resources.Load<ThemeStyleSheet>("QuadSimWorkspaceTheme");
                var document = host.AddComponent<UIDocument>(); document.panelSettings = settings; document.sortingOrder = 10000;
                var root = document.rootVisualElement;
                var viewport = new VisualElement { focusable = true }; viewport.style.height = 200; root.Add(viewport);
                var hud = new VisualElement { focusable = true }; hud.style.height = 100; root.Add(hud);
                bool active = true;
                WorkspaceInputRouting.Install(root, viewport, value => active = value);
                var director = host.AddComponent<CameraDirector>(); director.SetMode(CameraViewMode.Free);
                yield return null;
                viewport.Focus();
                InputState.Change(keyboard, new KeyboardState(Key.W));
                var before = director.ViewCamera.transform.position;
                WorkspaceInputRouting.SampleCamera(director, viewport, active);
                yield return null;
                Assert.That(Vector3.Distance(before, director.ViewCamera.transform.position), Is.GreaterThan(0), "Held W moves the camera");
                using (var click = PointerDownEvent.GetPooled(new Event { type = EventType.MouseDown, button = 0, mousePosition = hud.worldBound.center }))
                { click.target = hud; hud.SendEvent(click); }
                yield return null;
                Assert.That(active, Is.False, "Clicking HUD deactivates camera navigation even if a control does not take focus");
                before = director.ViewCamera.transform.position;
                for (int i = 0; i < 3; i++)
                {
                    InputState.Change(keyboard, new KeyboardState(Key.W));
                    WorkspaceInputRouting.SampleCamera(director, viewport, active);
                    yield return null;
                }
                Assert.That(Vector3.Distance(before, director.ViewCamera.transform.position), Is.LessThan(.0001f));
                viewport.Focus();
                WorkspaceInputRouting.SampleCamera(director, viewport, false);
                yield return null;
                Assert.That(Vector3.Distance(before, director.ViewCamera.transform.position), Is.LessThan(.0001f), "Inactive application does not fly");
            }
            finally
            {
                bindings.Dispose(); InputSystem.RemoveDevice(keyboard);
                Object.Destroy(host); Object.Destroy(settings);
            }
            yield return null;
        }
    }
}
