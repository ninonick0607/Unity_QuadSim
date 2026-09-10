#if UNITY_EDITOR
using System.Collections;
using System.Linq;
using System.Reflection;
using DroneCore;
using DroneCore.Core;
using DroneCore.CameraSystem;
using NUnit.Framework;
using QuadSim.RuntimeUI.Workspace;
using SimCore;
using SimCore.Common;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.UIElements;

namespace Nino.QuadSim.PlayModeTests
{
    public sealed class WorkspaceSceneEditorTests
    {
        private const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Instance;
        private static T Field<T>(RuntimeWorkspace w, string name) => (T)w.GetType().GetField(name, Private).GetValue(w);
        private static void Call(RuntimeWorkspace w, string name, params object[] args) => w.GetType().GetMethod(name, Private).Invoke(w, args);
        private static TextElement Input(Vector3Field vector, int axis) =>
            vector.Query<FloatField>().ToList()[axis].Q("unity-text-input").Q<TextElement>();
        private static void Key(VisualElement target, KeyCode key, char character = '\0', EventModifiers modifiers = EventModifiers.None)
        {
            using var e = KeyDownEvent.GetPooled(new Event { type = EventType.KeyDown, keyCode = key, character = character, modifiers = modifiers });
            e.target = target; target.SendEvent(e);
        }
        private static void Type(TextElement input, string value)
        {
            input.Focus(); Key(input, KeyCode.A, 'a', EventModifiers.Control);
            foreach (char c in value) Key(input, KeyCode.None, c);
        }

        [UnityTest]
        public IEnumerator HierarchySelectionAndInspectorTextCommitEditTheSameObject()
        {
            var host = new GameObject("Scene editor integration");
            RuntimeWorkspace workspace = null;
            QuadPawn pawn = null;
            try
            {
                var manager = host.AddComponent<SimulationManager>();
                manager.SetRunMode(SimulationManager.RunMode.Manual); manager.SetPaused(true);
                host.AddComponent<ControlAuthorityManager>();
                host.AddComponent<SimCoreApi>();
                var drones = host.AddComponent<DroneManager>();
                var prefab = AssetDatabase.LoadAssetAtPath<QuadPawn>("Packages/com.nino.quadsim/Runtime/Prefabs/Quad Prefabs/flightmare_pawn.prefab");
                drones.SetSceneEnv(12345, false, 47, 8, 400, StartupSpawnPolicy.SceneDronesOnly, prefab, null, false, "", MathUtil.SimFrame.FLU, GoalMode.Velocity);
                pawn = drones.SpawnDroneWithConfig("flightmare_quad", new Vector3(0, 30, 0), Quaternion.identity);
                workspace = host.AddComponent<RuntimeWorkspace>();
                workspace.GetType().GetField("restoringLayout", Private).SetValue(workspace, true);
                yield return null;
                Call(workspace, "SetUiHidden", false);
                Call(workspace, "OpenPanel", "Inspector");
                Call(workspace, "OpenPanel", "Scene");
                Call(workspace, "EnableSceneEditing");
                RuntimeWorkspaceStartup.Ensure();
                Assert.That(Object.FindObjectsByType<RuntimeWorkspace>(FindObjectsSortMode.None).Length, Is.EqualTo(1), "Startup reuses the existing workspace");
                RuntimeWorkspaceStartup.SetDeveloperHud(true);
                Assert.That(workspace.enabled, Is.False);
                RuntimeWorkspaceStartup.SetDeveloperHud(false);
                Assert.That(workspace.enabled, Is.True);
                workspace.GetType().GetField("restoringLayout", Private).SetValue(workspace, true);
                Call(workspace, "OpenPanel", "Inspector");
                Call(workspace, "OpenPanel", "Scene");
                var backend = Field<IWorkspaceBackend>(workspace, "backend");
                Assert.That(backend.Level.AddObject("cube", new Vector3(2, 3, 4), Vector3.one, true, out var cube), Is.True);
                Assert.That(backend.Level.AddObject("sphere", new Vector3(8, 9, 10), Vector3.one, true, out var sphere), Is.True);
                Call(workspace, "RefreshObjects");
                var hierarchy = Field<WorkspaceHierarchy>(workspace, "hierarchy");
                hierarchy.Tree.SetSelectionById(hierarchy.GetItemId(cube.transform));
                yield return null;
                Assert.That(backend.Scene.Selected, Is.EqualTo(cube.transform));
                Assert.That(Field<bool>(workspace, "hierarchyHighlight"), Is.True);
                var position = Field<Vector3Field>(workspace, "position");
                var rotation = Field<Vector3Field>(workspace, "rotation");
                var scale = Field<Vector3Field>(workspace, "scale");
                var x = Input(position, 0);
                Type(x, "7"); Key(x, KeyCode.Return, '\n');
                yield return null; yield return null;
                Assert.That(cube.transform.position, Is.EqualTo(new Vector3(7, 4, 3)), "Enter applies the typed ENU coordinate without an Apply button");
                var y = Input(position, 1);
                Type(y, "6");
                Input(rotation, 0).Focus();
                yield return null; yield return null;
                Assert.That(cube.transform.position, Is.EqualTo(new Vector3(7, 4, 6)), "Leaving a field commits its text");
                Type(Input(rotation, 1), "45"); Key(Input(rotation, 1), KeyCode.Return, '\n');
                yield return null; yield return null;
                Assert.That(Quaternion.Angle(cube.transform.rotation, Quaternion.Euler(0, 45, 0)), Is.LessThan(.01f));
                Type(Input(scale, 2), "2"); Key(Input(scale, 2), KeyCode.Return, '\n');
                yield return null; yield return null;
                Assert.That(cube.transform.localScale, Is.EqualTo(new Vector3(1, 1, 2)));
                Field<DropdownField>(workspace, "gizmoFrame").value = "FRD / NED";
                hierarchy.Tree.Focus(); Call(workspace, "ReadPose");
                Assert.That(position.value, Is.EqualTo(new Vector3(6, 7, -4)));
                Type(Input(position, 2), "-5"); Key(Input(position, 2), KeyCode.Return, '\n');
                yield return null; yield return null;
                Assert.That(cube.transform.position.y, Is.EqualTo(5), "NED positive-down conversion matches the displayed axes");

                var spherePose = sphere.transform.position;
                Type(Input(position, 0), "99");
                hierarchy.Tree.SetSelectionById(hierarchy.GetItemId(sphere.transform));
                hierarchy.Tree.Focus();
                yield return null; yield return null;
                Assert.That(sphere.transform.position, Is.EqualTo(spherePose), "A pending text edit never transfers to a new selection");
                Assert.That(backend.SelectedId, Is.EqualTo(pawn.AgentId), "Scene asset selection preserves the controlled drone");
                Call(workspace, "SelectSceneObject", cube.transform, false);
                Assert.That(Field<bool>(workspace, "hierarchyHighlight"), Is.False, "Viewport selection does not tint the asset");
                var row = hierarchy.Tree.Query<Label>().ToList().First(label =>
                    label.userData is WorkspaceHierarchy.Entry entry && entry.Target == cube.transform);
                using (var click = PointerDownEvent.GetPooled(new Event { type = EventType.MouseDown, button = 0, mousePosition = row.worldBound.center }))
                { click.target = row; row.SendEvent(click); }
                Assert.That(Field<bool>(workspace, "hierarchyHighlight"), Is.True, "Re-clicking the selected row restores highlighting");
                hierarchy.Tree.Focus();
                Key(hierarchy.Tree, KeyCode.Escape);
                yield return null;
                Assert.That(backend.Scene.HasSelection, Is.False, "Escape from the hierarchy clears the inspector and handles");
                Assert.That(Field<bool>(workspace, "hierarchyHighlight"), Is.False);
                Assert.That(hierarchy.Tree.selectedIndex, Is.EqualTo(-1));
                Assert.That(backend.SelectedId, Is.EqualTo(pawn.AgentId));
                Assert.That(backend.Level.RemoveObject(cube.ObjectId), Is.True);
                Call(workspace, "RefreshObjects");
                Assert.That(hierarchy.Tree.GetItemDataForId<WorkspaceHierarchy.Entry>(hierarchy.GetItemId(sphere.transform)).Target, Is.EqualTo(sphere.transform));
                Assert.That(hierarchy.Tree.GetItemDataForIndex<WorkspaceHierarchy.Entry>(0).Name, Is.EqualTo("Scene Objects (1)"));
            }
            finally
            {
                if (workspace != null) workspace.enabled = false;
                if (pawn != null) Object.Destroy(pawn.gameObject);
                Object.Destroy(host);
            }
            yield return null;
        }
    }
}
#endif
