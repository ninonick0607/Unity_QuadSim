using QuadSim.RuntimeUI.Workspace;
using UnityEditor;
using UnityEngine;

namespace QuadSim.Editor
{
    public static class RuntimeWorkspaceMenu
    {
        [MenuItem("QuadSim/Developer/Use ImGui HUD")]
        private static void DeveloperHud() => RuntimeWorkspaceStartup.SetDeveloperHud(true);
        [MenuItem("QuadSim/Developer/Use Workspace HUD")]
        private static void WorkspaceHud() => RuntimeWorkspaceStartup.SetDeveloperHud(false);
        [MenuItem("QuadSim/Developer/Use ImGui HUD", true)]
        [MenuItem("QuadSim/Developer/Use Workspace HUD", true)]
        private static bool CanSwitch() => EditorApplication.isPlaying;

        [MenuItem("GameObject/QuadSim/Runtime Workspace", false, 10)]
        private static void Create()
        {
            var existing = Object.FindAnyObjectByType<RuntimeWorkspace>();
            if (existing != null) { Selection.activeGameObject = existing.gameObject; return; }
            var host = new GameObject("QuadSim Runtime Workspace");
            Undo.RegisterCreatedObjectUndo(host, "Create QuadSim runtime workspace");
            Undo.AddComponent<RuntimeWorkspace>(host);
            Selection.activeGameObject = host;
        }
    }
}
