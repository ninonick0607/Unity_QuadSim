using QuadSim.RuntimeUI.Workspace;
using UnityEditor;
using UnityEngine;

namespace QuadSim.Editor
{
    public static class RuntimeWorkspaceMenu
    {
        [MenuItem("GameObject/QuadSim/Runtime Workspace (Preview)", false, 10)]
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
