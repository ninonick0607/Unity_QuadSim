using QuadSim.RuntimeUI.ImGuiHud;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace QuadSim.Editor
{
    /// <summary>Strip legacy HUD components from release scenes; retain opt-in tools in development builds.</summary>
    public sealed class RuntimeWorkspaceBuildProcessor : IProcessSceneWithReport
    {
        public int callbackOrder => 0;
        public void OnProcessScene(Scene scene, BuildReport report)
        {
            if (report == null) return;
            bool development = (report.summary.options & UnityEditor.BuildOptions.Development) != 0;
            foreach (var root in scene.GetRootGameObjects())
            foreach (var legacy in root.GetComponentsInChildren<ImGuiRuntimeBootstrap>(true))
            {
                var renderer = legacy.GetComponent<UImGui.UImGui>();
                if (development)
                {
                    legacy.enabled = false;
                    if (renderer != null) renderer.enabled = false;
                }
                else
                {
                    if (renderer != null) Object.DestroyImmediate(renderer);
                    Object.DestroyImmediate(legacy);
                }
            }
        }
    }
}
