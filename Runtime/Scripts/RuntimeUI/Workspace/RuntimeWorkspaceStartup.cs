using System;
using SimCore;
using QuadSim.RuntimeUI.ImGuiHud;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Chooses presentation at scene startup without changing simulation ownership.</summary>
    public static class RuntimeWorkspaceStartup
    {
        public static bool DeveloperHud { get; private set; }
        public static bool GraphicsAvailable => !Application.isBatchMode &&
            SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void Reset()
        {
            DeveloperHud = (Application.isEditor || Debug.isDebugBuild) &&
                Array.Exists(Environment.GetCommandLineArgs(), arg => arg == "--quadsim-dev-imgui");
            SceneManager.sceneLoaded -= OnSceneLoaded;
        }
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Subscribe() => SceneManager.sceneLoaded += OnSceneLoaded;
        // Enter Play Mode can skip scene reload, so sceneLoaded alone is insufficient.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void StartWorkspace() => Ensure();
        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) => Ensure();
        public static void SetDeveloperHud(bool enabled)
        {
            DeveloperHud = enabled && (Application.isEditor || Debug.isDebugBuild);
            Ensure();
        }
        public static void Ensure()
        {
            if (!GraphicsAvailable) return;
            var sim = SimCoreApi.Get();
            if (sim == null) return;
            var workspace = UnityEngine.Object.FindAnyObjectByType<RuntimeWorkspace>();
            if (DeveloperHud)
            {
                if (workspace != null) workspace.enabled = false;
                foreach (var legacy in UnityEngine.Object.FindObjectsByType<ImGuiRuntimeBootstrap>(FindObjectsSortMode.None))
                {
                    var renderer = legacy.GetComponent<UImGui.UImGui>();
                    if (renderer != null) renderer.enabled = true;
                    legacy.enabled = true;
                }
                return;
            }
            if (workspace == null) workspace = sim.gameObject.AddComponent<RuntimeWorkspace>();
            else workspace.enabled = true;
        }
    }
}
