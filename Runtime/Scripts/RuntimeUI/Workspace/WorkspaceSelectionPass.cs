using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace QuadSim.RuntimeUI.Workspace
{
    // Draw only the explicitly selected entity, only into the display camera.
    // DrawRenderer retains animated mesh/rotor transforms and bypasses wall occlusion.
    public sealed class WorkspaceSelectionPass : CustomPass
    {
        public Camera DisplayCamera;
        public bool Visible;
        private readonly List<(Renderer Renderer, int Submeshes)> meshes = new List<(Renderer, int)>();
        private Material overlay;
        public void Select(Transform target)
        {
            meshes.Clear();
            if (target == null) return;
            foreach (var renderer in target.GetComponentsInChildren<Renderer>(true))
            {
                var mesh = renderer is SkinnedMeshRenderer skin ? skin.sharedMesh : renderer.GetComponent<MeshFilter>()?.sharedMesh;
                if (mesh != null) meshes.Add((renderer, mesh.subMeshCount));
            }
        }
        protected override bool executeInSceneView => false;
        protected override void Execute(CustomPassContext ctx)
        {
            if (!Visible || meshes.Count == 0 || ctx.hdCamera.camera != DisplayCamera) return;
            if (overlay == null)
            {
                var shader = Resources.Load<Shader>("QuadSimSelectionOverlay");
                if (shader == null) return;
                overlay = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            }
            foreach (var entry in meshes)
                if (entry.Renderer != null && entry.Renderer.enabled && entry.Renderer.gameObject.activeInHierarchy)
                    for (int i = 0; i < entry.Submeshes; i++) ctx.cmd.DrawRenderer(entry.Renderer, overlay, i, 0);
        }
        protected override void Cleanup()
        {
            UnityEngine.Rendering.CoreUtils.Destroy(overlay); overlay = null;
        }
    }
}
