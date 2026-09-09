using UnityEngine;

namespace QuadSim.RL
{
    [DisallowMultipleComponent]
    public sealed class RLSceneObject : MonoBehaviour
    {
        public string ObjectId { get; private set; }
        public string Shape { get; private set; }
        public bool CollisionEnabled { get; private set; }

        public void Configure(string objectId, string shape, bool collisionEnabled, Color color)
        {
            ObjectId = objectId;
            Shape = shape;
            CollisionEnabled = collisionEnabled;
            name = objectId;

            Collider collider = GetComponent<Collider>();
            if (collider != null) collider.enabled = collisionEnabled;

            Renderer renderer = GetComponent<Renderer>();
            if (renderer != null)
            {
                var material = Resources.Load<Material>("QuadSim/ScenarioObject");
                if (material != null) renderer.sharedMaterial = material;
                var properties = new MaterialPropertyBlock();
                properties.SetColor("_BaseColor", color); properties.SetColor("_Color", color);
                renderer.SetPropertyBlock(properties);
            }
        }
    }
}
