using System;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Core
{
    [CreateAssetMenu(menuName = "QuadSim/Drone prefab catalog")]
    public sealed class DronePrefabCatalog : ScriptableObject
    {
        [Serializable]
        public struct Entry
        {
            public string Key;
            public QuadPawn Prefab;
            [Tooltip("Shown when this entry deliberately substitutes another model.")]
            public string Notice;
        }
        public Entry[] Entries = Array.Empty<Entry>();

        public bool TryResolve(DroneConfig config, out QuadPawn prefab, out string notice)
        {
            // An explicit key must resolve exactly; do not silently spawn the wrong model.
            if (!string.IsNullOrWhiteSpace(config.ModelParams.PrefabKey))
                return Find(config.ModelParams.PrefabKey, out prefab, out notice);
            if (Find(config.SourceFile, out prefab, out notice)) return true;
            return Find(config.ModelParams.BodyMesh, out prefab, out notice);
        }
        private bool Find(string key, out QuadPawn prefab, out string notice)
        {
            if (!string.IsNullOrWhiteSpace(key))
                foreach (var entry in Entries)
                    if (string.Equals(entry.Key, key, StringComparison.OrdinalIgnoreCase) && entry.Prefab != null)
                    { prefab = entry.Prefab; notice = entry.Notice; return true; }
            prefab = null; notice = null; return false;
        }
    }
}
