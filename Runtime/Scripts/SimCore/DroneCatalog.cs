// Runtime/SimCore/DroneCatalog.cs
using System;
using UnityEngine;
using DroneCore;

namespace QuadSim.SimCore
{
    [CreateAssetMenu(fileName = "DroneCatalog", menuName = "QuadSim/Drone Catalog", order = 1)]
    public sealed class DroneCatalog : ScriptableObject
    {
        [Serializable]
        public struct Entry
        {
            [Tooltip("YAML name (no extension) in StreamingAssets/Configs/Drones or package Resources.")]
            public string configName;

            [Tooltip("Chassis prefab. Physics comes from the YAML; this is the body, mounts and meshes.")]
            public QuadPawn pawn;
        }

        [Tooltip("Used when a config has no entry. Should be QuadPawn_Base.")]
        public QuadPawn fallbackPawn;

        public Entry[] entries;

        public QuadPawn Resolve(string configName)
        {
            if (entries != null && !string.IsNullOrWhiteSpace(configName))
            {
                for (int i = 0; i < entries.Length; i++)
                {
                    if (string.Equals(entries[i].configName, configName,
                            StringComparison.OrdinalIgnoreCase) &&
                        entries[i].pawn != null)
                        return entries[i].pawn;
                }
            }

            if (fallbackPawn == null)
                Debug.LogError($"[DroneCatalog] No pawn for '{configName}' and no fallback set.");
            return fallbackPawn;
        }
    }
}