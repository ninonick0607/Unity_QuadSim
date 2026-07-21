using UnityEditor;
using UnityEngine;

namespace Editor
{
    public static class FindMissingScripts
    {
        [MenuItem("Tools/QuadSim/Find Missing Scripts In Open Scenes")]
        private static void FindInOpenScenes()
        {
            int total = 0;
            foreach (var go in Object.FindObjectsOfType<GameObject>(true))
            {
                var components = go.GetComponents<Component>();
                for (int i = 0; i < components.Length; i++)
                {
                    if (components[i] == null)
                    {
                        total++;
                        Debug.LogWarning($"Missing script on GameObject: {GetPath(go)}", go);
                    }
                }
            }

            Debug.Log($"FindMissingScripts: Found {total} missing scripts in open scenes.");
        }

        private static string GetPath(GameObject go)
        {
            string path = go.name;
            Transform t = go.transform;
            while (t.parent != null)
            {
                t = t.parent;
                path = t.name + "/" + path;
            }
            return path;
        }
    }
}