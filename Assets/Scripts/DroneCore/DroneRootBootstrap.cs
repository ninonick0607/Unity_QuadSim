#if UNITY_EDITOR
using UnityEditor;
#endif
using DroneCore.Common;
using UnityEngine;

namespace DroneCore
{
    [DisallowMultipleComponent]
    public sealed class DroneRootBootstrap : MonoBehaviour
    {
#if UNITY_EDITOR
        private bool _buildQueued;
#endif

        private void Reset() => Build();
        private void Awake()
        {
            // Runtime only; OnValidate handles editor-time.
            if (Application.isPlaying) Build();
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (Application.isPlaying) return;
            if (_buildQueued) return;

            _buildQueued = true;
            EditorApplication.delayCall += () =>
            {
                _buildQueued = false;
                if (this == null) return;
                if (Application.isPlaying) return;
                Build();
            };
        }
#endif

        [ContextMenu("Build DroneRoot")]
        public void Build()
        {
            // DEDUPE FIRST (important)
            RemoveDuplicates<Rigidbody>(gameObject);
            RemoveDuplicates<DroneBody>(gameObject);
            RemoveDuplicates<ThrusterSet>(gameObject);
            RemoveDuplicates<Controllers.CascadedController>(gameObject);
            RemoveDuplicates<RobotCore.SensorManager>(gameObject);
            RemoveDuplicates<RobotCore.RobotCore>(gameObject);
            RemoveDuplicates<Interfaces.FlightCommandProxy>(gameObject);

            var rb      = GetOrAdd<Rigidbody>(gameObject);
            var body    = GetOrAdd<DroneBody>(gameObject);
            var thr     = GetOrAdd<ThrusterSet>(gameObject);
            var ctrl    = GetOrAdd<Controllers.CascadedController>(gameObject);
            var sensors = GetOrAdd<RobotCore.SensorManager>(gameObject);
            var core    = GetOrAdd<RobotCore.RobotCore>(gameObject);
            var cmd     = GetOrAdd<Interfaces.FlightCommandProxy>(gameObject);

            body.AutoWireIfNeeded();
            body.ValidateOrThrow();

            rb.mass = 1.28f;
            rb.interpolation = RigidbodyInterpolation.None;
            rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
        }

        private static void RemoveDuplicates<T>(GameObject go) where T : Component
        {
            var comps = go.GetComponents<T>();
            for (int i = 1; i < comps.Length; i++)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying)
                    Object.DestroyImmediate(comps[i]);
                else
                    Object.Destroy(comps[i]);
#else
                Object.Destroy(comps[i]);
#endif
            }
        }

        private static T GetOrAdd<T>(GameObject go) where T : Component
        {
            if (!go.TryGetComponent<T>(out var c))
                c = go.AddComponent<T>();
            return c;
        }
    }
}
