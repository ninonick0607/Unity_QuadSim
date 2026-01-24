using UnityEngine;
using DroneCore;
using DroneCore.Common;
using DroneCore.Controllers;
using RobotCore;

[DisallowMultipleComponent]
public sealed class DroneRootBootstrap : MonoBehaviour
{
    public bool ensureMotorChildren = false;

    private void Reset() => Build();
    private void Awake() => Build();

#if UNITY_EDITOR
    private void OnValidate()
    {
        if (!Application.isPlaying) Build();
    }
#endif

    [ContextMenu("Build DroneRoot")]
    public void Build()
    {
        var rb       = GetOrAdd<Rigidbody>(gameObject);
        var body     = GetOrAdd<DroneBody>(gameObject);
        var thr      = GetOrAdd<ThrusterSet>(gameObject);
        var ctrl     = GetOrAdd<CascadedController>(gameObject);
        var sensors  = GetOrAdd<SensorManager>(gameObject);
        var core     = GetOrAdd<RobotCore.RobotCore>(gameObject);

        //if (ensureMotorChildren) EnsureMotorChildren(transform);

        // Let components self-wire
        body.AutoWireIfNeeded();
        body.ValidateOrThrow();

        rb.mass = 1.28f;
        // Basic RB defaults (optional)
        rb.interpolation = RigidbodyInterpolation.None;
        rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
        //rb.maxAngularVelocity = 100f;
    }

    private static T GetOrAdd<T>(GameObject go) where T : Component
    {
        if (!go.TryGetComponent<T>(out var c))
            c = go.AddComponent<T>();
        return c;
    }

    private static void EnsureMotorChildren(Transform root)
    {
        EnsureChild(root, "MotorFL");
        EnsureChild(root, "MotorFR");
        EnsureChild(root, "MotorBL");
        EnsureChild(root, "MotorBR");
    }

    private static Transform EnsureChild(Transform root, string name)
    {
        var t = root.Find(name);
        if (t != null) return t;

        var go = new GameObject(name);
        go.transform.SetParent(root, false);
        return go.transform;
    }
}