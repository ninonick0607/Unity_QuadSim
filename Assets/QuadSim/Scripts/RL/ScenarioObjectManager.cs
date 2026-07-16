using System.Collections.Generic;
using MathUtil;
using UnityEngine;

namespace QuadSim.RL
{
    public readonly struct ScenarioObjectSpec
    {
        public readonly string ObjectId;
        public readonly string Shape;
        public readonly Vector3 PositionEnu;
        public readonly float YawDeg;
        public readonly Vector3 SizeEnu;
        public readonly bool CollisionEnabled;
        public readonly Color Color;

        public ScenarioObjectSpec(
            string objectId,
            string shape,
            Vector3 positionEnu,
            float yawDeg,
            Vector3 sizeEnu,
            bool collisionEnabled,
            Color color)
        {
            ObjectId = objectId;
            Shape = shape;
            PositionEnu = positionEnu;
            YawDeg = yawDeg;
            SizeEnu = sizeEnu;
            CollisionEnabled = collisionEnabled;
            Color = color;
        }
    }

    [DisallowMultipleComponent]
    public sealed class ScenarioObjectManager : MonoBehaviour
    {
        private static ScenarioObjectManager _instance;
        private readonly Dictionary<string, Stack<RLSceneObject>> _pool = new();
        private readonly List<RLSceneObject> _active = new();
        private readonly Dictionary<string, ScenarioObjectSpec> _specs = new();

        public static ScenarioObjectManager GetOrCreate(GameObject host)
        {
            if (_instance != null) return _instance;
            _instance = FindFirstObjectByType<ScenarioObjectManager>();
            if (_instance == null && host != null)
                _instance = host.AddComponent<ScenarioObjectManager>();
            return _instance;
        }

        private void Awake() => _instance = this;

        private void OnDestroy()
        {
            if (_instance == this) _instance = null;
        }

        public void Apply(IReadOnlyList<ScenarioObjectSpec> specs)
        {
            ReleaseActive();
            _specs.Clear();

            for (int i = 0; i < specs.Count; i++)
            {
                ScenarioObjectSpec spec = specs[i];
                RLSceneObject sceneObject = Acquire(spec.Shape);
                Transform objectTransform = sceneObject.transform;
                objectTransform.SetPositionAndRotation(
                    Frames.InverseTransformWorldPosition(spec.PositionEnu, SimFrame.FLU),
                    Frames.InverseTransformQuaternion(
                        Quaternion.AngleAxis(spec.YawDeg, Vector3.forward), SimFrame.FLU));
                objectTransform.localScale = ScaleForShape(spec.Shape, spec.SizeEnu);
                sceneObject.Configure(
                    spec.ObjectId, spec.Shape, spec.CollisionEnabled, spec.Color);
                sceneObject.gameObject.SetActive(true);
                _active.Add(sceneObject);
                _specs[spec.ObjectId] = spec;
            }
        }

        public object[] Snapshot()
        {
            object[] result = new object[_active.Count];
            for (int i = 0; i < _active.Count; i++)
            {
                RLSceneObject sceneObject = _active[i];
                ScenarioObjectSpec spec = _specs[sceneObject.ObjectId];
                result[i] = new Dictionary<string, object>
                {
                    ["object_id"] = spec.ObjectId,
                    ["shape"] = spec.Shape,
                    ["position_enu"] = new object[]
                    {
                        spec.PositionEnu.x, spec.PositionEnu.y, spec.PositionEnu.z,
                    },
                    ["yaw_deg"] = spec.YawDeg,
                    ["size_enu"] = new object[]
                    {
                        spec.SizeEnu.x, spec.SizeEnu.y, spec.SizeEnu.z,
                    },
                    ["collision_enabled"] = spec.CollisionEnabled,
                    ["color_rgba"] = new object[]
                    {
                        spec.Color.r, spec.Color.g, spec.Color.b, spec.Color.a,
                    },
                };
            }
            return result;
        }

        private RLSceneObject Acquire(string shape)
        {
            if (_pool.TryGetValue(shape, out Stack<RLSceneObject> objects)
                && objects.Count > 0)
                return objects.Pop();

            PrimitiveType primitive = shape switch
            {
                "sphere" => PrimitiveType.Sphere,
                "cylinder" => PrimitiveType.Cylinder,
                "goal" => PrimitiveType.Cylinder,
                _ => PrimitiveType.Cube,
            };
            GameObject gameObject = GameObject.CreatePrimitive(primitive);
            gameObject.transform.SetParent(transform, false);
            return gameObject.AddComponent<RLSceneObject>();
        }

        private void ReleaseActive()
        {
            for (int i = 0; i < _active.Count; i++)
            {
                RLSceneObject sceneObject = _active[i];
                if (sceneObject == null) continue;
                sceneObject.gameObject.SetActive(false);
                if (!_pool.TryGetValue(sceneObject.Shape, out Stack<RLSceneObject> objects))
                {
                    objects = new Stack<RLSceneObject>();
                    _pool[sceneObject.Shape] = objects;
                }
                objects.Push(sceneObject);
            }
            _active.Clear();
        }

        private static Vector3 ScaleForShape(string shape, Vector3 sizeEnu)
        {
            Vector3 unitySize = new Vector3(sizeEnu.x, sizeEnu.z, sizeEnu.y);
            if (shape == "cylinder" || shape == "goal")
                unitySize.y *= 0.5f;
            return unitySize;
        }
    }
}
