using System.Collections.Generic;
using DroneCore;
using DroneCore.Core;
using MathUtil;
using QuadSim.RL;
using UnityEngine;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Editor selection and transforms; simulation and object lifecycles stay with their managers.</summary>
    public sealed class WorkspaceSceneService
    {
        private readonly WorkspaceLevelService level;
        private readonly WorkspaceFlightSession flight;
        private readonly List<Transform> authored = new List<Transform>();
        public event System.Action<Transform> TransformChanging;
        public Transform Selected { get; private set; }
        public bool HasSelection => Selected != null && Selected.gameObject.activeInHierarchy;
        public bool CanEdit => level.CanEdit && HasSelection;
        public bool CanScale => CanEdit && Selected.GetComponent<QuadPawn>() == null;
        public WorkspaceSceneService(WorkspaceLevelService level, WorkspaceFlightSession flight)
        {
            this.level = level; this.flight = flight;
            RefreshAuthoredObjects();
        }
        public void Select(Transform target) => Selected = target;
        public void RefreshAuthoredObjects()
        {
            authored.Clear();
            var seen = new HashSet<Transform>();
            foreach (var collider in Object.FindObjectsByType<Collider>(FindObjectsSortMode.None))
            {
                if (collider.GetComponentInParent<QuadPawn>() != null || collider.GetComponentInParent<RLSceneObject>() != null) continue;
                var target = collider.attachedRigidbody != null ? collider.attachedRigidbody.transform : collider.transform;
                if (seen.Add(target)) authored.Add(target);
            }
        }
        public void ReadObjects(List<Transform> destination)
        {
            destination.Clear();
            foreach (var target in authored) if (target != null && target.gameObject.activeInHierarchy) destination.Add(target);
            foreach (var item in level.Objects.ActiveObjects) if (item != null && item.gameObject.activeInHierarchy) destination.Add(item.transform);
        }
        public Transform Pick(Ray ray)
        {
            Transform target = null;
            float distance = float.PositiveInfinity;
            if (Physics.Raycast(ray, out var hit))
            {
                distance = hit.distance;
                var pawn = hit.collider.GetComponentInParent<QuadPawn>();
                var item = hit.collider.GetComponentInParent<RLSceneObject>();
                target = pawn != null ? pawn.transform : item != null ? item.transform :
                    hit.rigidbody != null ? hit.rigidbody.transform : hit.collider.transform;
            }
            // Markers with collision disabled remain selectable without adding physics colliders.
            foreach (var item in level.Objects.ActiveObjects)
            {
                if (item == null || item.CollisionEnabled) continue;
                var renderer = item.GetComponent<Renderer>();
                if (renderer != null && renderer.bounds.IntersectRay(ray, out float d) && d < distance)
                { target = item.transform; distance = d; }
            }
            return target;
        }
        public bool ApplyTransform(Transform expected, Vector3 worldPosition, Quaternion worldRotation, Vector3 localScale)
        {
            if (!CanEdit || expected == null || expected != Selected ||
                !Finite(worldPosition) || !Finite(localScale) || localScale.x <= 0 || localScale.y <= 0 || localScale.z <= 0 ||
                !WorkspaceBackend.IsFinite(worldRotation.x) || !WorkspaceBackend.IsFinite(worldRotation.y) ||
                !WorkspaceBackend.IsFinite(worldRotation.z) || !WorkspaceBackend.IsFinite(worldRotation.w)) return false;
            var pawn = Selected.GetComponent<QuadPawn>();
            if (pawn != null)
            {
                if (pawn.Api == null || !pawn.Api.IsInitialized) return false;
                TransformChanging?.Invoke(Selected);
                pawn.Api.ResetPose(worldPosition, worldRotation);
                flight.Forget(pawn);
            }
            else
            {
                TransformChanging?.Invoke(Selected);
                var body = Selected.GetComponent<Rigidbody>();
                if (body != null)
                {
                    body.position = worldPosition; body.rotation = worldRotation;
                    if (!body.isKinematic) { body.linearVelocity = Vector3.zero; body.angularVelocity = Vector3.zero; }
                }
                Selected.SetPositionAndRotation(worldPosition, worldRotation);
                Selected.localScale = localScale;
            }
            Physics.SyncTransforms();
            return true;
        }
        private static bool Finite(Vector3 v) => WorkspaceBackend.IsFinite(v.x) && WorkspaceBackend.IsFinite(v.y) && WorkspaceBackend.IsFinite(v.z);
    }
}
