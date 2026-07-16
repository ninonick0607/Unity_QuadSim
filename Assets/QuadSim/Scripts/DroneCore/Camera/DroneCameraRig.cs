using UnityEngine;

namespace DroneCore.CameraSystem
{
    [DisallowMultipleComponent]
    public sealed class DroneCameraRig : MonoBehaviour
    {
        [SerializeField] private Transform fpvMount;
        [SerializeField] private Transform followTarget;
        [SerializeField] private Transform orbitTarget;
        [SerializeField] private Vector3 defaultFpvOffset = new Vector3(0.15f, 0.05f, 0f);

        private QuadPawn _owner;

        public QuadPawn Owner => _owner;
        public string OwnerAgentId => _owner != null ? _owner.AgentId : string.Empty;
        public Transform FpvMount => fpvMount != null ? fpvMount : transform;
        public Transform FollowTarget => followTarget != null ? followTarget : transform;
        public Transform OrbitTarget => orbitTarget != null ? orbitTarget : FollowTarget;

        private void Awake()
        {
            Initialize(GetComponent<QuadPawn>(), null);
        }

        public void Initialize(QuadPawn owner, Transform legacyFpvMount)
        {
            if (owner != null)
                _owner = owner;

            if (_owner == null)
                _owner = GetComponentInParent<QuadPawn>();

            Transform root = _owner != null ? _owner.transform : transform;
            if (followTarget == null) followTarget = root;
            if (orbitTarget == null) orbitTarget = followTarget;

            if (fpvMount == null && IsUsableMount(legacyFpvMount))
                fpvMount = legacyFpvMount;

            if (fpvMount == null)
            {
                var mount = new GameObject("FPVMount").transform;
                mount.SetParent(root, false);
                mount.localPosition = defaultFpvOffset;
                mount.localRotation = Quaternion.LookRotation(Vector3.right, Vector3.up);
                fpvMount = mount;
            }
        }

        private static bool IsUsableMount(Transform candidate)
        {
            return candidate != null && candidate.GetComponent<Camera>() == null;
        }
    }
}
