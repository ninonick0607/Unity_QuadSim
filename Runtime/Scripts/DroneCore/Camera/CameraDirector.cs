using DroneCore.Core;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.CameraSystem
{
    public enum CameraViewMode
    {
        Free = 0,
        Orbit = 1,
        Follow = 2,
        FPV = 3,
    }

    [DisallowMultipleComponent]
    [DefaultExecutionOrder(-50)]
    public sealed class CameraDirector : MonoBehaviour
    {
        [Header("View")]
        [SerializeField] private Camera viewCamera;
        [SerializeField] private CameraViewMode startupMode = CameraViewMode.Follow;

        [Header("Follow")]
        [SerializeField] private float followDistance = 0.9f;
        [SerializeField] private float followPitchDeg = -15f;
        [SerializeField] private float lookHeight = 0.15f;

        [Header("Orbit Damping")]
        [Tooltip("Orbit camera position response. Follow and FPV track exactly to avoid visual lag behind the Rigidbody.")]
        [SerializeField] private float positionSharpness = 10f;
        [Tooltip("Orbit camera rotation response. Follow and FPV track exactly to avoid visual lag behind the Rigidbody.")]
        [SerializeField] private float rotationSharpness = 14f;

        [Header("Orbit / Free")]
        [SerializeField] private float orbitYawDeg;
        [SerializeField] private float orbitPitchDeg = 20f;
        [SerializeField] private float orbitMouseSensitivity = 0.2f;
        [SerializeField] private float zoomWheelSensitivity = 2f;
        [SerializeField] private float freeLookSensitivity = 0.15f;
        [SerializeField] private float freeMoveSpeed = 10f;
        [SerializeField] private float freeBoostMultiplier = 3f;
        [SerializeField] private float freeWheelDollyDistance = 3f;

        private static CameraDirector _instance;
        private static RuntimeViewState _savedState;
        private DroneCameraRig _targetRig;
        private Transform _entityTarget;
        private Transform _mount;
        private bool _hasBoundTarget;
        private Vector3 _orbitPivot;
        private Vector3 _panOffset;
        public Transform Target => _entityTarget;
        public Vector3 FocusPoint => _entityTarget != null ? _entityTarget.position + _panOffset : _orbitPivot;
        public float FreeMoveSpeed { get => freeMoveSpeed; set => freeMoveSpeed = Mathf.Clamp(value, .01f, 1000f); }
        public bool Orthographic => viewCamera != null && viewCamera.orthographic;
        private QuadPawn _cameraSourceDrone;
        private CameraViewMode _mode;
        private Vector3 _socketOffset;
        private Vector3 _pendingFreeMovement;
        private Vector2 _pendingFreeLook;
        private float _pendingFreeDolly;
        private float _freeYawDeg;
        private float _freePitchDeg;
        private QuadPawn _freeFollowTarget;
        private Vector3 _lastFreeFollowPosition;
        private float _fieldOfView = 60f;
        private bool _cameraSettingsInitialized;
        private bool _restoreFreePose;

        private struct RuntimeViewState
        {
            public bool Valid;
            public CameraViewMode Mode;
            public float FollowDistance;
            public float FollowPitchDeg;
            public float OrbitYawDeg;
            public float OrbitPitchDeg;
            public float FieldOfView;
            public Vector3 SocketOffset;
            public bool HasFreePose;
            public Vector3 FreePosition;
            public Quaternion FreeRotation;
        }

        public Camera ViewCamera => viewCamera;
        public CameraViewMode Mode => _mode;
        public QuadPawn TargetDrone => _targetRig != null ? _targetRig.Owner : null;
        public QuadPawn CameraSourceDrone => _cameraSourceDrone;
        public QuadPawn FreeFollowTarget => _freeFollowTarget;
        public float FollowDistance
        {
            get => followDistance;
            set
            {
                followDistance = Mathf.Clamp(value, 0.05f, 10000f);
                SaveRuntimeState();
            }
        }
        public float OrbitYawDeg
        {
            get => orbitYawDeg;
            set
            {
                orbitYawDeg = Mathf.Repeat(value + 180f, 360f) - 180f;
                SaveRuntimeState();
            }
        }
        public float OrbitPitchDeg
        {
            get => orbitPitchDeg;
            set
            {
                orbitPitchDeg = Mathf.Clamp(value, -90f, 90f);
                SaveRuntimeState();
            }
        }
        public float FieldOfView
        {
            get => _fieldOfView;
            set
            {
                _fieldOfView = Mathf.Clamp(value, 20f, 140f);
                if (viewCamera != null) viewCamera.fieldOfView = _fieldOfView;
                SaveRuntimeState();
            }
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ClearRuntimeState()
        {
            _instance = null;
            _savedState = default;
        }

        private void Awake()
        {
            if (_instance != null && _instance != this) { enabled = false; return; }
            _instance = this;
            if (_savedState.Valid)
            {
                _mode = _savedState.Mode;
                followDistance = _savedState.FollowDistance;
                followPitchDeg = _savedState.FollowPitchDeg;
                orbitYawDeg = _savedState.OrbitYawDeg;
                orbitPitchDeg = _savedState.OrbitPitchDeg;
                _fieldOfView = _savedState.FieldOfView;
                _socketOffset = _savedState.SocketOffset;
                _cameraSettingsInitialized = true;
                _restoreFreePose = _savedState.Mode == CameraViewMode.Free && _savedState.HasFreePose;
            }
            else
            {
                _mode = startupMode;
            }
        }

        private void OnDestroy()
        {
            if (_instance == this)
            {
                SaveRuntimeState();
                _instance = null;
            }
        }

        public static CameraDirector Get()
        {
            if (_instance != null) return _instance;
            _instance = FindAnyObjectByType<CameraDirector>();
            return _instance;
        }

        public static CameraDirector GetOrCreate(GameObject host)
        {
            CameraDirector director = Get();
            if (director != null) return director;
            return host != null ? host.AddComponent<CameraDirector>() : null;
        }

        public void EnsureBoundToFirstDrone(DroneManager manager)
        {
            if (viewCamera == null) AdoptCamera(Camera.main);
            if (manager == null || manager.DroneCount == 0)
            {
                if (viewCamera == null)
                    AdoptCamera(Camera.main);
                return;
            }

            QuadPawn first = FindFirstSceneDrone(manager);
            if (first == null) return;

            if (viewCamera == null)
            {
                _cameraSourceDrone = first;
                AdoptCamera(first.DroneCamera != null ? first.DroneCamera : Camera.main);
            }

            if (!_hasBoundTarget)
                Bind(first, snap: true);

            DisableOtherDroneCameras(manager);
        }

        public void Bind(QuadPawn drone, bool snap = true)
        {
            if (drone == null) return;

            if (viewCamera == null)
            {
                _cameraSourceDrone = drone;
                AdoptCamera(drone.DroneCamera != null ? drone.DroneCamera : Camera.main);
            }

            _targetRig = drone.CameraRig;
            _entityTarget = _targetRig.FollowTarget;
            _mount = _targetRig.FpvMount;
            _hasBoundTarget = true;
            _panOffset = Vector3.zero;
            if (drone.HasConfigApplied)
                ApplyCameraSettings(drone.Config.CamParams);

            if (snap)
                UpdateCameraPose(1f);
        }

        public void SetMode(CameraViewMode mode, bool snap = true)
        {
            if (viewCamera == null) AdoptCamera(Camera.main);
            if (mode == CameraViewMode.FPV && _entityTarget == null) return;
            if (mode == CameraViewMode.Free || mode == CameraViewMode.FPV) viewCamera.orthographic = false;
            if (mode == CameraViewMode.Orbit && _mode != mode && viewCamera != null)
            {
                var angles = viewCamera.transform.eulerAngles;
                orbitYawDeg = angles.y;
                orbitPitchDeg = Mathf.DeltaAngle(0, angles.x);
                if (_entityTarget == null) _orbitPivot = viewCamera.transform.position + viewCamera.transform.forward * followDistance;
            }
            if (_mode == CameraViewMode.Free && mode != CameraViewMode.Free)
                _freeFollowTarget = null;

            if (mode == CameraViewMode.Free && _mode != CameraViewMode.Free)
                InitializeFreeView();

            _mode = mode;
            SaveRuntimeState();
            if (snap && mode != CameraViewMode.Free)
                UpdateCameraPose(1f);
        }

        public void ToggleFollowFpv()
        {
            SetMode(_mode == CameraViewMode.FPV ? CameraViewMode.Follow : CameraViewMode.FPV);
        }

        public void AddLookInput(Vector2 delta)
        {
            OrbitYawDeg += delta.x;
            OrbitPitchDeg += delta.y;
        }

        public void AddZoomInput(float delta)
        {
            FollowDistance -= delta;
        }

        public void AddMouseCameraInput(Vector2 pixelDelta, float wheelDelta)
        {
            if (_mode == CameraViewMode.Orbit && pixelDelta.sqrMagnitude > 0f)
                AddLookInput(new Vector2(pixelDelta.x, -pixelDelta.y) * orbitMouseSensitivity);

            if (_mode == CameraViewMode.Free && pixelDelta.sqrMagnitude > 0f)
                _pendingFreeLook += pixelDelta * freeLookSensitivity;

            if (!Mathf.Approximately(wheelDelta, 0f))
            {
                if (_mode == CameraViewMode.Follow || _mode == CameraViewMode.Orbit)
                    Zoom(wheelDelta);
                else if (_mode == CameraViewMode.Free)
                    _pendingFreeDolly += wheelDelta * freeWheelDollyDistance;
                else if (_mode == CameraViewMode.FPV)
                    FieldOfView -= wheelDelta * 5f;
            }
        }

        public void ResetOrbit()
        {
            orbitYawDeg = 0f;
            orbitPitchDeg = 20f;
            SaveRuntimeState();
        }

        public void AddFreeMovementInput(Vector3 localDirection, bool boosted)
        {
            _pendingFreeMovement += localDirection * (boosted ? freeBoostMultiplier : 1f);
        }

        public void ToggleFreeFollow(QuadPawn drone)
        {
            if (_mode != CameraViewMode.Free || drone == null) return;

            if (_freeFollowTarget == drone)
            {
                _freeFollowTarget = null;
                return;
            }

            _freeFollowTarget = drone;
            _lastFreeFollowPosition = GetFollowPosition(drone);

            if (viewCamera != null)
            {
                Vector3 lookDirection = _lastFreeFollowPosition - viewCamera.transform.position;
                if (lookDirection.sqrMagnitude > 0.0001f)
                {
                    viewCamera.transform.rotation = Quaternion.LookRotation(lookDirection, Vector3.up);
                    InitializeFreeView();
                }
            }
        }

        public void ApplyCameraSettings(CamParameters parameters)
        {
            Vector3 ueOffsetCm = parameters.CamSocket_Offset;
            _socketOffset = new Vector3(ueOffsetCm.x, ueOffsetCm.z, -ueOffsetCm.y) * 0.01f;

            if (!_cameraSettingsInitialized)
            {
                if (parameters.CamArm_Length > 0f)
                    followDistance = parameters.CamArm_Length;
                if (parameters.FOV > 0f)
                    _fieldOfView = Mathf.Clamp(parameters.FOV, 20f, 140f);

                followPitchDeg = Mathf.Clamp(parameters.CamPitch, -80f, 80f);
                _cameraSettingsInitialized = true;
            }

            if (viewCamera != null) viewCamera.fieldOfView = _fieldOfView;
            SaveRuntimeState();
        }

        private void LateUpdate()
        {
            if (Application.isBatchMode)
            {
                if (viewCamera != null) viewCamera.enabled = false;
                return;
            }

            UpdateCameraPose(Time.unscaledDeltaTime);
        }

        private void UpdateCameraPose(float deltaTime)
        {
            if (viewCamera == null) return;

            if (_mode == CameraViewMode.Free)
            {
                if (_freeFollowTarget != null)
                {
                    Vector3 followPosition = GetFollowPosition(_freeFollowTarget);
                    viewCamera.transform.position += followPosition - _lastFreeFollowPosition;
                    _lastFreeFollowPosition = followPosition;
                }

                if (_pendingFreeLook.sqrMagnitude > 0f)
                {
                    _freeYawDeg += _pendingFreeLook.x;
                    _freePitchDeg = Mathf.Clamp(_freePitchDeg + _pendingFreeLook.y, -89f, 89f);
                    viewCamera.transform.rotation = Quaternion.Euler(_freePitchDeg, _freeYawDeg, 0f);
                }

                if (_pendingFreeMovement.sqrMagnitude > 0f)
                {
                    Vector3 input = Vector3.ClampMagnitude(_pendingFreeMovement, freeBoostMultiplier);
                    Vector3 worldMovement = viewCamera.transform.right * input.x
                                          + Vector3.up * input.y
                                          + viewCamera.transform.forward * input.z;
                    viewCamera.transform.position += worldMovement * freeMoveSpeed * deltaTime;
                }

                if (!Mathf.Approximately(_pendingFreeDolly, 0f))
                    viewCamera.transform.position += viewCamera.transform.forward * _pendingFreeDolly;

                _pendingFreeMovement = Vector3.zero;
                _pendingFreeLook = Vector2.zero;
                _pendingFreeDolly = 0f;
                return;
            }

            if (_entityTarget == null && _mode != CameraViewMode.Orbit)
            {
                SetMode(CameraViewMode.Free, false);
                return;
            }

            Transform target = _entityTarget;
            Vector3 desiredPosition;
            Quaternion desiredRotation;

            if (_mode == CameraViewMode.FPV)
            {
                Transform mount = _mount != null ? _mount : target;
                desiredPosition = mount.position;
                desiredRotation = mount.rotation;
            }
            else
            {
                // The chase/orbit rig follows heading, not the aircraft attitude.
                // In this project the drone's nose is its local +X (target.right).
                // Flattening it keeps pitch and roll out of the camera position,
                // while FPV above still inherits the real mount rotation.
                Vector3 nose = target == null ? Vector3.forward : Vector3.ProjectOnPlane(_targetRig != null ? target.right : target.forward, Vector3.up);
                if (nose.sqrMagnitude < 0.0001f)
                    nose = Vector3.right;
                else
                    nose.Normalize();

                float targetYaw = nose.sqrMagnitude > 0f
                    ? Mathf.Atan2(nose.x, nose.z) * Mathf.Rad2Deg
                    : 0f;
                Quaternion heading = Quaternion.Euler(0f, targetYaw - 90f, 0f);
                Vector3 focus = FocusPoint;
                if (_mode == CameraViewMode.Follow && _targetRig != null) focus += heading * _socketOffset + Vector3.up * lookHeight;

                if (_mode == CameraViewMode.Orbit)
                {
                    Quaternion orbit = Quaternion.Euler(orbitPitchDeg, orbitYawDeg, 0f);
                    desiredPosition = focus + orbit * (Vector3.back * followDistance);
                    desiredRotation = orbit;
                    _orbitPivot = focus;
                }
                else
                {
                    float height = Mathf.Tan(-followPitchDeg * Mathf.Deg2Rad) * followDistance;
                    desiredPosition = focus - nose * followDistance + Vector3.up * height;
                    desiredRotation = Quaternion.LookRotation(focus - desiredPosition, Vector3.up);
                }
            }

            // Follow and FPV must use the same rendered target pose as the drone.
            // Damping these modes adds a second delayed pose after Rigidbody
            // interpolation, making the aircraft visibly hitch ahead of its camera.
            // Orbit remains damped because its camera movement is user-directed and
            // does not need to keep the aircraft rigidly fixed in the frame.
            if (_mode != CameraViewMode.Orbit || deltaTime >= 1f)
            {
                viewCamera.transform.SetPositionAndRotation(desiredPosition, desiredRotation);
                return;
            }

            float positionT = 1f - Mathf.Exp(-positionSharpness * deltaTime);
            float rotationT = 1f - Mathf.Exp(-rotationSharpness * deltaTime);
            viewCamera.transform.position = Vector3.Lerp(viewCamera.transform.position, desiredPosition, positionT);
            viewCamera.transform.rotation = Quaternion.Slerp(viewCamera.transform.rotation, desiredRotation, rotationT);
        }

        private void InitializeFreeView()
        {
            if (viewCamera == null) return;

            Vector3 forward = viewCamera.transform.forward;
            _freeYawDeg = Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
            _freePitchDeg = -Mathf.Asin(Mathf.Clamp(forward.y, -1f, 1f)) * Mathf.Rad2Deg;
            _pendingFreeMovement = Vector3.zero;
            _pendingFreeLook = Vector2.zero;
            _pendingFreeDolly = 0f;
        }

        private static Vector3 GetFollowPosition(QuadPawn drone)
        {
            if (drone == null) return Vector3.zero;
            DroneCameraRig rig = drone.CameraRig;
            return rig != null && rig.FollowTarget != null
                ? rig.FollowTarget.position
                : drone.transform.position;
        }

        private void AdoptCamera(Camera candidate)
        {
            Camera previousMain = Camera.main;
            // Own a display camera, never move a prefab or sensor camera out of its entity.
            var cameraObject = new GameObject("QuadSim View Camera", typeof(Camera), typeof(AudioListener));
            viewCamera = cameraObject.GetComponent<Camera>();
            if (candidate != null)
            {
                viewCamera.CopyFrom(candidate);
                cameraObject.transform.SetPositionAndRotation(candidate.transform.position, candidate.transform.rotation);
            }
            else cameraObject.transform.SetPositionAndRotation(new Vector3(0, 3, -6), Quaternion.Euler(20, 0, 0));
            cameraObject.tag = "MainCamera";
            viewCamera.targetTexture = null;
            viewCamera.rect = new Rect(0, 0, 1, 1);
            viewCamera.nearClipPlane = Mathf.Min(viewCamera.nearClipPlane, .02f);
            var hd = cameraObject.AddComponent<UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData>();
            if (candidate != null && candidate.TryGetComponent<UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData>(out var sourceHd)) sourceHd.CopyTo(hd);
            viewCamera.transform.SetParent(transform, true);
            viewCamera.transform.localScale = Vector3.one;
            if (_restoreFreePose)
            {
                viewCamera.transform.SetPositionAndRotation(
                    _savedState.FreePosition,
                    _savedState.FreeRotation);
                _restoreFreePose = false;
                InitializeFreeView();
            }
            viewCamera.fieldOfView = _fieldOfView;
            viewCamera.enabled = !Application.isBatchMode;

            AudioListener listener = viewCamera.GetComponent<AudioListener>();
            if (listener != null)
                listener.enabled = !Application.isBatchMode;

            if (previousMain != null && previousMain != viewCamera)
            {
                previousMain.enabled = false;
                AudioListener previousListener = previousMain.GetComponent<AudioListener>();
                if (previousListener != null) previousListener.enabled = false;
            }
        }

        /// <summary>Attach the display rig to any scene entity. Optional mount supplies an attached/FPV pose.</summary>
        public void BindEntity(Transform entity, Transform mount = null, bool snap = true)
        {
            if (entity == null) return;
            var drone = entity.GetComponentInParent<QuadPawn>();
            if (drone != null) { Bind(drone, snap); return; }
            if (viewCamera == null) AdoptCamera(Camera.main);
            _targetRig = null; _entityTarget = entity; _mount = mount;
            _hasBoundTarget = true; _panOffset = Vector3.zero;
            if (snap) UpdateCameraPose(1f);
        }

        /// <summary>Leave a tracked entity while keeping the display pose and orbit pivot stationary.</summary>
        public void ReleaseTarget()
        {
            _orbitPivot = FocusPoint;
            _panOffset = Vector3.zero;
            _entityTarget = null; _targetRig = null; _mount = null; _freeFollowTarget = null;
            _hasBoundTarget = true;
            if (_mode == CameraViewMode.Follow || _mode == CameraViewMode.FPV) SetMode(CameraViewMode.Free, false);
        }

        public void Navigate(Vector2 pixels, bool pan, bool zoom, float viewportHeight)
        {
            if (viewCamera == null) AdoptCamera(Camera.main);
            SetMode(CameraViewMode.Orbit, false);
            if (zoom) { Zoom(-pixels.y * .03f); return; }
            if (pan)
            {
                float span = Orthographic ? viewCamera.orthographicSize * 2 : 2 * followDistance * Mathf.Tan(FieldOfView * Mathf.Deg2Rad * .5f);
                Vector3 displacement = (-viewCamera.transform.right * pixels.x + viewCamera.transform.up * pixels.y) * span / Mathf.Max(1, viewportHeight);
                if (_entityTarget != null) _panOffset += displacement; else _orbitPivot += displacement;
            }
            else
            {
                if (Orthographic) viewCamera.orthographic = false;
                AddLookInput(pixels * orbitMouseSensitivity);
            }
        }

        public void Zoom(float steps)
        {
            float scale = Mathf.Exp(-steps * .15f);
            FollowDistance *= scale;
            if (Orthographic) viewCamera.orthographicSize = Mathf.Clamp(viewCamera.orthographicSize * scale, .025f, 5000f);
        }

        public void ToggleProjection()
        {
            if (viewCamera == null) AdoptCamera(Camera.main);
            SetMode(CameraViewMode.Orbit, false);
            viewCamera.orthographic = !viewCamera.orthographic;
            viewCamera.orthographicSize = followDistance * Mathf.Tan(FieldOfView * Mathf.Deg2Rad * .5f);
        }

        public void AxisView(Vector3 direction)
        {
            SetMode(CameraViewMode.Orbit, false);
            var rotation = Quaternion.LookRotation(direction, Mathf.Abs(direction.y) > .99f ? Vector3.forward : Vector3.up);
            orbitYawDeg = rotation.eulerAngles.y; orbitPitchDeg = Mathf.DeltaAngle(0, rotation.eulerAngles.x);
            viewCamera.orthographic = true;
            viewCamera.orthographicSize = followDistance * Mathf.Tan(FieldOfView * Mathf.Deg2Rad * .5f);
            UpdateCameraPose(1f);
        }

        public void FrameTarget(Transform entity = null)
        {
            if (entity != null) BindEntity(entity, snap: false);
            if (_entityTarget == null) return;
            var bounds = new Bounds(_entityTarget.position, Vector3.one * .2f);
            bool found = false;
            foreach (var renderer in _entityTarget.GetComponentsInChildren<Renderer>())
            {
                if (!renderer.enabled || !renderer.gameObject.activeInHierarchy) continue;
                if (!found) { bounds = renderer.bounds; found = true; } else bounds.Encapsulate(renderer.bounds);
            }
            SetMode(CameraViewMode.Orbit, false);
            _panOffset = bounds.center - _entityTarget.position;
            float halfFov = Mathf.Atan(Mathf.Tan(FieldOfView * Mathf.Deg2Rad * .5f) * Mathf.Min(1, viewCamera.aspect));
            FollowDistance = Mathf.Max(.2f, bounds.extents.magnitude * 1.2f / Mathf.Sin(halfFov));
            viewCamera.orthographicSize = Mathf.Max(.1f, bounds.extents.magnitude * 1.2f / Mathf.Min(1, viewCamera.aspect));
            UpdateCameraPose(1f);
        }

        private void DisableOtherDroneCameras(DroneManager manager)
        {
            for (int i = 0; i < manager.RegisteredDrones.Count; i++)
            {
                QuadPawn drone = manager.RegisteredDrones[i];
                if (drone == null) continue;

                Camera camera = drone.DroneCamera;
                if (camera == null || camera == viewCamera) continue;

                camera.enabled = false;
                AudioListener listener = camera.GetComponent<AudioListener>();
                if (listener != null) listener.enabled = false;
            }
        }

        private static QuadPawn FindFirstSceneDrone(DroneManager manager)
        {
            QuadPawn first = null;
            int firstRootIndex = int.MaxValue;

            for (int i = 0; i < manager.RegisteredDrones.Count; i++)
            {
                QuadPawn drone = manager.RegisteredDrones[i];
                if (drone == null) continue;

                int rootIndex = drone.transform.root.GetSiblingIndex();
                if (first == null || rootIndex < firstRootIndex)
                {
                    first = drone;
                    firstRootIndex = rootIndex;
                }
            }

            return first;
        }

        private void SaveRuntimeState()
        {
            bool hasFreePose = _mode == CameraViewMode.Free && viewCamera != null;
            _savedState = new RuntimeViewState
            {
                Valid = true,
                Mode = _mode,
                FollowDistance = followDistance,
                FollowPitchDeg = followPitchDeg,
                OrbitYawDeg = orbitYawDeg,
                OrbitPitchDeg = orbitPitchDeg,
                FieldOfView = _fieldOfView,
                SocketOffset = _socketOffset,
                HasFreePose = hasFreePose,
                FreePosition = hasFreePose ? viewCamera.transform.position : Vector3.zero,
                FreeRotation = hasFreePose ? viewCamera.transform.rotation : Quaternion.identity,
            };
        }
    }
}
