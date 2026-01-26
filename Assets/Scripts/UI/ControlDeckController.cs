using System;
using UnityEngine;
using UnityEngine.UIElements;
using DroneCore.Controllers;

namespace QuadSim.UI
{
    [DisallowMultipleComponent]
    public sealed class ControlDeckController : MonoBehaviour
    {
        [Header("Optional refs")]
        [SerializeField] private CascadedController cascadedController;
        [SerializeField] private Rigidbody targetRigidbody; // if null, auto-resolve from DroneBody or any RB

        private VisualElement _hudRoot;
        private VisualElement _root;
        private VisualElement _titleBar;

        private DockableWindowController _dock;
        private VisualElement _resizeOverlay;
        private VisualElement _tl, _tr, _bl, _br, _l, _r, _t, _b;

        // Controller selection
        private Button _btnCtrlCascade, _btnCtrlGeo, _btnCtrlApi;

        // Mode selection
        private Button _btnModePosition, _btnModeVelocity, _btnModeAngle, _btnModeAcro, _btnModePassthrough;

        // Foldouts
        private Foldout _foldClamps, _foldInputs;
        private VisualElement _inputsContainer;

        // Bottom actions
        private Button _btnSwitchCamera, _btnResetRot, _btnResetPos, _btnResetDrone;
        private Button _btnResetInputs;

        // Dropdowns
        private DropdownField _droneDropdown;
        private readonly System.Collections.Generic.List<DroneCore.DroneBody> _droneBodies = new();

        // WIP badge
        private Label _wipLabel;

        // Axes (phase-1: roll/pitch/yaw only; later swap to x/y/z per mode)
        private AxisWidgets _roll;
        private AxisWidgets _pitch;
        private AxisWidgets _yaw;

        private bool _suppressAxisEvents;

        private enum ControllerChoice { Cascade, Geometric, Api }
        private enum ModeChoice { Position, Velocity, Angle, Acro, Passthrough }

        private ControllerChoice _controller = ControllerChoice.Cascade;
        private ModeChoice _mode = ModeChoice.Acro;

        public void Initialize(VisualElement hudRoot)
        {
            _hudRoot = hudRoot ?? throw new ArgumentNullException(nameof(hudRoot));

            
            _root = _hudRoot.Q<VisualElement>("ControlRoot");
            if (_root == null)
                throw new Exception("[ControlDeck] UXML missing ControlRoot.");

            _titleBar = _root.Q<VisualElement>("ControlTitleBar");
            _resizeOverlay = _root.Q<VisualElement>("ResizeOverlay");

            _tl = _root.Q<VisualElement>("ResizeTL");
            _tr = _root.Q<VisualElement>("ResizeTR");
            _bl = _root.Q<VisualElement>("ResizeBL");
            _br = _root.Q<VisualElement>("ResizeBR");

            _l  = _root.Q<VisualElement>("ResizeL");
            _r  = _root.Q<VisualElement>("ResizeR");
            _t  = _root.Q<VisualElement>("ResizeT");
            _b  = _root.Q<VisualElement>("ResizeB");

            _btnCtrlCascade = _root.Q<Button>("CtrlCascade");
            _btnCtrlGeo     = _root.Q<Button>("CtrlGeometric");
            _btnCtrlApi     = _root.Q<Button>("CtrlApi");

            _btnModePosition    = _root.Q<Button>("ModePosition");
            _btnModeVelocity    = _root.Q<Button>("ModeVelocity");
            _btnModeAngle       = _root.Q<Button>("ModeAngle");
            _btnModeAcro        = _root.Q<Button>("ModeAcro");
            _btnModePassthrough = _root.Q<Button>("ModePassthrough");

            _wipLabel = _root.Q<Label>("WipLabel");

            _foldClamps = _root.Q<Foldout>("FoldClamps");
            _foldInputs = _root.Q<Foldout>("FoldInputs");
            _inputsContainer = _root.Q<VisualElement>("InputsContainer");

            _btnSwitchCamera = _root.Q<Button>("BtnSwitchCamera");
            _btnResetRot     = _root.Q<Button>("BtnResetRotation");
            _btnResetPos     = _root.Q<Button>("BtnResetPosition");
            _btnResetDrone   = _root.Q<Button>("BtnResetDrone");

            _btnResetInputs  = _root.Q<Button>("BtnResetInputs");
            _droneDropdown   = _root.Q<DropdownField>("DroneDropdown");

            
            // Dock controller
            _dock = new DockableWindowController(_hudRoot, _root, _titleBar, _tl, _tr, _bl, _br, _l, _r, _t, _b);

            // Bind axes (expects SliderRoll/InputRoll/ReadoutRoll/ClampMinRoll/ClampMaxRoll etc.)
            _roll  = BindAxis("Roll");
            _pitch = BindAxis("Pitch");
            _yaw   = BindAxis("Yaw");

            SetAxisClampDefaults(_roll,  -250f, 250f);
            SetAxisClampDefaults(_pitch, -250f, 250f);
            SetAxisClampDefaults(_yaw,   -250f, 250f);

            // Button wiring
            if (_btnCtrlCascade != null) _btnCtrlCascade.clicked += () => SetController(ControllerChoice.Cascade);
            if (_btnCtrlGeo != null)     _btnCtrlGeo.clicked += () => SetController(ControllerChoice.Geometric);
            if (_btnCtrlApi != null)     _btnCtrlApi.clicked += () => SetController(ControllerChoice.Api);

            if (_btnModePosition != null)    _btnModePosition.clicked += () => SetMode(ModeChoice.Position);
            if (_btnModeVelocity != null)    _btnModeVelocity.clicked += () => SetMode(ModeChoice.Velocity);
            if (_btnModeAngle != null)       _btnModeAngle.clicked += () => SetMode(ModeChoice.Angle);
            if (_btnModeAcro != null)        _btnModeAcro.clicked += () => SetMode(ModeChoice.Acro);
            if (_btnModePassthrough != null) _btnModePassthrough.clicked += () => SetMode(ModeChoice.Passthrough);

            if (_btnSwitchCamera != null) _btnSwitchCamera.clicked += SwitchCamera;
            if (_btnResetRot != null)     _btnResetRot.clicked += ResetRotation;
            if (_btnResetPos != null)     _btnResetPos.clicked += ResetPosition;
            if (_btnResetDrone != null)   _btnResetDrone.clicked += ResetDrone;

            if (_btnResetInputs != null) _btnResetInputs.clicked += ResetAllAxesToZero;

            if (_droneDropdown != null)
            {
                RefreshDroneDropdown();
                _droneDropdown.RegisterValueChangedCallback(evt =>
                {
                    if (!string.IsNullOrEmpty(evt.newValue))
                        SelectDroneByName(evt.newValue);
                });
            }

            // Start hidden (expected)
            _root.AddToClassList("is-hidden");

            RefreshButtonStates();
            PushOutputs(); // ensure command sink matches UI on start
        }

        public void Toggle()
        {
            if (_root == null) return;

            bool hidden = _root.ClassListContains("is-hidden");
            if (hidden)
            {
                _root.RemoveFromClassList("is-hidden");
                if (_droneDropdown != null) RefreshDroneDropdown();
                _dock?.BringToFront();
                BringResizeToFront();
            }
            else
            {
                _root.AddToClassList("is-hidden");
            }
        }


        // ----------------------------
        // Axis widgets
        // ----------------------------

        private sealed class AxisWidgets
        {
            public string Name;

            public Slider Slider;        // float slider (optional)
            public SliderInt SliderInt;  // int slider (optional)

            public FloatField Input;
            public Label Readout;

            public FloatField ClampMin;
            public FloatField ClampMax;

            public float DefaultClampMin;
            public float DefaultClampMax;

            public float CurrentRaw;

            public float ClampMinValue => (ClampMin != null) ? ClampMin.value : DefaultClampMin;
            public float ClampMaxValue => (ClampMax != null) ? ClampMax.value : DefaultClampMax;

            public void SetUIValue(float v)
            {
                if (Input != null) Input.value = v;
                if (Slider != null) Slider.value = v;
                if (SliderInt != null) SliderInt.value = Mathf.RoundToInt(v);
            }

            public void SetSliderRange(float low, float high)
            {
                if (Slider != null) { Slider.lowValue = low; Slider.highValue = high; }
                if (SliderInt != null)
                {
                    SliderInt.lowValue = Mathf.RoundToInt(low);
                    SliderInt.highValue = Mathf.RoundToInt(high);
                }
            }
        }

        private AxisWidgets BindAxis(string name)
        {
            var w = new AxisWidgets
            {
                Name = name,
                Slider = _root.Q<Slider>($"Slider{name}"),
                SliderInt = _root.Q<SliderInt>($"Slider{name}"),
                Input = _root.Q<FloatField>($"Input{name}"),
                Readout = _root.Q<Label>($"Readout{name}"),
                ClampMin = _root.Q<FloatField>($"ClampMin{name}"),
                ClampMax = _root.Q<FloatField>($"ClampMax{name}")
            };

            if ((w.Slider == null && w.SliderInt == null) || w.Input == null || w.Readout == null || w.ClampMin == null || w.ClampMax == null)
                throw new Exception($"[ControlDeck] Missing axis widgets for '{name}'. Check UXML names (Slider/Input/Readout/ClampMin/ClampMax).");

            if (w.Slider != null)
            {
                w.Slider.RegisterValueChangedCallback(evt =>
                {
                    if (_suppressAxisEvents) return;
                    w.CurrentRaw = evt.newValue;
                    SyncAxisUIFromRaw(w);
                    PushOutputs();
                });
            }

            if (w.SliderInt != null)
            {
                w.SliderInt.RegisterValueChangedCallback(evt =>
                {
                    if (_suppressAxisEvents) return;
                    w.CurrentRaw = evt.newValue;
                    SyncAxisUIFromRaw(w);
                    PushOutputs();
                });
            }

            if (w.Input != null)
            {
                w.Input.RegisterValueChangedCallback(evt =>
                {
                    if (_suppressAxisEvents) return;
                    w.CurrentRaw = evt.newValue;
                    SyncAxisUIFromRaw(w);
                    PushOutputs();
                });
            }

            if (w.ClampMin != null)
                w.ClampMin.RegisterValueChangedCallback(_ => { if (!_suppressAxisEvents) PushOutputs(); });

            if (w.ClampMax != null)
                w.ClampMax.RegisterValueChangedCallback(_ => { if (!_suppressAxisEvents) PushOutputs(); });

            return w;
        }

        private void SetAxisClampDefaults(AxisWidgets w, float min, float max)
        {
            if (w == null) return;
            w.DefaultClampMin = min;
            w.DefaultClampMax = max;

            _suppressAxisEvents = true;
            try
            {
                if (w.ClampMin != null) w.ClampMin.value = min;
                if (w.ClampMax != null) w.ClampMax.value = max;
                w.SetSliderRange(min, max);
            }
            finally
            {
                _suppressAxisEvents = false;
            }
        }

        private void SyncAxisUIFromRaw(AxisWidgets w)
        {
            if (w == null) return;

            _suppressAxisEvents = true;
            try
            {
                w.SetSliderRange(w.ClampMinValue, w.ClampMaxValue);
                w.SetUIValue(w.CurrentRaw);
            }
            finally
            {
                _suppressAxisEvents = false;
            }
        }

        private float ApplyClamp(AxisWidgets w)
        {
            if (w == null) return 0f;
            float lo = Mathf.Min(w.ClampMinValue, w.ClampMaxValue);
            float hi = Mathf.Max(w.ClampMinValue, w.ClampMaxValue);
            return Mathf.Clamp(w.CurrentRaw, lo, hi);
        }

        private void UpdateReadout(AxisWidgets w)
        {
            if (w == null || w.Readout == null) return;
            float v = ApplyClamp(w);
            w.Readout.text = v.ToString("0.00");
        }

        // ----------------------------
        // UI state
        // ----------------------------

        private void SetController(ControllerChoice choice)
        {
            _controller = choice;
            RefreshButtonStates();

            // Phase 1: only Cascade is actually writable; others are informational.
            PushOutputs();
        }

        private void SetMode(ModeChoice mode)
        {
            _mode = mode;
            RefreshButtonStates();

            // Phase 1: only Acro is writable under Cascade
            PushOutputs();
        }

        private bool IsActiveWritable()
        {
            return _controller == ControllerChoice.Cascade && _mode == ModeChoice.Acro;
        }

        private void RefreshButtonStates()
        {
            SetActive(_btnCtrlCascade, _controller == ControllerChoice.Cascade);
            SetActive(_btnCtrlGeo,     _controller == ControllerChoice.Geometric);
            SetActive(_btnCtrlApi,     _controller == ControllerChoice.Api);

            SetActive(_btnModePosition,    _mode == ModeChoice.Position);
            SetActive(_btnModeVelocity,    _mode == ModeChoice.Velocity);
            SetActive(_btnModeAngle,       _mode == ModeChoice.Angle);
            SetActive(_btnModeAcro,        _mode == ModeChoice.Acro);
            SetActive(_btnModePassthrough, _mode == ModeChoice.Passthrough);

            bool writable = IsActiveWritable();

            if (_inputsContainer != null)
                _inputsContainer.SetEnabled(true); // inputs still editable, but we gate writing in PushOutputs

            if (_wipLabel != null)
                _wipLabel.style.display = writable ? DisplayStyle.None : DisplayStyle.Flex;
        }

        private static void SetActive(Button b, bool active)
        {
            if (b == null) return;
            if (active) b.AddToClassList("active");
            else b.RemoveFromClassList("active");
        }

        private void PushOutputs()
        {
            ResolveRefsIfNeeded();

            float roll = ApplyClamp(_roll);
            float pitch = ApplyClamp(_pitch);
            float yaw = ApplyClamp(_yaw);

            UpdateReadout(_roll);
            UpdateReadout(_pitch);
            UpdateReadout(_yaw);

            if (!IsActiveWritable()) return;

            // Phase 1 command sink: CascadedController desiredRatesDeg
            if (cascadedController != null)
            {
                cascadedController.desiredRatesDeg = new Vector3(roll, pitch, yaw);
            }
        }

        private void BringResizeToFront()
        {
            _resizeOverlay?.BringToFront();
            _tl?.BringToFront();
            _tr?.BringToFront();
            _bl?.BringToFront();
            _br?.BringToFront();
            _l?.BringToFront();
            _r?.BringToFront();
            _t?.BringToFront();
            _b?.BringToFront();
        }

        // ----------------------------
        // Drone selection (optional)
        // ----------------------------

        private void RefreshDroneDropdown()
        {
            _droneBodies.Clear();

#if UNITY_2023_1_OR_NEWER
            var bodies = UnityEngine.Object.FindObjectsByType<DroneCore.DroneBody>(FindObjectsSortMode.None);
#else
            var bodies = UnityEngine.Object.FindObjectsOfType<DroneCore.DroneBody>();
#endif
            if (bodies != null)
                _droneBodies.AddRange(bodies);

            _droneDropdown.choices = new System.Collections.Generic.List<string>();
            for (int i = 0; i < _droneBodies.Count; i++)
            {
                var b = _droneBodies[i];
                var name = b != null ? b.gameObject.name : $"Drone {i}";
                _droneDropdown.choices.Add(name);
            }

            if (_droneDropdown.choices.Count == 0)
            {
                _droneDropdown.value = "(none)";
                return;
            }

            if (string.IsNullOrEmpty(_droneDropdown.value) || !_droneDropdown.choices.Contains(_droneDropdown.value))
                _droneDropdown.value = _droneDropdown.choices[0];

            SelectDroneByName(_droneDropdown.value);
        }

        private void SelectDroneByName(string name)
        {
            for (int i = 0; i < _droneBodies.Count; i++)
            {
                var b = _droneBodies[i];
                if (b == null) continue;
                if (b.gameObject.name != name) continue;

                targetRigidbody = b.Rigidbody;
                return;
            }
        }

        private void ResetAllAxesToZero()
        {
            _suppressAxisEvents = true;
            try
            {
                if (_roll != null)  { _roll.CurrentRaw = 0f;  _roll.SetUIValue(0f); }
                if (_pitch != null) { _pitch.CurrentRaw = 0f; _pitch.SetUIValue(0f); }
                if (_yaw != null)   { _yaw.CurrentRaw = 0f;   _yaw.SetUIValue(0f); }

                SyncAxisUIFromRaw(_roll);
                SyncAxisUIFromRaw(_pitch);
                SyncAxisUIFromRaw(_yaw);
            }
            finally
            {
                _suppressAxisEvents = false;
            }

            PushOutputs();
        }

        // ----------------------------
        // Ref resolution (phase 1)
        // ----------------------------

        private void ResolveRefsIfNeeded()
        {
            if (cascadedController == null)
                cascadedController = UnityEngine.Object.FindFirstObjectByType<CascadedController>();

            if (targetRigidbody == null)
            {
                var body = UnityEngine.Object.FindFirstObjectByType<DroneCore.DroneBody>();
                if (body != null) targetRigidbody = body.Rigidbody;
                if (targetRigidbody == null)
                    targetRigidbody = UnityEngine.Object.FindFirstObjectByType<Rigidbody>();
            }
        }

        // ----------------------------
        // Bottom actions
        // ----------------------------

        private void SwitchCamera()
        {
            Debug.Log("[ControlDeck] SwitchCamera (WIP)");
        }

        private void ResetRotation()
        {
            ResolveRefsIfNeeded();
            if (targetRigidbody == null) return;

            targetRigidbody.angularVelocity = Vector3.zero;
            targetRigidbody.rotation = Quaternion.identity;
        }

        private void ResetPosition()
        {
            ResolveRefsIfNeeded();
            if (targetRigidbody == null) return;

            targetRigidbody.linearVelocity = Vector3.zero;

            // Conservative default: reset to origin + 1m up.
            targetRigidbody.position = new Vector3(0f, 1f, 0f);
        }

        private void ResetDrone()
        {
            ResolveRefsIfNeeded();
            if (targetRigidbody == null) return;

            targetRigidbody.linearVelocity = Vector3.zero;
            targetRigidbody.angularVelocity = Vector3.zero;
            targetRigidbody.position = new Vector3(0f, 1f, 0f);
            targetRigidbody.rotation = Quaternion.identity;
        }
    }
}
