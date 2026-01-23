using System;
using UnityEngine;
using UnityEngine.UIElements;
using SimCore;

namespace QuadSim.UI
{
    public interface IDroneSpawner
    {
        void SpawnDrone();
    }

    [DisallowMultipleComponent]
    public sealed class SimTopBarController : MonoBehaviour
    {
        [Header("Refs (optional)")]
        [SerializeField] private SimulationManager sim;
        [SerializeField] private MonoBehaviour droneSpawner; // must implement IDroneSpawner

        [Header("Callbacks (optional)")]
        public Action OnToggleSettings;
        public Action OnToggleInspector;
        public Action OnToggleTelemetry;

        private Button _btnSettings, _btnSlower, _btnPlayPause, _btnStep, _btnFaster, _btnReset, _btnSpawn, _btnTelemetry, _btnInspector;
        private Label _lblSpeed;

        private float _currentSpeedScale = 1.0f;

        private void Awake()
        {
            if (sim == null) sim = FindFirstObjectByType<SimulationManager>();

            var doc = GetComponent<UIDocument>();
            if (doc == null) throw new Exception("SimTopBarController requires a UIDocument on the same GameObject.");

            var root = GetComponent<UIDocument>().rootVisualElement;
            root.style.backgroundColor = new Color(1, 0, 0, 0.5f);
            
            _btnSettings  = root.Q<Button>("BtnSettings");
            _btnSlower    = root.Q<Button>("BtnSlower");
            _btnPlayPause = root.Q<Button>("BtnPlayPause");
            _btnStep      = root.Q<Button>("BtnStep");
            _btnFaster    = root.Q<Button>("BtnFaster");
            _btnReset     = root.Q<Button>("BtnReset");
            _btnSpawn     = root.Q<Button>("BtnSpawn");
            _btnTelemetry = root.Q<Button>("BtnTelemetry");
            _btnInspector = root.Q<Button>("BtnInspector");
            _lblSpeed     = root.Q<Label>("LblSpeed");

            if (_btnSettings == null || _btnPlayPause == null || _lblSpeed == null)
                throw new Exception("SimTopBar UXML is missing expected named elements.");

            // Tooltips (optional)
            _btnStep.tooltip = "Advance one physics step";

            // Wire UI actions
            _btnSettings.clicked  += () => OnToggleSettings?.Invoke();
            _btnTelemetry.clicked += () => OnToggleTelemetry?.Invoke();
            _btnInspector.clicked += () => OnToggleInspector?.Invoke();

            _btnSlower.clicked += Slower;
            _btnFaster.clicked += Faster;
            _btnStep.clicked += StepOnce;
            _btnPlayPause.clicked += PlayPause;
            _btnReset.clicked += ResetSim;
            _btnSpawn.clicked += SpawnDrone;

            RefreshLabels();
        }

        private void Slower()
        {
            _currentSpeedScale = Mathf.Max(0.05f, _currentSpeedScale - 0.10f);
            ApplyTimeScale();
        }

        private void Faster()
        {
            _currentSpeedScale += 0.25f;
            ApplyTimeScale();
        }

        private void StepOnce()
        {
            if (sim == null) return;
            sim.StepOnce();
        }

        private void PlayPause()
        {
            if (sim == null) return;

            // Your SimulationManager already has TogglePaused()
            sim.TogglePaused();

            RefreshLabels();
        }

        private void ResetSim()
        {
            if (sim == null) return;

            
            _currentSpeedScale = 1.0f;
            sim.SetRunMode(SimulationManager.RunMode.FreeRun);
            sim.SetPaused(false);

            // Ensure timescale is applied (ClockFactory)
            ApplyTimeScale();

            sim.ResetSimulation();
            RefreshLabels();
            
        }

        private void SpawnDrone()
        {
            if (droneSpawner is IDroneSpawner spawner)
            {
                spawner.SpawnDrone();
                return;
            }
            // optional fallback: do nothing
        }

        private void ApplyTimeScale()
        {
            // SimulationManager syncs timeScale in Update -> ClockFactory.SetTimeScale(timeScale)
            // So we need to write to its serialized field or expose a method.
            // Add this method to SimulationManager: SetTimeScale(double ts)
            if (sim == null) return;
            sim.SetTimeScale(_currentSpeedScale);

            RefreshLabels();
        }

        private void RefreshLabels()
        {
            if (_lblSpeed != null)
                _lblSpeed.text = $"x{_currentSpeedScale:0.00}";

            if (_btnPlayPause != null && sim != null)
                _btnPlayPause.text = sim.IsPaused ? "RESUME" : "PAUSE";
        }
    }
}
