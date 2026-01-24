using System;
using UnityEngine;
using UnityEngine.UIElements;
using SimCore;

namespace QuadSim.UI
{
    [DisallowMultipleComponent]
    public sealed class SimTopBarController : MonoBehaviour
    {
        [Header("Optional refs")]
        [SerializeField] private SimulationManager sim;
        [SerializeField] private MonoBehaviour droneSpawner; // must implement IDroneSpawner

        // Exposed hooks (wired by SimHUDController)
        public Action OnSettingsPressed;
        public Action OnInspectorPressed;
        public Action OnTelemetryPressed;

        private Button _btnSettings, _btnSlower, _btnPlayPause, _btnStep, _btnFaster, _btnReset, _btnSpawn, _btnTelemetry, _btnInspector;
        private Label _lblSpeed;

        private float _currentSpeedScale = 1.0f;

        public void Initialize(VisualElement hudRoot)
        {
            if (sim == null) sim = FindFirstObjectByType<SimulationManager>();

            // IMPORTANT: hudRoot is your compositor root, not document root
            var top = hudRoot.Q<VisualElement>("TopBarRoot");
            if (top == null) Debug.LogError("[TopBar] Could not find TopBarRoot in UXML.");

            _btnSettings  = hudRoot.Q<Button>("BtnSettings");
            _btnSlower    = hudRoot.Q<Button>("BtnSlower");
            _btnPlayPause = hudRoot.Q<Button>("BtnPlayPause");
            _btnStep      = hudRoot.Q<Button>("BtnStep");
            _btnFaster    = hudRoot.Q<Button>("BtnFaster");
            _btnReset     = hudRoot.Q<Button>("BtnReset");
            _btnSpawn     = hudRoot.Q<Button>("BtnSpawn");
            _btnTelemetry = hudRoot.Q<Button>("BtnTelemetry");
            _btnInspector = hudRoot.Q<Button>("BtnInspector");
            _lblSpeed     = hudRoot.Q<Label>("LblSpeed");

            if (_btnSettings == null || _btnPlayPause == null || _lblSpeed == null)
                throw new Exception("[TopBar] UXML is missing expected named elements.");

            _btnStep.tooltip = "Advance one physics step";

            // UI actions -> events
            _btnSettings.clicked  += () => OnSettingsPressed?.Invoke();
            _btnInspector.clicked += () => OnInspectorPressed?.Invoke();
            _btnTelemetry.clicked += () => OnTelemetryPressed?.Invoke();

            // Time controls
            _btnSlower.clicked    += Slower;
            _btnFaster.clicked    += Faster;
            _btnStep.clicked      += StepOnce;
            _btnPlayPause.clicked += PlayPause;
            _btnReset.clicked     += ResetSim;
            _btnSpawn.clicked     += SpawnDrone;

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
            sim.TogglePaused();
            RefreshLabels();
        }

        private void ResetSim()
        {
            if (sim == null) return;

            _currentSpeedScale = 1.0f;
            sim.SetRunMode(SimulationManager.RunMode.FreeRun);
            sim.SetPaused(false);

            ApplyTimeScale();
            sim.ResetSimulation();

            RefreshLabels();
        }

        private void SpawnDrone()
        {
            // if (droneSpawner is IDroneSpawner spawner)
            // {
            //     spawner.SpawnDrone();
            // }
        }

        private void ApplyTimeScale()
        {
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
