using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.UI
{
    /// <summary>
    /// The compositor/controller for the whole HUD.
    /// Owns wiring between TopBar and Telemetry drawer.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SimHUDController : MonoBehaviour
    {
        [Header("Scene refs")]
        [SerializeField] private UIDocument document;

        [Header("Child controllers (same GO)")]
        [SerializeField] private SimTopBarController topBar;
        [SerializeField] private TelemetryDeckController telemetry;

        private VisualElement _docRoot;
        private VisualElement _hudRoot;

        private void Awake()
        {
            if (document == null) document = GetComponent<UIDocument>();
            if (topBar == null) topBar = GetComponent<SimTopBarController>();
            if (telemetry == null) telemetry = GetComponent<TelemetryDeckController>();

            if (document == null)
            {
                Debug.LogError("[SimHUD] Missing UIDocument.");
                enabled = false;
                return;
            }

            _docRoot = document.rootVisualElement;
            if (_docRoot == null)
            {
                Debug.LogError("[SimHUD] document.rootVisualElement is null.");
                enabled = false;
                return;
            }

            // This MUST match the name in Sim_Hud.uxml: <ui:VisualElement name="HUDRoot" ...>
            _hudRoot = _docRoot.Q<VisualElement>("HUDRoot");
            if (_hudRoot == null)
            {
                Debug.LogError("[SimHUD] Could not find HUDRoot. Check Sim_Hud.uxml element name.");
                enabled = false;
                return;
            }

            // Hard guarantee that HUDRoot is a true full-screen anchor.
            ForceFullScreen(_hudRoot);

            // Initialize child controllers against the same HUD root
            if (telemetry != null) telemetry.Initialize(_hudRoot);
            if (topBar != null) topBar.Initialize(_hudRoot);

            // Wire TopBar -> Telemetry here (TopBar does NOT FindFirstObjectByType).
            if (topBar != null && telemetry != null)
            {
                topBar.OnTelemetryPressed = telemetry.Toggle;
            }
        }

        private static void ForceFullScreen(VisualElement ve)
        {
            ve.style.position = Position.Absolute;
            ve.style.left = 0;
            ve.style.right = 0;
            ve.style.top = 0;
            ve.style.bottom = 0;
        }
    }
}
