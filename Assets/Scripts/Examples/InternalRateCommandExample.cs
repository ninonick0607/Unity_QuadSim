// Assets/Scripts/Examples/InternalRateCommandExample.cs
//
// EXAMPLE: How to control the drone from a user C# script (InputSource.Internal)
//
// HOW TO USE:
//   1. Drop this script onto any GameObject in your scene (NOT on the drone prefab)
//      - e.g., create an empty GameObject called "MyController" and attach this
//   2. In ControlAuthorityManager inspector, set Default Source = Internal
//      - OR leave it as UI and this script will request authority on Start
//   3. Hit Play — the drone should spin its yaw at the configured rate
//
// WHAT THIS DEMONSTRATES:
//   - Registering as InputSource.Internal with the authority system
//   - Requesting authority from ControlAuthorityManager
//   - Sending rate commands every physics tick via FlightCommandProxy
//   - Clean lifecycle (register on Start, cleanup on Destroy)
//
// This script is completely decoupled from the QuadSim codebase.
// It only uses public APIs: DroneManager, ControlAuthorityManager, FlightCommandProxy.

using UnityEngine;
using SimCore;
using SimCore.Common;
using DroneCore.Core;
using DroneCore.Interfaces;

public class InternalRateCommandExample : MonoBehaviour
{
    // ============================================================================
    // Configuration (tweak in Inspector)
    // ============================================================================
    
    [Header("Command Settings")]
    [Tooltip("Roll rate in deg/s")]
    [SerializeField] private float rollRate = 0f;
    
    [Tooltip("Pitch rate in deg/s")]
    [SerializeField] private float pitchRate = 0f;
    
    [Tooltip("Yaw rate in deg/s")]
    [SerializeField] private float yawRate = 30f;
    
    [Tooltip("Throttle [0..1]. ~0.41 is hover for the standard quad.")]
    [SerializeField] private float throttle = 0.41f;
    
    [Header("Behavior")]
    [Tooltip("Automatically request authority on Start. If false, you must grant it manually.")]
    [SerializeField] private bool requestAuthorityOnStart = true;
    
    [Tooltip("Which GoalMode to command in. Rate is simplest for testing.")]
    [SerializeField] private GoalMode commandMode = GoalMode.Rate;

    // ============================================================================
    // Runtime State
    // ============================================================================
    
    private FlightCommandProxy _proxy;
    private ControlAuthorityManager _authority;
    private bool _isActive;

    // ============================================================================
    // Lifecycle
    // ============================================================================
    
    private void Start()
    {
        StartCoroutine(InitializeDelayed());
    }

    private System.Collections.IEnumerator InitializeDelayed()
    {
        // Wait one frame for drone spawning to complete
        yield return null;

        // 1. Find the authority manager
        _authority = ControlAuthorityManager.Get();
        if (_authority == null)
        {
            Debug.LogError("[InternalExample] No ControlAuthorityManager in scene!");
            enabled = false;
            yield break;
        }

        _authority.NotifyInternalConnected();

        // 2. Find the drone
        var droneManager = DroneManager.Get();
        if (droneManager == null || droneManager.SelectedDrone == null)
        {
            Debug.LogError("[InternalExample] No DroneManager or no drone spawned!");
            enabled = false;
            yield break;
        }

        _proxy = droneManager.SelectedDrone.CommandProxy;

        // 3. Request authority
        if (requestAuthorityOnStart)
        {
            _isActive = _authority.RequestAuthority(InputSource.Internal);
            if (_isActive)
                Debug.Log("[InternalExample] Authority granted. Sending commands.");
            else
                Debug.LogWarning("[InternalExample] Authority request denied.");
        }
    }

    /// <summary>
    /// Send commands in Update. The proxy validates the source — if we don't have
    /// authority, commands are silently rejected.
    /// 
    /// NOTE: Commands are consumed by the controller in PrePhysicsStep (250Hz).
    /// Sending in Update (~60Hz) means the same command persists across multiple
    /// physics ticks, which is fine for rate commands. For time-critical control,
    /// you'd register as an ISimulatable and send in PrePhysicsStep instead.
    /// </summary>
    private void Update()
    {
        if (!_isActive || _proxy == null) return;
        
        // Build the command
        Axis4 cmd = new Axis4(rollRate, pitchRate, yawRate, throttle);
        
        // Send it — the proxy validates that we're InputSource.Internal
        // and that Internal has authority. If not, it returns false.
        _proxy.SetCommand(cmd, commandMode, InputSource.Internal);
    }

    private void OnDestroy()
    {
        // Clean up: tell authority system we're gone
        if (_authority != null)
        {
            _authority.NotifyInternalDisconnected();
        }
        
        Debug.Log("[InternalExample] Destroyed. Internal source disconnected.");
    }
}