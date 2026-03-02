// Assets/Scripts/SimCore/Rpc/ExternalRpcAdapter.cs
// Phase 9a: External RPC Adapter (Foundation)
//
// PURPOSE:
//   MonoBehaviour that lives on SimRoot. Bridges the background ZMQ server thread
//   to the Unity main thread where SimCoreApi and QuadSimApi calls must happen.
//
// RESPONSIBILITIES:
//   - Start/stop the RpcServerThread
//   - Drain incoming requests in Update() and dispatch to method handlers
//   - Manage External source lifecycle through SimCoreApi
//   - Track connection state and heartbeat timeout
//
// PHASE 9a SCOPE:
//   - connect / disconnect / heartbeat / get_status
//   - Connection lifecycle through ControlAuthorityManager
//   - Heartbeat-based liveness detection
//
// FUTURE PHASES:
//   - 9b: set_mode, send_command
//   - 9c: PUB telemetry streaming
//   - 9d: reset, time control, on-demand queries

using System;
using System.Collections.Generic;
using UnityEngine;
using SimCore.Common;

namespace SimCore.Rpc
{
    /// <summary>
    /// Bridges External Python clients to the simulation via ZeroMQ.
    /// Attach to SimRoot alongside SimCoreApi.
    ///
    /// The adapter starts a background thread with REP + PUB sockets.
    /// Requests arrive on the background thread, get queued to main thread,
    /// processed against SimCoreApi/QuadSimApi, and responses are sent back.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ExternalRpcAdapter : MonoBehaviour
    {
        // ============================================================================
        // Configuration
        // ============================================================================

        [Header("Server Settings")]
        [Tooltip("Port for the REQ/REP command socket.")]
        [SerializeField] private int commandPort = 5555;

        [Tooltip("Port for the PUB telemetry socket.")]
        [SerializeField] private int telemetryPort = 5556;

        [Tooltip("Start the RPC server automatically on Awake.")]
        [SerializeField] private bool autoStart = true;

        [Header("Connection Settings")]
        [Tooltip("Seconds without a heartbeat before considering the client disconnected.")]
        [SerializeField] private float heartbeatTimeoutSec = 5.0f;

        [Tooltip("Maximum requests to process per frame (prevents stalling).")]
        [SerializeField] private int maxRequestsPerFrame = 10;

        // ============================================================================
        // Runtime State
        // ============================================================================

        private RpcServerThread _server;
        private SimCoreApi _sim;

        // Connection tracking
        private bool _clientConnected;
        private string _clientName;
        private float _lastHeartbeatTime;

        // Pending accept/deny (when External is not auto-accepted)
        private bool _pendingConnectionApproval;
        private QueuedRequest _pendingConnectRequest;

        // Method dispatch table
        private Dictionary<string, Func<RpcRequest, RpcResponse>> _handlers;

        // ============================================================================
        // Public Accessors
        // ============================================================================

        /// <summary>True if the ZMQ server thread is running.</summary>
        public bool IsServerRunning => _server?.IsRunning ?? false;

        /// <summary>True if an External client is currently connected.</summary>
        public bool IsClientConnected => _clientConnected;

        /// <summary>Name of the connected client (from connect message), or null.</summary>
        public string ClientName => _clientName;

        /// <summary>Command port number.</summary>
        public int CommandPort => commandPort;

        /// <summary>Telemetry port number.</summary>
        public int TelemetryPort => telemetryPort;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            BuildHandlerTable();
        }

        private void Start()
        {
            _sim = SimCoreApi.Get();
            if (_sim == null)
            {
                Debug.LogError("[RpcAdapter] No SimCoreApi found. ExternalRpcAdapter requires SimCoreApi on SimRoot.");
                enabled = false;
                return;
            }

            // Subscribe to authority events for accept/deny flow
            _sim.OnAuthorityChanged += HandleAuthorityChanged;

            if (autoStart)
            {
                StartServer();
            }
        }

        private void Update()
        {
            if (_server == null) return;

            // Drain log messages from background thread
            DrainLogs();

            // Process incoming requests
            ProcessRequests();

            // Check heartbeat timeout
            CheckHeartbeatTimeout();
        }

        private void OnDestroy()
        {
            StopServer();

            if (_sim != null)
            {
                _sim.OnAuthorityChanged -= HandleAuthorityChanged;
            }
        }

        private void OnApplicationQuit()
        {
            StopServer();
        }

        // ============================================================================
        // Server Lifecycle
        // ============================================================================

        /// <summary>Start the ZMQ server thread.</summary>
        public void StartServer()
        {
            if (_server != null && _server.IsRunning)
            {
                Debug.LogWarning("[RpcAdapter] Server already running.");
                return;
            }

            _server = new RpcServerThread(commandPort, telemetryPort);
            _server.Start();

            Debug.Log($"[RpcAdapter] Starting RPC server — REP:{commandPort} PUB:{telemetryPort}");
        }

        /// <summary>Stop the ZMQ server thread and clean up.</summary>
        public void StopServer()
        {
            if (_server == null) return;

            // If a client was connected, notify disconnect
            if (_clientConnected)
            {
                HandleClientDisconnect("server_shutdown");
            }

            _server.Stop();
            _server.Dispose();
            _server = null;

            Debug.Log("[RpcAdapter] RPC server stopped.");
        }

        // ============================================================================
        // Request Processing (Main Thread)
        // ============================================================================

        private void ProcessRequests()
        {
            int processed = 0;

            while (processed < maxRequestsPerFrame && _server.TryDequeue(out var queued))
            {
                processed++;

                RpcResponse response;

                try
                {
                    if (_handlers.TryGetValue(queued.Request.Method, out var handler))
                    {
                        response = handler(queued.Request);
                    }
                    else
                    {
                        response = RpcResponse.Error(
                            $"unknown_method: {queued.Request.Method}",
                            queued.Request.RequestId);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[RpcAdapter] Exception handling '{queued.Request.Method}': {ex}");
                    response = RpcResponse.Error("internal_error", queued.Request.RequestId);
                }

                // Serialize and signal the background thread
                byte[] responseBytes = RpcSerializer.SerializeResponse(response);
                queued.SetResponse(responseBytes);
            }
        }

        private void DrainLogs()
        {
            while (_server.TryDequeueLog(out var level, out var message))
            {
                switch (level)
                {
                    case RpcServerThread.LogLevel.Info:
                        Debug.Log(message);
                        break;
                    case RpcServerThread.LogLevel.Warning:
                        Debug.LogWarning(message);
                        break;
                    case RpcServerThread.LogLevel.Error:
                        Debug.LogError(message);
                        break;
                }
            }
        }

        // ============================================================================
        // Heartbeat Monitoring
        // ============================================================================

        private void CheckHeartbeatTimeout()
        {
            if (!_clientConnected) return;
            if (heartbeatTimeoutSec <= 0f) return;

            float elapsed = Time.unscaledTime - _lastHeartbeatTime;

            if (elapsed > heartbeatTimeoutSec)
            {
                Debug.LogWarning($"[RpcAdapter] Heartbeat timeout ({elapsed:F1}s > {heartbeatTimeoutSec}s). " +
                                 $"Disconnecting client '{_clientName}'.");
                HandleClientDisconnect("heartbeat_timeout");
            }
        }

        // ============================================================================
        // Method Handlers (Phase 9a)
        // ============================================================================

        private void BuildHandlerTable()
        {
            _handlers = new Dictionary<string, Func<RpcRequest, RpcResponse>>
            {
                ["connect"]    = HandleConnect,
                ["disconnect"] = HandleDisconnect,
                ["heartbeat"]  = HandleHeartbeat,
                ["get_status"] = HandleGetStatus,
            };
        }

        /// <summary>
        /// Register additional method handlers (for Phase 9b, 9c, 9d extensions).
        /// Call this before or after server start — handlers are on the main thread.
        /// </summary>
        public void RegisterHandler(string method, Func<RpcRequest, RpcResponse> handler)
        {
            _handlers[method] = handler;
        }

        // --- connect ---

        private RpcResponse HandleConnect(RpcRequest req)
        {
            if (_clientConnected)
            {
                return RpcResponse.Error("already_connected", req.RequestId);
            }

            string clientName = req.GetString("client_name") ?? "unknown";

            Debug.Log($"[RpcAdapter] Connection request from '{clientName}'");

            // Check if External is denied for this session
            if (_sim.IsExternalDenied)
            {
                Debug.Log($"[RpcAdapter] Rejecting '{clientName}' — External is denied for this session.");
                return RpcResponse.Error("denied", req.RequestId);
            }

            // Notify the authority manager
            _sim.NotifyExternalConnectionAttempt();

            // Check if it was auto-accepted (External is default + autoAccept)
            var externalStatus = _sim.GetSourceStatus(InputSource.External);
            if (externalStatus == SourceStatus.Connected)
            {
                // Auto-accepted
                CompleteConnection(clientName);
                return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
                {
                    ["message"] = "connected",
                    ["server"] = "QuadSim",
                    ["command_port"] = commandPort,
                    ["telemetry_port"] = telemetryPort,
                });
            }

            if (externalStatus == SourceStatus.Denied)
            {
                return RpcResponse.Error("denied", req.RequestId);
            }

            // Pending approval — for Phase 9a, we auto-accept since there's no UI popup yet.
            // Phase 10 will add the UI popup. For now, accept immediately.
            // TODO(Phase 10): Hold the request and wait for UI approval.
            _sim.AcceptExternalConnection();
            CompleteConnection(clientName);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["message"] = "connected",
                ["server"] = "QuadSim",
                ["command_port"] = commandPort,
                ["telemetry_port"] = telemetryPort,
            });
        }

        private void CompleteConnection(string clientName)
        {
            _clientConnected = true;
            _clientName = clientName;
            _lastHeartbeatTime = Time.unscaledTime;

            Debug.Log($"[RpcAdapter] Client '{clientName}' connected. External source active.");
        }

        // --- disconnect ---

        private RpcResponse HandleDisconnect(RpcRequest req)
        {
            if (!_clientConnected)
            {
                return RpcResponse.Ok(req.RequestId); // Idempotent
            }

            Debug.Log($"[RpcAdapter] Client '{_clientName}' requested disconnect.");
            HandleClientDisconnect("client_request");

            return RpcResponse.Ok(req.RequestId);
        }

        // --- heartbeat ---

        private RpcResponse HandleHeartbeat(RpcRequest req)
        {
            if (!_clientConnected)
            {
                return RpcResponse.Error("not_connected", req.RequestId);
            }

            _lastHeartbeatTime = Time.unscaledTime;

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["sim_time"] = _sim.SimTimeSec,
                ["is_paused"] = _sim.IsPaused,
            });
        }

        // --- get_status ---

        private RpcResponse HandleGetStatus(RpcRequest req)
        {
            var status = _sim.GetStatus();

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["is_paused"] = status.IsPaused,
                ["time_scale"] = status.TimeScale,
                ["sim_time"] = status.SimTimeSec,
                ["fixed_dt"] = status.FixedDtSec,
                ["authority"] = status.CurrentAuthority.ToString(),
                ["ui_status"] = status.UIStatus.ToString(),
                ["internal_status"] = status.InternalStatus.ToString(),
                ["external_status"] = status.ExternalStatus.ToString(),
                ["client_connected"] = _clientConnected,
                ["client_name"] = _clientName ?? "",
            });
        }

        // ============================================================================
        // Disconnect Handling
        // ============================================================================

        private void HandleClientDisconnect(string reason)
        {
            string name = _clientName ?? "unknown";

            _clientConnected = false;
            _clientName = null;
            _lastHeartbeatTime = 0f;

            // Notify authority manager
            if (_sim != null)
            {
                _sim.NotifyExternalDisconnected();
            }

            Debug.Log($"[RpcAdapter] Client '{name}' disconnected. Reason: {reason}");
        }

        // ============================================================================
        // Event Handlers
        // ============================================================================

        private void HandleAuthorityChanged(InputSource oldAuth, InputSource newAuth)
        {
            // If authority moved away from External while a client is connected,
            // the client's commands will be rejected by the proxy. We don't
            // disconnect them — they can still read telemetry and get_status.
            if (_clientConnected && oldAuth == InputSource.External && newAuth != InputSource.External)
            {
                Debug.Log($"[RpcAdapter] Authority changed from External to {newAuth}. " +
                          "Client remains connected but commands will be rejected.");
            }
        }
    }
}