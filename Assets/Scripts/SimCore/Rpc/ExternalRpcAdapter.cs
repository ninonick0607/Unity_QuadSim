// Assets/Scripts/SimCore/Rpc/ExternalRpcAdapter.cs
// Phase 9a + 9b: External RPC Adapter
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
// PHASE 9a: connect / disconnect / heartbeat / get_status
// PHASE 9b: set_mode, send_command, get_sensor_data, get_telemetry,
//           set_controller, reset_all, reset_pose, reset_rotation,
//           reset_physics, reset_controller, pause, resume, step,
//           set_time_scale, reset_simulation
//
// FUTURE PHASES:
//   - 9c: PUB telemetry streaming
//   - 9d: Python client library

using System;
using System.Collections.Generic;
using UnityEngine;
using SimCore.Common;
using DroneCore.Api;

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
        // String ↔ Enum Maps (Phase 9b)
        // ============================================================================

        private static readonly Dictionary<string, GoalMode> GoalModeMap = new()
        {
            ["none"]        = GoalMode.None,
            ["position"]    = GoalMode.Position,
            ["velocity"]    = GoalMode.Velocity,
            ["angle"]       = GoalMode.Angle,
            ["rate"]        = GoalMode.Rate,
            ["passthrough"] = GoalMode.Passthrough,
            ["wrench"]      = GoalMode.Wrench,
        };

        private static readonly Dictionary<GoalMode, string> GoalModeReverseMap = new()
        {
            [GoalMode.None]        = "none",
            [GoalMode.Position]    = "position",
            [GoalMode.Velocity]    = "velocity",
            [GoalMode.Angle]       = "angle",
            [GoalMode.Rate]        = "rate",
            [GoalMode.Passthrough] = "passthrough",
            [GoalMode.Wrench]      = "wrench",
        };

        private static readonly Dictionary<string, ControllerKind> ControllerKindMap = new()
        {
            ["cascade"]   = ControllerKind.Cascade,
            ["geometric"] = ControllerKind.Geometric,
        };

        private static readonly Dictionary<ControllerKind, string> ControllerKindReverseMap = new()
        {
            [ControllerKind.Cascade]   = "cascade",
            [ControllerKind.Geometric] = "geometric",
        };

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
                        // Any valid request from a connected client proves liveness —
                        // refresh heartbeat so active clients don't time out.
                        if (_clientConnected)
                        {
                            _lastHeartbeatTime = Time.unscaledTime;
                        }

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
        // Handler Table
        // ============================================================================

        private void BuildHandlerTable()
        {
            _handlers = new Dictionary<string, Func<RpcRequest, RpcResponse>>
            {
                // --- Phase 9a: Connection lifecycle ---
                ["connect"]          = HandleConnect,
                ["disconnect"]       = HandleDisconnect,
                ["heartbeat"]        = HandleHeartbeat,
                ["get_status"]       = HandleGetStatus,

                // --- Phase 9b: Drone control ---
                ["set_mode"]         = HandleSetMode,
                ["send_command"]     = HandleSendCommand,
                ["get_sensor_data"]  = HandleGetSensorData,
                ["get_telemetry"]    = HandleGetTelemetry,
                ["set_controller"]   = HandleSetController,

                // --- Phase 9b: Reset operations ---
                ["reset_all"]        = HandleResetAll,
                ["reset_pose"]       = HandleResetPose,
                ["reset_rotation"]   = HandleResetRotation,
                ["reset_physics"]    = HandleResetPhysics,
                ["reset_controller"] = HandleResetController,

                // --- Phase 9b: Simulation control ---
                ["pause"]            = HandlePause,
                ["resume"]           = HandleResume,
                ["step"]             = HandleStep,
                ["set_time_scale"]   = HandleSetTimeScale,
                ["reset_simulation"] = HandleResetSimulation,
            };
        }

        /// <summary>
        /// Register additional method handlers (for Phase 9c, 9d extensions).
        /// Call this before or after server start — handlers are on the main thread.
        /// </summary>
        public void RegisterHandler(string method, Func<RpcRequest, RpcResponse> handler)
        {
            _handlers[method] = handler;
        }

        // ============================================================================
        // Phase 9a Handlers: Connection Lifecycle
        // ============================================================================

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
        // Phase 9b Handlers: Drone Control
        // ============================================================================

        // --- set_mode ---

        private RpcResponse HandleSetMode(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            string modeStr = req.GetString("mode");
            if (modeStr == null)
            {
                return RpcResponse.Error("missing_param: mode", req.RequestId);
            }

            if (!GoalModeMap.TryGetValue(modeStr.ToLowerInvariant(), out var goalMode))
            {
                return RpcResponse.Error($"invalid_mode: {modeStr}", req.RequestId);
            }

            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.SetMode(goalMode);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["mode"] = GoalModeReverseMap[drone.CurrentMode],
            });
        }

        // --- send_command ---

        private RpcResponse HandleSendCommand(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            // Pre-check authority: QuadSimApi.SendCommand doesn't surface the proxy's
            // rejection bool, so we check authority here to give a clear error to the client.
            if (_sim.CurrentAuthority != InputSource.External)
            {
                return RpcResponse.Error("authority_rejected", req.RequestId);
            }

            // Extract Axis4 values
            float x = req.GetFloat("x");
            float y = req.GetFloat("y");
            float z = req.GetFloat("z");
            float w = req.GetFloat("w");
            var axis4 = new Axis4(x, y, z, w);

            // Optional mode override — set mode + send in one call
            string modeStr = req.GetString("mode");
            bool success;

            if (modeStr != null)
            {
                if (!GoalModeMap.TryGetValue(modeStr.ToLowerInvariant(), out var goalMode))
                {
                    return RpcResponse.Error($"invalid_mode: {modeStr}", req.RequestId);
                }

                success = drone.SendCommand(InputSource.External, axis4, goalMode);
            }
            else
            {
                success = drone.SendCommand(InputSource.External, axis4);
            }

            if (!success)
            {
                // SendCommand returns false if drone isn't initialized or proxy is missing
                return RpcResponse.Error("command_failed", req.RequestId);
            }

            return RpcResponse.Ok(req.RequestId);
        }

        // --- get_sensor_data ---

        private RpcResponse HandleGetSensorData(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            var s = drone.GetSensorData();

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["imu_ang_vel"]     = Vec3ToArray(s.ImuAngVel),
                ["imu_attitude"]    = Vec3ToArray(s.ImuAttitude),
                ["imu_accel"]       = Vec3ToArray(s.ImuAccel),
                ["imu_vel"]         = Vec3ToArray(s.ImuVel),
                ["imu_orientation"] = QuatToArray(s.ImuOrientation),
                ["imu_timestamp"]   = s.ImuTimestampSec,
                ["imu_valid"]       = s.ImuValid,
                ["gps_position"]    = Vec3ToArray(s.GpsPosition),
                ["gps_timestamp"]   = s.GpsTimestampSec,
                ["gps_valid"]       = s.GpsValid,
            });
        }

        // --- get_telemetry ---

        private RpcResponse HandleGetTelemetry(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            var t = drone.GetTelemetry();

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["drone_id"]           = t.DroneID ?? "",
                ["mode"]               = GoalModeReverseMap.GetValueOrDefault(t.Mode, "unknown"),
                ["controller"]         = ControllerKindReverseMap.GetValueOrDefault(t.Controller, "unknown"),
                ["motors"]             = new object[] { t.MotorOutputs.FL, t.MotorOutputs.FR,
                                                        t.MotorOutputs.BL, t.MotorOutputs.BR },
                ["desired_rates_deg"]  = Vec3ToArray(t.ControllerTelemetry.DesiredRatesDeg),
                ["desired_angles_deg"] = Vec3ToArray(t.ControllerTelemetry.DesiredAnglesDeg),
                ["desired_vel"]        = Vec3ToArray(t.ControllerTelemetry.DesiredVel),
                ["external_cmd"]       = new object[] { t.ControllerTelemetry.ExternalCmd.X,
                                                        t.ControllerTelemetry.ExternalCmd.Y,
                                                        t.ControllerTelemetry.ExternalCmd.Z,
                                                        t.ControllerTelemetry.ExternalCmd.W },
            });
        }

        // --- set_controller ---

        private RpcResponse HandleSetController(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            string kindStr = req.GetString("controller");
            if (kindStr == null)
            {
                return RpcResponse.Error("missing_param: controller", req.RequestId);
            }

            if (!ControllerKindMap.TryGetValue(kindStr.ToLowerInvariant(), out var kind))
            {
                return RpcResponse.Error($"invalid_controller: {kindStr}", req.RequestId);
            }

            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.SetController(kind);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["controller"] = ControllerKindReverseMap[drone.CurrentController],
            });
        }

        // ============================================================================
        // Phase 9b Handlers: Reset Operations
        // ============================================================================

        // --- reset_all ---

        private RpcResponse HandleResetAll(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetAll();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_pose ---

        private RpcResponse HandleResetPose(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            float px = req.GetFloat("x", 0f);
            float py = req.GetFloat("y", 3f);  // default spawn height
            float pz = req.GetFloat("z", 0f);
            var position = new Vector3(px, py, pz);

            // Quaternion defaults to identity if not provided
            float qx = req.GetFloat("qx", 0f);
            float qy = req.GetFloat("qy", 0f);
            float qz = req.GetFloat("qz", 0f);
            float qw = req.GetFloat("qw", 1f);
            var rotation = new Quaternion(qx, qy, qz, qw);

            drone.ResetPose(position, rotation);

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_rotation ---

        private RpcResponse HandleResetRotation(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetRotation();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_physics ---

        private RpcResponse HandleResetPhysics(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetPhysics();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_controller ---

        private RpcResponse HandleResetController(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetController();

            return RpcResponse.Ok(req.RequestId);
        }

        // ============================================================================
        // Phase 9b Handlers: Simulation Control
        // ============================================================================

        // --- pause ---

        private RpcResponse HandlePause(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.Pause();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- resume ---

        private RpcResponse HandleResume(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.Resume();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- step ---

        private RpcResponse HandleStep(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            int count = req.GetInt("count", 1);

            if (count <= 0)
            {
                return RpcResponse.Error("invalid_param: count must be > 0", req.RequestId);
            }

            if (count == 1)
            {
                _sim.StepOnce();
            }
            else
            {
                _sim.StepMany(count);
            }

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["sim_time"] = _sim.SimTimeSec,
                ["steps_queued"] = count,
            });
        }

        // --- set_time_scale ---

        private RpcResponse HandleSetTimeScale(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            float scale = req.GetFloat("scale", 1.0f);

            if (scale <= 0f)
            {
                return RpcResponse.Error("invalid_param: scale must be > 0", req.RequestId);
            }

            _sim.SetTimeScale(scale);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["time_scale"] = _sim.TimeScale,
            });
        }

        // --- reset_simulation ---

        private RpcResponse HandleResetSimulation(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.ResetSimulation();

            return RpcResponse.Ok(req.RequestId);
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

        // ============================================================================
        // Shared Helpers (Phase 9b)
        // ============================================================================

        /// <summary>
        /// Guard: require the client to be connected. Returns false and sets the
        /// error response if not connected.
        /// </summary>
        private bool RequireConnected(RpcRequest req, out RpcResponse errorResponse)
        {
            if (_clientConnected)
            {
                errorResponse = default;
                return true;
            }

            errorResponse = RpcResponse.Error("not_connected", req.RequestId);
            return false;
        }

        /// <summary>
        /// Guard: resolve the selected drone via QuadSimApi.Selected().
        /// Returns false and sets the error response if no drone is available.
        /// Phase 9b is single-drone — multi-drone index parameter is a future addition.
        /// </summary>
        private bool TryGetSelectedDrone(RpcRequest req, out QuadSimApi drone, out RpcResponse errorResponse)
        {
            drone = QuadSimApi.Selected();

            if (drone != null)
            {
                errorResponse = default;
                return true;
            }

            errorResponse = RpcResponse.Error("no_drone", req.RequestId);
            return false;
        }

        /// <summary>Convert Vector3 to float[] for MessagePack serialization.</summary>
        private static object[] Vec3ToArray(Vector3 v)
        {
            return new object[] { v.x, v.y, v.z };
        }

        /// <summary>Convert Quaternion to float[] for MessagePack serialization.</summary>
        private static object[] QuatToArray(Quaternion q)
        {
            return new object[] { q.x, q.y, q.z, q.w };
        }
    }
}