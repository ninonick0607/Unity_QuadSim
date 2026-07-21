// Assets/Scripts/SimCore/Rpc/RpcServerThread.cs
// Phase 9a + 9c: Background ZMQ Server Thread
//
// PURPOSE:
//   Runs on a background thread. Owns the NetMQ REP and PUB sockets.
//   Polls REP for incoming requests, enqueues them for the main thread,
//   and waits for the main thread to provide responses.
//   Drains the publish queue and sends frames on the PUB socket (Phase 9c).
//
// THREADING CONTRACT:
//   - This class runs entirely on the background thread.
//   - Communication with Unity main thread via ConcurrentQueues.
//   - Main thread signals responses via ManualResetEventSlim.
//   - Shutdown is signaled via _shutdownToken.
//
// DOES NOT:
//   - Call any Unity API (no MonoBehaviour, no Debug.Log from bg thread)
//   - Process requests — just relays them
//   - Know about SimCoreApi, QuadSimApi, or any game logic

using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace SimCore.Rpc
{
    /// <summary>
    /// Queued request from background thread to main thread.
    /// </summary>
    public sealed class QueuedRequest
    {
        public RpcRequest Request;
        public readonly ManualResetEventSlim ResponseReady = new ManualResetEventSlim(false);
        public byte[] ResponseBytes;

        public void SetResponse(byte[] bytes)
        {
            ResponseBytes = bytes;
            ResponseReady.Set();
        }
    }

    /// <summary>
    /// Background thread that manages ZMQ sockets for the External RPC adapter.
    /// 
    /// Lifecycle:
    ///   1. Constructed with config (ports, timeouts)
    ///   2. Start() spins up the thread
    ///   3. Main thread calls TryDequeue() in Update() to get requests
    ///   4. Main thread processes request, calls SetResponse() on the QueuedRequest
    ///   5. Background thread wakes up and sends the response over ZMQ
    ///   6. Main thread calls EnqueuePublish() to push telemetry frames (Phase 9c)
    ///   7. Background thread drains publish queue onto PUB socket each cycle
    ///   8. Stop() signals shutdown, joins the thread
    /// </summary>
    public sealed class RpcServerThread : IDisposable
    {
        // ============================================================================
        // Configuration
        // ============================================================================

        private readonly int _repPort;
        private readonly int _pubPort;
        private readonly int _pollTimeoutMs;

        // ============================================================================
        // Thread Communication
        // ============================================================================

        /// <summary>
        /// Requests waiting for the main thread to process.
        /// Background thread enqueues, main thread dequeues.
        /// </summary>
        private readonly ConcurrentQueue<QueuedRequest> _pendingRequests = new();

        /// <summary>
        /// Log messages from the background thread (since we can't use Debug.Log).
        /// Main thread drains these in Update().
        /// </summary>
        private readonly ConcurrentQueue<(LogLevel, string)> _logQueue = new();

        /// <summary>
        /// Publish frames queued by the main thread for the PUB socket (Phase 9c).
        /// Main thread enqueues serialized MessagePack frames, background thread sends them.
        /// </summary>
        private readonly ConcurrentQueue<byte[]> _publishQueue = new();

        public enum LogLevel { Info, Warning, Error }

        // ============================================================================
        // Thread State
        // ============================================================================

        private Thread _thread;
        private volatile bool _shutdownRequested;
        private volatile bool _isRunning;

        /// <summary>True if the background thread is actively running and sockets are bound.</summary>
        public bool IsRunning => _isRunning;

        /// <summary>True during startup, normal operation, or a delayed shutdown.</summary>
        public bool IsThreadAlive => _thread != null && _thread.IsAlive;

        // ============================================================================
        // Construction
        // ============================================================================

        /// <param name="repPort">Port for REQ/REP command socket (default 5555).</param>
        /// <param name="pubPort">Port for PUB telemetry socket (default 5556).</param>
        /// <param name="pollTimeoutMs">ZMQ poll interval in ms. Lower = more responsive, higher = less CPU (default 10).</param>
        public RpcServerThread(int repPort = 5555, int pubPort = 5556, int pollTimeoutMs = 10)
        {
            _repPort = repPort;
            _pubPort = pubPort;
            _pollTimeoutMs = pollTimeoutMs;
        }

        // ============================================================================
        // Lifecycle
        // ============================================================================

        /// <summary>
        /// Start the background thread. Binds ZMQ sockets and begins polling.
        /// </summary>
        public void Start()
        {
            // IsRunning becomes true only after both sockets bind. Guard on the
            // actual thread so another Start cannot enter during that window.
            if (_thread != null && _thread.IsAlive)
            {
                Log(LogLevel.Warning, "Server thread already starting or running.");
                return;
            }

            _shutdownRequested = false;

            _thread = new Thread(Run)
            {
                Name = "QuadSim-RPC-Server",
                IsBackground = true
            };
            _thread.Start();
        }

        /// <summary>
        /// Signal the background thread to shut down and wait for it to finish.
        /// </summary>
        public void Stop()
        {
            // Always stop and join, including while the thread is still creating
            // or binding sockets. The old IsRunning guard leaked that startup case.
            _shutdownRequested = true;

            Thread thread = _thread;
            if (thread != null && thread.IsAlive && Thread.CurrentThread != thread)
            {
                if (!thread.Join(TimeSpan.FromSeconds(3)))
                {
                    // Keep the reference alive. The adapter must not run global
                    // NetMQ cleanup while this worker may still own sockets.
                    Log(LogLevel.Warning, "Server thread did not exit within 3s.");
                    return;
                }
            }

            if (_thread == thread && (thread == null || !thread.IsAlive))
                _thread = null;
        }

        /// <summary>
        /// Try to dequeue a pending request for main-thread processing.
        /// Returns true if a request was available.
        /// </summary>
        public bool TryDequeue(out QueuedRequest request)
        {
            return _pendingRequests.TryDequeue(out request);
        }

        /// <summary>
        /// Drain log messages from the background thread.
        /// Call from main thread (e.g., in Update).
        /// </summary>
        public bool TryDequeueLog(out LogLevel level, out string message)
        {
            if (_logQueue.TryDequeue(out var entry))
            {
                level = entry.Item1;
                message = entry.Item2;
                return true;
            }

            level = default;
            message = null;
            return false;
        }

        // ============================================================================
        // PUB Socket — Phase 9c
        // ============================================================================

        /// <summary>
        /// Enqueue a serialized MessagePack frame for publishing on the PUB socket.
        /// Called from the main thread (ExternalRpcAdapter.PublishTelemetry).
        /// The background thread drains this queue each poll cycle.
        /// </summary>
        public void EnqueuePublish(byte[] frame)
        {
            _publishQueue.Enqueue(frame);
        }

        // ============================================================================
        // Background Thread Entry Point
        // ============================================================================

        private void Run()
        {
            ResponseSocket repSocket = null;
            PublisherSocket pubSocket = null;

            try
            {
                // Select the managed AsyncIO backend before creating sockets.
                AsyncIO.ForceDotNet.Force();

                if (_shutdownRequested) return;
                repSocket = new ResponseSocket();

                if (_shutdownRequested) return;
                pubSocket = new PublisherSocket();

                if (_shutdownRequested) return;

                string repAddress = $"tcp://*:{_repPort}";
                string pubAddress = $"tcp://*:{_pubPort}";

                repSocket.Bind(repAddress);
                pubSocket.Bind(pubAddress);

                Log(LogLevel.Info, $"REP socket bound to {repAddress}");
                Log(LogLevel.Info, $"PUB socket bound to {pubAddress}");

                _isRunning = true;

                // Main poll loop
                while (!_shutdownRequested)
                {
                    PollIteration(repSocket, pubSocket);
                }
            }
            catch (Exception ex)
            {
                if (_shutdownRequested)
                    Log(LogLevel.Info, $"Server stopped during startup/shutdown: {ex.Message}");
                else
                    Log(LogLevel.Error, $"Server thread exception: {ex.Message}\n{ex.StackTrace}");
            }
            finally
            {
                _isRunning = false;

                try { repSocket?.Unbind($"tcp://*:{_repPort}"); } catch { }
                try { pubSocket?.Unbind($"tcp://*:{_pubPort}"); } catch { }

                repSocket?.Dispose();
                pubSocket?.Dispose();

                // NetMQConfig.Cleanup is process-global, not socket-local. The
                // adapter performs one blocking cleanup after this thread joins.
                Log(LogLevel.Info, "Server thread stopped. Sockets closed.");
            }
        }

        private void PollIteration(ResponseSocket repSocket, PublisherSocket pubSocket)
        {
            // --- 1. Handle REP requests ---

            if (repSocket.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(_pollTimeoutMs),
                    out byte[] requestBytes))
            {
                // Deserialize
                var parsed = RpcSerializer.DeserializeRequest(requestBytes);
                if (parsed == null)
                {
                    // Bad message — send error response immediately (must reply to maintain REQ/REP)
                    var errorResponse = RpcSerializer.SerializeResponse(
                        RpcResponse.Error("invalid_message"));
                    repSocket.SendFrame(errorResponse);
                }
                else
                {
                    // Enqueue for main thread processing
                    var queued = new QueuedRequest { Request = parsed.Value };
                    _pendingRequests.Enqueue(queued);

                    // Wait in short slices so shutdown can interrupt this wait and
                    // Stop() can reliably join the worker instead of waiting 5 seconds.
                    bool gotResponse = false;
                    const int responseTimeoutMs = 5000;
                    const int waitSliceMs = 10;
                    for (int waitedMs = 0;
                         waitedMs < responseTimeoutMs && !_shutdownRequested;
                         waitedMs += waitSliceMs)
                    {
                        if (queued.ResponseReady.Wait(waitSliceMs))
                        {
                            gotResponse = true;
                            break;
                        }
                    }

                    if (_shutdownRequested) return;

                    if (gotResponse && queued.ResponseBytes != null)
                    {
                        repSocket.SendFrame(queued.ResponseBytes);
                    }
                    else
                    {
                        // Timeout — main thread didn't respond in time
                        Log(LogLevel.Warning, $"Response timeout for method '{parsed.Value.Method}'");
                        var timeoutResponse = RpcSerializer.SerializeResponse(
                            RpcResponse.Error("server_timeout"));
                        repSocket.SendFrame(timeoutResponse);
                    }
                }
            }

            // --- 2. Drain PUB queue (Phase 9c) ---

            while (_publishQueue.TryDequeue(out byte[] pubFrame))
            {
                pubSocket.SendFrame(pubFrame);
            }
        }

        // ============================================================================
        // Helpers
        // ============================================================================

        private void Log(LogLevel level, string message)
        {
            _logQueue.Enqueue((level, $"[RpcServer] {message}"));
        }

        public void Dispose()
        {
            Stop();
        }
    }
}