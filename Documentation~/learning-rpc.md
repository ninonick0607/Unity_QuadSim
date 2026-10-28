# Study track: RPC and ZeroMQ in QuadSim

Goal: understand QuadSim's Python ↔ Unity link well enough to explain it,
debug it, and extend it without AI. Do each stage yourself. Write the code by
hand, then ask for a review. Each stage ends with questions; answer them
out loud or in writing before moving on.

## Stage 0: Map it onto ROS (reading only)

| ROS concept | ZeroMQ / QuadSim equivalent |
|---|---|
| Service (request → response) | REQ/REP sockets on port 5555: commands, queries, stepping |
| Topic | PUB/SUB sockets on port 5556: telemetry stream |
| `.msg` / `.srv` definitions | None. Messages are MessagePack maps; the "schema" is the code in `RpcProtocol.cs` and `quadsim/protocol.py` |
| roscore / DDS discovery | None. Endpoints are explicit (`tcp://127.0.0.1:5555`) |
| Executor and callbacks | A background thread owns the sockets; handlers run on Unity's main thread |

The two big differences from ROS: ZeroMQ has **no discovery and no schema**.
Both sides must agree on addresses and field names by convention, and tests
are what enforce that agreement.

Questions: What happens in ROS if a service server dies mid-call? What would
you expect to happen in ZeroMQ?

## Stage 1: Build a REQ/REP echo (about 20 lines, pyzmq)

1. Write a REP server that replies with the uppercase of whatever it receives,
   and a REQ client that sends three messages.
2. Break it on purpose: call `send` twice on the REQ socket without a `recv`.
   Read the error.
3. Make the server never reply. Set `zmq.RCVTIMEO` on the client. After the
   timeout, try to send again. What happens?

Then read `QuadSimLib/quadsim/_transport.py` `_send_unlocked`. Step 3 is
backlog bug **B1**, a real bug in the SDK. Read "Lazy Pirate" in the ZeroMQ
Guide (chapter 4, Reliable Request-Reply) and fix B1 yourself, with a unit test.

Questions: Why does REQ enforce strict send/recv alternation? What does
`LINGER = 0` change when a socket closes?

## Stage 2: PUB/SUB

1. Write a publisher sending a counter at 100 Hz and a subscriber that prints
   it. Start the subscriber *after* the publisher. Which messages are missing?
   (This is the "slow joiner" problem.)
2. Subscribe with a topic prefix filter, then try `zmq.CONFLATE` on the
   subscriber.

Then read how the telemetry stream works: `ExternalRpcAdapter.PublishTick` and
the subscription code in `_transport.py`.

Questions: Why is telemetry on PUB/SUB, while stepping must use REQ/REP? Is
any telemetry message guaranteed to arrive?

## Stage 3: MessagePack and the envelope

1. `msgpack.packb({"method": "get_status", "request_id": 1})`. Print the bytes
   and decode them by hand against the MessagePack spec (a fixmap, then
   fixstr keys).
2. Read the request parsing in `RpcProtocol.cs` (it requires `method`, takes
   `request_id`, and treats the remaining keys as parameters) and compare it
   with `request = {"method": ..., "request_id": ..., **params}` in
   `_transport.py`.

Questions: What stops a Python typo in a parameter name from being silently
ignored by Unity? What enforces the size, field-count, and rate limits
(`rpc-security.md`)?

## Stage 4: Threads and Unity's main thread

Read `RpcServerThread.cs` (network thread, `ConcurrentQueue`s) and
`ExternalRpcAdapter.Update` → `ProcessRequests`. Draw a sequence diagram for
one `step_with_command` call, from Python `send` to the reply bytes, naming the
thread for each box.

Questions: Why can't the handler run on the network thread? Where is the
reply actually sent? Why does each lockstep round trip cost at least one
Unity frame (backlog A3)?

## Stage 5: Add a real RPC method end to end

Add `get_vehicle_params`. It should return the applied mass, inertia,
per-motor max thrust, hover throttle, and rotor layout (FLU) from the active
drone. This is useful right away: the geometric controller currently keeps a
separate Python vehicle profile and warns that selecting it does not switch
the Unity model. You will touch the handler table, a handler, frame conversion
through `Frames.cs`, an SDK method plus types, and a test on each side.

## Stage 6: Measure it

Write the throughput benchmark (backlog **P2**): lockstep ticks per second for
1, 4, and 16 drones, headless, recording hardware and versions. This is paper
evidence and closes the loop on stage 4.
