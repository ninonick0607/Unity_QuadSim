# RPC Security Boundary

## Supported configuration

QuadSim 0.3.0 supports RPC on the same host. `rpcBindAddress` defaults to
`127.0.0.1`; the SDK defaults to `localhost`. Do not forward ports 5555 or 5556
to another network or set `0.0.0.0` for ordinary use.

Loopback prevents another machine from reaching the sockets, while protocol
negotiation and request limits reduce accidental misuse by local processes. It
does not protect against an untrusted process already running as the user.

## Project AirSim reference

The local Project AirSim source was reviewed for the future remote case. Its
clients default to `127.0.0.1`, while the sim server can listen broadly and can
be configured with `client_auth_public_key`. Its authorization layer issues an
expiring signed/encrypted token and limits unauthenticated calls to the setup
and unsubscribe path. The relevant reference implementations are
`simserver/src/simserver.cpp` and `core_sim/src/client_authorization.cpp/.hpp`.

That is a useful architecture reference, but copying it now would add key,
token-lifetime, revocation, clock, and deployment obligations without a defined
remote user. QuadSim therefore stays loopback-only for the supported path.

## Threats and controls

| Threat | Current control | Residual risk |
|---|---|---|
| Accidental network exposure | Loopback default and explicit override warning | User can still deliberately bind broadly |
| Oversized or malformed input | 1 MiB limit, MessagePack failure handling, shape limits | Method-specific nested limits remain limited |
| Request flooding | 1,000 request/s process limit | A hostile local process can still consume resources |
| Incompatible client | Major-version handshake and feature/schema advertisement | Legacy clients are visible but not evidence-grade |
| Eavesdropping or command injection across a network | Remote use is unsupported | No authentication or encryption exists |

## Remote-use decision gate

Before supporting remote RPC, define the network boundary, users, credential
provisioning, process ownership, latency envelope, revocation behavior, and
whether an authenticated tunnel already solves the deployment. Only then choose
between a Project AirSim-style token handshake, mutual TLS, or a tunnel-only
policy.
