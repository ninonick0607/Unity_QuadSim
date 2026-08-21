# Phase 3/4 Runtime Contract

## Prepared release pair

| Unity package | QuadSimLib | Protocol | Frame schema | Time schema |
|---|---|---:|---:|---:|
| 0.3.0 | 0.16.0 | 1.0 | 1 | 1 |

These versions are local release candidates. They have not been tagged or
published.

## Protocol and compatibility

- The SDK connect request declares its SDK and protocol versions, supported
  features, and frame/time schemas.
- Unity rejects an advertised incompatible protocol major before granting
  external authority.
- Unity returns its package version, supported features, schemas, ports, and
  bind address. The SDK exposes these values as `sim.protocol_info`.
- A legacy server remains connectable but is marked `negotiated = false`;
  `quadsim doctor` treats that as a failed platform check.

## RPC security boundary

- REP and PUB bind to `127.0.0.1` by default. Remote binding is explicit and
  emits a warning.
- Requests are limited to 1 MiB, 64 top-level fields, a non-empty method no
  longer than 64 characters, and 1,000 requests per second per server process.
- Remote mode is not an authenticated or encrypted deployment. See
  `rpc-security.md` before changing the bind address.

## Vehicle authoring

- New vehicles use YAML schema v1 and declare physics and controller rates.
- `create-drone`, `validate-drone`, and `inspect-drone` create a complete
  template and an evidence-bearing report without C# edits.
- `extends` provides deep-map overlays. Lists replace the base list. Resolve an
  overlay with `resolve-drone` before copying it into Unity because the Unity
  runtime intentionally consumes a self-contained YAML file.
- Reports include all source files and their composite hash, derived plant and
  allocation values, rate checks, the configured flight envelope, warnings,
  and calibration-data provenance.
- The Unity editor window at **Tools > QuadSim > Drone YAML Inspector** previews
  the center-of-mass-relative rotor layout and effectiveness matrix.

## Offline calibration

`calibrate-drone` accepts CSV inputs for thrust, yaw torque, hover command,
motor step response, principal inertia, and linear coast-down drag. A generated
configuration records the input path, SHA-256, and updated fields. These are
simple offline fits; using Sentinel or Flightmare measurements does not by
itself establish a broad simulator-to-real fidelity claim.
