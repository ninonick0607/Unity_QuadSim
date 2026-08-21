# Phase 3/4 Validation Checkpoint

Status: the Python/package slice and Unity compiler gate pass. Unity test
execution is pending because the local license does not include the headless
editor entitlement.

## Passed locally

- QuadSimLib Phase 1/2 plus Phase 3/4 focused suite: 51/51.
- Python compile, CLI help, package/YAML mirror, JSON, and `git diff --check`
  gates.
- Creation, validation, inspection, overlay resolution, cycle detection,
  runtime-rate checks, and calibration provenance.
- Protocol capability parsing, legacy detection, and major mismatch rejection.
- Local 0.16.0 source and wheel builds; both pass `twine check`.
- A clean target install from the wheel reports 0.16.0 and passes
  `create-drone` plus `doctor --no-connect`.
- All four packaged Unity profiles and sample mirrors validate under schema v1.
- Unity 6000.5.8f1's generated Roslyn inputs compile `Nino.QuadSim`, `UImGui`,
  `Nino.QuadSim.Editor`, `Nino.QuadSim.Tests`, and
  `Nino.QuadSim.PlayModeTests` with zero diagnostics.

The current wheel and source distribution are local only under QuadSimLib's
`dist/`; no PyPI upload or Git tag was performed.

- wheel SHA-256: `70e1400316c9363829ad70ab387611c12cf681c570cdfc71277c18f5fe7b3fb9`
- source SHA-256: `23d68620a91af88fd405b31b4ba102bcba20ff22f296f05a70bd995e52464f8c`

## Prepared but not yet proven by execution

- Unity EditMode tests now include analytic Phase 1/2 coverage plus protocol,
  schema, and malformed-message checks.
- Unity PlayMode tests cover manual pre-physics/post-physics ordering, integer
  step time, reset, and the loopback default.
- CI builds Linux and Windows players, runs EditMode and PlayMode tests, gates
  Unity logs, scans secrets/dependencies, and runs a Linux built-player RPC
  smoke that connects, negotiates, acquires authority, resets, strict-steps,
  repeats a short seeded digest sequence, reads post-step sensors, disconnects,
  and writes a provenance manifest.

The Hub-launched GUI editor can authenticate. A command-line EditMode run
starts Unity and Package Manager, but the licensing client reports that
`com.unity.editor.headless` is unavailable. Run the prepared tests from the
GUI Test Runner or add a headless-capable local/CI license. The CI jobs also
require Unity license secrets before they can become merge gates.

## Known non-blocking baseline warnings

- Crazyflie's raw physical-units effectiveness condition number is about
  4865.6.
- Standard quad's declared `propulsion.max_thrust` differs from the
  coefficient-derived runtime value by about 45.3%. The runtime plant continues
  using the coefficient-derived value.

## Remaining Phase 3 work

- Execute the new EditMode and PlayMode suites from the Hub-launched GUI editor.
- Add a headless-capable Unity license for batchmode and the local built-player
  gates.
- Enable the workflows as required GitHub checks and supply CI license secrets.
- Run the Python matrix on GitHub after the QuadSimLib changes are pushed.
- Approve, tag, and publish the 0.3.0/0.16.0 pair only after those gates pass.
- Add the optional scheduled PX4 mission when CI capacity and a pinned PX4
  baseline are available.

## Remaining Phase 4 work

- Prove a newly generated and resolved YAML can be selected, spawned, and flown
  in the built player without C# edits.
- Run the calibration commands against chosen Sentinel and/or Flightmare source
  CSVs and review the resulting coefficients and supported envelope.
- Expand calibration fitting only when the available measurement format or a
  specific research decision requires it.
