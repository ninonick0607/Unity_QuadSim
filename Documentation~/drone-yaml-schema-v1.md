# Drone YAML Schema v1

Schema v1 describes one conventional fixed-rotor quad. New files must set
`schema_version: 1`. Unity continues to accept legacy unversioned files with a
warning, but rejects unknown explicit schema versions.

## Units and conventions

| Field family | Unit or convention |
|---|---|
| `mass_kg` | kilograms |
| positions, offsets, arm length, propeller diameter | metres, Unity body axes: +x forward, +y up, +z left |
| inertia x/y/z | principal diagonal in kg m², expressed in Unity body axes |
| drag coefficients | Unity Rigidbody linear/angular damping coefficients |
| `max_rpm` | revolutions per minute |
| motor time constants | seconds |
| velocity limits | metres per second |
| angle and angular-rate limits | degrees and degrees per second |
| thrust | newtons |
| torque | newton-metres |
| rotor `dir` | reactive body-yaw sign: `-1` or `1` |
| `physics_rate_hz`, `controller_rate_hz` | hertz |

Static per-rotor thrust is `Ct * rho * n² * D⁴` at `rho = 1.225 kg/m³`.
Yaw torque follows the explicitly selected `legacy_2_5_pi` or
`power_coefficient_2_pi` convention. `limits.max_thrust_newtons` remains the
controller's total-thrust limit; the plant derives its per-rotor maximum from
the propulsion coefficients.

## Required v1 structure

- `schema_version`, `name`, and optional `body_mesh`.
- `physical_properties`: mass, center-of-mass offset, diagonal inertia, arm
  length, linear damping, and angular damping.
- `propulsion`: diameter, maximum RPM, thrust/torque coefficients, torque
  convention, optional asymmetric motor time constants, and exactly four
  propellers.
- `runtime`: positive physics and controller rates. The controller rate must
  not exceed the physics rate; an integer physics/controller ratio is preferred.
- `controller`: position, velocity, attitude, and rate PID groups.
- `camera_parameters`.
- `limits`, including positive roll/pitch and yaw rate limits.

Rotor IDs are canonical `0, 1, 2, 3` for front-left, front-right, back-left,
and back-right. Locations must be unique and a conventional quad must have two
rotors in each spin direction.

Use `quadsim create-drone` for a unit-annotated template. `validate-drone`
checks schema, physical/propulsion values, inertia, rotor identity and layout,
spin balance, effectiveness rank and conditioning, hover feasibility, and
axis authority. It also warns when a configured motor time constant is sampled
by fewer than two physics steps. `inspect-drone` emits the same validation plus
a JSON vehicle report, supported envelope, calibration provenance, and source
SHA-256.

## Overlays

An authoring-time overlay may set `extends` to an absolute path or a path
relative to the overlay. Mappings merge recursively; scalar values and lists
replace the base value. Cycles are rejected. Use `quadsim resolve-drone` to
write a self-contained YAML before loading it in Unity.

## Calibration provenance

`quadsim calibrate-drone` can save a resolved configuration containing a
`calibration.sources` list. Each entry records the calibration kind, source CSV
path, SHA-256, and fitted fields. Unity ignores this metadata while the vehicle
report preserves it as experiment provenance.
