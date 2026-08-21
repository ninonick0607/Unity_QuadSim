# Testing Phases 3 and 4

## 1. Confirm the baseline

```bash
git -C /path/to/Unity_QuadSim branch --show-current
git -C /path/to/Unity_QuadSim log -5 --oneline
git -C /path/to/Unity_QuadSim status --short
git -C /path/to/QuadSimLib branch --show-current
git -C /path/to/QuadSimLib log -5 --oneline
git -C /path/to/QuadSimLib status --short
```

Do not clean or reset either worktree. The Phase 1/2 modifications are part of
the baseline for these tests.

## 2. Run the SDK gate

From QuadSimLib:

```bash
python -m pip install -e ".[test]"
python -m pytest -q \
  Testing/test_lockstep.py \
  Testing/test_multi_agent.py \
  Testing/test_replay.py \
  Testing/test_rl_foundation_verifier.py \
  Testing/test_drone_config.py \
  Testing/test_protocol.py
```

Expected result for this checkpoint: `51 passed`.

## 3. Exercise vehicle authoring

```bash
quadsim create-drone my_quad.yaml --name MyQuad
quadsim validate-drone my_quad.yaml
quadsim inspect-drone my_quad.yaml --output my_quad.report.json
quadsim doctor --no-connect --config my_quad.yaml
```

For a tuning overlay:

```bash
quadsim create-drone my_overlay.yaml --name SentinelTuned \
  --extends /path/to/sentinel_quad.yaml
quadsim resolve-drone my_overlay.yaml sentinel_tuned.resolved.yaml
```

Edit only the fields being overridden in `my_overlay.yaml`. Copy the resolved
file into the Unity project's `StreamingAssets/Configs/Drones` folder.

## 4. Exercise calibration

```bash
quadsim calibrate-drone my_quad.yaml thrust thrust.csv --output thrust.yaml
quadsim calibrate-drone my_quad.yaml yaw yaw.csv --output yaw.yaml
quadsim calibrate-drone my_quad.yaml hover hover.csv --output hover.yaml
quadsim calibrate-drone my_quad.yaml motor-step motor.csv --output motor.yaml
quadsim calibrate-drone my_quad.yaml inertia inertia.csv --output inertia.yaml
quadsim calibrate-drone my_quad.yaml drag drag.csv --output drag.yaml
```

Required CSV columns are respectively:

- thrust: `command_01,thrust_n`
- yaw: `command_01,yaw_torque_nm`
- hover: `hover_command_01`
- motor-step: `time_s,target_01,response_01`
- inertia: `axis,torque_nm,angular_accel_rad_s2`
- drag: `time_s,velocity_m_s`

Every saved result records the CSV path, SHA-256, and updated fields.

## 5. Build the local PyPI candidate

```bash
python -m build
python -m twine check \
  dist/quadsim_sdk-0.16.0.tar.gz \
  dist/quadsim_sdk-0.16.0-py3-none-any.whl
```

Install the wheel in a fresh virtual environment and run `quadsim doctor
--no-connect`. Do not run `twine upload` during local preparation.

## 6. Test in Unity

Open `/Users/nninofernandez/QuadSim` from Unity Hub. Its package manifest points
directly at the local `Unity_QuadSim` repository, so saved package changes are
picked up without copying files.

After Unity finishes importing:

1. Open **Window > General > Console**, clear the old entries, and use
   **Assets > Refresh** if Unity has not recompiled automatically.
2. Confirm there are no red compiler entries. Do not evaluate runtime behavior
   while script compilation is failing.
3. Run **Window > General > Test Runner** and run all EditMode tests.
4. Switch the Test Runner to PlayMode and run all PlayMode tests.
5. Open **Tools > QuadSim > Drone YAML Inspector**, select each packaged YAML,
   and verify rank 4, finite derived values, and the intended rotor layout.
6. Put the resolved custom YAML in `StreamingAssets/Configs/Drones`, select its
   basename in `Default_SceneConfig`, and start the scene.

The current local license works through Unity Hub but does not expose the
`com.unity.editor.headless` entitlement. Use the GUI Test Runner for now;
command-line `-batchmode -runTests` remains a CI/headless-license gate.

With the scene running:

```bash
quadsim doctor --config /path/to/resolved.yaml
python /path/to/Unity_QuadSim/ci~/built_player_smoke.py \
  --output /tmp/quadsim-smoke-manifest.json
```

Then command a low-altitude takeoff, hover, and landing. Confirm that no C# file
was edited, the player log has no rejected gate patterns, and the smoke manifest
records loopback binding, protocol 1.0, strict step, and post-step sensors.
The CI invocation also records both repository revisions, scene/vehicle hashes,
and two matching seeded digest runs.

## 7. GitHub gates

Configure Unity license secrets, push both repositories, and run the Phase 3
workflow. Require package-contract, all Python versions, Unity EditMode,
Unity PlayMode, Linux/Windows player builds, built-player smoke, and security
jobs before creating release tags.
