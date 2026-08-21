from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import quadsim
from quadsim import QuadSim
from quadsim.replay import ZERO_DIGEST, chained_digest, step_result_payload


def connect(args):
    deadline = time.monotonic() + args.startup_timeout
    last_error = None
    while time.monotonic() < deadline:
        sim = QuadSim(
            host=args.host,
            command_port=args.command_port,
            telemetry_port=args.telemetry_port,
            timeout_ms=1000,
            client_name="Phase3BuiltPlayerSmoke",
        )
        try:
            sim.connect()
            return sim
        except Exception as exc:
            last_error = exc
            sim.disconnect()
            time.sleep(1.0)
    raise RuntimeError(f"player did not accept RPC within the timeout: {last_error}")


def source_file(path):
    if not path:
        return None
    source = Path(path).resolve()
    content = source.read_bytes()
    return {
        "path": str(source),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def first_difference(left, right, path="$", tolerance=1e-6):
    if (isinstance(left, (int, float)) and not isinstance(left, bool)
            and isinstance(right, (int, float)) and not isinstance(right, bool)):
        return None if abs(float(left) - float(right)) <= tolerance else path
    if isinstance(left, dict) and isinstance(right, dict):
        if set(left) != set(right):
            return path
        for key in sorted(left):
            difference = first_difference(
                left[key], right[key], f"{path}.{key}", tolerance)
            if difference is not None:
                return difference
        return None
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        if len(left) != len(right):
            return path
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            difference = first_difference(
                left_item, right_item, f"{path}[{index}]", tolerance)
            if difference is not None:
                return difference
        return None
    return None if left == right else path


def deterministic_sequence(sim, drones, agents, steps):
    reset = sim.reset_episode(agents, seed=12345, settle_steps=2)
    digest = ZERO_DIGEST
    last_step = None
    events_seen = 0
    records = []
    for offset in range(steps):
        target_step = reset.step_index + offset + 1
        actions = {drone.agent_id: (0.0, 0.0, 0.0, 0.0) for drone in drones}
        last_step = sim.step_multi(
            actions,
            count=1,
            target_step=target_step,
            strict_lockstep=True,
        )
        state, events = step_result_payload(last_step)
        state["episode_id"] = 0
        records.append({"state": state, "events": events})
        digest = chained_digest(
            digest,
            {
                "mode": "acceleration",
                "count": 1,
                "target_step": target_step,
                "actions": actions,
            },
            state,
            None,
            events,
        )
        events_seen += len(events)
    return reset, last_step, digest, events_seen, records


def run(args) -> dict:
    sim = connect(args)
    try:
        protocol = sim.protocol_info
        if protocol is None or not protocol.negotiated:
            raise RuntimeError("player did not negotiate protocol capabilities")
        if protocol.bind_address not in ("127.0.0.1", "localhost", "::1"):
            raise RuntimeError(f"player unexpectedly exposed RPC on {protocol.bind_address}")

        initial = sim.get_status()
        drones = sim.list_drones()
        if not drones or not all(drone.is_initialized for drone in drones):
            raise RuntimeError("player did not register an initialized drone")

        sim.pause()
        agents = [
            {
                "agent_id": drone.agent_id,
                "team_id": drone.team_id or "default",
                "drone_index": drone.drone_index,
                "position_enu": (0.0, index * 2.0, 1.0),
                "yaw_deg": 0.0,
            }
            for index, drone in enumerate(drones)
        ]
        reset, step, first_digest, first_events, first_records = deterministic_sequence(
            sim, drones, agents, args.replay_steps)
        repeated_reset, repeated_step, second_digest, second_events, second_records = \
            deterministic_sequence(sim, drones, agents, args.replay_steps)
        difference = first_difference(first_records, second_records)
        if difference is not None:
            raise RuntimeError(
                f"seeded replay exceeded 1e-6 at {difference}")
        sensors = sim.drone().get_sensors()
        final = sim.get_status()

        expected_step = reset.step_index + args.replay_steps
        if step.step_index != expected_step or repeated_step.step_index != expected_step:
            raise RuntimeError(
                f"strict sequence ended at {step.step_index}, expected {expected_step}")
        if (sensors.episode_id != repeated_step.episode_id
                or sensors.step_index < repeated_step.step_index):
            raise RuntimeError("sensor snapshot predates the completed strict step")

        return {
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "sdk_version": quadsim.__version__,
            "unity_revision": args.unity_revision,
            "sdk_revision": args.sdk_revision,
            "scene_config": source_file(args.scene_config),
            "vehicle_config": source_file(args.vehicle_config),
            "protocol": asdict(protocol),
            "initial_status": asdict(initial),
            "drones": [asdict(drone) for drone in drones],
            "reset": {
                "episode_id": repeated_reset.episode_id,
                "step_index": repeated_reset.step_index,
                "seed": repeated_reset.seed,
                "settle_steps": repeated_reset.settle_steps,
            },
            "strict_step": {
                "episode_id": repeated_step.episode_id,
                "step_index": repeated_step.step_index,
                "sim_time_ns": repeated_step.sim_time_ns,
                "sensor_step_index": sensors.step_index,
            },
            "deterministic_replay": {
                "steps_per_run": args.replay_steps,
                "runs": 2,
                "matches_within_absolute_tolerance": True,
                "absolute_tolerance": 1e-6,
                "digests": [first_digest, second_digest],
                "events_per_run": [first_events, second_events],
            },
            "final_status": asdict(final),
        }
    finally:
        sim.disconnect()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=5555)
    parser.add_argument("--telemetry-port", type=int, default=5556)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--replay-steps", type=int, default=8)
    parser.add_argument("--unity-revision", default=os.environ.get("GITHUB_SHA", "unknown"))
    parser.add_argument("--sdk-revision", default="unknown")
    parser.add_argument("--scene-config")
    parser.add_argument("--vehicle-config")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    manifest = run(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
