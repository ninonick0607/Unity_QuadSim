from __future__ import annotations

import argparse
from pathlib import Path


REJECTED = (
    "NullReferenceException",
    "InvalidOperationException",
    "error CS",
    "Scripts have compiler errors",
    "Compilation failed",
    "[RpcProtocol] Failed",
    "invalid collider",
    "dropped_step",
    "request_too_large",
    "rate_limited",
    "incompatible_protocol",
    "server_timeout",
)


def check(paths):
    failures = []
    for supplied in paths:
        path = Path(supplied)
        if not path.exists():
            failures.append(f"{path}: log path does not exist")
            continue
        files = sorted(path.rglob("*.log")) if path.is_dir() else [path]
        if not files:
            failures.append(f"{path}: no Unity log files found")
            continue
        for log in files:
            text = log.read_text(encoding="utf-8", errors="replace")
            for pattern in REJECTED:
                if pattern.lower() in text.lower():
                    failures.append(f"{log}: unexpected {pattern}")
    return failures


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args(argv)
    failures = check(args.paths)
    for failure in failures:
        print(failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
