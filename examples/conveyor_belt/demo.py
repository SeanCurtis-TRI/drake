#!/usr/bin/env python3

"""Runs numbered conveyor belt simulation demos."""

from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
import sys


BINARY = "./bazel-bin/examples/conveyor_belt/conveyor_belt_simulation"


@dataclass(frozen=True)
class Demo:
    description: str
    params: list[str]


DEMOS = [
    Demo(
        description="Both belts, point contact",
        params=[
            "--conveyor_belts=both",
            "--contact_model=point",
            "--target_belt_speed=0.5",
            "--target_realtime_rate=1.0",
        ],
    ),
    Demo(
        description="Physical belt only",
        params=[
            "--conveyor_belts=physical",
            "--contact_model=point",
            "--target_belt_speed=0.5",
            "--target_realtime_rate=1.0",
        ],
    ),
    Demo(
        description="Virtual surface-velocity belt only",
        params=[
            "--conveyor_belts=virtual",
            "--contact_model=point",
            "--target_belt_speed=0.5",
            "--target_realtime_rate=1.0",
        ],
    ),
    Demo(
        description="Both belts, hydroelastic contact",
        params=[
            "--conveyor_belts=both",
            "--contact_model=hydro",
            "--target_belt_speed=0.5",
            "--target_realtime_rate=1.0",
        ],
    ),
]


def find_repo_root() -> Path:
    path = Path(__file__).resolve().parent
    for candidate in [path, *path.parents]:
        if (candidate / ".git").exists():
            return candidate
    return path.parents[1]


def usage() -> str:
    lines = [
        "Usage:",
        "  ./demo.py <number> [extra simulation flags...]",
        "  ./demo.py list",
        "",
        f"Runs {BINARY} with a numbered group of command-line parameters.",
        "Extra flags are appended after the selected group.",
        "",
        "Available demos:",
    ]
    for i, demo in enumerate(DEMOS, start=1):
        lines.append(f"  {i}  {demo.description}")
    return "\n".join(lines)


def main() -> int:
    if len(sys.argv) < 2:
        print(usage(), file=sys.stderr)
        return 2

    demo_id = sys.argv[1]
    extra_args = sys.argv[2:]
    if demo_id in {"list", "--list", "-l", "help", "--help", "-h"}:
        print(usage())
        return 0

    try:
        demo_index = int(demo_id) - 1
        demo = DEMOS[demo_index]
    except (ValueError, IndexError):
        print(f"Unknown demo '{demo_id}'.", file=sys.stderr)
        print(usage(), file=sys.stderr)
        return 2

    repo_root = find_repo_root()
    binary = repo_root / BINARY
    if not binary.exists():
        print(
            f"Missing {BINARY}. Build //examples/conveyor_belt:"
            "conveyor_belt_simulation first.",
            file=sys.stderr,
        )
        return 1

    params = [*demo.params, *extra_args]
    command = [BINARY, *params]
    print(f"Running demo {demo_id}: {' '.join(command)}")
    os.chdir(repo_root)
    return subprocess.call(command)


if __name__ == "__main__":
    sys.exit(main())
