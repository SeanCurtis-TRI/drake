#!/usr/bin/env python3

"""Runs numbered conveyor belt simulation demos."""

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import sys


BINARY = "./bazel-bin/examples/conveyor_belt/conveyor_belt_simulation"


@dataclass(frozen=True)
class Demo:
    description: str
    notes: list[str]
    params: list[str]


DEMOS = [
    Demo(
        description="Initial, coarse belt",
        notes=(
            "This conveyor belt has some obvious behavior problems. The most "
            "obvious cause is the width of the belt's links. With only 20 "
            "links, we can't effectively wrap around the drums. This "
            "introduces all sorts of inconvenient forces, even without friction."
        ),
        params=[
            "--conveyor_belts=physical",
            "--drum_spacing=1.0",
            "--num_links=20",
            "--include_end_buttons=false",
            "--include_center_support=false",
            "--target_belt_speed=0.1",
        ],
    ),
    Demo(
        description="Finer belt",
        notes=(
            "Here we double the number of links from 20 to 40. We can already "
            "see that the belt slides over the drum more efficiently."
        ),
        params=["--num_links=40"],
    ),
    Demo(
        description="Even finer belt",
        notes=(
            "Doubled again to 80 links, we're starting to approximate a "
            "continuous belt.\n\nHowever, there is a limit to how far you can "
            "take this."
        ),
        params=["--num_links=80"],
    ),
    Demo(
        description="Even finer belt",
        notes=(
            "Doubled again to 160 links and the performance starts noticably "
            "degrading.\n\n"
            "So, we want the minimum number of links that give us good "
            "behavior. However, the width of the link is determined by the "
            "drum radius. As soon as we start creating longer belts, we're "
            "back to the performance problems."
        ),
        params=["--num_links=160"],
    ),
    Demo(
        description="Move to a 4-m belt",
        notes=(
            "This belt is four times as long. Clearly, the 40-links are no "
            "longer sufficient. In this case, the gains we have on the "
            "controller aren't even sufficient to drive the belt.\n\n"
            "In fact, even cranking up the gains on the controller and the "
            "maximum drive force is insufficient."
        ),
        params=[
            "--num_links=40",
            "--drum_spacing=4.0",
        ],
    ),
    Demo(
        description="80 links for a 4-m belt",
        notes=(
            "By increasing the number of links to 80, the controller can now "
            "push the belt again. However, we've taken a performance hit.\n\n"
            "We're not going to delve much into the stability problems (the "
            "belt flying off). Ultimately, we could handle that by putting "
            "some frictionless rims on the drums.\n\n"
            "It is worth noting, that *these* 80 links are more performant "
            "than 80 links on the 2-m belt. I don't know why, but I suspect "
            "it is related to geometry culling."
        ),
        params=[
            "--num_links=80",
        ],
    ),
    ## Reset back to 40-links on 1-m belt.
    Demo(
        description="Back to 40-link, 1-m belt",
        notes=(
            "Let's return to our toy conveyor belt. What happens if we play "
            "with the mass of the box?"
        ),
        params=[
            "--drum_spacing=1.0",
            "--num_links=60",
        ],
    ),
    Demo(
        description="Much more massive box",
        notes=(
            "This box is 40X heavier. It warps the belt so much that the motor "
            "can't drive it over the drum."
        ),
        params=[
            "--box_mass=4.0",
        ],
    ),
    Demo(
        description="Crank the motor",
        notes=(
            "If we crank the gains, the controller can push the box. "
            "Eventually."
        ),
        params=[
            "--belt_speed_kp=200",
            "--belt_speed_ki=200",
        ],
    ),
    ## Reset back to 40-links on 1-m belt.
    Demo(
        description="Back to 40-link, 1-m belt",
        notes=(
            "Let's return to our toy conveyor belt. This runs well, but this "
            "runs well because we're in point contact."
        ),
        params=[
            "--drum_spacing=1.0",
            "--num_links=60",
            "--box_mass=0.1",
            "--belt_speed_kp=50",
            "--belt_speed_ki=50",
        ],
    ),
    Demo(
        description="Swap to hydro",
        notes=(
            "Changing to hydroelastic contact *kills* us."
        ),
        params=[
            "--contact_model=hydro",
        ],
    ),
    # Swapping to virtual conveyor belt.
    Demo(
        description="Virtual conveyor belt",
        notes=(
            "With the virtual conveyor belt, we get arbitrarily smooth "
            "behavior because the belt is defined to follow the geometry. We "
            "don't have to worry about belt resolution.\n\n"
            "Because we're using *point* contact on this belt, we're using "
            "the belt's convex hull. You can see the box moving above the "
            "visible surface."
        ),
        params=[
            "--contact_model=point",
            "--conveyor_belts=virtual",
        ],
    ),
    Demo(
        description="Virtual belt with hydro contact",
        notes=(
            "The virtual conveyor belt scales much better with hydroelastic "
            "contact. The single surface reduces the amount of work we need "
            "to do."
        ),
        params=[
            "--contact_model=hydro",
        ],
    ),
    Demo(
        description="Virtual belt with hydro contact and massive box",
        notes=(
            "This box is 100X heavier. We don't have to specify a higher gain "
            "because the virtual belt has infinite torque and its speed "
            "is always exactly as specified."
        ),
        params=[
            "--box_mass=10.0",
        ],
    ),
    Demo(
        description="Final demo",
        notes=(
            "Side by side comparison."
        ),
        params=[
            "--box_mass=0.1",
            "--contact_model=point",
            "--drum_spacing=2.0",
            "--num_links=60",
            "--conveyor_belts=both",
            "--include_end_buttons=true",
            "--target_belt_speed=0.5",
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
        "  ./demo.py --start_at <number> [extra simulation flags...]",
        "  ./demo.py list",
        "",
        f"Runs {BINARY} with a numbered group of command-line parameters.",
        "Each demo overlays its parameters onto the demos before it.",
        "Extra flags are appended after the selected group.",
        "",
        "Available demos:",
    ]
    for i, demo in enumerate(DEMOS, start=1):
        lines.append(f"  {i}  {demo.description}")
    return "\n".join(lines)


def flag_key(param: str) -> str:
    """Returns the gflags key used to overlay command-line parameters."""
    if not param.startswith("--"):
        return param
    flag = param[2:]
    name, has_value, _ = flag.partition("=")
    if not has_value and name.startswith("no"):
        return name[2:]
    return name


def resolve_params(demo_index: int) -> list[str]:
    """Overlays demo params through demo_index, with later flags winning."""
    params_by_key = {}
    for demo in DEMOS[: demo_index + 1]:
        for param in demo.params:
            key = flag_key(param)
            params_by_key.pop(key, None)
            params_by_key[key] = param
    return list(params_by_key.values())


def parse_demo_index(demo_id: str) -> int:
    try:
        demo_index = int(demo_id) - 1
        if demo_index < 0 or demo_index >= len(DEMOS):
            raise IndexError
        return demo_index
    except (ValueError, IndexError) as e:
        raise ValueError(f"Unknown demo '{demo_id}'.") from e


def parse_start_at(argv: list[str]) -> tuple[int, list[str]]:
    start_arg = argv[1]
    if start_arg in {"--start_at", "--start-at"}:
        if len(argv) < 3:
            raise ValueError(f"Missing value for {start_arg}.")
        demo_id = argv[2]
        extra_args = argv[3:]
    else:
        name, has_value, demo_id = start_arg.partition("=")
        if name not in {"--start_at", "--start-at"} or not has_value:
            raise ValueError(f"Unknown demo '{start_arg}'.")
        extra_args = argv[2:]
    return parse_demo_index(demo_id), extra_args


def command_for_demo(demo_index: int, extra_args: list[str]) -> list[str]:
    return [BINARY, *resolve_params(demo_index), *extra_args]


def run_command(command: list[str]) -> int:
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                return process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                return process.wait()
        return process.returncode or 130


def wait_for_next_demo(next_demo_index: int | None) -> str:
    key_cmds = (
        "R replays; C or N for next; P for previous; number jumps; "
        "anything else exits"
    )
    if next_demo_index is None:
        prompt = f"Press Enter to exit; {key_cmds}: "
    else:
        prompt = (
            f"Press Enter for demo {next_demo_index + 1}; {key_cmds}: "
        )
    while True:
        try:
            response = input(prompt).strip()
            response_lower = response.lower()
            if response == "" or response_lower == "c" or response_lower == "n":
                return "next"
            if response_lower == "r":
                return "replay"
            if response_lower == "p":
                return "previous"
            try:
                demo_index = parse_demo_index(response)
                return f"jump:{demo_index}"
            except ValueError:
                if response.lstrip("-").isdigit():
                    print(f"Choose a demo from 1 to {len(DEMOS)}.")
                    continue
            return "exit"
        except KeyboardInterrupt:
            print()
        except EOFError:
            print()
            return "exit"


def run_demo(demo_index: int, extra_args: list[str]) -> int:
    demo = DEMOS[demo_index]
    command = command_for_demo(demo_index, extra_args)
    print()
    print(f"Demo {demo_index + 1}: {demo.description}")
    if demo.notes:
        print()
        print(demo.notes)
        print()
    print(f"Running demo {demo_index + 1}: {' '.join(command)}", flush=True)
    return run_command(command)


def run_sequence(start_index: int, extra_args: list[str]) -> int:
    returncode = 0
    demo_index = start_index
    while demo_index < len(DEMOS):
        returncode = run_demo(demo_index, extra_args)
        next_demo_index = demo_index + 1
        if next_demo_index >= len(DEMOS):
            next_demo_index = None
        action = wait_for_next_demo(next_demo_index)
        if action == "next":
            if next_demo_index is None:
                return returncode
            demo_index = next_demo_index
        elif action == "replay":
            continue
        elif action == "previous":
            if demo_index > 0:
                demo_index -= 1
            else:
                print("Already at the first demo.")
        elif action.startswith("jump:"):
            demo_index = int(action.removeprefix("jump:"))
        else:
            return returncode
    return returncode


def main() -> int:
    if len(sys.argv) < 2:
        print(usage(), file=sys.stderr)
        return 2

    demo_id = sys.argv[1]
    extra_args = sys.argv[2:]
    if demo_id in {"list", "--list", "-l", "help", "--help", "-h"}:
        print(usage())
        return 0

    start_at = demo_id.startswith("--start_at") or demo_id.startswith(
        "--start-at"
    )
    try:
        if start_at:
            demo_index, extra_args = parse_start_at(sys.argv)
        else:
            demo_index = parse_demo_index(demo_id)
    except ValueError as e:
        print(e, file=sys.stderr)
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

    os.chdir(repo_root)
    if start_at:
        return run_sequence(demo_index, extra_args)
    return run_demo(demo_index, extra_args)


if __name__ == "__main__":
    sys.exit(main())
