#!/usr/bin/env python3
"""Run a command and its descendants on a shared, bounded Linux CPU set."""

import argparse
import os
from pathlib import Path
import re
import sys


def parse_cpuset(value, allowed):
    """Parse a Linux-style CPU list without allowing CPUs outside our mask."""
    cpus = set()
    for part in value.split(","):
        match = re.fullmatch(r"([0-9]+)(?:-([0-9]+))?", part.strip())
        if not match:
            raise ValueError("NAV_CPUSET must be a CPU list such as 2,4,6 or 2-4")
        first = int(match.group(1))
        last = int(match.group(2)) if match.group(2) else first
        if last < first:
            raise ValueError("NAV_CPUSET contains a reversed CPU range")
        # Bound range expansion even when the supplied endpoint is very large.
        if last - first + 1 > len(allowed):
            raise ValueError("NAV_CPUSET includes CPUs outside the currently allowed mask")
        cpus.update(range(first, last + 1))
    if not cpus <= set(allowed):
        raise ValueError("NAV_CPUSET includes CPUs outside the currently allowed mask")
    return sorted(cpus)


def choose_cpus(allowed, count, topology_root=Path("/sys/devices/system/cpu")):
    """Prefer distinct physical cores, leaving the lowest CPU's core until last."""
    allowed = sorted(set(allowed))
    if count < 1 or count > len(allowed):
        raise ValueError(
            f"NAV_CPU_COUNT must be between 1 and {len(allowed)} currently allowed CPUs"
        )
    cores = {}
    try:
        for cpu in allowed:
            topology = topology_root / f"cpu{cpu}" / "topology"
            core = tuple(
                int((topology / name).read_text().strip())
                for name in ("physical_package_id", "core_id")
            )
            if any(value < 0 for value in core):
                raise ValueError("CPU topology is unknown")
            cores.setdefault(core, []).append(cpu)
    except (OSError, ValueError):
        candidates = allowed[1:] + allowed[:1]
    else:
        groups = list(cores.values())
        groups = groups[1:] + groups[:1]
        candidates = [group[0] for group in groups]
        candidates.extend(cpu for group in groups for cpu in group[1:])
    return sorted(candidates[:count])


def configured_cpus(environ, allowed):
    if "NAV_CPUSET" in environ:
        return parse_cpuset(environ["NAV_CPUSET"], allowed)
    value = environ.get("NAV_CPU_COUNT", "3")
    if not re.fullmatch(r"[0-9]+", value):
        raise ValueError("NAV_CPU_COUNT must be a positive integer")
    return choose_cpus(allowed, int(value))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-cpus", action="store_true", help="print the selected CPUs without running a command")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if args.print_cpus and command:
        parser.error("--print-cpus cannot be combined with a command")
    if not args.print_cpus and not command:
        parser.error("a command is required (or use --print-cpus)")
    try:
        if sys.platform != "linux":
            raise ValueError("CPU affinity requires Linux")
        cpus = configured_cpus(os.environ, os.sched_getaffinity(0))
        normalized = ",".join(str(cpu) for cpu in cpus)
        if args.print_cpus:
            print(normalized)
            return 0
        os.sched_setaffinity(0, cpus)
        if os.sched_getaffinity(0) != set(cpus):
            raise ValueError("the requested CPU mask could not be applied exactly")
        os.environ["NAV_CPUSET"] = normalized
        os.execvp(command[0], command)
    except (OSError, ValueError) as error:
        print(f"navigation CPU affinity: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
