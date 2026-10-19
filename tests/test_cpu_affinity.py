"""CPU selection and real process/thread inheritance, without ROS dependencies."""

import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


WRAPPER = Path(__file__).resolve().parents[1] / "scripts/run_on_nav_cpus.py"
SPEC = importlib.util.spec_from_file_location("run_on_nav_cpus", WRAPPER)
affinity = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(affinity)


class CpuSelectionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.topology = Path(self.directory.name)

    def write_topology(self, cores):
        for cpu, (package, core) in cores.items():
            target = self.topology / f"cpu{cpu}" / "topology"
            target.mkdir(parents=True)
            (target / "physical_package_id").write_text(str(package))
            (target / "core_id").write_text(str(core))

    def test_elf3_distinct_cores_and_leave_first_core(self):
        self.write_topology({cpu: (0, cpu // 2 if cpu < 8 else cpu - 4) for cpu in range(16)})
        self.assertEqual(affinity.choose_cpus(range(16), 3, self.topology), [2, 4, 6])
        self.assertEqual(affinity.choose_cpus(range(16), 2, self.topology), [2, 4])

    def test_current_allowed_mask_is_respected(self):
        self.write_topology({cpu: (0, cpu // 2) for cpu in range(8)})
        self.assertEqual(affinity.choose_cpus([1, 3, 4, 5, 6], 3, self.topology), [3, 4, 6])

    def test_smt_siblings_only_after_all_available_cores(self):
        self.write_topology({cpu: (0, cpu // 2) for cpu in range(6)})
        self.assertEqual(affinity.choose_cpus(range(6), 3, self.topology), [0, 2, 4])
        self.assertEqual(affinity.choose_cpus(range(6), 4, self.topology), [0, 2, 3, 4])

    def test_same_core_id_on_different_packages_is_distinct(self):
        self.write_topology({0: (0, 0), 1: (0, 0), 2: (1, 0), 3: (1, 0), 4: (0, 1)})
        self.assertEqual(affinity.choose_cpus(range(5), 2, self.topology), [2, 4])

    def test_unavailable_or_partial_topology_falls_back(self):
        self.assertEqual(affinity.choose_cpus([3, 5, 7], 2, self.topology), [5, 7])
        self.write_topology({3: (0, 0)})
        self.assertEqual(affinity.choose_cpus([3, 5, 7], 2, self.topology), [5, 7])

    def test_single_available_cpu(self):
        self.assertEqual(affinity.choose_cpus([9], 1, self.topology), [9])

    def test_explicit_ranges_are_normalized_and_override_count(self):
        self.assertEqual(
            affinity.configured_cpus({"NAV_CPUSET": "6, 2-4,3", "NAV_CPU_COUNT": "invalid"}, range(8)),
            [2, 3, 4, 6],
        )

    def test_default_count_is_three(self):
        with mock.patch.object(affinity, "choose_cpus", return_value=[2, 4, 6]) as choose:
            self.assertEqual(affinity.configured_cpus({}, range(16)), [2, 4, 6])
            choose.assert_called_once_with(range(16), 3)

    def test_invalid_configuration(self):
        for cpuset in ("", "2,", "-1", "4-2", "1--3", "1-9999999999", "8", "1,3"):
            with self.subTest(cpuset=cpuset), self.assertRaises(ValueError):
                affinity.configured_cpus({"NAV_CPUSET": cpuset}, {0, 1, 2, 4})
        for count in ("0", "-1", "", "1.5", "three", "5"):
            with self.subTest(count=count), self.assertRaises(ValueError):
                affinity.configured_cpus({"NAV_CPU_COUNT": count}, {0, 1, 2, 4})

    def test_print_does_not_mutate_affinity_or_environment(self):
        with mock.patch.dict(os.environ, {"NAV_CPUSET": "4,2"}, clear=True), \
                mock.patch.object(os, "sched_getaffinity", return_value={2, 4}), \
                mock.patch.object(os, "sched_setaffinity") as apply_mask, \
                mock.patch("sys.stdout", new_callable=io.StringIO) as output:
            self.assertEqual(affinity.main(["--print-cpus"]), 0)
            self.assertEqual(output.getvalue(), "2,4\n")
            self.assertEqual(os.environ["NAV_CPUSET"], "4,2")
            apply_mask.assert_not_called()


@unittest.skipUnless(sys.platform == "linux", "requires Linux affinity")
class CpuExecutionTests(unittest.TestCase):
    def run_wrapper(self, settings, *command):
        environ = {key: value for key, value in os.environ.items() if key not in ("NAV_CPUSET", "NAV_CPU_COUNT")}
        environ.update(settings)
        return subprocess.run(
            [sys.executable, str(WRAPPER), *command], env=environ,
            capture_output=True, text=True, timeout=10,
        )

    def test_process_thread_and_nested_launcher_inherit_the_same_mask(self):
        cpus = sorted(os.sched_getaffinity(0))[:3]
        code = """
import json, os, subprocess, sys, threading
result = {'process': sorted(os.sched_getaffinity(0)), 'env': os.environ['NAV_CPUSET']}
thread = threading.Thread(target=lambda: result.update(thread=sorted(os.sched_getaffinity(0))))
thread.start()
thread.join()
result['child'] = json.loads(subprocess.check_output([sys.executable, '-c', 'import json, os; print(json.dumps(sorted(os.sched_getaffinity(0))))'], text=True))
result['nested'] = subprocess.check_output([sys.executable, sys.argv[1], '--print-cpus'], text=True).strip()
print(json.dumps(result))
"""
        result = self.run_wrapper({"NAV_CPUSET": ",".join(map(str, reversed(cpus)))}, sys.executable, "-c", code, str(WRAPPER))
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        for key in ("process", "thread", "child"):
            self.assertEqual(data[key], cpus)
        self.assertEqual(data["env"], ",".join(map(str, cpus)))
        self.assertEqual(data["nested"], data["env"])

    def test_invalid_configuration_never_executes_command(self):
        allowed = os.sched_getaffinity(0)
        cases = (
            {"NAV_CPU_COUNT": str(len(allowed) + 1)},
            {"NAV_CPUSET": str(max(allowed) + 1)},
            {"NAV_CPUSET": ""},
        )
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "executed"
            for settings in cases:
                with self.subTest(settings=settings):
                    result = self.run_wrapper(settings, sys.executable, "-c", "import pathlib, sys; pathlib.Path(sys.argv[1]).touch()", str(marker))
                    self.assertEqual(result.returncode, 2)
                    self.assertIn("navigation CPU affinity:", result.stderr)
                    self.assertFalse(marker.exists())

    def test_command_exit_code_is_preserved(self):
        cpu = min(os.sched_getaffinity(0))
        result = self.run_wrapper({"NAV_CPUSET": str(cpu)}, sys.executable, "-c", "raise SystemExit(17)")
        self.assertEqual(result.returncode, 17)


if __name__ == "__main__":
    unittest.main()
