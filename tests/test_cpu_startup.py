"""Check startup affinity through an existing tmux server, using ROS stubs."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
TMUX = shutil.which("tmux")


@unittest.skipUnless(TMUX and len(os.sched_getaffinity(0)) >= 3, "needs tmux and three allowed CPUs")
class StartupAffinityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="nav cpu startup ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.socket = str(self.root / "tmux.sock")
        self.env = {k: v for k, v in os.environ.items()
                    if k not in ("NAV_CPUSET", "NAV_CPU_COUNT", "TMUX", "TMUX_PANE")}
        self.env.update(ROS_SETUP=str(self.root / "setup.bash"), SENSOR_BACKEND="mid360",
                        SESSION="cpu_test", NAV_CPU_COUNT="3", START_RVIZ="1",
                        MAP_DIR=str(self.root / "maps/test"), TEST_ROOT=str(self.root))
        self.env["PATH"] = str(self.root / "bin") + os.pathsep + self.env["PATH"]
        for name in ("start_mapping.sh", "start_odometry.sh", "start_odometry_rviz.sh",
                     "scripts/run_on_nav_cpus.py"):
            destination = self.root / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, destination)
        for name in ("setup.bash", "install/setup.bash", "config/mapping.rviz",
                     "config/relocalization.rviz", "src/planner/config/navigation.rviz",
                     "src/planner/config/odometry.rviz", "src/Point-LIO/config/mid360.yaml",
                     "src/livox_ros_driver2/config/MID360s_config.json", "maps/test/metadata.yaml",
                     "maps/test/poses.csv", "maps/test/static_map.pcd", "maps/test/btc/manifest.csv",
                     "maps/test/btc/0.btc", "prefix/share/planner/launch/mid360_odometry.launch.py"):
            self.write(name, "")
        for package, executable in (("map", "map_node"), ("relocalization", "relocalization_node"),
                                    ("planner", "scan_planner_node"), ("planner", "octo_global_planner_node"),
                                    ("point_lio", "pointlio_mapping"), ("rviz2", "rviz2"),
                                    ("livox_ros_driver2", "livox_ros_driver2_node")):
            self.write(f"prefix/lib/{package}/{executable}", "#!/bin/sh\nexit 0\n", executable=True)
        self.write("bin/ros2", """#!/usr/bin/env python3
import json, os, pathlib, sys
root = pathlib.Path(os.environ['TEST_ROOT'])
if sys.argv[1:3] == ['pkg', 'prefix']:
    print(root / 'prefix')
else:
    with (root / 'calls.jsonl').open('a') as stream:
        stream.write(json.dumps({'args': sys.argv[1:], 'cpus': sorted(os.sched_getaffinity(0))}) + '\\n')
""", executable=True)
        self.write("save_map.sh", '#!/bin/bash\nros2 test save\n', executable=True)
        self.write("bin/tmux", f"""#!/usr/bin/env python3
import os, sys
if sys.argv[1] in ('attach-session', 'switch-client'):
    sys.exit(0)
os.execv({TMUX!r}, [{TMUX!r}, '-S', os.environ['TEST_ROOT'] + '/tmux.sock', *sys.argv[1:]])
""", executable=True)
        # Start the server without the CPU wrapper, as on a user's existing desktop.
        subprocess.run([TMUX, "-S", self.socket, "new-session", "-d", "-s", "keep_server"],
                       env=self.env, check=True)
        self.addCleanup(lambda: subprocess.run([TMUX, "-S", self.socket, "kill-server"],
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    def write(self, name, content, executable=False):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        if executable:
            path.chmod(0o755)

    def run_startup(self, script, mode=None, count=3):
        self.env["NAV_CPU_COUNT"] = str(count)
        expected = subprocess.check_output(
            ["python3", str(self.root / "scripts/run_on_nav_cpus.py"), "--print-cpus"],
            env=self.env, text=True).strip()
        expected = [int(cpu) for cpu in expected.split(",")]
        result = subprocess.run(["bash", str(self.root / script)], env=self.env,
                                input=f"{mode}\n" if mode else "", text=True,
                                capture_output=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        minimum = 4 if mode == 1 else 3 if mode else 1
        deadline = time.monotonic() + 5
        calls = []
        while time.monotonic() < deadline:
            path = self.root / "calls.jsonl"
            if path.exists():
                calls = [json.loads(line) for line in path.read_text().splitlines()]
            if len(calls) >= minimum:
                break
            time.sleep(0.05)
        self.assertEqual(len(calls), minimum, calls)
        for call in calls:
            self.assertEqual(call["cpus"], expected, call)
        server_pid = subprocess.check_output(
            [TMUX, "-S", self.socket, "display-message", "-p", "#{pid}"], text=True).strip()
        self.assertEqual(os.sched_getaffinity(int(server_pid)), os.sched_getaffinity(0))

    def test_mapping_all_panes(self):
        self.run_startup("start_mapping.sh", mode=1)

    def test_relocalization_all_panes(self):
        self.run_startup("start_mapping.sh", mode=2)

    def test_navigation_all_panes(self):
        self.run_startup("start_mapping.sh", mode=3)

    def test_standalone_odometry_two_cpus(self):
        self.run_startup("start_odometry.sh", count=2)

    def test_standalone_odometry_viewer(self):
        self.run_startup("start_odometry_rviz.sh")


if __name__ == "__main__":
    unittest.main()
