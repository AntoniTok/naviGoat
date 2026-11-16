"""Exercise the actual launcher preflight with a fake ROS setup, without ROS."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class CameraLauncherTests(unittest.TestCase):
    def run_preflight(self, setup_text):
        launcher = Path(__file__).resolve().parents[1] / "run_camera_test.sh"
        # Stop before any processes are launched. Run the real preflight against
        # temporary paths and stub dependency commands instead of /opt or USB.
        preflight = launcher.read_text().split("export TRACK_MAX_AGE=", 1)[0]
        with tempfile.TemporaryDirectory(prefix="following-launcher-") as temporary:
            root = Path(temporary)
            workspace = root / "workspace"
            for name in ("ros/src/autonomy/person_following/ros_camera_test.py",
                         "enid/social_perception/jetson_pipeline.sh"):
                path = workspace / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            setup = root / "setup.bash"
            setup.write_text(setup_text)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            for name in ("ros2", "python3"):
                stub = bin_dir / name
                stub.write_text("#!/bin/sh\nexit 0\n")
                stub.chmod(0o755)
            preflight = preflight.replace("/workspace", str(workspace))
            preflight = preflight.replace("/opt/ros/jazzy/setup.bash", str(setup))
            preflight += '''
[[ "${FOLLOWING_TEST_SETUP_LOADED:-}" == yes ]]
[[ -o nounset && -o errexit && -o errtrace && -o pipefail ]]
printf 'preflight completed with strict options restored\\n'
'''
            environment = os.environ.copy()
            environment["PATH"] = str(bin_dir) + os.pathsep + environment["PATH"]
            environment.pop("AMENT_TRACE_SETUP_FILES", None)
            environment.pop("FOLLOWING_TEST_SETUP_LOADED", None)
            return subprocess.run(["bash", "-c", preflight], env=environment,
                                  capture_output=True, text=True, timeout=5)

    def test_ros_setup_may_read_unset_variables_then_strict_checks_resume(self):
        # This optional-variable access is the pattern in Jazzy's setup scripts.
        result = self.run_preflight('''
if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then :; fi
export FOLLOWING_TEST_SETUP_LOADED=yes
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("preflight completed", result.stdout)

    def test_setup_failure_still_aborts_startup(self):
        result = self.run_preflight("return 42\n")
        self.assertEqual(result.returncode, 42, result.stderr)
        self.assertNotIn("preflight completed", result.stdout)


if __name__ == "__main__":
    unittest.main()
