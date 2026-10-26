"""Exercise School launcher ownership using disposable CPU-only processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "src/custom_corridor/worlds/school_arena/school_runtime.sh"


def live(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] not in ("Z", "X")
    except FileNotFoundError:
        return False


@unittest.skipUnless(sys.platform == "linux", "Bash launcher process groups require Linux")
class SchoolRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.ready = self.folder / "ready.json"
        self.worker = self.folder / "worker.py"
        self.worker.write_text("""import json, os, signal, subprocess, sys, time
from pathlib import Path
child = subprocess.Popen([sys.executable, '-c',
    'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); '
    'signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'])
Path(sys.argv[1]).write_text(json.dumps([os.getpid(), child.pid, os.getpgrp()]))
if sys.argv[2] != 'stay':
    time.sleep(0.1)
    sys.exit(int(sys.argv[2]))
time.sleep(60)
""")
        self.unrelated = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(60)"], start_new_session=True)
        self.addCleanup(self.stop_unrelated)

    def stop_unrelated(self):
        self.unrelated.terminate()
        self.unrelated.wait(timeout=5)

    def launch(self, action, *, monitor=False):
        script = self.folder / "launch.sh"
        script.write_text('set -euo pipefail\nsource "$1"\nprocess_init\n'
                          'process_start "$2" "$3" "$4" "$5"\nsleep 0.2\nprocess_wait\n')
        process = subprocess.Popen(["bash", *(["-m"] if monitor else []), str(script), str(HELPER), sys.executable,
                                    str(self.worker), str(self.ready), action],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                   start_new_session=True)
        self.addCleanup(self.reap, process)
        deadline = time.monotonic() + 5
        while not self.ready.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.ready.exists())
        self.owned = json.loads(self.ready.read_text())
        return process

    def reap(self, process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)
        if self.ready.exists():
            try:
                os.killpg(json.loads(self.ready.read_text())[2], signal.SIGKILL)
            except ProcessLookupError:
                pass

    def verify(self, process, status):
        _, error = process.communicate(timeout=12)
        self.assertEqual(process.returncode, status, error.decode())
        deadline = time.monotonic() + 2
        while any(live(p) for p in self.owned[:2]) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(any(live(p) for p in self.owned[:2]))
        self.assertIsNone(self.unrelated.poll(), "An unrelated session was signalled")

    def test_failure_status_and_stubborn_descendant(self):
        self.verify(self.launch("23"), 23)

    def test_exit_before_child_session_exists(self):
        delayed = self.folder / "setsid"
        delayed.write_text('#!/bin/bash\nsleep 0.2\nexec /usr/bin/setsid "$@"\n')
        delayed.chmod(0o755)
        child_pid = self.folder / "child_pid"
        script = self.folder / "early_exit.sh"
        script.write_text('set -euo pipefail\nsource "$1"\nprocess_init\n'
                          'process_start "$2" -c "import time;time.sleep(60)"\n'
                          'printf "%s" "${PROCESS_PIDS[0]}" > "$3"\nexit 29\n')
        process = subprocess.Popen(["bash", str(script), str(HELPER), sys.executable, str(child_pid)],
                                   env=dict(os.environ, PATH=str(self.folder) + ":" + os.environ["PATH"]),
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True)
        self.addCleanup(self.reap, process)
        _, error = process.communicate(timeout=12)
        self.assertEqual(process.returncode, 29, error.decode())
        self.assertFalse(live(int(child_pid.read_text())))
        self.assertIsNone(self.unrelated.poll())

    def test_job_control_does_not_change_owned_group(self):
        self.verify(self.launch("23", monitor=True), 23)

    def test_success_status(self):
        self.verify(self.launch("0"), 0)

    def test_sigint(self):
        process = self.launch("stay")
        process.send_signal(signal.SIGINT)
        self.verify(process, 130)

    def test_sigterm(self):
        process = self.launch("stay")
        process.terminate()
        self.verify(process, 143)


if __name__ == "__main__":
    unittest.main()
