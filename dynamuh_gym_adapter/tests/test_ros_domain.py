"""Cheap real-kernel lease/lifecycle tests; no Gazebo or ROS participant required."""
import os
from pathlib import Path
import select
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ros_domain
from ros_domain import DomainInUse, DomainLease, DomainLeaseError
import real_backend
from core import Scenario, SimulatorFault, SensorSnapshot, require_fresh


def child_code(tmp_path, domain=90):
    return ('from pathlib import Path\nfrom ros_domain import DomainLease, DomainInUse\n'
            f'p=Path({str(tmp_path)!r})\n'
            f'd={domain}\n')


def test_same_domain_rejected_in_another_process(tmp_path):
    lease = DomainLease.acquire(90, lock_dir=tmp_path)
    try:
        code = child_code(tmp_path) + ('try: DomainLease.acquire(d,lock_dir=p)\n'
                                     'except DomainInUse: print("BUSY")\n'
                                     'else: raise SystemExit(2)\n')
        r = subprocess.run([sys.executable, '-B', '-c', code], capture_output=True,
                           text=True, timeout=5, check=True)
        assert r.stdout.strip() == 'BUSY'
    finally:
        lease.close()
    with_lease = DomainLease.acquire(90, lock_dir=tmp_path)
    with_lease.close()


def test_automatic_domains_are_distinct_and_close_is_idempotent(tmp_path):
    a = DomainLease.acquire(lock_dir=tmp_path)
    b = DomainLease.acquire(lock_dir=tmp_path)
    try:
        assert a.domain_id != b.domain_id
        assert a.domain_id != 0 and b.domain_id != 0
        assert not os.get_inheritable(a.fileno())
    finally:
        a.close(); a.close(); b.close()
    assert a.path.exists()  # Never unlink: that would permit two locked inodes.


def test_kernel_releases_lease_on_sigkill(tmp_path):
    code = child_code(tmp_path) + ('import time\nlease=DomainLease.acquire(d,lock_dir=p)\n'
                                 'print("READY",flush=True)\ntime.sleep(20)\n')
    p = subprocess.Popen([sys.executable, '-B', '-c', code], stdout=subprocess.PIPE, text=True)
    try:
        assert select.select([p.stdout], [], [], 5)[0]
        assert p.stdout.readline().strip() == 'READY'
        with pytest.raises(DomainInUse):
            DomainLease.acquire(90, lock_dir=tmp_path)
        p.kill(); p.wait(timeout=3)
        fresh = DomainLease.acquire(90, lock_dir=tmp_path)
        fresh.close()
    finally:
        if p.poll() is None: p.kill(); p.wait(timeout=3)


def test_inherited_launch_fd_prevents_unsafe_reuse(tmp_path):
    lease = DomainLease.acquire(90, lock_dir=tmp_path)
    p = subprocess.Popen([sys.executable, '-B', '-c', 'import time; time.sleep(20)'],
                         pass_fds=(lease.fileno(),))
    try:
        lease.close()
        with pytest.raises(DomainInUse):
            DomainLease.acquire(90, lock_dir=tmp_path)
        p.terminate(); p.wait(timeout=3)
        fresh = DomainLease.acquire(90, lock_dir=tmp_path)
        fresh.close()
    finally:
        if p.poll() is None: p.kill(); p.wait(timeout=3)
        lease.close()


def test_port_policy_and_invalid_requests(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'read_text', lambda *a, **k: '32768 60999')
    safe = ros_domain.safe_domain_ids()
    assert 0 in safe and 61 in safe and 100 in safe and 215 in safe and 231 in safe
    assert 101 not in safe and 171 not in safe and 232 not in safe
    for value in (-1, 233, 171, True, '61'):
        with pytest.raises(ValueError):
            DomainLease.acquire(value, lock_dir=tmp_path)
    monkeypatch.setattr(ros_domain.sys, 'platform', 'unsupported')
    with pytest.raises(DomainLeaseError, match='Linux'):
        ros_domain.safe_domain_ids()


def test_unsafe_lock_inode_fails_closed(tmp_path):
    target = tmp_path / 'target'; target.write_text('untouched')
    (tmp_path / 'dynamuh_gym_ros_domain_90.lock').symlink_to(target)
    with pytest.raises(DomainLeaseError):
        DomainLease.acquire(90, lock_dir=tmp_path)
    assert target.read_text() == 'untouched'


class LifecycleProbe(real_backend.GazeboROSBackend):
    """Only episode resources are stubbed; the OS lease/reset/close are real."""
    def _stop_episode(self):
        self.stop_count = getattr(self, 'stop_count', 0) + 1
        self.process = None

    def _ros_callbacks(self):
        self.initialized_domains = getattr(self, 'initialized_domains', []) + [self.domain_id]
        if getattr(self, 'fail_init', False): raise RuntimeError('ROS initialization failed')

    def _wait_service(self): pass
    def _world_steps(self, steps): assert steps == 50
    def _snapshot(self, **kwargs): return SimpleNamespace(epoch=self.epoch)


def probe(tmp_path, monkeypatch):
    monkeypatch.setattr(real_backend, 'DomainLease', SimpleNamespace(
        acquire=lambda d: DomainLease.acquire(d, lock_dir=tmp_path)))
    launches = []
    def launch(cmd, **kwargs):
        launches.append(kwargs)
        return SimpleNamespace(pid=-1)
    monkeypatch.setattr(real_backend.subprocess, 'Popen', launch)
    return LifecycleProbe(ros_domain_id=90), launches


def test_resets_hold_domain_and_apply_it_before_ros_and_to_launch(tmp_path, monkeypatch):
    b, launches = probe(tmp_path, monkeypatch)
    fd = b._domain_lease.fileno()
    try:
        for epoch in range(1, 4):
            assert b.reset(Scenario(), epoch).epoch == epoch
            assert b.domain_id == 90 and b._domain_lease.fileno() == fd
            with pytest.raises(DomainInUse): DomainLease.acquire(90, lock_dir=tmp_path)
        assert b.initialized_domains == [90, 90, 90]
        assert all(p['env']['ROS_DOMAIN_ID'] == '90' for p in launches)
        assert all(p['pass_fds'] == (fd,) for p in launches)
        assert len({p['env']['GZ_PARTITION'] for p in launches}) == 3
    finally:
        b.close()
    b.close()
    fresh = DomainLease.acquire(90, lock_dir=tmp_path); fresh.close()
    with pytest.raises(DomainLeaseError): b.reset(Scenario(), 4)


def test_initialization_exception_releases_lease(tmp_path, monkeypatch):
    b, launches = probe(tmp_path, monkeypatch); b.fail_init = True
    with pytest.raises(RuntimeError, match='initialization failed'):
        b.reset(Scenario(), 1)
    assert not launches and b._closed
    fresh = DomainLease.acquire(90, lock_dir=tmp_path); fresh.close()


def test_failed_cleanup_does_not_unlock_live_worker(tmp_path, monkeypatch):
    b, _ = probe(tmp_path, monkeypatch)
    original = b._stop_episode
    def fail(): raise SimulatorFault('cannot prove cleanup')
    b._stop_episode = fail
    with pytest.raises(SimulatorFault): b.close()
    with pytest.raises(DomainInUse): DomainLease.acquire(90, lock_dir=tmp_path)
    b._stop_episode = original; b.close()
    fresh = DomainLease.acquire(90, lock_dir=tmp_path); fresh.close()


def test_owned_children_stop_before_lease_release(tmp_path, monkeypatch):
    monkeypatch.setattr(real_backend, 'DomainLease', SimpleNamespace(
        acquire=lambda d: DomainLease.acquire(d, lock_dir=tmp_path)))
    b = real_backend.GazeboROSBackend(ros_domain_id=90)
    child = 'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)'
    parent = ('import subprocess,sys,time; '
              f'p=subprocess.Popen([sys.executable,"-c",{child!r}]); '
              'time.sleep(.2); print(p.pid,flush=True); time.sleep(30)')
    p = subprocess.Popen([sys.executable, '-B', '-c', parent], start_new_session=True,
                         pass_fds=(b._domain_lease.fileno(),), stdout=subprocess.PIPE, text=True)
    b.process = p
    try:
        assert select.select([p.stdout], [], [], 5)[0]
        child_pid = int(p.stdout.readline())
        b.close()
        assert not b._live_group(p.pid)
        assert p.poll() is not None
        fresh = DomainLease.acquire(90, lock_dir=tmp_path); fresh.close()
    finally:
        if not b._closed: b.close()


def test_forked_worker_cannot_close_parent_resources(tmp_path, monkeypatch):
    b, _ = probe(tmp_path, monkeypatch)
    parent_pid = os.getpid()
    monkeypatch.setattr(real_backend.os, 'getpid', lambda: parent_pid + 1)
    try:
        with pytest.raises(DomainLeaseError, match='fork'):
            b.close()
    finally:
        monkeypatch.setattr(real_backend.os, 'getpid', lambda: parent_pid)
        b.close()


def test_invalid_imu_time_still_rejected():
    s = SensorSnapshot(1, 35.2, 35.2, np.ones(360), .1, 3.5, 35.172,
                       np.zeros(6), .02)
    with pytest.raises(SimulatorFault, match='imu'):
        require_fresh(s, 1)
    s.imu_time = s.sim_time + .03
    with pytest.raises(SimulatorFault, match='imu'):
        require_fresh(s, 1)


@pytest.mark.parametrize('reply', ['ack', 'clock_fallback', 'rejected'])
def test_original_worldcontrol_branches_preserved(tmp_path, monkeypatch, reply):
    # Unit stub only: no simulator/world request is ever sent by this test.
    monkeypatch.setattr(real_backend, 'DomainLease', SimpleNamespace(
        acquire=lambda d: DomainLease.acquire(d, lock_dir=tmp_path)))
    b = real_backend.GazeboROSBackend(ros_domain_id=90); b.clock = 1.
    calls = []
    b._ensure_alive = lambda: None
    def request(args, timeout):
        calls.append((args, timeout))
        if reply == 'clock_fallback': b.clock = 1.2
        return subprocess.CompletedProcess(args, 0, 'data: true\n' if reply == 'ack' else '',
                                           '' if reply == 'ack' else 'Service call timed out\n')
    b._gz = request
    try:
        if reply == 'rejected':
            with pytest.raises(SimulatorFault, match='WorldControl step rejected'):
                b._world_steps(50)
            assert len(calls) == 3  # Existing behavior, pure unit stub, not live retry.
        else:
            b._world_steps(50); assert len(calls) == 1
        assert all(c[0][-1] == 'pause: true multi_step: 50' and c[1] == 4.5 for c in calls)
    finally:
        b.close()
