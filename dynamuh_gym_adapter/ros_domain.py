"""Linux, host-local ROS domain leases; lock files are never unlinked.

flock, not the metadata/PID, is authoritative. Unrelated or remote ROS
participants do not honor this lease and cannot be excluded by a local lock.
"""
from __future__ import annotations

import errno
import fcntl
import json
import os
from pathlib import Path
import stat
import sys
import weakref


class DomainLeaseError(RuntimeError):
    pass


class DomainInUse(DomainLeaseError):
    pass


def safe_domain_ids() -> tuple[int, ...]:
    """Reserve a full RTPS block (120 participants), outside ephemeral ports.

    This deliberately excludes boundary domains with smaller participant
    budgets. Read the actual Linux setting instead of assuming its default.
    """
    if sys.platform != 'linux':
        raise DomainLeaseError('ROS domain allocation is supported on Linux only')
    try:
        low, high = map(int, Path('/proc/sys/net/ipv4/ip_local_port_range').read_text().split())
        if not 0 <= low <= high <= 65535:
            raise ValueError('invalid ephemeral port range')
    except (OSError, ValueError) as exc:
        raise DomainLeaseError('Cannot establish the Linux ephemeral port range') from exc
    return tuple(d for d in range(233)
                 if (end := 7400 + 250*d + 249) <= 65535
                 and (end < low or 7400 + 250*d > high))


def _open_lock(path: Path) -> int:
    flags = os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW
    while True:
        try:
            fd = os.open(path, flags)
            break
        except FileNotFoundError:
            try:
                fd = os.open(path, flags | os.O_CREAT | os.O_EXCL, 0o666)
            except FileExistsError:
                continue
            # Cooperating workers under other local UIDs use the same inode.
            try:
                os.fchmod(fd, 0o666)
            except BaseException:
                os.close(fd)
                raise
            break
    info = os.fstat(fd)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        os.close(fd)
        raise DomainLeaseError(f'Unsafe ROS domain lock file: {path}')
    return fd


class DomainLease:
    def __init__(self, domain_id: int, fd: int, path: Path):
        self.domain_id = domain_id
        self.owner_pid = os.getpid()
        self.path = path
        self._fd = fd
        self._finalizer = weakref.finalize(self, os.close, fd)

    @classmethod
    def acquire(cls, requested: int | None = None, *, lock_dir: Path = Path('/tmp')):
        safe = safe_domain_ids()
        if requested is not None:
            if isinstance(requested, bool) or not isinstance(requested, int):
                raise ValueError('ROS domain must be an integer or None (automatic)')
            if requested not in safe:
                raise ValueError(f'ROS domain {requested} is outside the safe full-block '
                                 'DDS/UDP range for this Linux ephemeral-port configuration')
            candidates = (requested,)
        else:
            # Leave the conventional, non-isolated ROS default domain 0 alone.
            candidates = tuple(d for d in safe if d != 0)
        for domain in candidates:
            path = lock_dir / f'dynamuh_gym_ros_domain_{domain}.lock'
            try:
                fd = _open_lock(path)
            except OSError as exc:
                raise DomainLeaseError(f'Cannot open ROS domain lease {path}: {exc}') from exc
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                os.close(fd)
                if exc.errno not in (errno.EACCES, errno.EAGAIN):
                    raise DomainLeaseError(f'Cannot lock ROS domain {domain}') from exc
                if requested is not None:
                    raise DomainInUse(f'ROS domain {domain} is already owned by a cooperating '
                                      'local worker; choose a different --domain or omit it '
                                      'for automatic allocation') from exc
                continue
            try:
                data = json.dumps({'domain': domain, 'pid': os.getpid()}).encode()
                os.ftruncate(fd, 0)
                if os.write(fd, data) != len(data):
                    raise OSError('Incomplete lease metadata write')
                return cls(domain, fd, path)
            except BaseException:
                os.close(fd)
                raise
        raise DomainLeaseError('No safe, unowned local ROS domain is available')

    def assert_owner(self):
        if os.getpid() != self.owner_pid:
            raise DomainLeaseError('A ROS worker/lease cannot be reused across fork')
        if not self._finalizer.alive:
            raise DomainLeaseError('ROS domain lease has been released')

    def fileno(self) -> int:
        self.assert_owner()
        return self._fd

    def close(self):
        # Do not LOCK_UN: an owned launch may still hold an inherited reference
        # after a worker crash. The kernel releases flock on the last FD close.
        self._finalizer()
