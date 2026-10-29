"""Ownership, child-watcher evidence and shutdown ordering for capture bridges."""
import os
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from dynamuhsim.runtime.shutdown import stop_ground_truth_bridge

ARGS = b'/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge\0/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg\0--ros-args\0-r\0__node:=nav2_pilot_ground_truth_bridge\0'


class BridgeShutdownTests(unittest.TestCase):
    def test_clean_and_failed_child_watcher_receipts_remain_distinct(self):
        for clean in [True, False]:
            with self.subTest(clean=clean), tempfile.TemporaryDirectory() as directory:
                log = Path(directory) / 'launch.log'
                line = ('[INFO] [parameter_bridge-9]: process has finished cleanly [pid 123]' if clean else
                        '[ERROR] [parameter_bridge-9]: process has died [pid 123, exit code -6, cmd bridge]')
                # A different child's clean record must never mask this child failure.
                log.write_text(line + '\n[INFO] [other-8]: process has finished cleanly [pid 124]\n')
                guard = NS(members=lambda: [dict(pid=123, command='owned')])
                with patch.object(Path, 'read_bytes', return_value=ARGS), patch.object(os, 'pidfd_open', return_value=42) as opened, patch.object(signal, 'pidfd_send_signal') as sent, patch.object(os, 'close') as closed:
                    result = stop_ground_truth_bridge(guard, log)
                self.assertEqual(result['clean'], clean)
                self.assertEqual(result['record'], line)
                opened.assert_called_once_with(123)
                sent.assert_called_once_with(42, signal.SIGINT)
                closed.assert_called_once_with(42)

    def test_missing_duplicate_wrong_topic_and_changed_identity_never_signal(self):
        cases = [([], ARGS), ([dict(pid=123), dict(pid=124)], ARGS),
                 ([dict(pid=123)], ARGS.replace(b'/common/ground_truth@', b'/other@'))]
        for members, args in cases:
            with patch.object(Path, 'read_bytes', return_value=args), patch.object(signal, 'pidfd_send_signal') as sent:
                result = stop_ground_truth_bridge(NS(members=lambda: members), Path('unused'))
                self.assertFalse(result['clean']); sent.assert_not_called()
        with patch.object(Path, 'read_bytes', return_value=ARGS), patch.object(os, 'pidfd_open', return_value=42), patch.object(os, 'close') as closed, patch.object(signal, 'pidfd_send_signal') as sent:
            guard = NS(members=unittest.mock.Mock(side_effect=[[dict(pid=123)], []]))
            result = stop_ground_truth_bridge(guard, Path('unused'))
            self.assertEqual(result['status'], 'IDENTITY_CHANGED')
            sent.assert_not_called(); closed.assert_called_once_with(42)

    def test_timeout_is_not_clean_even_if_process_disappears(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(Path, 'read_bytes', return_value=ARGS), patch.object(os, 'pidfd_open', return_value=42), patch.object(os, 'close'), patch.object(signal, 'pidfd_send_signal'):
            result = stop_ground_truth_bridge(NS(members=lambda: [dict(pid=123)]), Path(directory)/'missing', timeout=0)
            self.assertEqual(result['status'], 'TIMEOUT'); self.assertFalse(result['clean'])
