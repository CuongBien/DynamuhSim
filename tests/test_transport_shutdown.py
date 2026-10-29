"""Exact launch bridge cardinality, ownership and child-watcher evidence."""
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from dynamuhsim.runtime import transport_shutdown as stop

ARGS={101:[stop.PARAM,b'--ros-args',b'--params-file',b'/tmp/robot.yaml'],102:[stop.IMAGE,b'/camera/image',b'/camera/depth_image',b'__node:=camera_image_bridge'],103:[stop.PARAM,b'/obstacle_type@std_msgs/msg/String]gz.msgs.StringMsg',b'/obstacle_type/status@std_msgs/msg/String[gz.msgs.StringMsg',b'__node:=obstacle_type_bridge'],104:[stop.PARAM,b'/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg',b'__node:=nav2_pilot_ground_truth_bridge']}
LABELS={101:'parameter_bridge-3',102:'image_bridge-4',103:'parameter_bridge-8',104:'parameter_bridge-9'}


def log_text(failed=None):
 text='\n'.join(f'[INFO] [{LABELS[p]}]: process started with pid [{p}]' for p in ARGS)+'\n'
 for pid in ARGS:
  text+=(f'[ERROR] [{LABELS[pid]}]: process has died [pid {pid}, exit code -11, cmd bridge]' if pid==failed else f'[INFO] [{LABELS[pid]}]: process has finished cleanly [pid {pid}]')+'\n'
 return text


class TransportShutdownTests(unittest.TestCase):
 def test_ordered_single_pidfd_signals_and_failure_not_masked(self):
  for failed in [None,103]:
   with self.subTest(failed=failed),tempfile.TemporaryDirectory() as directory:
    log=Path(directory)/'launch.log';log.write_text(log_text(failed));guard=NS(members=lambda:[dict(pid=p,command='owned') for p in ARGS])
    with patch.object(stop,'argv',side_effect=lambda p:ARGS[p]),patch.object(stop.os,'pidfd_open',side_effect=lambda p:p) as opened,patch.object(stop.os,'close'),patch.object(stop.signal,'pidfd_send_signal') as sent:
     result=stop.stop_launch_bridges(guard,log)
    self.assertEqual(result['clean'],failed is None);self.assertEqual([v['initial']['pid'] for v in result['bridges']],[104,103,102,101]);self.assertEqual([c.args[0] for c in sent.call_args_list],[104,103,102,101]);self.assertEqual(opened.call_count,4)

 def test_missing_ownership_wrong_argv_and_incomplete_log_never_signal(self):
  for mode in ['ownership','argv','log']:
   with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
    log=Path(directory)/'launch.log';log.write_text(log_text().replace('[INFO] [image_bridge-4]: process started with pid [102]','no startup') if mode=='log' else log_text());guard=NS(members=lambda:[dict(pid=p,command='owned') for p in ARGS if mode!='ownership' or p!=103])
    def args(pid):return [b'/unrelated/program'] if mode=='argv' and pid==103 else ARGS[pid]
    with patch.object(stop,'argv',side_effect=args),patch.object(stop.signal,'pidfd_send_signal') as sent:
     result=stop.stop_launch_bridges(guard,log)
    self.assertFalse(result['clean']);sent.assert_not_called()

 def test_identity_change_after_pidfd_open_is_not_signalled(self):
  with tempfile.TemporaryDirectory() as directory:
   log=Path(directory)/'launch.log';log.write_text(log_text());state=dict(opened=False)
   def args(pid):return [b'/unrelated/program'] if pid==104 and state['opened'] else ARGS[pid]
   def opened(pid):
    if pid==104:state['opened']=True
    return pid
   with patch.object(stop,'argv',side_effect=args),patch.object(stop.os,'pidfd_open',side_effect=opened),patch.object(stop.os,'close'),patch.object(stop.signal,'pidfd_send_signal') as sent:
    result=stop.stop_launch_bridges(NS(members=lambda:[dict(pid=p,command='owned') for p in ARGS]),log)
   self.assertFalse(result['clean']);self.assertEqual(result['bridges'][0]['status'],'IDENTITY_CHANGED');self.assertNotIn(104,[c.args[0] for c in sent.call_args_list])

 def test_missing_terminal_ack_is_not_success(self):
  with tempfile.TemporaryDirectory() as directory:
   log=Path(directory)/'launch.log';log.write_text('\n'.join(v for v in log_text().splitlines() if 'finished cleanly' not in v));guard=NS(members=lambda:[dict(pid=p,command='owned') for p in ARGS])
   with patch.object(stop,'argv',side_effect=lambda p:ARGS[p]),patch.object(stop.os,'pidfd_open',side_effect=lambda p:p),patch.object(stop.os,'close'),patch.object(stop.signal,'pidfd_send_signal'):
    result=stop.stop_launch_bridges(guard,log,timeout=0)
   self.assertFalse(result['clean']);self.assertTrue(all(v['status']=='TIMEOUT' for v in result['bridges']))
