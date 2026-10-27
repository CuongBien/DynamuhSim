"""Velocity-pilot CLI boundaries and terminal-pause regression checks."""
from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from dynamuhsim.runtime.velocity_trial import parser,observe_interlock


class VelocityTrialTests(unittest.TestCase):
    def test_backend_and_fault_options_stay_specific_to_each_pilot(self):
        args=['--report-dir','/tmp/new run','--model','crowd_orca_plus','--case','crossing_near']
        self.assertEqual(parser(False).parse_args(args).sim_seconds,10.)
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):parser(True).parse_args(args)
        args[3]='soc_guarded_orca'
        self.assertEqual(parser(True).parse_args(args+['--fault-after-updates','25','--fault-mode','disconnect']).fault_mode,'disconnect')
        with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):parser(False).parse_args(args+['--fault-after-updates','25'])

    def test_terminal_pause_rejects_clock_drift_and_sustained_resume(self):
        for clocks,valid in [([0.,0.,.0025,.0025],True),([0.,.001,.0025,.0025],False),
                             ([0.,0.,.01,.01],False),([0.,0.,.0025,.003],False)]:
            with self.subTest(clocks=clocks):
                latest={'clock':NS(clock=NS(sec=0,nanosec=0))};remaining=iter(clocks);receipt={}
                def spin(seconds):
                    stamp=next(remaining);latest['clock']=NS(clock=NS(sec=int(stamp),nanosec=round(stamp*1e9)))
                with patch('dynamuhsim.runtime.velocity_trial.subprocess.run',return_value=NS(returncode=0,stdout='data: true')) as call:
                    if valid:
                        check=observe_interlock(spin,latest,[],receipt)
                        self.assertTrue(check['manual_resume_attempted'])
                        self.assertEqual(check['repaused_clock_delta_s'],0.)
                        self.assertEqual(call.call_args.args[0][-1],'pause: false')
                    else:
                        with self.assertRaises(AssertionError):observe_interlock(spin,latest,[],receipt)
                        self.assertIn('clock_before',receipt['interlock_check'])
                        self.assertIn('clock_after',receipt['interlock_check'])


if __name__=='__main__':unittest.main()
