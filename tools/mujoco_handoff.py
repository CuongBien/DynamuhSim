#!/usr/bin/env python3
"""Receiver preflight: reuse existing guards; no simulator launch or auto-install."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, text=True, capture_output=True, timeout=30)


def delegate(arguments):
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    result = subprocess.run([sys.executable, str(ROOT/'tools/handoff.py'), *arguments],
                            cwd=ROOT, env=env, text=True, capture_output=True, timeout=120)
    try:
        receipt = json.loads(result.stdout)
    except json.JSONDecodeError:
        receipt = dict(status='ERROR', output=(result.stdout+result.stderr)[-4000:])
    receipt['exit_code'] = result.returncode
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check'])
    data = parser.add_mutually_exclusive_group()
    data.add_argument('--archive', type=Path, help='Portable pilot ZIP received separately')
    data.add_argument('--dataset', type=Path, help='Previously extracted portable dataset')
    parser.add_argument('--restore-dir', type=Path, help='NEW extraction directory, required with --archive')
    parser.add_argument('--output', type=Path, help='NEW JSON receipt; never overwrite')
    args = parser.parse_args()
    if bool(args.archive) != bool(args.restore_dir):
        parser.error('--archive and --restore-dir must be used together')
    if args.output and args.output.exists():
        parser.error('Receipt already exists: '+str(args.output))
    plan = json.loads((ROOT/'configs/handoff/mujoco_port.json').read_text())
    anchor = plan['base_commit']
    ancestry = git('merge-base', '--is-ancestor', anchor, 'HEAD')
    head = git('rev-parse', 'HEAD')
    branch = git('branch', '--show-current')
    checks = dict(reference_ancestry=dict(status='PASS' if ancestry.returncode==0 else 'FAIL',
                  reference_commit=anchor, note='A shallow clone may need additional Git history'))
    for key, command in [('protected_baseline',['baseline']),('data_dependencies',['doctor','--scope','data']),
                         ('contract_smoke',['smoke-contract'])]:
        checks[key] = delegate(command)
    if args.archive:
        checks['portable_data'] = delegate(['smoke-data','--archive',str(args.archive.resolve()),
                                          '--work-dir',str(args.restore_dir.resolve())])
    elif args.dataset:
        checks['portable_data'] = delegate(['smoke-data','--dataset',str(args.dataset.resolve())])
    else:
        checks['portable_data'] = dict(status='NOT_CHECKED', note='Provide the separate ZIP or an extracted dataset; Git does not include the pilot')
    required = ['reference_ancestry','protected_baseline','data_dependencies','contract_smoke']
    if args.archive or args.dataset: required.append('portable_data')
    good = all(checks[name]['status']=='PASS' for name in required)
    report = dict(schema='mujoco_receiver_preflight/1', status='PASS' if good else 'FAIL',
                  scope='CODE_AND_DATA_CHECKS' if args.archive or args.dataset else 'CODE_CHECKS_ONLY',
                  branch=branch.stdout.strip(), head=head.stdout.strip(), checks=checks,
                  mujoco_backend='NOT_IMPLEMENTED', simulator_started=False, software_installed=False,
                  training_authorized=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps(report, indent=2))
    return 0 if good else 1


if __name__=='__main__':
    raise SystemExit(main())
