#!/usr/bin/env python3
"""Three sequential fixed-adapter episodes; retain native references and failed attempts."""
import argparse,hashlib,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;REPO=SOCIAL.parent.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();out=a.report_dir.resolve();out.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
config=SOCIAL/'configs/formation.yaml';(out/'formation.yaml').write_bytes(config.read_bytes())
protocol=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),seeds=[42,43,44],steps=1200,reset_prefix_steps_seed42=500,
    source_sha256={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),HERE/'worker.py',HERE/'continuous_groups.py',config,*list((SOCIAL/'vendor/SocNavGym/socnavgym').rglob('*.py'))]},
    native_reference=str(SOCIAL/'benchmark/socnavgym_lifecycle/reports/run01'),variant='fixed',
    vendor_commit=subprocess.check_output(['git','-C',str(SOCIAL/'vendor/SocNavGym'),'rev-parse','HEAD'],text=True).strip(),
    scope='Native lifecycle with explicit fixed adapter: preserve positions at formation commit, bound yaw, global individual-disc ORCA proposal plus analytic swept collision limiting. Stationary robot and native square room only; no Gazebo.',
    resources='Sequential, affinity two CPUs, BLAS/OMP one thread; native CPU workload, no Gazebo process')
(out/'protocol.json').write_text(json.dumps(protocol,indent=2));attempts=[]
for seed in protocol['seeds']:
    directory=out/f'seed{seed}';print('START',seed,flush=True)
    with (out/f'seed{seed}.log').open('w') as log:
        process=subprocess.run([str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--config',str(out/'formation.yaml'),'--output',str(directory),'--seed',str(seed)],stdout=log,stderr=subprocess.STDOUT,cwd=REPO)
    result=json.loads((directory/'result.json').read_text()) if (directory/'result.json').exists() else {}
    attempts.append(dict(seed=seed,returncode=process.returncode,status=result.get('status','MISSING'),steps=result.get('recorded_steps'),events=len(result.get('events',[]))))
    (out/'progress.json').write_text(json.dumps(dict(planned=3,attempts=attempts),indent=2));print('END',json.dumps(attempts[-1]),flush=True)
raise SystemExit(0 if all(a['returncode']==0 for a in attempts) else 1)
