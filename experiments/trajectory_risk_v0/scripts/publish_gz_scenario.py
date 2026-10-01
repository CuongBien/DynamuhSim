#!/usr/bin/env python3
import argparse, json, subprocess, pathlib
p=argparse.ArgumentParser(); p.add_argument('scenario'); p.add_argument('--command',default='reset_start'); a=p.parse_args()
path=pathlib.Path(a.scenario).expanduser().resolve()
payload=json.dumps(json.loads(path.read_text()),separators=(',',':'))
def esc(s): return s.replace('\\','\\\\').replace('"','\\"').replace('\n','\\n')
subprocess.run(['gz','topic','-t','/multi_human/scenario','-m','gz.msgs.StringMsg','-p',f'data: "{esc(payload)}"'],check=True)
subprocess.run(['gz','topic','-t','/multi_human/command','-m','gz.msgs.StringMsg','-p',f'data: "{a.command}"'],check=True)
print(f'Published {path}')
print(f'Command: {a.command}')
