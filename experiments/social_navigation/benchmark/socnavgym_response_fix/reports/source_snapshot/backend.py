import ast,copy,hashlib,json,math,random,sys,types
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response'
source=COMMON/'model_worker.py';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
locked=json.loads((COMMON/'reports/run01/protocol.json').read_text());expected=next(v for k,v in locked['source_sha256'].items() if k.endswith('/source_snapshot/model_worker.py'));assert sha(source)==expected
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Runner')
ns=dict(HERE=COMMON,a=types.SimpleNamespace(model='soc_guarded_orca'),protocol=json.loads((COMMON/'inputs/protocol.json').read_text()),model={},dt=.1,layout=json.loads((COMMON/'inputs/layout.json').read_text()),sha=sha,Path=Path,np=np,random=random,copy=copy,math=math,sys=sys)
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
def create(case,legacy=False):
    runner=ns['Runner'](case)
    if not legacy:
        sys.path.insert(0,str(HERE))
        from responsive_motion import enable
        enable(runner)
    return runner
