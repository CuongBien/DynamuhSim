#!/usr/bin/env python3
"""Read-only control probes: frozen model/engine bindings and synthetic sensor input.

No train, held-out evaluation, corpus/raw access, report snapshot execution or
subprocesses are allowed during the audited probe. Output is written afterwards.
"""
import argparse,hashlib,importlib.util,json,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
sys.dont_write_bytecode=True
from dynamuhsim.runtime.profiles import PROFILES
from dynamuhsim.artifacts.layout import logical_report_name
from dynamuhsim.workspace import stage_directory


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--consumer',choices=['bundle','legacy'],default='bundle')
    args=parser.parse_args()
    if args.output.exists() or args.output.is_symlink():raise FileExistsError('Preserve existing probe')
    benchmark=ROOT/'experiments/social_navigation/benchmark'
    active=True;reads=set();denied=[]
    def audit(event,values):
        if not active:return
        if event=='open':
            path,mode,flags=values
            if isinstance(path,(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(path)).absolute()
                if flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND):
                    denied.append(event);raise PermissionError('Control probe is read-only')
                if path.is_relative_to(ROOT):
                    name=logical_report_name(path.relative_to(ROOT).as_posix())
                    if '/generated/' in name or '/episodes/' in name or 'LABEL_REVIEW_ONLY' in name:
                        denied.append(name);raise PermissionError('No raw/corpus or held-out predictions in control probe')
                    if args.consumer=='bundle' and any(name.startswith('experiments/social_navigation/benchmark/'+stage+'/reports/') for stage in ['depth_person_classifier','depth_identity_guard','depth_track_refinement']):
                        denied.append(name);raise PermissionError('Bundle consumer cannot read legacy model/engine reports')
                    reads.add(name)
        if event in ['os.remove','os.rename','os.mkdir','os.rmdir','os.link','os.symlink','subprocess.Popen','os.system','socket.connect','socket.bind']:
            denied.append(event+(':'+str(values[0]) if event=='subprocess.Popen' else ''));raise PermissionError('Control probe forbids side effects')
        if event=='exec' and '/reports/' in values[0].co_filename.replace('\\','/'):
            denied.append(values[0].co_filename);raise PermissionError('Do not execute report snapshots')
    sys.addaudithook(audit)
    def load(name,path):
        spec=importlib.util.spec_from_file_location(name,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
    result={}
    for name,profile in PROFILES.items():profile.validate_gates(benchmark)
    result['runtime_gates']=dict(status='PASS',profiles=sorted(PROFILES))
    if args.consumer=='bundle':
        from dynamuhsim.controls.bundle import registered_bundle
        from dynamuhsim.perception.depth_classifier import FrozenDepthClassifier
        bundle=registered_bundle(ROOT)
        engines={}
        for name,stage in [('identity','depth_identity_guard'),('refinement','depth_track_refinement')]:
            value=bundle.verify_engine(name,ROOT)
            metadata=bundle.manifest['engines'][name]['metadata']
            engines[stage]=dict(status='PASS',engine_files=len(value['engine_sha256']),freeze_sha256=bundle.manifest['files'][metadata]['sha256'])
        classifier=FrozenDepthClassifier(bundle);frozen=classifier.metadata
        seal_sha=classifier.evaluation_receipt_sha256
        logits=classifier.logits(np.linspace(-1,1,90,dtype=np.float32).reshape(3,30))
    else:
        engines={}
        for stage,relative in [('depth_identity_guard','reports/final_engine/frozen.json'),('depth_track_refinement','reports/frozen02.json')]:
            module=load('_control_'+stage,stage_directory(ROOT,stage)/'freeze.py')
            value=module.verify(stage_directory(ROOT,stage)/relative)
            engines[stage]=dict(status='PASS',engine_files=len(value['engine_sha256']),freeze_sha256=sha(stage_directory(ROOT,stage)/relative))
        folder=stage_directory(ROOT, 'depth_person_classifier') / 'reports'
        frozen=json.loads((folder/'selected_frozen.json').read_text())
        for name,key in [('selected_model.npz','model_sha256'),('selected_model.pt','torch_weights_sha256'),('training.json','training_sha256')]:
            assert sha(folder/name)==frozen[key]
        assert sha(stage_directory(ROOT, 'depth_person_classifier') / 'frozen.json')==frozen['source_freeze_sha256']
        seal=folder/'test_opened.json';assert seal.is_file()
        seal_sha=sha(seal)
        assert json.loads(seal.read_text())['selected_frozen_sha256']==sha(folder/'selected_frozen.json')
        arrays=dict(np.load(folder/'selected_model.npz',allow_pickle=False))
        inference=load('_control_classifier_inference',stage_directory(ROOT, 'depth_person_classifier') / 'inference.py')
        synthetic=dict(depth=np.linspace(-1,1,90,dtype=np.float32).reshape(3,30),lidar=np.zeros((3,8),dtype=np.float32),lidar_mask=np.array([True,False,True]))
        logits=inference.numpy_logits(synthetic,arrays,frozen['hidden'],frozen['modality'])
        assert sha(seal)==seal_sha
    result['engine_bindings']=engines
    assert logits.shape==(3,) and np.isfinite(logits).all()
    result['classifier']=dict(status='PASS',candidate=frozen['candidate'],threshold=frozen['threshold'],model_sha256=frozen['model_sha256'],test_seal_sha256=seal_sha,
                              synthetic_rows=3,logits_sha256=hashlib.sha256(logits.tobytes()).hexdigest(),training_run=False,held_out_data_opened=False,test_evaluation_run=False)
    # Exercise the actual dynamic imports and tracker with fabricated geometry only.
    guard=load('_control_identity_guard',stage_directory(ROOT, 'depth_identity_guard') / 'guard_tracker.py')
    tracker=guard.IdentityGuard();rows=[]
    for i in range(4):
        detection=dict(xy=np.array([1.+.01*i,.2]),shape=np.array([1.5,.2,.15]),points=80,extent=np.array([.4,.3,1.5]))
        rows.append(tracker.update((i+1)*100_000_000,[detection],[0.,0.,0.]))
    assert all(len(row)==1 for row in rows) and rows[-1][0]['confirmed']
    result['dynamic_identity_tracker']=dict(status='PASS',synthetic_frames=4,output_sha256=hashlib.sha256(json.dumps(rows,sort_keys=True).encode()).hexdigest())
    result['trace']=dict(status='PASS',reads=sorted(reads),blocked_events=denied,report_snapshots_executed=False)
    active=False
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:stream.write(json.dumps(result,sort_keys=True,indent=2,allow_nan=False)+'\n')
    print('CONTROL PROBE PASS',len(reads),'repository file reads; no train/test/corpus access')


if __name__=='__main__':main()
