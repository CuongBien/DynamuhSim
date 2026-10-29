"""Package the audited portable dataset; no simulator/raw mutation or training."""
import argparse,hashlib,importlib.util,json,shutil,subprocess,sys,zipfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('--pilot',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();audit=json.loads(a.audit.read_text());assert audit['status']=='PASS';dataset=a.pilot.resolve()/'dataset';a.output.mkdir(parents=True,exist_ok=False)
 shutil.copy2(HERE/'inspect_pilot.py',dataset/'tools/inspect_pilot.py');shutil.copy2(ROOT/'docs/HANDOFF_MUJOCO.md',dataset/'HANDOFF_MUJOCO.md');shutil.copy2(a.audit,dataset/'audit.json');shutil.copy2(HERE/'audit_freeze.json',dataset/'audit_freeze.json');(dataset/'README_HANDOFF.md').write_text('Two-episode pilot, not a training split. Python + NumPy only.\nRun: python tools/inspect_pilot.py --dataset .\nSee HANDOFF_MUJOCO.md for schema/port checklist and audit.json for integrity results. Raw MCAP is retained at source_run on original machine; not included in this portable bundle.\n')
 spec=importlib.util.spec_from_file_location('pack_loader',dataset/'tools/loader.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);ds=m.Dataset(dataset);fig,axes=plt.subplots(2,5,figsize=(20,8),layout='constrained');cmap=ListedColormap(['#4d5661','#edf0f3','#e26b35']);norm=BoundaryNorm([-1.5,-.5,.5,1.5],3);examples=[]
 for row,meta in enumerate(ds.metadata['episodes']):
  e=ds.episode(meta['episode_id']);index=len(e)//2;x=e.observation(index);y=e.supervision(index);masks=e.task_masks(index);target=dataset/'examples'/meta['episode_id'];target.mkdir(parents=True,exist_ok=False);np.savez_compressed(target/'observation_SENSOR_ONLY.npz',**x['history'],depth_history=x['depth_history'],bev_anchor_estimated=x['bev_anchor_estimated'],bev_frame_valid=np.array(x['bev_frame_valid']));np.savez_compressed(target/'supervision_LABEL_ONLY.npz',**{'trajectory__'+k:v for k,v in y['trajectories'].items()},**{'future_bev__'+k:v for k,v in y['future_bev'].items()});np.savez_compressed(target/'task_masks_LABEL_ONLY.npz',**masks);(target/'calibration_SENSOR_ONLY.json').write_text(json.dumps(x['calibration'],indent=2)+'\n');metadata=dict(episode_id=meta['episode_id'],window_index=index,anchor_ns=int(x['history']['anchor_ns']),split='pilot_only',training_authorized=False,selection='Fixed middle index for schema/visual QA only; no model selection',depth_source='Recorded metric optical-Z, no synthesis',bev_source='Computed observed occupancy from recorded depth');(target/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n');examples.append(metadata)
  depth=x['depth_history'][-1];good=np.isfinite(depth)&(depth>=.28)&(depth<=5.);depth_cmap=plt.get_cmap('viridis').copy();depth_cmap.set_bad('#000000');im=axes[row,0].imshow(np.ma.array(depth,mask=~good),cmap=depth_cmap,vmin=.28,vmax=5.,interpolation='nearest');axes[row,0].set(title=meta['episode_id']+'\nDepth history last (m; invalid black)',xlabel='Optical pixel u',ylabel='Optical pixel v');fig.colorbar(im,ax=axes[row,0],shrink=.6)
  grids=[x['bev_anchor_estimated'],*y['future_bev']['occupancy']]
  for col,(grid,title) in enumerate(zip(grids,['INPUT: estimated anchor','LABEL: +0.5s true anchor','LABEL: +1.0s true anchor','LABEL: +3.0s true anchor']),1):
   ax=axes[row,col];ax.imshow(grid,origin='lower',extent=(-3.2,3.2,0,6.4),cmap=cmap,norm=norm,interpolation='nearest');ax.invert_xaxis();ax.set(title=title,xlabel='y left (m)',ylabel='x forward (m)',aspect='equal')
 fig.suptitle('Real recorded depth and calculated BEV: gray unknown / white free / orange occupied\nFixed middle windows; both pilot_only. Future ego GT used in LABEL warps only, never input BEV. No AI image generation.',fontsize=12);fig.savefig(a.output/'pilot_samples.png',dpi=140);plt.close(fig);(a.output/'example_metadata.json').write_text(json.dumps(examples,indent=2)+'\n');shutil.copy2(a.output/'pilot_samples.png',dataset/'pilot_samples.png')
 result=subprocess.run([sys.executable,'-I',str(dataset/'tools/inspect_pilot.py'),'--dataset',str(dataset)],cwd='/tmp',capture_output=True,text=True,check=True);inspection=json.loads(result.stdout);(a.output/'portable_inspection.json').write_text(json.dumps(inspection,indent=2)+'\n');assert inspection['ROS_imported'] is False
 inventory={str(p.relative_to(dataset)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(dataset.rglob('*')) if p.is_file() and '__pycache__' not in p.parts};(dataset/'bundle_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n');archive=a.output/'handoff_data_pilot01.zip'
 with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for name in [*inventory,'bundle_inventory.json']:z.write(dataset/name,'handoff_data_pilot01/'+name)
 with zipfile.ZipFile(archive) as z:
  assert z.testzip() is None
  for name,info in inventory.items():
   with z.open('handoff_data_pilot01/'+name) as f:
    h=hashlib.sha256()
    for block in iter(lambda:f.read(1<<20),b''):h.update(block)
   assert h.hexdigest()==info['sha256']
 receipt=dict(status='PASS',archive=str(archive.resolve()),archive_bytes=archive.stat().st_size,archive_sha256=sha(archive),bundle_files=len(inventory)+1,uncompressed_bytes=sum(v['bytes'] for v in inventory.values()),zip_crc_and_all_member_sha256='PASS',portable_inspection='PASS, Python isolated mode, no ROS imports',pilot_only=True,training_ready=False,raw_mcap_included=False);(a.output/'package_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt),flush=True)

if __name__=='__main__':main()
