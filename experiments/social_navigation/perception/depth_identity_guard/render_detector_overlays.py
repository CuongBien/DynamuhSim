#!/usr/bin/env python3
"""Draw actual detector cluster footprints on recorded depth; GT is a separate view."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
from compare_recordings import sha, old_proposals
from snapshots.sensor import project
from snapshots.association import associate
from generalized_detector import GeneralizedPersonDetector
from frozen_classifier import Classifier


def pixel_support(points, calibration, stride):
    """Project only measured candidate points back to the original camera image."""
    tf = np.array(calibration['optical_to_base_link'])
    optical = (np.asarray(points)-tf[:3, 3]) @ tf[:3, :3]
    good = np.isfinite(optical).all(axis=1) & (optical[:, 2]>0)
    optical = optical[good]
    k = calibration['k']
    uv = optical[:, :2]/optical[:, 2, None]*[k[0], k[4]]+[k[2], k[5]]
    good = (uv[:, 0]>=-.01)&(uv[:, 0]<calibration['width'])&(uv[:, 1]>=-.01)&(uv[:, 1]<calibration['height'])
    uv = uv[good]
    if not len(uv): return None, uv
    lo = np.maximum(uv.min(axis=0)-stride/2, 0.)
    hi = np.minimum(uv.max(axis=0)+stride/2, [calibration['width']-1, calibration['height']-1])
    return np.r_[lo, hi].tolist(), uv


def frame_predictions(points, calibration, protocol, old_row, new_row, classifier, detector):
    old = []
    for candidate in old_proposals(points):
        score = classifier.predict(dict(points_xyz=candidate['raw_points'], scan_valid=False, scan_ranges=[]))['score']
        old.append(dict(candidate, score=score, base_score=score,
                        accepted=score>=protocol['legacy_2d_classifier_threshold']))
    candidates = detector.detect_candidates(points)
    decisions = detector.detect(points)
    if len(candidates)!=len(decisions): raise ValueError('New candidate order changed')
    new = []
    for candidate, decision in zip(candidates, decisions):
        if not np.allclose(candidate['xy'], decision['xy'], atol=1e-10): raise ValueError('Candidate mismatch')
        new.append(dict(candidate, score=decision['score'], base_score=decision['base_score'],
                        accepted=decision['is_person']))
    result = {}
    for variant, proposals, row in [('old',old,old_row),('new',new,new_row)]:
        accepted = [d for d in proposals if d['accepted']]
        if len(accepted)!=len(row['tracks']): raise ValueError('Overlay differs from sealed track count')
        for d, track in zip(accepted,row['tracks']):
            if not np.allclose(d['xy'],track['xy_base'],atol=1e-10) or d['points']!=track['point_count']:
                raise ValueError('Overlay does not reproduce sealed detection')
            if variant=='new' and not np.isclose(d['score'],track['person_score'],atol=1e-12):
                raise ValueError('Overlay score differs from sealed result')
            d['track_id'] = track['track_id']
        records = []
        for i,d in enumerate(proposals):
            bbox, uv = pixel_support(d['raw_points'],calibration,protocol['stride'])
            if bbox is None: raise ValueError('Accepted/candidate points do not project into sensor image')
            records.append(dict(candidate_index=i,accepted=bool(d['accepted']),track_id=d.get('track_id'),
                                score=float(d['score']),base_score=float(d['base_score']),bbox_uv=bbox,
                                xy_base=np.asarray(d['xy']).tolist(),point_count=int(d['points']),pixel_uv=uv))
        result[variant]=records
    return result


def background(ax,depth,title,color):
    ax.imshow(np.ma.masked_invalid(depth),cmap='viridis',vmin=.28,vmax=5.,interpolation='nearest')
    ax.set_title(title,color=color,fontsize=13,fontweight='bold',pad=14)
    ax.set_xlim(-.5,depth.shape[1]-.5);ax.set_ylim(depth.shape[0]-.5,-.5)
    ax.set_xticks([]);ax.set_yticks([])


def draw_candidate(ax,d,color,debug=False):
    accepted=d['accepted']
    if not accepted and not debug:return
    x0,y0,x1,y1=d['bbox_uv']
    edge=color if accepted else '#e05252'
    ax.add_patch(Rectangle((x0,y0),max(1,x1-x0),max(1,y1-y0),fill=False,
                           edgecolor=edge,lw=2.4 if accepted else 1.5,ls='-' if accepted else '--'))
    uv=d['pixel_uv'];ax.scatter(uv[:,0],uv[:,1],s=3,color=edge,alpha=.28 if accepted else .13)
    ident=str(d['track_id']) if d['track_id'] is not None and d['track_id']>0 else '?'
    label=(f"NGƯỜI #{ident} | điểm {d['score']:.2f}" if accepted else f"LOẠI C{d['candidate_index']} | {d['score']:.2f}")
    if debug:
        label=(f"NGƯỜI #{ident}" if accepted else f"LOẠI C{d['candidate_index']}")+f"\n{d['base_score']:.2f} → {d['score']:.2f}"
    ax.text(min(max(x0,2),260),max(y0,26 if debug else 12),label,fontsize=8.4,color='white',va='bottom',clip_on=True,
            bbox=dict(facecolor='#112033' if accepted else '#6c2020',edgecolor=edge,alpha=.9,pad=2))


def save_predictions(path,depth,predictions,episode,index,stamp,debug=False):
    fig,axes=plt.subplots(1,2,figsize=(13.4,5.0))
    for ax,key,title,color in [(axes[0],'old','BẢN CŨ: 2D + classifier','#1672df'),
                               (axes[1],'new','BẢN MỚI: 3D + prior','#ce6800')]:
        count=sum(d['accepted'] for d in predictions[key])
        background(ax,depth,f'{title} — {count} phát hiện',color)
        for d in predictions[key]:draw_candidate(ax,d,color,debug)
        if count==0:
            ax.text(.98,.94,'0 NGƯỜI ĐƯỢC PHÁT HIỆN',transform=ax.transAxes,ha='right',va='top',fontsize=10,color='white',
                    bbox=dict(facecolor='#a72b37',alpha=.95,pad=6))
    fig.suptitle(f'{episode} · frame {index} · {stamp/1e9:.2f} s mô phỏng',fontsize=13)
    caption=('Nét liền = được nhận là người; nét đứt đỏ = ứng viên bị loại. Điểm sau prior là điểm heuristic.' if debug else
             'Khung liền = đầu ra dự đoán NGƯỜI + ID + điểm. Khung ôm các điểm depth đã đo; không suy đoán phần thân bị khuất.')
    fig.text(.5,.035,caption,ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.08,1,.93]);fig.savefig(path,dpi=115);plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--comparison',type=Path,required=True)
    parser.add_argument('--normalized-root',type=Path,required=True)
    parser.add_argument('--previous-media',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((args.comparison/'plan.json').read_text());protocol=plan['protocol']
    if any(sha(Path(p))!=h for p,h in plan['source_sha256'].items()):raise ValueError('Frozen inference sources changed')
    seals=json.loads((args.comparison/'sensor_seal.json').read_text())
    previous=json.loads((args.previous_media/'frames.json').read_text())
    manifest,examples={},{}
    detector=GeneralizedPersonDetector(min_person_score=protocol['generalized_3d_classifier_threshold'])
    classifier=Classifier(Path(__file__).resolve().parent.parent/'depth_person_classifier/artifacts')
    for episode in protocol['episodes']:
        normalized=args.normalized_root/episode
        if any(sha(normalized/name)!=h for name,h in plan['input_sha256'][episode].items()):raise ValueError('Recorded input changed')
        sensor=args.comparison/episode
        for variant,h in seals[episode]['sensor_sha256'].items():
            if sha(sensor/(variant+'.jsonl'))!=h:raise ValueError('Recorded sensor outputs changed')
        old=[json.loads(l) for l in (sensor/'legacy_2d_classifier.jsonl').read_text().splitlines()]
        new=[json.loads(l) for l in (sensor/'generalized_3d_classifier.jsonl').read_text().splitlines()]
        labels={d['depth_index']:d for d in map(json.loads,(sensor/'evaluation_LABEL_ONLY.jsonl').read_text().splitlines())}
        best=max(labels,key=lambda i:(sum(v>=0 for v in labels[i]['geometric_matches']),len(new[i]['tracks']),-i))
        miss=253 if episode=='pilot_conversation' else 121
        selection=sorted({d['depth_index'] for d in previous[episode]}|{best,miss})
        examples[episode]=dict(positive=best,miss=miss)
        depth=np.load(normalized/'depth.npy',mmap_mode='r',allow_pickle=False)
        calibration=json.loads((normalized/'calibration.json').read_text())
        entries=[];gif=[]
        for index in selection:
            row=old[index]
            if not row['odom_valid']:continue
            points=project(depth[index],calibration['k'],np.array(calibration['optical_to_base_link']),protocol['stride'])
            predictions=frame_predictions(points,calibration,protocol,row,new[index],classifier,detector)
            stem=f'{episode}_{index:04d}'
            names=dict(detections=stem+'_detections.png',candidates=stem+'_candidates.png',reference=stem+'_reference.png')
            save_predictions(args.output/names['detections'],depth[index],predictions,episode,index,row['stamp_ns'])
            save_predictions(args.output/names['candidates'],depth[index],predictions,episode,index,row['stamp_ns'],True)
            lab=labels.get(index)
            reference=[]
            if lab:
                for ident,xy,support in zip(lab['gt_ids'],lab['gt_xy_base_LABEL_ONLY'],lab['depth_supported_proxy']):
                    if not support:continue
                    mask=(np.linalg.norm(points[:,:2]-xy,axis=1)<=.4)&(points[:,2]>=.06)&(points[:,2]<=1.95)
                    bbox,uv=pixel_support(points[mask],calibration,protocol['stride'])
                    if bbox is not None:reference.append(dict(gt_id=ident,bbox_uv=bbox))
            fig,ax=plt.subplots(figsize=(9.2,5.8));background(ax,depth[index],'THAM CHIẾU GT + DEPTH — không phải detector','#087d65')
            for d in reference:
                x0,y0,x1,y1=d['bbox_uv'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,ls='--',lw=2,edgecolor='#13e3b7'))
                ax.text(min(x0,340),max(y0,12),f"GT {d['gt_id']}",color='white',fontsize=10,va='bottom',bbox=dict(facecolor='#005947',alpha=.9,pad=3))
            fig.text(.5,.02,'Khung tham chiếu là vùng điểm trong cylinder quanh GT. Chưa có instance mask chuẩn.',ha='center',fontsize=9)
            fig.tight_layout(rect=[0,.05,1,1]);fig.savefig(args.output/names['reference'],dpi=110);plt.close(fig)
            payload={}
            for key,ds in predictions.items():payload[key]=[{k:v for k,v in d.items() if k!='pixel_uv'} for d in ds]
            entry=dict(depth_index=index,stamp_ns=row['stamp_ns'],files=names,
                       counts=dict(old=len(row['tracks']),new=len(new[index]['tracks']),gt_proxy=len(reference)),
                       detections=payload,reference_LABEL_ONLY=reference,
                       semantic_validated=False,training_authorized=False)
            entries.append(entry)
            with Image.open(args.output/names['detections']) as image:gif.append(image.convert('RGB').copy())
        gif[0].save(args.output/(episode+'_detector.gif'),save_all=True,append_images=gif[1:],duration=900,loop=0)
        for image in gif:image.close()
        manifest[episode]=entries
    receipt=dict(schema='depth_detector_pixel_overlays/1',episodes=manifest,examples=examples,
                 selection='10 existing uniform samples plus max geometric 3D matches and named miss examples; illustrative selection, all-frame metrics unchanged',
                 box_source='Inverse projection of actual measured proposal points; no assumed human-height boxes',
                 replay_verification='Every displayed accepted proposal count, XY and point count matches sealed outputs; new scores also match exactly',
                 gt_policy='GT is only in separate reference view/diagnostic example selection, never used to create detector boxes',
                 old_threshold=protocol['legacy_2d_classifier_threshold'],new_threshold=protocol['generalized_3d_classifier_threshold'],
                 comparison_plan_sha256=sha(args.comparison/'plan.json'),comparison_json_sha256=sha(args.comparison/'comparison.json'),
                 generated_ai_images=False,renderer_sha256=sha(Path(__file__)),
                 assets={p.name:sha(p) for p in args.output.iterdir() if p.suffix in ('.png','.gif')})
    (args.output/'detector_overlays.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(frames={k:len(v) for k,v in manifest.items()},examples=examples,output=str(args.output)),indent=2))


if __name__=='__main__':main()
