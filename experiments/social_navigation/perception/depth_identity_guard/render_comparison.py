#!/usr/bin/env python3
"""Scientific previews of recorded depth and label-side pairing; never generates AI images."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from PIL import Image
from compare_recordings import sha, scene_at
from snapshots.sensor import project
from generalized_detector import GeneralizedPersonDetector, adaptive_3d_clustering


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--normalized-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    plan = json.loads((args.comparison/'plan.json').read_text())
    protocol = plan['protocol']
    manifest, diagnosis = {}, {}
    detector = GeneralizedPersonDetector()
    for ep in protocol['episodes']:
        norm = args.normalized_root/ep
        depth = np.load(norm/'depth.npy', mmap_mode='r', allow_pickle=False)
        with np.load(norm/'observations.npz', allow_pickle=False) as o: times = o['depth__stamp_ns']
        cal = json.loads((norm/'calibration.json').read_text())
        scene = [json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()]
        rows = {v:[json.loads(l) for l in (args.comparison/ep/(v+'.jsonl')).read_text().splitlines()] for v in protocol['variants']}
        labels = {v['depth_index']:v for v in [json.loads(l) for l in (args.comparison/ep/'evaluation_LABEL_ONLY.jsonl').read_text().splitlines()]}
        manifest[ep] = []
        gif_frames = []
        for index in np.linspace(40, len(depth)-40, 10, dtype=int):
            points = project(depth[index], cal['k'], np.array(cal['optical_to_base_link']), 4)
            fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), gridspec_kw=dict(width_ratios=[1.35, 1]))
            ax, bev = axes
            im = ax.imshow(np.ma.masked_invalid(depth[index]), cmap='viridis', vmin=.28, vmax=5.)
            ax.set_title('Recorded depth — meters, invalid pixels masked')
            ax.set_xlabel('pixel u'); ax.set_ylabel('pixel v')
            fig.colorbar(im, ax=ax, shrink=.65, label='optical depth Z [m]')
            p = points[(points[:,2]>=.06)&(points[:,2]<=1.95)]
            bev.scatter(p[::2,1], p[::2,0], c=p[::2,2], s=2, cmap='Greys', vmin=0, vmax=1.8, alpha=.45)
            gt = scene_at(scene, int(times[index]), protocol['gt_max_bracket_ns'])
            if gt:
                ids, xy = gt
                for j, (ident, center) in enumerate(zip(ids, xy)):
                    support = labels.get(int(index), {}).get('depth_supported_proxy', [False]*len(ids))[j]
                    bev.add_patch(Circle((center[1],center[0]), .36, fill=False, color='#22a06b' if support else '#9aa4af', lw=1.6, ls='--'))
                    bev.text(center[1], center[0]+.4, 'GT '+str(ident), ha='center', fontsize=7, color='#14754c', clip_on=True)
            for variant, color, marker, title in [('legacy_2d_classifier','#2563eb','o','2D + frozen classifier'),('generalized_3d_classifier','#e67e22','x','3D + frozen classifier/prior')]:
                tracks = rows[variant][index]['tracks']
                xy = np.array([v['xy_base'] for v in tracks]).reshape(-1,2)
                bev.scatter(xy[:,1], xy[:,0], s=70, marker=marker, edgecolors=color if marker=='o' else None,
                            facecolors='none' if marker=='o' else color, label=title, linewidths=1.8)
            bev.add_patch(Rectangle((-.14,-.12),.28,.24,color='#111827'))
            bev.set_xlim(3.2,-3.2); bev.set_ylim(-.3,5.5); bev.set_aspect('equal')
            bev.set_xlabel('y left [m]'); bev.set_ylabel('x forward [m]')
            bev.set_title('Depth point projection + evaluator-only GT')
            bev.legend(loc='upper right', fontsize=7)
            bev.grid(alpha=.15)
            fig.suptitle(f'{ep} | frame {index} | simulation timestamp {times[index]/1e9:.2f} s', fontsize=13)
            fig.text(.5,.015,'Circles GT are label-side geometric references. This is not a costmap or a World Model prediction.',ha='center',fontsize=8)
            fig.tight_layout(rect=[0,.04,1,.94])
            name=f'{ep}_{index:04d}.png';fig.savefig(args.output/name,dpi=110);plt.close(fig)
            with Image.open(args.output/name) as image: gif_frames.append(image.convert('RGB').copy())
            manifest[ep].append(dict(file=name,depth_index=int(index),stamp_ns=int(times[index])))
        gif_frames[0].save(args.output/(ep+'.gif'),save_all=True,append_images=gif_frames[1:],duration=700,loop=0)
        for image in gif_frames: image.close()
        # Fixed 32 uniformly spaced frames; diagnostic only, never changes threshold/gates.
        heights=[]; stage_counts=dict(supported_person_frames=0,raw_3d_cluster_near_gt=0,candidate_after_planar_filter_near_gt=0,
                                     person_score_pass_near_gt=0,base_score_pass_but_prior_reject=0)
        for index in np.linspace(0,len(depth)-1,32,dtype=int):
            if int(index) not in labels: continue
            points=project(depth[index],cal['k'],np.array(cal['optical_to_base_link']),4)
            raw=adaptive_3d_clustering(points);candidates=detector.detect_candidates(points);detections=detector.detect(points)
            lab=labels[int(index)]
            for center,supported in zip(lab['gt_xy_base_LABEL_ONLY'],lab['depth_supported_proxy']):
                if not supported:continue
                stage_counts['supported_person_frames']+=1
                center=np.array(center)
                near=(np.linalg.norm(points[:,:2]-center,axis=1)<=.40)&(points[:,2]>=.06)&(points[:,2]<=1.95)
                if near.any():heights.append(float(np.ptp(points[near,2])))
                stage_counts['raw_3d_cluster_near_gt']+=any(np.linalg.norm(np.median(q[:,:2],axis=0)-center)<=.6 for q in raw)
                stage_counts['candidate_after_planar_filter_near_gt']+=any(np.linalg.norm(q['xy']-center)<=.6 for q in candidates)
                stage_counts['person_score_pass_near_gt']+=any(q['is_person'] and np.linalg.norm(q['xy']-center)<=.6 for q in detections)
            stage_counts['base_score_pass_but_prior_reject']+=sum(q['base_score']>=.45 and not q['is_person'] for q in detections)
        diagnosis[ep]=dict(sampled_frames=32,stage_counts=stage_counts,visible_depth_vertical_span_median_m=float(np.median(heights)) if heights else None,
                            note='Approximate cylinder-support geometry; counts by stage are diagnostic, not semantic error attribution')
    (args.output/'frames.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (args.output/'diagnosis.json').write_text(json.dumps(diagnosis,indent=2)+'\n')
    (args.output/'media_provenance.json').write_text(json.dumps(dict(
        source='Existing normalized Gazebo depth.npy + recorded CameraInfo/TF; GT only for drawing/evaluation',
        transformations='Depth colormap and calibrated base-frame projection; GIF is a slideshow of uniformly selected recorded frames',
        generated_ai_images=False,changes_to_raw=False,comparison_plan_sha256=sha(args.comparison/'plan.json'),
        images={p.name:sha(p) for p in args.output.iterdir() if p.suffix in ('.png','.gif')}
    ),indent=2)+'\n')
    print(json.dumps(diagnosis,indent=2))


if __name__=='__main__':main()
