#!/usr/bin/env python3
"""Local ablation report with real measured-point boxes and explicit readiness limits."""
import argparse
import copy
import html
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ablation import GUARD,CLASSIFIER,sha
from render_detector_overlays import frame_predictions,pixel_support,background,draw_candidate
from generalized_detector import GeneralizedPersonDetector
from frozen_classifier import Classifier
from snapshots.sensor import project


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    output=a.output.resolve();output.mkdir(parents=True,exist_ok=False)
    result=json.loads((a.run/'result.json').read_text());plan=json.loads((a.run/'plan.json').read_text());protocol=plan['protocol']
    if any(sha(path)!=h for path,h in plan['source_sha256'].items()):raise ValueError('Inference source changed')
    normalized=Path(plan['normalized_root']);manifest={};verified=0
    detector=GeneralizedPersonDetector();classifier=Classifier(CLASSIFIER/'artifacts')
    indices=[40,113,121,180,253,320]
    for ep in protocol['episodes']:
        norm=normalized/ep;depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
        cal=json.loads((norm/'calibration.json').read_text())
        rows={name:[json.loads(l) for l in (a.run/ep/(name+'.jsonl')).read_text().splitlines()] for name in protocol['variants']}
        manifest[ep]=[]
        for index in indices:
            points=project(depth[index],cal['k'],np.array(cal['optical_to_base_link']),protocol['stride'])
            original=frame_predictions(points,cal,protocol,rows['legacy_2d_classifier'][index],rows['generalized_3d_classifier'][index],classifier,detector)
            without=copy.deepcopy(original['new'])
            for d in without:d.update(accepted=bool(d['base_score']>=.45),score=d['base_score'],track_id=None)
            accepted=[d for d in without if d['accepted']];recorded=rows['three_d_no_prior_045'][index]['tracks']
            if len(accepted)!=len(recorded):raise ValueError('No-prior overlay count differs from sealed output')
            for d,t in zip(accepted,recorded):
                if not np.allclose(d['xy_base'],t['xy_base'],atol=1e-10) or not np.isclose(d['score'],t['person_score'],atol=1e-12):raise ValueError('No-prior overlay differs')
                d['track_id']=t['track_id']
            verified+=1
            fig,axes=plt.subplots(1,3,figsize=(18,5.2))
            views=[(original['old'],'2D cũ (0,50)','#1672df'),(original['new'],'3D giữ prior (0,45)','#ce6800'),(without,'3D bỏ prior (0,45)','#00866c')]
            counts=[]
            for ax,(ds,title,color) in zip(axes,views):
                count=sum(d['accepted'] for d in ds);counts.append(count)
                background(ax,depth[index],f'{title}: {count} phát hiện',color)
                for d in ds:
                    draw_candidate(ax,d,color)
                    if d['accepted']:
                        ident=str(d['track_id']) if d['track_id'] is not None and d['track_id']>0 else '?'
                        ax.texts[-1].set_text(f"#{ident} {d['score']:.2f}")
                if count==0:ax.text(.98,.94,'0 PHÁT HIỆN',transform=ax.transAxes,ha='right',color='white',bbox=dict(facecolor='#a72b37',alpha=.95,pad=5))
            stamp=rows['legacy_2d_classifier'][index]['stamp_ns']
            fig.suptitle(f'{ep} · frame {index} · {stamp/1e9:.2f} s · cùng một ảnh depth đã ghi',fontsize=13)
            fig.text(.5,.03,'Khung = cụm điểm detector đã dùng, kèm ID/điểm. GT không tạo khung. Ví dụ minh họa không thay số liệu 910 frame.',ha='center',fontsize=10)
            fig.tight_layout(rect=[0,.08,1,.93]);name=f'{ep}_{index:04d}.png';fig.savefig(output/name,dpi=105);plt.close(fig)
            manifest[ep].append(dict(file=name,depth_index=index,stamp_ns=stamp,counts=counts))
    # Stage counts include reassociation gains/losses; display net totals only.
    fig,axes=plt.subplots(1,2,figsize=(12,4.7))
    for ax,(ep,stats) in zip(axes,result['episodes'].items()):
        s=stats['stage_attribution'];names=['Depth\nhỗ trợ GT','Proposal\n3D','Sau lọc\nmặt phẳng','Classifier\nkhông prior','Sau\nprior']
        values=[s[k] for k in ['supported','proposal_3d','planar_kept','three_d_no_prior_045','generalized_3d_classifier']]
        bars=ax.bar(names,values,color=['#94a3b8','#5fa7e1','#77b7ce','#23a38a','#e78b57'])
        for bar,value in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,value+15,str(value),ha='center',fontsize=9)
        ax.set_title(ep);ax.set_ylabel('Lượt GT/frame hỗ trợ, ghép hình học');ax.set_ylim(0,max(values)*1.18);ax.tick_params(axis='x',labelsize=8)
    fig.suptitle('Mất người ở cả proposal, lọc mặt phẳng, classifier và prior — proxy hình học',fontsize=12)
    fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(output/'stages.png',dpi=120);plt.close(fig)
    source=json.loads((Path(__file__).parent/'source_review.json').read_text())
    readiness=json.loads((Path(__file__).parent/'generated/benchmark_readiness.json').read_text())
    cuda=json.loads((Path(__file__).parent/'generated/candidate_cuda.json').read_text())
    inventory=json.loads((Path(__file__).parent/'generated/asset_inventory.json').read_text())
    table=[]
    titles={'legacy_2d_classifier':'2D cũ / ngưỡng 0,50','generalized_3d_classifier':'3D + prior / 0,45','three_d_no_prior_045':'3D bỏ prior / 0,45','three_d_no_prior_050':'3D bỏ prior / 0,50 control'}
    for ep,stats in result['episodes'].items():
        for name,v in stats['variants'].items():
            table.append(f"<tr><td>{html.escape(ep)}</td><td>{titles[name]}</td><td>{100*v['supported_gt_proxy_coverage']:.1f}%</td><td>{v['paired']}</td><td>{v['unpaired']}</td><td>{v['fragmentation_proxy']}</td></tr>")
    links=''.join(f"<tr><td><a href='{html.escape(s['url'],quote=True)}' target='_blank' rel='noopener'>{html.escape(s['name'])}</a></td><td>{html.escape(s['facts'])}</td><td>{html.escape(s['decision'])}</td></tr>" for s in source['sources'])
    body=r"""<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Depth detector — prior ablation & generalization</title>
<style>:root{font-family:system-ui,sans-serif;color:#e1eaf6;background:#0d1422;color-scheme:dark}body{max-width:1320px;margin:auto;padding:25px;line-height:1.65}a{color:#8acaff}h1{font-size:30px;line-height:1.25}h2{font-size:21px;margin-top:30px}p{color:#b7c8dc}.box{padding:17px;border:1px solid #30445f;border-radius:12px;background:#14223a;margin:18px 0}.good{color:#82e9b4}.warn{color:#ffc28e}img{width:100%;height:auto;background:white;border-radius:8px}table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:10px;border-bottom:1px solid #304159;text-align:left}th{color:#9ed6f1}select,button,input{padding:8px;background:#20334d;border:1px solid #46607d;border-radius:7px;color:#e7f0fb}input[type=range]{width:38%;vertical-align:middle}.scroll{overflow:auto}.small{font-size:13px;color:#a7bbd1}code{background:#23334c;padding:2px 4px}pre{white-space:pre-wrap;font-size:12px;max-height:350px;overflow:auto}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}@media(max-width:800px){body{padding:12px}.grid{display:block}h1{font-size:25px}}</style>
<p class="small">30/10/2026 · nhánh <code>feat/depth-detector-generalization</code> · dữ liệu cũ chỉ là development/regression</p>
<h1>Bỏ prior cải thiện rõ, nhưng 3D hiện tại<br>vẫn chưa vượt bản cũ.</h1>
<div class="box"><strong>Kết luận vòng này:</strong> giữ tọa độ XYZ và bộ kiểm chứng, dừng ưu tiên clustering/prior 3D hiện tại. Ứng viên train thử tiếp theo là <b>proposal 2D có recall cao + head PointNet-style học trên XYZ</b>. Head mới chỉ qua CUDA smoke, chưa train và chưa có kết quả generalization.</div>
<h2>Xem trực tiếp cùng một frame</h2><p>Trái: bản cũ. Giữa: 3D giữ prior. Phải: 3D bỏ prior, giữ nguyên proposal và classifier. Khung người/ID/điểm được đối chiếu với output đã seal; không vẽ khung dự đoán từ GT.</p>
<div><select id="episode"><option value="pilot_walking">Nhóm đi bộ</option><option value="pilot_conversation">Nhóm trò chuyện</option></select> <button id="prev">←</button> <input id="frame" type="range" min="0" max="5" value="2" aria-label="Chọn frame ablation"> <button id="next">→</button> <span id="counts"></span></div>
<a id="large" target="_blank" rel="noopener"><img id="image" alt="Khung người trên ảnh depth, ba cấu hình ablation"></a><p id="caption" class="small"></p>
<h2>Toàn bộ 910 frame: cùng weights, không sweep ngưỡng</h2><div class="scroll"><table><thead><tr><th>Episode</th><th>Cấu hình</th><th>Coverage proxy</th><th>Paired</th><th>Unpaired</th><th>Fragment proxy</th></tr></thead><tbody>__TABLE__</tbody></table></div>
<p class="small">905 frame đủ odom/GT để đo; 1.820 dòng baseline/prior cũ tái lập khớp. Coverage là proxy GT có hỗ trợ depth, không phải semantic recall chuẩn. Paired/unpaired dùng gate 0,60 m và margin 0,15 m. Hai episode cùng world/mesh, tương quan theo thời gian; không coi số lượt GT/frame là số người độc lập. Không có timing so sánh vì runner được instrument chung.</p>
<h2>Người bị mất ở bước nào?</h2><img src="stages.png" alt="Số ghép hình học còn lại qua proposal, lọc mặt phẳng, classifier và prior">
<p>Trên cả hai episode, prior làm mất thêm <b>831 lượt ghép GT/frame</b> so với cùng classifier không prior. Nhưng trước prior đã có mất ở proposal/planar filter. Retrain classifier không tự sửa những bước này.</p>
<p class="small">Stage “proposal 3D” bao gồm envelope, density/size và quy tắc cắt torso; không phải clustering thuần. Ghép hình học có thể mất và lấy lại cặp khi lọc ứng viên, nên không gọi các cột là upper bound chặt. JSON có cả lost/gained, không giấu reassociation.</p>
<h2>Chốt ứng viên để train thử</h2><div class="grid"><div class="box"><b>Giữ:</b> proposal 2D cũ làm đối chứng và nguồn patch XYZ; calibration/timestamp/GT separation; mask unknown và tracker.<br><b>Thử học:</b> point encoder nhỏ trên điểm quan sát, không đưa ID người/world/simulator vào input, không nhân prior 0,15 phía sau.<br><b>Giới hạn:</b> head không cứu người không có proposal hoặc hai người bị gộp; cần đo trên corpus mới.</div><div class="box"><b class="good">CUDA compatibility: PASS</b><br>18.881 tham số; Python3.12.3, Torch2.7.1+cu128, RTX4060.<br>Forward/backward, permutation, padding mask, empty patch kiểm được.<br><b class="warn">0 optimizer step · weights chưa train</b><br>Đây là implementation PointNet-style riêng, không phải checkpoint person pretrained và không chứng minh chất lượng.</div></div>
<h2>Benchmark tổng quát hóa: đã chuẩn bị contract, chưa có dữ liệu mới</h2><div class="box warn"><b>NOT_READY — chưa thu các miền độc lập.</b> Manifest gồm 8 case train/validation/test với scene/geometry/avatar tách hẳn và no-human controls. Validator chặn dùng chung morphology dù đổi màu áo, trùng episode/window group, label từ chính detector và file/hash thiếu. Không đánh giá test trước khi freeze model chọn bằng validation.</div>
<p>Inventory xác nhận <code>walk / walk_blue / walk_orange / walk_red / stand / push_trolley</code> có cùng geometry-array fingerprint. Chúng không phải sáu kiểu người độc lập. Corpus cũ tự ghi rõ một warehouse/một mesh; các test cũ đã mở không được đổi tên thành test mới.</p>
<p>LiDAR mô phỏng hiện nhìn human collision cylinder, không phải chân người có khớp. Vì vậy pretrained LiDAR trên dữ liệu thật chưa được coi là ứng viên thay thế trực tiếp. Nhánh depth và benchmark LiDAR phải có dữ liệu/nhãn đúng modality; FROG không cung cấp depth cho head XYZ này.</p>
<h2>Dataset/model đã đối chiếu nguồn gốc</h2><div class="scroll"><table><thead><tr><th>Nguồn tác giả</th><th>Thông tin kiểm được</th><th>Khả năng dùng hiện tại</th></tr></thead><tbody>__SOURCES__</tbody></table></div>
<p class="small">Chưa tải external dataset/checkpoint mới hoặc xác nhận chất lượng ngoài miền. SDT có camera cao, cần kiểm calibration/đơn vị và nhãn patch trước khi dùng. LFE author runtime Humble/22.04 chưa được nghiệm thu Jazzy/24.04; lượt này không cài package hệ thống.</p>
<h2>Điểm dừng và việc kế tiếp</h2><ol><li>Ablation và quyết định proposal: đã có bằng chứng, giữ mọi FAIL.</li><li>Contract benchmark và validator: đã có; còn cần ít nhất các mesh morphology mới cùng capture/label QA cho các case.</li><li>Ứng viên train: đã chốt và kiểm CUDA. Khi train/validation mới đủ, chạy một lượt train theo protocol, freeze rồi mới mở test.</li></ol><p>Không có WM/RL training, không bật semantic/H1–H5/association masks. Workspace MuJoCo được giữ nguyên trên nhánh riêng; vòng này không mở Gazebo.</p>
<h2>Receipts</h2><p><a href="result.json">Số liệu ablation</a> · <a href="plan.json">Source/input freeze</a> · <a href="sensor_seal.json">Sensor seal</a> · <a href="candidate_cuda.json">CUDA probe</a> · <a href="benchmark_manifest.json">8 planned cases</a> · <a href="benchmark_readiness.json">Data readiness</a> · <a href="asset_inventory.json">Mesh fingerprints</a> · <a href="source_review.json">Nguồn/compatibility</a> · <a href="integrity.json">Kiểm tra độc lập</a></p>
<script>const views=__FRAMES__;const ep=document.getElementById('episode'),range=document.getElementById('frame');function update(){const list=views[ep.value];range.max=list.length-1;const f=list[+range.value];document.getElementById('image').src=f.file;document.getElementById('large').href=f.file;document.getElementById('counts').textContent='Cũ '+f.counts[0]+' | Giữ prior '+f.counts[1]+' | Bỏ prior '+f.counts[2];document.getElementById('caption').textContent=ep.value+' · frame '+f.depth_index+' · '+(f.stamp_ns/1e9).toFixed(2)+' s mô phỏng · khung từ điểm đo thật, ví dụ minh họa';}ep.addEventListener('change',update);range.addEventListener('input',update);document.getElementById('prev').onclick=()=>{range.value=Math.max(0,+range.value-1);update()};document.getElementById('next').onclick=()=>{range.value=Math.min(+range.max,+range.value+1);update()};update();</script></html>"""
    body=body.replace('__TABLE__',''.join(table)).replace('__SOURCES__',links).replace('__FRAMES__',json.dumps(manifest).replace('<',r'\u003c'))
    (output/'index.html').write_text(body)
    for name in ['result.json','plan.json','sensor_seal.json']:shutil.copyfile(a.run/name,output/name)
    stage=Path(__file__).parent
    for name in ['candidate_cuda.json','asset_inventory.json','benchmark_readiness.json']:
        shutil.copyfile(stage/'generated'/name,output/name)
    for name in ['benchmark_manifest.json','source_review.json','protocol.json']:shutil.copyfile(stage/name,output/name)
    (output/'frames.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'media_receipt.json').write_text(json.dumps(dict(verified_overlay_frames=verified,generated_ai_images=False,
        source='Existing Gazebo depth + measured candidate points; exact sealed detection comparison, no GT for boxes',
        report_source_sha256=sha(Path(__file__)),assets={p.name:sha(p) for p in output.glob('*.png')}),indent=2)+'\n')
    print(output/'index.html')


if __name__=='__main__':main()
