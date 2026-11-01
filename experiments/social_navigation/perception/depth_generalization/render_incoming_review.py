#!/usr/bin/env python3
"""Render sealed incoming-review replay with measured-point boxes, never GT-derived boxes."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from review_incoming import CLASSIFIER, sha, old_proposals, project, GeneralizedPersonDetector, Classifier
from render_detector_overlays import frame_predictions, pixel_support, background, draw_candidate

NAMES = dict(legacy_2d_classifier='Cũ · 2D · 0,50',incoming_2d_050='Weights mới · 2D · 0,50',
             incoming_2d_085='Weights mới · 2D · 0,85',generalized_3d_classifier='Weights mới · 3D + prior · 0,45')

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((a.run/'plan.json').read_text());res=json.loads((a.run/'result.json').read_text());seals=json.loads((a.run/'sensor_seal.json').read_text())
    for path,h in plan['source_sha256'].items():assert sha(Path(path))==h
    protocol=plan['protocol'];root=Path(plan['normalized_root']);manifest={}
    legacy=Classifier(Path(plan['model_dirs']['baseline']));incoming=Classifier(Path(plan['model_dirs']['incoming']))
    detector=GeneralizedPersonDetector(artifacts_dir=Path(plan['model_dirs']['incoming']),min_person_score=.45)
    for ep in protocol['episodes']:
        norm=root/ep
        for name,h in plan['input_sha256'][ep].items():assert sha(norm/name)==h
        for name,h in seals[ep].items():assert sha(a.run/ep/(name+'.jsonl'))==h
        depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False);cal=json.loads((norm/'calibration.json').read_text())
        rows={name:[json.loads(l) for l in (a.run/ep/(name+'.jsonl')).read_text().splitlines()] for name in protocol['variants']}
        manifest[ep]=[]
        for index in [113,121,253,320]:
            points=project(depth[index],cal['k'],np.array(cal['optical_to_base_link']),protocol['stride'])
            two=frame_predictions(points,cal,protocol,rows['legacy_2d_classifier'][index],rows['generalized_3d_classifier'][index],legacy,detector)
            views={'legacy_2d_classifier':two['old'],'generalized_3d_classifier':two['new']}
            props=old_proposals(points)
            scores=[incoming.predict(dict(points_xyz=q['raw_points'],scan_valid=False,scan_ranges=[]))['score'] for q in props]
            for name,tau in [('incoming_2d_050',.5),('incoming_2d_085',.85)]:
                ds=[];accepted=[(i,q,score) for i,(q,score) in enumerate(zip(props,scores)) if score>=tau]
                tracks=rows[name][index]['tracks'];assert len(accepted)==len(tracks)
                for (i,q,score),track in zip(accepted,tracks):
                    assert np.allclose(q['xy'],track['xy_base'],atol=1e-10) and np.isclose(score,track['person_score'],atol=1e-12)
                    bbox,uv=pixel_support(q['raw_points'],cal,protocol['stride'])
                    ds.append(dict(candidate_index=i,accepted=True,track_id=track['track_id'],score=score,base_score=score,bbox_uv=bbox,pixel_uv=uv))
                views[name]=ds
            fig,axes=plt.subplots(1,4,figsize=(21,4.9));counts=[]
            for ax,name,color in zip(axes,NAMES,['#1976d2','#008473','#a36108','#b64242']):
                ds=views[name];count=sum(q['accepted'] for q in ds);counts.append(count)
                background(ax,depth[index],NAMES[name]+f'\n{count} phát hiện',color)
                accepted_index=0
                for q in ds:
                    draw_candidate(ax,q,color)
                    if q['accepted']:
                        ax.texts[-1].set_text(f"#{q['track_id']} · {q['score']:.2f}")
                        ax.texts[-1].set_position((6,16+accepted_index*18))
                        accepted_index+=1
            fig.suptitle(f'{ep} · frame {index} · depth đã ghi; mỗi cột dùng cùng input',fontsize=13)
            fig.text(.5,.025,'Khung từ đúng điểm detector đã dùng, đối chiếu output đã seal. Không dùng GT để tạo khung; hình không chứng minh semantic correctness.',ha='center',fontsize=10)
            fig.tight_layout(rect=[0,.06,1,.94]);name=f'{ep}_{index:04d}.png';fig.savefig(a.output/name,dpi=110);plt.close(fig)
            manifest[ep].append(dict(file=name,depth_index=index,counts=counts))
    table=[]
    for ep,stats in res['episodes'].items():
        for name in NAMES:
            q=stats['variants'][name];gate=stats['local_nonregression'].get(name)
            table.append(f"<tr><td>{ep}</td><td>{NAMES[name]}</td><td>{q['supported_gt_proxy_coverage']*100:.1f}%</td><td>{q['paired']}</td><td>{q['unpaired']}</td><td>{q['fragmentation_proxy']}</td><td>{'baseline' if gate is None else ('PASS' if gate['passed'] else 'FAIL')}</td></tr>")
    body=r'''<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Review code detector mới — 31/10/2026</title><style>:root{font-family:system-ui,sans-serif;color-scheme:dark;background:#111b2b;color:#e6f0fd}body{max-width:1420px;margin:auto;padding:26px;line-height:1.65}a{color:#9adaff}h1{font-size:32px;line-height:1.25}h2{margin-top:32px}p,li{color:#bfcee1}.box{background:#203149;padding:18px;border-radius:12px;margin:20px 0}.warn{border-left:4px solid #efb16c}.ok{border-left:4px solid #6dc9aa}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:11px;border-bottom:1px solid #3b506b;text-align:left}.scroll{overflow:auto}img{width:100%;background:#fff;border-radius:9px}button,select{font:inherit;background:#263d59;color:#eaf5ff;padding:9px;border:1px solid #587592;border-radius:6px}pre{white-space:pre-wrap;font-size:12px;background:#1c2c43;padding:16px;max-height:480px;overflow:auto}code{color:#b7dcff}</style><a href="../">← Tất cả báo cáo</a><p>31/10/2026 · nguồn bab853ea + 2ffd7c75 · nhánh feat/depth-detector-generalization</p><h1>Đã nhập code mới để kiểm tra.<br>Weights mới có cải thiện, nhưng chưa đủ thay bản cũ.</h1><div class="box warn"><b>Kết luận:</b> model mới giảm unpaired và cải thiện nhẹ walking ở ngưỡng0,50; nhưng conversation bị mất coverage và tăng phân mảnh ID. Cả0,50 lẫn0,85 đều FAIL gate tổng hai episode. Giữ model thử nghiệm riêng, giữ weights/world mặc định cũ.</div>
<h2>Đối chiếu cùng ảnh depth thật</h2><p>Bốn cột: bản cũ; weights mới cùng threshold0,50; weights mới threshold0,85 tác giả chọn; weights mới trên pipeline3D + prior0,45 như default của code được gửi. Không quét ngưỡng hoặc train trong vòng review này.</p><p><select id="ep"><option value="pilot_walking">Nhóm đi bộ</option><option value="pilot_conversation">Nhóm trò chuyện</option></select> <button id="prev">←</button> <button id="next">→</button> <span id="caption"></span></p><a id="large" target="_blank"><img id="frame" alt="Khung người và điểm số từ bốn cấu hình trên cùng ảnh depth đã ghi"></a>
<h2>Kết quả trên toàn bộ recordings cũ</h2><div class="scroll"><table><thead><tr><th>Episode</th><th>Cấu hình</th><th>Coverage proxy</th><th>Paired</th><th>Unpaired</th><th>Fragment proxy</th><th>Gate cục bộ</th></tr></thead><tbody>__TABLE__</tbody></table></div><p>910 frame, 905 đủ odom/GT; baseline khớp910/910 frame với reference cũ. Coverage/paired là proxy hình học, không phải semantic recall. Unpaired không đồng nghĩa chắc chắn đồ vật/false positive. Gate cục bộ giữ≥95% paired, không tăng unpaired/ID switch/fragmentation. Hai recording đã mở trước đây chỉ dùng regression, không phải test tổng quát hóa.</p>
<h2>Điểm đáng giữ</h2><ul><li>Có trainer PointNet-style và trainer linear; có pipeline tạo depth/mask, QA và render báo cáo. Trước vòng này mới có recipe/head smoke.</li><li>Weights linear mới thực sự khác bản cũ và tải được bằng Classifier API, không phải chỉ đổi tên model.</li><li>Trên walking: cùng threshold0,50, coverage90,7→91,4%, unpaired160→133; ở0,85 là89,2% và116. Các kết quả này đáng giữ để nghiên cứu.</li></ul>
<h2>Vì sao chưa chấp nhận kết luận tổng quát hóa?</h2><ul><li>Dataset80 frame được dựng bằng renderer NumPy/SciPy với point-splat từ DAE, không phải depth capture của bốn world Gazebo.</li><li>A1/A2/B1/C1 vẫn dùng walk_blue/orange/red/stand cùng geometry và cùng height1,716m. Đổi family label/file hash không tạo morphology độc lập giữa train/validation/test.</li><li>Scene hash là literal trong code; hai giá trị dài63 ký tự, không phải SHA25664 ký tự. Readiness READY hiện chỉ chứng minh metadata/file hashes đủ, chưa xác nhận provenance.</li><li>PointNet checkpoint, model_freeze và training/test receipts không có trong commit và cũng không có trên máy review. Chưa tái lập được kết quả hoặc quyết định riêng cho PointNet; không train bù trên bộ dữ liệu này để hợp thức hóa.</li><li>Step-D gốc dùng weights mới cho cả các nhánh được gọi là3D baseline, matching có thể tính nhiều TP cho một người, timing chưa gồm projection/ROS, và các gate được đặt trong code sau evaluation. Không dùng các số đó làm nghiệm thu deployment.</li><li>Train selection hiện là macro-F1 trên patch gộp, chưa phải whole-scene macro-F1 và equal scene/family sampling đúng recipe.</li></ul>
<h2>Đã sửa khi tích hợp</h2><div class="box ok"><ul><li>Nhập hai commit bằng fast-forward vào checkout detector; không merge sang MuJoCo.</li><li>Giữ model mới ở artifacts_experimental_2ffd7c75, validation_gate=false; weights/config default và5world giữ byte gốc879aaf4. World mới vẫn có trong history2ffd7c75.</li><li>Trainer linear bắt buộc output-dir mới và cấm ghi vào default/legacy; không tự deploy. Sửa matching dùng one-to-one, tách baseline3D đúng weights và ghi rõ timing chưa end-to-end.</li><li>Full suite đầu vào338PASS/2skip; full suite sau sửa344PASS/2skip. Protected baseline116filePASS. Không mở Gazebo hoặc fit model.</li></ul></div>
<h2>Phần WM đi kèm commit</h2><p>Commit còn mang theo snapshot wm_restore với báo cáo8episode/2quarantine và một samplewindow. Đây là bằng chứng do nhánh gửi cung cấp, chưa phải full dataset3GB đã được phục hồi/audit trong vòng này; chưa trainWM/RL. Giữ tách khỏi kết luận detector.</p>
<h2>Bước tiếp theo có giới hạn</h2><p>Yêu cầu provenance/assets có geometry thật sự độc lập và checkpoint+receipts còn thiếu. Thu depth/ownership từ pipeline đã kiểm trên train/validation, seal test mới, rồi mới train/đánh giá. Không tăng threshold hoặc thêm luật theo từng cảnh để làm đẹp kết quả hiện tại.</p>
<p><a href="result.json">Kết quả910frame</a> · <a href="plan.json">Protocol/source/input hashes và source audit</a> · <a href="sensor_seal.json">Sensor seal</a> · <a href="frames.json">Manifest ảnh</a> · <a href="integrated_full_tests.log">Suite sau sửa</a> · <a href="integrated_baseline.json">116protected files</a> · <a href="REVIEW.md">Tài liệu review và cách chạy</a></p><details><summary>Source audit chi tiết</summary><pre id="audit"></pre></details><script>const frames=__FRAMES__,audit=__AUDIT__;let i=1;const ep=document.getElementById('ep');function show(){const f=frames[ep.value][i];document.getElementById('frame').src=f.file;document.getElementById('large').href=f.file;document.getElementById('caption').textContent='Frame '+f.depth_index+' · số khung: '+f.counts.join(' / ');}ep.onchange=()=>{i=1;show()};document.getElementById('prev').onclick=()=>{i=(i+3)%4;show()};document.getElementById('next').onclick=()=>{i=(i+1)%4;show()};document.getElementById('audit').textContent=JSON.stringify(audit,null,2);show();</script></html>'''
    body=body.replace('__TABLE__',''.join(table)).replace('__FRAMES__',json.dumps(manifest).replace('<',r'\u003c')).replace('__AUDIT__',json.dumps(plan['source_audit']).replace('<',r'\u003c'))
    (a.output/'index.html').write_text(body)
    (a.output/'frames.json').write_text(json.dumps(manifest,indent=2)+'\n')
    for name in ['plan.json','result.json','sensor_seal.json']:shutil.copyfile(a.run/name,a.output/name)
    for name in ['integrated_full_tests.log','integrated_baseline.json']:shutil.copyfile(a.run.parent/name,a.output/name)
    doc=Path(__file__).resolve().parents[4]/'docs/INCOMING_DETECTOR_REVIEW_2026-10-31.md';shutil.copyfile(doc,a.output/'REVIEW.md')
    print(json.dumps(dict(rendered_frames=sum(map(len,manifest.values())),all_boxes_match_sealed_outputs=True)))

if __name__=='__main__':main()
