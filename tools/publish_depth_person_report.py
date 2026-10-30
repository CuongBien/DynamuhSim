#!/usr/bin/env python3
"""Publish a new static localhost report directory; never replace an older report."""
import argparse
import html
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--media-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--detector-overlays', type=Path)
    args = parser.parse_args()
    run, media, output = args.run_dir, args.media_dir, args.output
    output.mkdir(parents=True, exist_ok=False)
    comparison = json.loads((run/'comparison/comparison.json').read_text())
    ros = json.loads((run/'ros_smoke.json').read_text())
    integrity = json.loads((run/'replay_integrity.json').read_text())
    frames = json.loads((media/'frames.json').read_text())
    diagnosis = json.loads((media/'diagnosis.json').read_text())
    for source in media.iterdir():
        if source.is_file(): shutil.copyfile(source, output/source.name)
    for name in ['comparison.json','plan.json','sensor_seal.json']:
        shutil.copyfile(run/'comparison'/name, output/name)
    for name in ['ros_smoke.json','replay_integrity.json','build.log','unit_tests.log','benchmark_scenarios.log','evaluate_multi_environments.log','launch_arguments.log']:
        shutil.copyfile(run/name, output/name)
    table = []
    for ep, result in comparison['episodes'].items():
        for variant, q in result['variants'].items():
            table.append('<tr><td>'+html.escape(ep)+'</td><td>'+html.escape(variant)+'</td>'+
                         f"<td>{q['paired']:,}</td><td>{q['unpaired']:,}</td><td>{100*q['supported_gt_proxy_coverage']:.1f}%</td>"+
                         f"<td>{q['fragmentation_proxy']}</td><td>{q['id_switches_500ms']}</td><td>{q['latency_p95_ms']:.1f} ms</td></tr>")
    contents = r"""<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Depth person detector — integration & recorded A/B</title>
<style>
:root{font-family:Inter,system-ui,sans-serif;color:#dce7f6;background:#0c1221;color-scheme:dark}body{max-width:1220px;margin:auto;padding:30px 24px;line-height:1.6}h1{font-size:30px;line-height:1.25;margin-bottom:12px}h2{font-size:21px;margin-top:32px}p{max-width:1050px;color:#b8c9df}a{color:#8ac7ff}code{background:#17243a;padding:2px 5px;border-radius:4px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.card{background:#142036;border:1px solid #283d58;border-radius:14px;padding:18px}.card b{display:block;font-size:25px}.good{color:#72e6b3}.bad{color:#ff9d8f}.notice{border-left:4px solid #f79874;background:#322328;padding:14px 18px;border-radius:8px;color:#ffddce}.small{font-size:13px;color:#a7bbd1}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:11px 9px;border-bottom:1px solid #2a3b51}th{color:#8ed7f0;background:#15233b;position:sticky;top:0}.scroll{overflow:auto}select,button,input{padding:8px 12px;background:#1c2e46;color:#edf5ff;border:1px solid #365272;border-radius:8px}input[type=range]{width:45%;vertical-align:middle}figure{margin:12px 0}img{display:block;width:100%;height:auto;background:white;border-radius:10px}.two{display:grid;grid-template-columns:1fr 1fr;gap:16px}figcaption{padding:8px 0;color:#b8c9df;font-size:13px}li{margin:8px 0}.badge{padding:3px 10px;background:#1d3932;border-radius:20px;font-size:13px;color:#92e9c0}pre{white-space:pre-wrap;background:#142036;padding:15px;border-radius:10px;overflow:auto}summary{cursor:pointer;color:#b4d8ff}@media(max-width:800px){.grid,.two{grid-template-columns:1fr}h1{font-size:25px}body{padding:18px 12px}}
</style>
<header><span class="badge">30/10/2026 · Depth + LiDAR project · detector thử nghiệm</span><h1>Đã sửa tích hợp ROS.<br>Detector 3D chưa đạt để thay bản cũ.</h1>
<p>Đọc nhánh <code>feat/depth-person-detector</code>, sửa các lỗi đầu vào và replay cùng dữ liệu đã ghi. Đây là báo cáo benchmark perception; không có lượt chạy Gazebo hoặc huấn luyện mới.</p></header>
<div class="grid"><div class="card"><b class="good">323 tests · PASS</b>2 skip theo dependency; thêm 5 test pipeline gốc PASS.</div><div class="card"><b class="good">8/8 ROS checks</b>Node cài vào prefix riêng; CameraInfo, TF, depth 16/32-bit, odom và reset.</div><div class="card"><b class="bad">2/2 promotion FAIL</b>910 frame replay, 905 frame đủ odom/GT để đo. Detector mới bỏ sót nhiều.</div></div>
<h2>Kết luận sử dụng</h2><div class="notice">Giữ detector mới ở launch thử nghiệm riêng, không thay Nav2/default perception và không dùng output làm nhãn train. Điểm 100% trên hình trụ/hộp tổng hợp không chuyển thành chất lượng tốt trên depth camera hiện tại.</div>
<p><strong>Đã sửa:</strong> lấy calibration từ CameraInfo + TF; chuẩn hóa 16UC1/32FC1; chặn odom tương lai/quá cũ; inference lỗi → invalid; giữ ID/velocity/ambiguity trong JSON; marker unknown không ghi đè nhau; cài resources độc lập với checkout. Benchmark giờ matching theo vị trí, reset giữa cảnh tĩnh.</p>
<h2>So sánh cùng hai recordings</h2><p class="small">Paired = proposal ghép một-một với tâm người GT, gate 0,60 m + ambiguity margin 0,15 m. Unpaired có thể do sai vị trí, đồ vật hoặc ghép mơ hồ. Coverage dùng GT có ≥18 điểm depth trong bán kính 0,40 m làm proxy khả năng quan sát; chưa có pixel-instance truth nên không gọi đây là semantic precision/recall. Legacy geometry là proposal + IdentityGuard; 2D classifier là comparator offline dựng từ proposal cũ và classifier đóng băng.</p>
<div class="scroll"><table><thead><tr><th>Episode</th><th>Pipeline</th><th>Paired</th><th>Unpaired</th><th>Coverage proxy</th><th>Fragment proxy</th><th>ID switch ≤0,5s</th><th>p95 CPU</th></tr></thead><tbody>__TABLE__</tbody></table></div>
<p class="small">Timing gồm projection + detection + tracking, thread caps = 1; chưa gồm ROS/hiển thị. Giữ nguyên weights và ngưỡng đã khai báo trước khi replay. Các gate promotion được giữ nguyên khi thất bại.</p>
<h2>Xem frame depth thật và phát hiện cũ/mới</h2><p>Chọn episode, kéo thanh hoặc bấm bước để đối chiếu. Xanh dương: 2D + classifier; cam: 3D mới; vòng nét đứt: GT chỉ dùng phía đánh giá. Nền BEV là điểm depth chiếu theo calibration, không phải costmap.</p>
<div><select id="episode"><option value="pilot_conversation">Nhóm đứng trò chuyện</option><option value="pilot_walking">Nhóm đi bộ</option></select> <button id="prev">←</button> <input id="frame" type="range" min="0" max="9" value="2" aria-label="Frame đã ghi"> <button id="next">→</button> <span id="position"></span></div>
<figure><img id="snapshot" alt="Depth ghi thật và point projection so sánh"><figcaption id="caption"></figcaption></figure>
<details><summary>GIF tổng quan hai episode</summary><p class="small">Mỗi GIF là slideshow 10 frame lấy cách đều từ recording, 700 ms/hình; không phải video realtime, không dùng để đánh giá tốc độ dáng đi.</p><div class="two"><figure><img src="pilot_conversation.gif" loading="lazy" alt="Các frame depth nhóm trò chuyện"><figcaption>Conversation — depth đã ghi</figcaption></figure><figure><img src="pilot_walking.gif" loading="lazy" alt="Các frame depth nhóm đi bộ"><figcaption>Walking — depth đã ghi</figcaption></figure></div></details>
<h2>Vì sao vẫn chưa đạt?</h2><p>Các rule hình học đang loại nhiều proposal của người: có mất cụm ở clustering/bộ lọc mặt phẳng, sau đó score/prior tiếp tục loại. Trên 32 frame chẩn đoán mỗi episode, conversation có 73 trường hợp GT được depth hỗ trợ → 57 có cụm gần GT → 47 qua lọc mặt phẳng → 6 còn được dự đoán người. Walking tương ứng 31 → 30 → 24 → 5. Đây là chẩn đoán hình học gần GT, không thay thế nhãn semantic.</p>
<p>Có proposal vượt ngưỡng classifier trước prior nhưng bị loại sau prior. Chưa thể quy toàn bộ lỗi cho chiều cao camera; cần thiết kế lại trên tập development với nhiều kiểu người và che khuất, rồi đánh giá bằng bộ sensor riêng. Lượt này không chỉnh ngưỡng để làm đẹp kết quả.</p>
<details><summary>JSON chẩn đoán và checks</summary><pre id="diagnosis"></pre><pre id="ros"></pre></details>
<h2>Các phần đã nghiệm thu và giới hạn</h2><ul><li>Build package vào prefix /tmp riêng, launch tùy chọn load được và ROS transport test PASS. Đây chưa phải nghiệm thu live perception trong Gazebo.</li><li>Integrity PASS: 2.730 sensor rows của 3 pipeline; source/input/seal không đổi; odom luôn nhân quả hoặc ghi invalid rõ ràng.</li><li>116 file baseline và hai capture freezes cũ vẫn PASS. Raw pilot và bộ ZIP bàn giao không bị ghi lại.</li><li>Classifier weights cũ, tracker cũ, H1–H5/association/train/WM/RL chưa được nâng trạng thái.</li><li>Ảnh và GIF lấy từ depth Gazebo đã ghi, vẽ bằng Matplotlib/Pillow; không có ảnh AI. GT chỉ ở evaluator/overlay.</li></ul>
<h2>Receipts để kiểm lại</h2><p><a href="comparison.json">Kết quả số</a> · <a href="plan.json">Protocol + source/input hashes</a> · <a href="sensor_seal.json">Sensor seal</a> · <a href="ros_smoke.json">ROS checks</a> · <a href="replay_integrity.json">Integrity</a> · <a href="unit_tests.log">323 tests</a> · <a href="evaluate_multi_environments.log">30 cảnh tổng hợp, metric mới</a> · <a href="media_provenance.json">Nguồn ảnh/GIF</a></p>
<p class="small">MuJoCo handoff: có thể mang API detector nhận point cloud/depth + calibration sang backend khác. Giữ detector này ở trạng thái experimental; pilot cũ và hợp đồng dữ liệu giữ nguyên.</p>
<script>const frames=__FRAMES__;const diagnostic=__DIAG__;const checks=__ROS__;const e=document.getElementById('episode'),r=document.getElementById('frame');function update(){const list=frames[e.value];r.max=list.length-1;const i=Math.min(+r.value,list.length-1),f=list[i];document.getElementById('snapshot').src=f.file;document.getElementById('position').textContent=(i+1)+' / '+list.length;document.getElementById('caption').textContent=e.value+' · depth frame '+f.depth_index+' · simulation timestamp '+(f.stamp_ns/1e9).toFixed(2)+' s · các mốc GT chỉ dùng đánh giá';}e.addEventListener('change',update);r.addEventListener('input',update);document.getElementById('prev').onclick=()=>{r.value=Math.max(0,+r.value-1);update()};document.getElementById('next').onclick=()=>{r.value=Math.min(+r.max,+r.value+1);update()};document.getElementById('diagnosis').textContent=JSON.stringify(diagnostic,null,2);document.getElementById('ros').textContent=JSON.stringify(checks,null,2);update();</script></html>"""
    contents = contents.replace('__TABLE__', ''.join(table))
    for key, obj in [('__FRAMES__',frames),('__DIAG__',diagnosis),('__ROS__',ros)]:
        contents = contents.replace(key,json.dumps(obj).replace('<',r'\u003c'))
    if args.detector_overlays:
        overlays = json.loads((args.detector_overlays/'detector_overlays.json').read_text())
        for source in args.detector_overlays.iterdir():
            if source.is_file():
                if (output/source.name).exists():
                    raise FileExistsError('Overlay would replace existing report asset: '+source.name)
                shutil.copyfile(source, output/source.name)
        section = r"""
<section id="detector" class="card" style="margin:24px 0;padding:20px">
<h2 style="margin-top:0">Đây là khung người do detector dự đoán</h2>
<p>Ảnh bên trái là <b style="color:#7bbaff">bản cũ 2D + classifier</b>; bên phải là <b style="color:#ffbd73">bản mới 3D + prior</b>. Mỗi khung liền có <b>NGƯỜI + ID + điểm</b>. Nếu bản mới không có khung và ghi <b>0 phát hiện</b>, đó là kết quả nó bỏ sót ở frame đó.</p>
<p class="small">Depth camera → gom cụm điểm → classifier chấm điểm → bản mới áp thêm prior hình dạng → giữ các cụm vượt ngưỡng → tracker gán ID. Ngưỡng bản cũ 0,50; bản mới 0,45. Điểm sau prior chưa phải xác suất đã calibration.</p>
<p class="small">Mở mặc định một frame có phát hiện để nhìn rõ khung. Bấm “Ví dụ bản mới bỏ sót” để thấy lỗi; các ví dụ minh họa này không được dùng thay cho thống kê toàn bộ recordings.</p><div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap"><select id="det-episode" aria-label="Chọn episode detector"><option value="pilot_walking" selected>Nhóm đi bộ</option><option value="pilot_conversation">Nhóm trò chuyện</option></select><button id="det-positive">Ví dụ bản mới bắt được người</button><button id="det-miss">Ví dụ bản mới bỏ sót</button></div>
<div style="margin:12px 0"><button id="det-prev">←</button> <input id="det-frame" type="range" min="0" max="10" value="0" aria-label="Chọn frame có khung người"> <button id="det-next">→</button> <span id="det-position"></span></div>
<div id="det-summary" style="font-size:19px;font-weight:700;color:#e6f1ff;margin-bottom:12px"></div>
<label style="display:block;margin:8px 0"><input id="det-rejected" type="checkbox"> Xem cả ứng viên bị loại và điểm trước/sau prior (khung nét đứt đỏ)</label>
<figure><a id="det-open" target="_blank" rel="noopener"><img id="det-image" alt="Khung dự đoán người trực tiếp trên ảnh depth, so sánh hai detector"></a><figcaption id="det-caption"></figcaption></figure>
<p class="small">Bấm ảnh để mở lớn. Khung được chiếu từ đúng cụm điểm detector đã dùng: có thể chỉ ôm phần người camera nhìn thấy. Chưa có bounding-box head dự đoán toàn thân bị khuất. ID được gán riêng bởi từng tracker, không phải ID GT.</p>
<div class="scroll"><table><thead><tr><th>Bản</th><th>ID / ứng viên</th><th>Kết quả</th><th>Điểm gốc</th><th>Điểm cuối</th><th>Ngưỡng</th></tr></thead><tbody id="det-scores"></tbody></table></div>
<details style="margin-top:18px"><summary>Đối chiếu GT riêng: camera đang nhìn thấy người ở đâu?</summary><p class="small">Nét đứt xanh là vùng điểm depth quanh vị trí người GT, chỉ là tham chiếu hình học. Đây không phải khung detector, không phải instance mask chuẩn và không dùng để tạo các khung dự đoán ở trên.</p><img id="det-reference" alt="GT tham chiếu tách riêng với kết quả detector"></details>
<details style="margin-top:14px"><summary>GIF có khung người: xem thay đổi qua các frame</summary><p class="small">Slideshow các frame đã ghi, có thêm ví dụ phát hiện/bỏ sót để minh họa; không phải video realtime và không thay thế số liệu toàn bộ 910 frame.</p><div class="two"><img src="pilot_walking_detector.gif" loading="lazy" alt="GIF khung người nhóm đi bộ"><img src="pilot_conversation_detector.gif" loading="lazy" alt="GIF khung người nhóm trò chuyện"></div></details>
<p class="small"><a href="detector_overlays.json">Tọa độ khung, điểm số và đối chiếu với output đã seal</a>. Overlay khớp chính xác số proposal, vị trí và số điểm của các detection đã ghi; GT không được dùng để tạo khung detector.</p>
</section>
"""
        contents = contents.replace('</header>', '</header>'+section)
        contents = contents.replace('Đã sửa tích hợp ROS.<br>Detector 3D chưa đạt để thay bản cũ.', 'Xem detector khoanh người<br>trực tiếp trên ảnh depth.')
        contents = contents.replace('<h2>Xem frame depth thật và phát hiện cũ/mới</h2>', '<details><summary>BEV và ảnh không có khung — báo cáo tham khảo cũ</summary><h2>Ảnh depth và BEV</h2>')
        contents = contents.replace('<details><summary>GIF tổng quan hai episode</summary>', '</details><details><summary>GIF tổng quan hai episode (BEV)</summary>')
        javascript = r"""<script>
const detectorMedia=__DETECTOR_JSON__;
const de=document.getElementById('det-episode'),dr=document.getElementById('det-frame'),dj=document.getElementById('det-rejected');
function setDetectorExample(kind){const target=detectorMedia.examples[de.value][kind];dr.value=detectorMedia.episodes[de.value].findIndex(f=>f.depth_index===target);updateDetector();}
function updateDetector(){const list=detectorMedia.episodes[de.value];dr.max=list.length-1;const i=Math.min(Math.max(0,+dr.value),list.length-1),f=list[i];
const file=dj.checked?f.files.candidates:f.files.detections;document.getElementById('det-image').src=file;document.getElementById('det-open').href=file;document.getElementById('det-reference').src=f.files.reference;
document.getElementById('det-position').textContent=(i+1)+' / '+list.length;
document.getElementById('det-summary').textContent='Bản cũ: '+f.counts.old+' phát hiện   |   Bản mới: '+f.counts.new+' phát hiện';
document.getElementById('det-caption').textContent=de.options[de.selectedIndex].text+' · frame '+f.depth_index+' · '+(f.stamp_ns/1e9).toFixed(2)+' giây mô phỏng · '+(dj.checked?'Khung nét đứt đỏ = ứng viên bị loại, không phải phát hiện cuối.':'Khung liền = dự đoán người của detector.');
const body=document.getElementById('det-scores');body.replaceChildren();
for(const key of ['old','new']){for(const d of f.detections[key]){if(!d.accepted&&!dj.checked)continue;const row=document.createElement('tr');const values=[key==='old'?'Cũ (2D)':'Mới (3D)',d.accepted?(d.track_id>0?'ID '+d.track_id:'ID chưa rõ'):'Ứng viên '+d.candidate_index,d.accepted?'DỰ ĐOÁN NGƯỜI':'BỊ LOẠI',d.base_score.toFixed(3),d.score.toFixed(3),(key==='old'?detectorMedia.old_threshold:detectorMedia.new_threshold).toFixed(2)];for(const value of values){const td=document.createElement('td');td.textContent=value;row.appendChild(td);}body.appendChild(row);}}
if(!f.counts.old&&!f.counts.new&&!dj.checked){const row=document.createElement('tr'),td=document.createElement('td');td.colSpan=6;td.textContent='Cả hai detector không xuất detection ở frame này.';row.appendChild(td);body.appendChild(row);}}
de.addEventListener('change',()=>setDetectorExample('positive'));dr.addEventListener('input',updateDetector);dj.addEventListener('change',updateDetector);
document.getElementById('det-positive').onclick=()=>setDetectorExample('positive');document.getElementById('det-miss').onclick=()=>setDetectorExample('miss');
document.getElementById('det-prev').onclick=()=>{dr.value=Math.max(0,+dr.value-1);updateDetector();};document.getElementById('det-next').onclick=()=>{dr.value=Math.min(+dr.max,+dr.value+1);updateDetector();};
setDetectorExample('positive');
</script>"""
        javascript = javascript.replace('__DETECTOR_JSON__', json.dumps(overlays).replace('<', r'\u003c'))
        contents = contents.replace('</html>',javascript+'</html>')

    (output/'index.html').write_text(contents)
    print(output/'index.html')


if __name__=='__main__':main()
