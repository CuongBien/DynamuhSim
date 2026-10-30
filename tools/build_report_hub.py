#!/usr/bin/env python3
"""Build a report directory's home page, preserving the original demo and its assets."""
import argparse
import html
import json
from pathlib import Path
import re
from urllib.parse import quote

GROUPS = [
    ('detector', 'Detector người và khả năng tổng quát hóa'),
    ('handoff', 'Dữ liệu mẫu và bàn giao'),
    ('people', 'Hành vi người và nhóm'),
    ('navigation', 'Robot, đường đi và costmap'),
    ('runtime', 'Độ ổn định của mô phỏng và ghi dữ liệu'),
    ('archive', 'Bản nháp và báo cáo cũ'),
    ('other', 'Các trang khác'),
]
# Descriptions are navigation aids, not new acceptance or quality claims.
INFO = {
 'detector_handoff': ('handoff', 'Bàn giao công việc detector tiếp theo', 'Context đầy đủ, nhánh code, dữ liệu cần chuyển, kế hoạch và điểm dừng; có file tải xuống.'),
 'source_report_audit_v1': ('detector', 'Vì sao báo cáo gốc tốt nhưng replay kém?', 'Đối chiếu bài test tổng hợp, recordings và code trước/sau merge.'),
 'depth_generalization_v1': ('detector', 'So sánh detector cũ, 3D có prior và bỏ prior', 'Khung người trên depth, số liệu bỏ sót và chuẩn bị benchmark tổng quát hóa. Chưa train.'),
 'depth_person_detector_v2': ('detector', 'Xem detector khoanh người trực tiếp', 'Khung dự đoán, ID, điểm số và các ví dụ phát hiện/bỏ sót trên depth đã ghi.'),
 'depth_person_detector_v1': ('detector', 'Tích hợp detector thử nghiệm và kiểm tra ROS', 'Báo cáo tích hợp ban đầu; xem v2 để thấy khung người rõ hơn.'),
 'handoff_pilot': ('handoff', 'Pilot dữ liệu bàn giao', 'Cấu trúc dữ liệu, kiểm tra mẫu và giới hạn sử dụng cho huấn luyện.'),
 'group_cohesion_heading': ('people', 'Giảm quay liên tục và giữ nhóm', 'Đối chiếu hành vi nhóm đi bộ, video và lỗi còn lại.'),
 'group_goal_ablation': ('people', 'Kiểm tra vòng bao và mục tiêu nhóm', 'So sánh cấu hình trong giai đoạn chẩn đoán hành vi nhóm.'),
 'endpoint_humans': ('people', 'Chuyển động người và hội tụ ở cuối đường', 'So sánh hiện tượng quay liên tục và các lượt mô phỏng.'),
 'sim_goal_clock': ('people', 'Đồng hồ mục tiêu người', 'Kiểm tra thời gian mô phỏng dùng cho mục tiêu và chuyển động nhóm.'),
 'persistent_conversation': ('people', 'Nhóm đứng trò chuyện', 'Kiểm tra nhóm giữ vị trí hội thoại và các giới hạn đã ghi.'),
 'geometry_ab': ('navigation', 'Kích thước robot và inflation', 'Hai lượt chạy A/B, footprint, đường đi và costmap.'),
 'planner_review': ('navigation', 'Vì sao NavFn chọn đi vòng?', 'Đường toàn cục, global costmap và ảnh camera.'),
 'static_gap': ('navigation', 'Robot đi qua khe hẹp', 'Đối chiếu các lượt thành công/thất bại, video thật và costmap.'),
 'factory_regression': ('navigation', 'Kiểm tra lại trong nhà xưởng', 'Kết quả tới đích, recovery và giới hạn hội tụ.'),
 'encounter_regression': ('navigation', 'Gặp người gần và đi qua khe', 'Các lượt kiểm tra MPPI, biên hở và video.'),
 'bridge_shutdown': ('runtime', 'Đóng bridge khi kết thúc phiên', 'Kiểm tra lỗi shutdown và thứ tự đóng tiến trình.'),
 'readiness_atomic': ('runtime', 'Kiểm tra tín hiệu sẵn sàng', 'Báo cáo kiểm tra readiness của phiên mô phỏng.'),
 'seed_regression': ('runtime', 'Kiểm tra các seed 46–48', 'Kết quả theo từng lượt; giữ nguyên các lần FAIL/ERROR.'),
 'transport_order': ('runtime', 'Thứ tự ghi và đóng capture', 'Kiểm tra transport và dữ liệu ghi nhận.'),
 'seed_regression_progress': ('archive', 'Ảnh chụp tiến độ seed regression', 'Trang tiến độ cũ, không phản ánh một công việc đang chạy. Xem seed_regression để đối chiếu.'),
}

def build(root):
    root = root.resolve(strict=True)
    home = root / 'index.html'
    original = root / 'simulation_20261029.html'
    marker = '<!-- dynamuh-report-hub -->'
    if home.exists() and marker not in home.read_text(encoding='utf-8'):
        if original.exists():
            raise FileExistsError('Refusing to replace the preserved original home page')
        original.write_bytes(home.read_bytes())
    pages = []
    for path in sorted(root.rglob('*.html')):
        if path == home or not path.is_file() or not path.resolve().is_relative_to(root):
            continue
        rel = path.relative_to(root).as_posix()
        content = path.read_text(encoding='utf-8')
        match = re.search(r'<title[^>]*>(.*?)</title>', content, re.I | re.S)
        title = html.unescape(re.sub(r'\s+', ' ', match.group(1)).strip()) if match else rel
        folder = path.relative_to(root).parts[0]
        group, label, description = INFO.get(folder, ('other', title, 'Báo cáo được tự động tìm thấy trong thư mục localhost.'))
        if rel == original.name:
            group, label, description = 'navigation', 'Demo gốc: camera, depth, classifier và costmap', 'Giữ nguyên trang chính trước đây, gồm các mục footprint và video mô phỏng ngày 29/10.'
        elif '_draft' in folder or path.name != 'index.html':
            group, label, description = 'archive', title + ' — bản lưu', 'Bản nháp hoặc bản trước chỉnh sửa; giữ lại để truy vết, ưu tiên báo cáo chính ở trên.'
        pages.append(dict(path=rel, url=quote(rel, safe='/'), title=label, description=description, group=group))
    sections = []
    for key, label in GROUPS:
        items = [p for p in pages if p['group'] == key]
        if not items:
            continue
        cards = ''.join('<a class="card" href="'+html.escape(p['url'], quote=True)+'"><h3>'+html.escape(p['title'])+'</h3><p>'+html.escape(p['description'])+'</p><small>'+html.escape(p['path'])+'</small></a>' for p in items)
        sections.append('<section id="'+key+'"><h2>'+label+' <span>'+str(len(items))+'</span></h2><div class="grid">'+cards+'</div></section>')
    contents = r'''<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<!-- dynamuh-report-hub -->
<title>DynamuhSim — Tất cả báo cáo</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#101927;color:#e9f0fa}body{max-width:1200px;margin:auto;padding:32px 22px;line-height:1.6}h1{font-size:clamp(28px,5vw,42px);line-height:1.2;margin:12px 0}h2{font-size:23px;margin-top:36px}h2 span{font-size:14px;color:#acbdd1}p{color:#bac9dd}a{color:#96d5ff}nav{display:flex;gap:12px;flex-wrap:wrap;margin:24px 0}nav a{padding:7px 12px;border:1px solid #354b65;border-radius:20px;text-decoration:none}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,330px),1fr));gap:14px}.card{display:block;background:#1a293e;border:1px solid #314761;padding:18px;border-radius:12px;text-decoration:none;color:inherit}.card:hover,.card:focus-visible{border-color:#91d8ff;background:#21344e}.card h3{font-size:19px;margin:0 0 8px;color:#b5e1ff}.card p{margin:0 0 12px}.card small{color:#9bafc8;overflow-wrap:anywhere}.notice{border-left:4px solid #e3b66e;padding:12px 18px;background:#2d2b29;border-radius:6px}input{width:100%;box-sizing:border-box;padding:13px 16px;background:#16253a;color:#eff6ff;border:1px solid #5b7390;border-radius:8px;font:inherit}pre{overflow:auto;background:#17263a;padding:18px;border-radius:8px}code{color:#b6dfff;overflow-wrap:anywhere}.badge{color:#a8d6f6;font-size:14px}footer{margin-top:40px;color:#9eafc5}[hidden]{display:none!important}
</style><header><div class="badge">DynamuhSim · Thư viện báo cáo · 30/10/2026</div><h1>Tất cả báo cáo ở một nơi</h1><p>__COUNT__ trang đã lưu, gồm báo cáo chính, bản nháp và trang demo gốc. Chọn chủ đề hoặc tìm theo tên bên dưới.</p><p class="notice">Đây là các kết quả đã ghi, không phải mô phỏng đang chạy. Mỗi báo cáo giữ nguyên phạm vi và kết luận tại thời điểm thực hiện; PASS của một bước không có nghĩa toàn hệ thống đã đạt. Detector mới chưa được chấp nhận thay bản cũ, chưa train World Model/RL.</p></header>
<nav><a href="#detector">Detector người</a><a href="#handoff">Dữ liệu / bàn giao</a><a href="#people">Hành vi người</a><a href="#navigation">Điều hướng / costmap</a><a href="#runtime">Độ ổn định</a><a href="#archive">Bản lưu</a><a href="#howto">Cách mở lại</a></nav>
<label for="search">Tìm báo cáo</label><input id="search" type="search" placeholder="Ví dụ: detector, costmap, nhóm, seed…"><p id="counter" aria-live="polite"></p>
__SECTIONS__
<section id="howto"><h2>Báo cáo lưu ở đâu? Mở lại thế nào?</h2><p>Toàn bộ HTML, ảnh, video và số liệu được server phục vụ từ thư mục trên Ubuntu/WSL:</p><pre><code>__ROOT__</code></pre><p>Mở terminal Ubuntu rồi chạy:</p><pre><code>cd "__ROOT__"
python3 -m http.server 35315 --bind 127.0.0.1</code></pre><p>Sau đó mở <a href="http://localhost:35315/">http://localhost:35315/</a>. Giữ terminal mở để tiếp tục xem; Ctrl+C dừng server. Lệnh chỉ phục vụ báo cáo tĩnh, không mở Gazebo hoặc chạy lại benchmark. Nếu cổng đang được dùng bởi server này, chỉ cần mở link.</p><p>Thư mục <code>generated</code> hiện được Git bỏ qua: các media trên máy này không tự đi kèm khi clone GitHub. Code và receipts đã commit nằm riêng trong repository.</p><p><a href="hub_catalog.json">Danh mục đầy đủ (JSON)</a> · <a href="simulation_20261029.html">Trang chính gốc trước khi có mục lục</a></p></section>
<footer>Trang mục lục không sửa kết quả các báo cáo con. Các báo cáo phát sinh sau này sẽ được thêm khi chạy lại công cụ tạo mục lục.</footer>
<script>
const input=document.getElementById('search'),cards=[...document.querySelectorAll('.card')];
function filter(){const q=input.value.normalize('NFD').replace(/[\u0300-\u036f]/g,'').replace(/đ/g,'d').toLowerCase();let count=0;for(const c of cards){const text=c.textContent.normalize('NFD').replace(/[\u0300-\u036f]/g,'').replace(/đ/g,'d').toLowerCase();c.hidden=!text.includes(q);if(!c.hidden)count++;}for(const s of document.querySelectorAll('section:not(#howto)'))s.hidden=![...s.querySelectorAll('.card')].some(c=>!c.hidden);document.getElementById('counter').textContent=count+' / '+cards.length+' trang';}
input.addEventListener('input',filter);filter();
// Preserve previously shared links to sections of the original home page.
const legacy=['footprint','navigation','group-cohesion-heading','sim-goal-clock','encounter-regression','endpoint-humans','factory-regression','static-gap','planner-review','geometry-ab'];
if(legacy.includes(location.hash.slice(1)) && (location.hash!=='#navigation'||location.search.includes('footprint=')))location.replace('simulation_20261029.html'+location.search+location.hash);
</script></html>'''
    contents = contents.replace('__COUNT__', str(len(pages))).replace('__SECTIONS__', ''.join(sections)).replace('__ROOT__', html.escape(str(root)))
    home.write_text(contents, encoding='utf-8')
    (root/'hub_catalog.json').write_text(json.dumps({'root':str(root),'count':len(pages),'pages':pages},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'home':str(home),'pages':len(pages),'original_preserved':str(original)},ensure_ascii=False))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True, help='Existing static report server directory')
    build(parser.parse_args().root)
