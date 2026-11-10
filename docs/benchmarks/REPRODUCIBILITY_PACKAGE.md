# Gói tái lập và manifest phát hành Human Navigation

**Phiên bản đề xuất · `human-nav-benchmark-v1.0.0`.** Manifest phát hành tham chiếu snapshot sạch `codex/task-08d-snapshot` ở `0d86e376fff8ec54c6c9dd3b08330e283b2ad042`, source digest `6cb053a26ec7d0bd8c55465532b9fc016c624b36b258f5590caee2aa34efdf7a`, binding 08B và ba manifest Phase 0/1/2. Bundle mỏng chứa commit nghiên cứu và yêu cầu có commit nền `b50d0844904e7b90cc6371dafbf601878bfa960a`. Artifact raw giữ tại run gốc. Freeze khóa danh tính qua checksum và verifier đọc-only. [Release manifest](../../runs/task_08f/release_manifest.json) · [Git bundle](../../runs/task_08f/research_source.bundle) · [Bundle verification](../../runs/task_08f/bundle_verify.log)

## Những tệp cần bàn giao

Manifest liệt kê đường dẫn tương đối từ workspace root. `artifact_checksums.sha256` khóa raw, snapshot source/config/model/asset, tài liệu, bảng thống kê, công cụ, native archive và replay. SHA-256 của file checksum nằm trong release manifest. File checksum rời của manifest là điểm đối chiếu ngoài chuỗi tham chiếu này. File lock chạy và log của chính lần xác minh được giữ ngoài danh sách artifact nội dung. [Checksum toàn bộ gói](../../runs/task_08f/artifact_checksums.sha256) · [Checksum release manifest](../../runs/task_08f/release_manifest.sha256)

| Đầu ra | Tệp chuẩn |
| --- | --- |
| Final QA và coverage | [episode_audit.csv](../../runs/task_08e2/report/episode_audit.csv), [coverage_breakdown.csv](../../runs/task_08e2/report/coverage_breakdown.csv) |
| Inventory và nguồn gốc attempt | [episode_inventory.csv](../../runs/task_08f/episode_inventory.csv), [raw attempt directories](../../runs/task_08d/benchmark_study/attempts/) |
| Backend, pair và action | [backend_comparison.csv](../../runs/task_08f/backend_comparison.csv), [paired_backend_audit.csv](../../runs/task_08e2/report/paired_backend_audit.csv), [paired_backend_summary.csv](../../runs/task_08e2/report/paired_backend_summary.csv), [action_contrasts.csv](../../runs/task_08e1/report/action_contrasts.csv) |
| Failure, thống kê và hạn chế | [failure_catalog.csv](../../runs/task_08f/failure_catalog.csv), [statistical_summary.json](../../runs/task_08f/statistical_summary.json), [limitations.json](../../runs/task_08f/limitations.json) |

Từ workspace root, chạy verifier bằng Python ≥3.11. Script kiểm hash file, schema phát hành, phase/card/seed, inventory, 900 execution key, provenance, config/asset, tập ghép và replay. Khi file thiếu hoặc lệch hash, exit 2 và in lỗi. Khi thành công, exit 0 và in SHA-256 của manifest. [Verifier](../../scripts/benchmarks/verify_release.py)

```bash
repo="$PWD"
out="$repo/runs/task_08f"
sha256sum -c "$out/release_manifest.sha256"
python3 -B scripts/benchmarks/verify_release.py "$out/release_manifest.json" --root "$repo"
```

## Runtime được xác minh

| Phần | Khóa |
| --- | --- |
| Nền tảng | Ubuntu 24.04.5 LTS x86_64, glibc 2.39, CPython 3.12.3, MuJoCo 4.0.0 |
| JuPedSim | `1.4.2+task04u`, nguồn commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`, extension SHA-256 `5ebdd4593786734392fefa0c3748f4431d3de53fb857614f6a36a8919b23ec65` |
| RVO2 | `pyrvo2==0.0.0`, nguồn `c2c46ba8d59556aa10faf03479293236efea154d`, extension SHA-256 `af2aac265a6fee5bd01fdb1dbeb14df557b3f9c36a43aff0ee0deb9b9012ccf3` |
| Build | CMake 3.28.3, GCC/G++ 13.3.0. RVO2 có pin Cython 3.1.4 và setuptools 80.9.0. |
| Dependency và metric | `uv.lock` snapshot, package version trong run lock, implementation metrics và KTC theo hash nguồn ở manifest |

Gói giữ [archive nguồn JuPedSim](../../runs/task_08f/native/jupedsim-1.4.2.tar.gz), [wheel 04U đã dùng](../../runs/task_08b/jupedsim_04u/jupedsim-1.4.2+task04u-cp312-cp312-linux_x86_64.whl), [nguồn RVO2 pin](../../runs/task_08f/native/Python-RVO2-source.tar.gz) và [RVO2 binary kèm metadata cài đặt](../../runs/task_08f/native/pyrvo2-cp312-installed.tar.gz). Source RVO2 được so byte giữa gitlink pin và input dùng để build, gồm CMake, Cython, C++ và các ví dụ trong source build. Binary nghiên cứu ban đầu nạp từ thư mục tạm. Archive phát hành giữ chính binary và metadata đó. JuPedSim stock thiếu external actor/rebase 04U. Dùng wheel 04U hoặc recipe rebuild với hai patch và hash đã khóa. [04U](../reports/human_models/TASK_04U_REPORT.md) · [Lockfile snapshot](../../runs/task_08d/checkout/uv.lock) · [Provenance build 08B](../../runs/task_08b/jupedsim_04u/provenance.log)

### Khôi phục vào môi trường riêng

Tạo một checkout riêng chứa commit nền trước khi fetch bundle. Lệnh sau minh họa cài đặt từ workspace có gói. Đường dẫn `restore` mới phải nằm ngoài raw run. Kiểm tra gói trước khi giải archive.

```bash
repo="$PWD"
out="$repo/runs/task_08f"
restore="$(mktemp -d /tmp/narrownav_restore.XXXXXX)"
git clone --no-checkout "$repo" "$restore/source"
git -C "$restore/source" fetch "$out/research_source.bundle" refs/heads/codex/task-08d-snapshot
git -C "$restore/source" checkout -b codex/task-08d-snapshot FETCH_HEAD
cd "$restore/source"
uv venv --python 3.12.3 --seed .venv
uv sync --locked --extra dev --extra sim --extra human-all
.venv/bin/python -m venv --without-pip "$restore/native_venv"
probe="$restore/native_venv/bin/python"
base_site="$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
probe_site="$("$probe" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
tar -xzf "$out/native/pyrvo2-cp312-installed.tar.gz" -C "$probe_site"
printf '%s\n' "$base_site" > "$probe_site/locked-deps.pth"
.venv/bin/python -m pip --python "$probe" install --no-index --no-deps --ignore-installed --no-compile \
  "$repo"/runs/task_08b/jupedsim_04u/*.whl
env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$PWD/src" "$probe" -B -m scripts.benchmarks.plan dry-run \
  --manifest "$repo/runs/task_08d/benchmark_study/phase_2.json"
```

Trong nghiệm thu 08F, checkout được phục hồi từ bundle, hai binary được cài vào venv riêng với các dependency khóa đã có trên máy. Dry-run đạt 900/900 và trường runtime khớp ngoài đường dẫn interpreter mới. Chuỗi lệnh giải dependency từ đầu trên máy khác vẫn cần đối chiếu lockfile, package versions, ABI và native hash. Khi cần rebuild từ nguồn, dùng `bash scripts/diagnostics/build_jupedsim_04u.sh "$out/native/jupedsim-1.4.2.tar.gz" /path/to/new_build` từ checkout có `.venv`. RVO2 có thể build lại từ archive nguồn pin bằng toolchain đã ghi. Build lại có thể cho binary hash khác. Mọi resume nghiên cứu yêu cầu toàn bộ run lock khớp, gồm đường dẫn interpreter. [Log kiểm chứng môi trường phục hồi](../../runs/task_08f/native_restore.log) · [Hướng dẫn build RVO2](../setup/HUMAN_MODELS_ENV.md#cài-theo-lockfile)

## Replay và kiểm thử

Replay dùng interpreter gốc, cùng provenance và output mới. Chín case phủ bốn backend, sáu Card, một cặp FA Advance/Yield chung seed và các ca contact, overlap-only, guard, thiếu tiến độ và đạt an toàn. Giữ phase gốc từng slot. So byte config, intent, trace, contact và validity. Với GT/action, chỉ bỏ episode ID mới. Khi so manifest, bỏ run/episode ID và wall-clock duration. Phần còn lại khớp chính xác, không nới dung sai. [Danh sách và kết quả từng case](../../runs/task_08f/replay/replay_results.json) · [Log replay](../../runs/task_08f/replay.log)

```bash
repo=/home/haduckien/nav_ws
out="$repo/runs/task_08f"
checkout="$repo/runs/task_08d/checkout"
P="$repo/runs/task_08b/jupedsim_04u/venv/bin/python"
cd "$checkout"
headless() { env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src:$checkout" "$P" -B "$@"; }
headless "$out/replay.py" --source-run "$repo/runs/task_08d/benchmark_study" --output-dir "$out/replay_check"
headless -m pytest -q -rs --basetemp "$out/test_fixtures" \
  tests/unit/test_benchmark_08a.py tests/unit/test_module_dependencies.py tests/unit/test_seed.py \
  tests/integration/test_orca_runtime.py tests/integration/test_jupedsim_universal.py \
  tests/integration/test_simulation_contract.py tests/simulation/test_collision_monitor.py
```

Entry gate, audit 900 lượt, reporting regression, replay 9/9, restore dry-run và pytest đều exit 0. Pytest đạt 46 test, hai skip cho stock/04S. Báo cáo 08E-2 lưu thêm full-suite 251 pass, bốn skip trên cùng snapshot. [Entry gate](../../runs/task_08f/entry_gate.log) · [Audit](../../runs/task_08f/audit.log) · [Pytest](../../runs/task_08f/pytest.log)
