# Validation and deliverables

**FULL_PASS for acceptance criteria A–J.** Real clothed Quaternius characters were loaded, animated and displayed in MuJoCo, including a complete desktop GUI run at the default duration. Source clip coverage and foot grounding limitations are listed below.

## Source and build evidence

The official Google Drive listings contained `Smooth_Male_Casual.fbx` and `Smooth_Female_Casual.fbx`. Both normal Drive binary URL forms returned a quota HTML page during preparation. The actual files were obtained from the MRPT CC0 archives, verified as Kaydara FBX 7400 binaries, and imported using Blender 4.5.15 LTS. [Animated Men](https://quaternius.com/packs/animatedmen.html), [Animated Women](https://quaternius.com/packs/animatedwomen.html), [MRPT source listing](https://mvsimulator.readthedocs.io/en/latest/world_actors.html)

| File under `assets/source/` | SHA-256 |
| --- | --- |
| `Smooth_Male_Casual.fbx` | `e744bfbd0cc9df86e5b69848017c8554690aeb256cecd63e88f77d28cda9ec43` |
| `Smooth_Female_Casual.fbx` | `a47c3ef6bf71d1e1d61ae91864c0dd0fd7bc52667f948e64d08c38f3b255e4a6` |
| `AnimatedMen-License.txt`, `AnimatedWomen-License.txt` | `83d8959f9fc56353ed571fbe2dc52e4bcd64508e2399501cd45ac2ce3df0bf8c` |

Both licenses say CC0 1.0 Universal. Archive hashes and original Drive IDs are pinned in [assets/selection.json](assets/selection.json). Each source contains a 42-bone rig and eleven embedded actions. The imported idle, walk and run are sampled at 24 Hz. The normalizer found `-Y` facing in the Blender import, based on the foot-bone direction and rendered faces. The selected FBX files have no UV layers, image maps or vertex colors. Their skin, hair, eyes and clothes use original material slots. The generated male has six used material parts, and the female has eight.

A second Blender conversion produced byte-identical manifests and NPZ files. The generated male `character.npz` SHA-256 is `9916b80d6617c7d2a9dc08dc395239ea0d02743b887e52dd2993310863c28a7c`. The female hash is `10eca8deae057066c99f554cc65a598289f6d59e0364dc5301a302289f6d0e91`.

## Tests and replay

| Check | Observed result |
| --- | --- |
| Automated tests | **9 passed**, zero failed, skipped or blocked in the final suite. [pytest output](reports/pytest.txt) |
| Native skin validation | MuJoCo 4.0.0 compiled 14 skins and 84 mocap bones for the male/female pair. Native scene skin vertices changed by more than 5 cm at a fixed root pose while sampling the walk clip. The only non-skin geom is the preview floor. |
| Headless replay | **150.0 s**, 4,501 evaluated frames and all five stages. Six agents in the final stage. Wall time 12.00 s, update p50 1.53 ms, p95 4.93 ms and peak process RSS 81.7 MiB. [Headless JSON](reports/check-150s.json) |
| EGL replay and recording | **150.0 s**, 4,501 evaluated frames and 11 rendered PNGs at 800×600. [Record JSON](reports/record-150s.json) |
| Desktop smoke | 61 frames at 30 Hz over 2.0 simulated seconds, including six simultaneous actors. The window rendered and closed with process exit 0. [GUI smoke JSON](reports/gui-smoke.json) |
| Full desktop GUI | **150.0 s**, 4,501 rendered frames and process exit 0. Wall time 150.22 s, update p50 1.85 ms, p95 6.15 ms and peak RSS 142.2 MiB. [Full GUI JSON](reports/gui-150s.json) |

The headless, recorded and desktop GUI runs produced the same pose SHA-256, `1875e04090a8455ad6092617bdbe62dce174a45e63b5ecd210a2fa3c1a78b7bb`. The digest covers all published world bone positions and rotations in frame and actor order. The separate real-asset test checks position/yaw tracking, no jump at the start of a crossfade, loop continuity, normalized weights and rotations, and visible mesh deformation. Missing bundles are reported as `BLOCKED (assets)` for tests and exit code 2 in the CLI.

The first desktop implementation completed its frame loop but crashed during shutdown. The passive viewer render thread and GLFW termination ran concurrently. The final desktop path uses a single-threaded GLFW window with MuJoCo's `mjv_updateScene` and `mjr_render`, freeing the rendering context before destroying the window.

## Foot measurements

Each probe uses actual skinned mesh vertices within 8 cm of the bind floor with at least 20% influence from the matching foot or lower leg. Each frame computes the lowest probe vertex and the probe centroid using the source bone weights. Penetration is the largest negative minimum Z in the full replay.

A near-ground candidate requires consecutive lowest vertices at or below 4 cm, a moving root at least 0.2 m/s, and centroid vertical speed at most 0.12 m/s. The stance subset also requires the centroid to move backward faster than 0.05 m/s in the actor's heading-aligned frame. Sliding speed is the norm of consecutive centroid displacement **in world XY** divided by the simulation step. These are geometric indicators, with no contact solver in the preview. A separate broader near-ground p95 includes low swing poses.

- Maximum penetration was **15.5 mm** across the full replay.
- The **6,449** stance samples had median **0.056 m/s** and p95 **0.201 m/s** residual sliding.
- The broader **12,044** near-ground samples had p95 **2.854 m/s**.
- Walk reference speeds were measured by the median backward motion of near-floor stance candidates at native playback, resampled at 120 Hz. Male **1.57 m/s** (56 candidates), female **1.54 m/s** (55). Run references **4.43 m/s** and **4.39 m/s** use only seven and eight candidates and are provisional.

The source clips retain vertical bob and source gait. This implementation has no visual foot lock or ground IK. Source floor penetration and residual sliding remain observable.

## Offscreen performance

MuJoCo 4.0.0 with **llvmpipe (LLVM 20.1.2, 256 bits)**, 640×480 EGL. Each row is a separate process and 90 measured frames after 12 warmup frames. FPS includes animation, MuJoCo kinematics, scene update, rendering and RGB readback. Update latency excludes draw/readback and diagnostics. Peak RSS includes loading and the rendering context.

| People | FPS | Frame p95 | Update p50 | Update p95 | Peak RSS |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 111.1 | 11.82 ms | 0.92 ms | 2.22 ms | 262.3 MiB |
| 2 | 101.1 | 12.64 ms | 1.64 ms | 3.60 ms | 265.4 MiB |
| 5 | 69.1 | 19.68 ms | 3.64 ms | 7.52 ms | 274.4 MiB |
| 10 | 45.1 | 28.07 ms | 7.14 ms | 13.33 ms | 285.9 MiB |

[1 person](reports/benchmark-1.json), [2 people](reports/benchmark-2.json), [5 people](reports/benchmark-5.json), [10 people](reports/benchmark-10.json)

## Files

All paths are under `humans/`.

- Project and preparation files `.gitignore`, `README.md`, `VALIDATION.md`, `requirements.txt`, `requirements-dev.txt`, `pytest.ini`, `assets/selection.json`, `fetch_assets.py`, `prepare_assets.py`, `replay.json`.
- Runtime files `human_visual/__init__.py`, `human_visual/animation.py`, `human_visual/bundle.py`, `human_visual/mujoco_skin.py`, `human_visual/replay.py`, `human_visual/diagnostics.py`, `human_visual/demo.py`, `human_visual/window_view.py`, `human_visual/benchmark.py`.
- Tests `tests/test_human_visual.py`.
- Generated assets `assets/generated/casual_male/{manifest.json,character.npz}`, `assets/generated/casual_female/{manifest.json,character.npz}`. The four original source files are listed in the first table. Source and generated asset directories, `.venv/`, and work/cache files in `tmp/` are ignored locally.
- Measurement outputs `reports/pytest.txt`, `reports/check-150s.json`, `reports/record-150s.json`, `reports/gui-smoke.json`, `reports/gui-150s.json`, `reports/benchmark-1.json`, `reports/benchmark-2.json`, `reports/benchmark-5.json`, `reports/benchmark-10.json`.
- Actual MuJoCo renders in `reports/frames/` are `000000-straight.png`, `000450-straight.png`, `000900-speed_change.png`, `001350-speed_change.png`, `001800-turn.png`, `002250-turn.png`, `002700-start_stop_idle.png`, `003150-start_stop_idle.png`, `003600-ensemble.png`, `004050-ensemble.png`, `004500-ensemble.png`. [Two actors at 15 s](reports/frames/000450-straight.png), [Six actors at 135 s](reports/frames/004050-ensemble.png)

## Remaining limits

The official models are stylized low-poly meshes with authored materials. They have no PBR texture maps. Additional turn, slow walk, start, stop and backward source clips were absent from these two assets. Demo turns are externally authored headings over a real walk cycle. Slow walk changes cadence and start/stop crossfade real clips. No rig-to-rig retarget was demonstrated. The GLB/glTF import path and textured material path were not exercised with these FBX sources.

`git --no-optional-locks status --porcelain --untracked-files=all -- . ':!humans'` matched the pre-task status exactly on `dev/mujoco`. The existing root changes predate this task. No Git staging, commits or branch changes were made.
