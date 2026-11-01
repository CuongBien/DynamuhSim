# Experimental Warehouse human diversity checkpoint

Overall behavioral result: **PARTIAL**. This snapshot preserves existing Task 2/3 evidence; no measurement was regenerated for the Git checkpoint. `social_diversity_v1` remains opt-in; the human motion default is `legacy`. Nav2/MPPI configuration is unchanged. Task 2 intentionally changed the capture goal-clock default to `sim_v1` and enabled its motion contract in the maintained group fixture; these preserved fixes must not be confused with promotion of the Task 3 profile.

## Complete remote evidence

Host: `DESKTOP-NVIVQ2U`, MCP: `dynamuhsim-wsl / execute_bash`, `auto_source_ros=false`.

Complete root:

`/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree/experiments/social_navigation/benchmark/generated/human_behavior_diversity_20261101T073539_924830Z/`

[Complete remote manifest](../../../experiments/social_navigation/benchmark/generated/human_behavior_diversity_20261101T073539_924830Z/manifest.json) is available only on that remote worktree. `snapshot_inventory.json` records its SHA256, the source hashes and the original path/size/SHA256 of every copied file. All copies were checked against that finalized manifest and byte-compared through SHA256; no external symlinks are included. `manifest.initial.json` and the implementation reports retain historical pre-commit provenance, including statements that no commit had yet been made at experiment time.

**Full ignored raw evidence will NOT be transferred by `git pull`.** Raw MCAP, JSONL, runtime logs, native build binaries and the thousands of generated artifacts stay unchanged on the remote machine. Full controller/pose replay and saved-evidence tests require that remote evidence and the existing dependencies. No raw bag, virtual environment or vendor directory is included here.

## What this snapshot shows

- `comparison_final/metrics_before_after.json` contains the authoritative 300-step, seed45, crossing matched-input controller comparison. The trajectory/event PNG belongs to that comparison. The original CSV uses CRLF, which Git flags as trailing whitespace; its portable copy is omitted instead of altering original evidence. The original CSV remains unchanged remotely.
- `comparison_v2/replay_comparison.gif` is the original verified **2D recorded Gazebo pose playback**, 184 frames / 36.8 seconds, 2,748,351 bytes. SHA256: `346e75c13772f99febd0040fb207224f9ea53c641c6c71c891c4ef3429b16f38`. Every frame was decoded again before copying. It aligns the first active saved poses of different live robot trajectories; the shorter baseline is held at its final pose. It is qualitative, not a causal A/B controller comparison or camera footage.
- `replay/playback_verification_v2.json` preserves original keyframe reconstruction and media hashes. Absolute paths in original receipts refer to the complete remote evidence, not portable raw inputs.
- Per-run config, provenance, process/result and metrics JSON, component summaries, schema and controller receipts are unchanged original copies. Superseded comparison metrics are excluded; `CLEARANCE_UNITS.md` explains the corrected physical robot-radius units.

## Known regressions and acceptance limits

- Human-human physical surface clearance regresses from **0.355261 m to 0.029478 m** in the matched replay. Positive nominal-disc gaps do not establish safety or social comfort acceptance.
- All three native episodes are **INCOMPLETE**, external exit124 / runner ERROR. Successful cleanup and replay do not make them accepted navigation episodes.
- Native robot yielding is unverified (zero live transitions selected the robot as partner); overtaking completion is unverified. State labels are not maneuver success.
- The unchanged adapter executes piecewise-linear XY/yaw with interval-boundary velocity jumps. Continuous acceleration, full mesh/contact safety and general human naturalness remain unverified.
- One matched encounter/seed and bounded component fixtures do not validate general routing, native standing conversation, formation/dispersal or per-agent native goal expiry diversity.

## Exact replay / reproduction commands

Run remotely from the target worktree via MCP `execute_bash`, `auto_source_ros=false`. These are instructions only: the checkpoint task did not launch a new simulator, replay or benchmark. Choose absent output names; preserve existing raw files. See [Task 3 report](../../WAREHOUSE_HUMAN_BEHAVIOR_DIVERSITY_IMPLEMENTATION.md) section 18 for native capture commands and original limitations.

```bash
cd /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree
E="$PWD/experiments/social_navigation/benchmark/generated/human_behavior_diversity_20261101T073539_924830Z"
source /opt/ros/jazzy/setup.bash
source /home/cuongbien/DynamuhSim/install/setup.bash
# Native controller replay requires the existing dependency mappings.
# Only create an absent path; never overwrite a mapping or its external target.
test ! -e experiments/social_navigation/.venv-socnavgym && test ! -L experiments/social_navigation/.venv-socnavgym && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/.venv-socnavgym experiments/social_navigation/.venv-socnavgym
test ! -e experiments/social_navigation/vendor && test ! -L experiments/social_navigation/vendor && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/vendor experiments/social_navigation/vendor

# Outputs are outside the original evidence directory and must be new.
R="$PWD/experiments/social_navigation/benchmark/generated/warehouse_human_checkpoint_RECHECK_NEW"
mkdir "$R"
# Mode A: actual native controller, same recorded robot inputs.
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --output "$R/baseline.jsonl"
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --input-run "$E/candidate_v2/native" --output "$R/candidate_native.jsonl"
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --profile social_diversity_v1 --output "$R/candidate_matched.jsonl"

# Recompute authoritative comparison from saved controller outputs into new output.
mkdir "$R/comparison"
MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 python3 tools/warehouse_human_evidence.py --experiment "$E" --candidate-case candidate_v2 --comparison-dir "$R/comparison" --skip-render
# Mode B: saved-pose playback; existing per-run GIFs are reused and verified.
mkdir "$R/render"
MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 python3 tools/warehouse_human_evidence.py --experiment "$E" --candidate-case candidate_v2 --comparison-dir "$R/render" --verification-name "$R/playback_verification.json"

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python:tests DYNAMUHSIM_BEHAVIOR_EVIDENCE="$E" python3 -m unittest test_social_diversity test_warehouse_behavior_evidence test_goal_clock test_final_motion test_group_diagnostics test_group_projection_regressions test_group_orca test_group_intent test_group_cohesion test_group_cohesion_heading test_capture_runtime test_nav2_runner_integration test_capture_audit test_nav2_runtime test_encounters test_persistent_conversation -v
```

The checkpoint validation reran this focused suite: **124 tests PASS**, including all 7 retained-evidence tests; all 19 modified/new Python files compiled in memory; `git diff --check` passed. No C++ source changed and no C++ rebuild, training or long benchmark was run. This is an experimental WIP checkpoint for continued development and validation, not full human-naturalness acceptance.
