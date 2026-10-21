# Human-response replay, separate from navigation

[Measured run01 report](reports/run01/REPLAY_REPORT.md) contains6 CrowdSimPlus probes and18 SocNavGym group ablations. These are offline native simulator runs using prescribed robot inputs. The diagram GIFs are generated from recorded simulator states and labelled **OFFLINE REPLAY / NOT GAZEBO CAMERA**. They are not AI images or live camera recordings.

Run01 component commands were executed in sequence and source/input hashes recorded before each simulator stage. The convenience run.py wrapper was added afterward; it has not been used to replace any run01 outcome. Reproduce into a NEW directory:

```bash
python3 experiments/social_navigation/benchmark/human_response_replay/run.py \
  --report-dir experiments/social_navigation/benchmark/human_response_replay/reports/my_run
```

Requires the existing pinned CrowdSimPlus Docker image, SocNavGym Python3.12 venv, host Matplotlib/Pillow. No system packages, Python environments, vendor files, ROS robot source or baseline Nav2 config are changed. Native calculations run on CPU, at most2 CPUs/2GB for CrowdSimPlus and2-core affinity for group probes; they are sequential. This is not a CUDA workload. Gazebo rendering is tested separately on NVIDIA.

The wrapper reuses the exact committed inputs/configs, rather than regenerating a different scene. The three robot traces are far stationary(26,0), head-on(-27+0.5t,0), and crossing(-24,-2+0.4*min(t,10)) with a stop at10s. Samples0..20s at0.1s,201 inputs for200 updates; world-frame velocity matches segment derivatives. Robot/human initial overlaps and robot route/static geometry are checked before running. Input type is explicitly scripted, not Gazebo ground truth.

CrowdSimPlus: same individual fixture/private endpoints for ORCA+/SFM, native act/constrain/step retained.200 updates are compared with full upstream env.step human integration; both sides get the same external post-step patrol switches. Exact repeated rollout validates determinism. This does not replay rewards/episode logic or robot planning.

SocNavGym: one group3, walking/conversation × native/circle_contact/robot_orca × same3 traces. Geometry-only control separates contact-shape changes from adding the robot agent to ORCA. Native vendor unchanged. Group scenes differ from independent CrowdSimPlus endpoints, so do not rank frameworks using the resulting gaps. Fixed membership is a fixture, not formation/dispersal. Conversation has standing positions/inward headings, not spoken dialogue or a proven social response.

Measured limitations: native walking group can overlap internally even in far control. Robot-aware walking avoids the robot in these two near-input probes but retains member overlap in crossing. Conversation stays fixed even when a prescribed robot trace passes through it; a closed-loop Nav2 robot may instead go around. Signed footprint/proxy gaps and all outcomes remain in the report.

`summary.json`, `results.csv`, both protocols, raw trajectory/result files, `manifest.json`, `fixture_checks.json` and `validation.json` retain source hashes and checks. Gymnasium reset observation-space warning is preserved; this kernel path does not benchmark RL observations/reward/training. `validate.py` preserves the local original Git index/outside-worktree checkpoint created for run01; on another checkout, provide a fresh audit checkpoint before running rather than changing historical evidence.

Live Gazebo camera captures are under the separate [group demonstration](../group_gazebo_demo/README.md) folder; GUI and Nav2 startup outcomes must be reported separately from replay.
