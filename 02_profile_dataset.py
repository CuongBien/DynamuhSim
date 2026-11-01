#!/usr/bin/env python3
"""Read-only, LLM-ready dataset profiler for DynamuhSim WM sensor/action corpus.

Features: metadata/splits/provenance, complete NPZ/NPY field headers, sampled sensor/action/
BEV/human distributions, integrity checks, charts and AI prompt. Does not read test
arrays or labels by default. Depth loaded by NumPy mmap, NOT copied into RAM.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
import os
from pathlib import Path
import zipfile
import numpy as np

DESCRIPTIONS = {
    "anchor_ns": "Thoi diem neo cua cua so, nanosecond simulation.",
    "history_query_ns": "20 query tu t-1.9s toi t, 10Hz; timestamp cua observation history.",
    "scan_ranges_m": "[N,20,360] LiDAR range (m) co mask; invalid ray da duoc ghi 0.",
    "scan_ray_valid": "Mask cho moi ray LiDAR; khong lay value=0 lam khoang cach that.",
    "imu": "[N,20,10]: quaternion xyzw + angular velocity xyz + linear acceleration xyz.",
    "odom_local_pose_and_twist": "[N,20,9]: local XY/yaw + linear xyz + angular xyz; estimated, khong phai simulator GT.",
    "action": "[N,20,3]: lenh da thuc hien trong history, vx/vy/wz (base axes).",
    "depth_index": "[N,20] indices tro vao depth_SENSOR_ONLY.npy; khong phai mang pixel.",
    "goal_xy_from_amcl": "[N,2] goal trong he robot tinh theo AMCL nhan qua, co goal_valid.",
    "goal_valid": "Mask goal con hop le; stale >1 giay -> zero + false.",
    "bev_anchor_estimated": "BEV hiện tại duoc suy tu depth + odometry nhan qua; khong dung GT.",
    "occupancy": "BEV -1 unknown, 0 observed free, 1 observed occupied; vai tro input/label phu thuoc file.",
    "frame_valid": "Mask bo du lieu BEV o muc frame/horizon.",
    "cell_loss_valid": "Mask cell observed va frame supported; chi tinh loss tai mask true.",
    "future_stamp_ns": "[N,30] moc thoi gian nhan quy dao nguoi tu t+0.1..t+3.0s.",
    "human_xy_future_m": "[N,30,6,2] quy dao 6 nguoi trong he robot that tai anchor, LABEL GT.",
    "human_xy_at_anchor_m": "[N,6,2] vi tri nguoi tai anchor, LABEL GT.",
    "human_valid": "Support noi suy GT, KHONG phai nhan da detect duoc nguoi.",
    "human_ids": "Danh sach ID con nguoi chi on dinh trong episode.",
    "executed_action_future": "[N,30,3] lenh robot da thuc hien t..t+2.9s; action condition tren mot trajectory, khong phai counterfactual.",
    "executed_action_query_ns": "[N,30] moc hoi cua 30 lenh tuong lai.",
    "executed_action_stamp_ns": "Timestamp cua lenh tuong lai da ghi.",
    "executed_action_available_ns": "Timestamp availability cua lenh da ghi.",
    "group_id_future": "Nhom nguoi tuong lai (LABEL simulator); khong phai observation.",
    "group_mode_future": "Nhom 0:none, 1:walking, 2:standing (LABEL).",
    "persona_index": "Danh muc persona; hien tai -1/unavailable, KHONG phai H1-H5 ground truth.",
    "persona_valid": "Mask persona; hien tai tat ca false, khong train head H1-H5.",
    "m1_track_aligned_valid": "Mask lien ket tracker-sensor voi ground truth ID; hien tai false.",
    "wm_observed_bev_train_valid": "Mask ho tro task WM observed future BEV, KHONG phai quyen train test.",
    "executed_action_condition_valid": "Mask action condition hop le.",
    "rl_reward_valid": "Mask reward RL; hien tai false.",
    "rl_reward": "Reward RL; hien tai NaN/unavailable; KHONG doi NaN thanh zero.",
    "offset_ns": "[3] horizon +0.5/+1/+3 giay cho future observed BEV.",
    "query_ns": "Moc thoi gian can lay nhan Depth tuong lai.",
    "depth_stamp_ns": "Timestamp Depth frame da su dung.",
    "depth_time_error_ns": "Sai so thoi gian cua target Depth frame.",
    "anchor_pose_world_gt": "Simulator ground-truth robot pose tai anchor, CHI nhan/evaluation.",
    "future_pose_world_gt": "Simulator ground-truth robot poses cho future depth target, CHI nhan/evaluation.",
    "bev_frame_valid": "Mask du lieu current BEV hop le.",
    "calibration": "Camera intrinsics/extrinsics va thong tin phep chieu depth.",
    "history": "Tap observation da sample mot cua so; KHONG phai human/world future label.",
}

FILE_DESCRIPTIONS = [
    ("samples_SENSOR_ONLY.npz", "Sensor observation windows (LiDAR, IMU, odom, cmd_vel, goal, frame indices and masks)."),
    ("observations_SENSOR_ONLY.npz", "Full timestamped sensor arrays retained for audits; not the recommended causal reader."),
    ("depth_SENSOR_ONLY.npy", "Raw float32 depth frames [F,240,424]; NaN/Inf retained; index via depth_index."),
    ("bev_anchor_estimated_SENSOR_ONLY.npz", "Causal input BEV from Depth and odometry, estimated anchor."),
    ("calibration_SENSOR_ONLY.json", "Camera K/P/D, optical/base frames, extrinsics."),
    ("lidar_geometry_SENSOR_ONLY.json", "LiDAR angular/range geometry and units."),
    ("future_trajectories_actions_LABEL_ONLY.npz", "Future simulator human GT trajectories + executed future action conditioning."),
    ("future_observed_bev_LABEL_ONLY.npz", "Future observed-depth occupancy labels + per-cell/per-frame masks."),
    ("human_states_LABEL_ONLY.jsonl", "Per-time human/robot simulator GT; NEVER online robot input."),
    ("task_masks_LABEL_ONLY.npz", "Validity flags for WM, association, persona and RL reward."),
    ("episode_end_LABEL_ONLY.json", "Navigation terminated/truncated metadata; not validated RL reward."),
    ("episode.json", "Canonical episode metadata: ID, split, scenario, seed, counts and outcome."),
    ("inventory.json", "File checksums and size evidence at episode level."),
    ("dataset.json", "Canonical dataset manifest and episode splits."),
    ("capture_protocol.json", "Collection protocol and scene/control assumptions."),
    ("export_protocol.json", "Schema/target export definitions."),
    ("quarantine_review.json", "Quarantine decisions for failed capture attempts."),
    ("capture_result.json", "Capture runtime and navigation result provenance."),
    ("capture_audit.json", "Checks of capture validity."),
    ("raw_export_audit.json", "Checks of raw MCAP to normalized arrays."),
]


def file_role(path: Path) -> str:
    ss = set(path.parts)
    if "inputs" in ss or "SENSOR_ONLY" in path.name:
        return "OBSERVATION_INPUT"
    if "labels" in ss or "LABEL_ONLY" in path.name:
        return "SUPERVISION_GT_ONLY"
    if "provenance" in ss:
        return "PROVENANCE_AUDIT"
    if path.name in {"dataset.json", "episode.json", "capture_protocol.json", "export_protocol.json"}:
        return "CANONICAL_METADATA"
    return "OTHER_OR_SUPPORT"


def npy_header(stream):
    from numpy.lib import format as fmt
    ver = fmt.read_magic(stream)
    readers = {(1, 0): fmt.read_array_header_1_0, (2, 0): fmt.read_array_header_2_0}
    if hasattr(fmt, "read_array_header_3_0"):
        readers[(3, 0)] = fmt.read_array_header_3_0
    if ver not in readers:
        raise ValueError("Unsupported NumPy NPY header version: " + str(ver))
    shape, fortran, dtype = readers[ver](stream)
    return list(shape), str(dtype), bool(fortran)


def describe_arrays(root: Path, include_test: bool):
    """Read only .npy headers: NPZ members opened as streams, not materialized tensors."""
    schema, errors = [], []
    for p in sorted(root.rglob("*")):
        if not p.is_file() or p.suffix.lower() not in {".npy", ".npz"}:
            continue
        rel = p.relative_to(root)
        split_test = any(s.startswith("wm_") and s.endswith("_49") for s in rel.parts) # overridden by caller below
        if not include_test and p.parent.name == "labels" and split_test:
            continue
        role = file_role(rel)
        try:
            if p.suffix == ".npy":
                with p.open("rb") as f:
                    shape, dtype, order = npy_header(f)
                schema.append({"file": rel.as_posix(), "role": role,
                               "key": "<npy-array>", "shape": shape, "dtype": dtype,
                               "fortran_order": order, "description": next((s for k,s in FILE_DESCRIPTIONS if k==p.name), "Array file")})
            else:
                with zipfile.ZipFile(p, "r") as z:
                    for member in z.infolist():
                        if member.is_dir() or not member.filename.endswith(".npy"):
                            continue
                        with z.open(member) as f:
                            shape, dtype, order = npy_header(f)
                        key = Path(member.filename).name[:-4]
                        description = DESCRIPTIONS.get(key)
                        if description is None and key.endswith(("_stamp_ns", "_available_ns", "_valid")):
                            description = "Timestamp (ns), available time (ns) hoac validity mask tuong ung voi modality."
                        schema.append({"file": rel.as_posix(), "role": role, "key": key,
                                       "shape": shape, "dtype": dtype, "fortran_order": order,
                                       "description": description or "Chua co data dictionary; can doi chieu protocol/source."})
        except Exception as ex:
            errors.append({"file": rel.as_posix(), "error": str(ex)})
    return schema, errors


def describe_numeric(values: np.ndarray):
    v = np.asarray(values).reshape(-1)
    n = v.size
    if n == 0:
        return {"count": 0}
    v = v.astype(np.float64, copy=False)
    good = np.isfinite(v)
    out = {"count": int(n), "nonfinite": int((~good).sum())}
    if good.any():
        q = np.percentile(v[good], [0, 5, 25, 50, 75, 95, 100])
        out.update(dict(zip(("min", "p05", "p25", "median", "p75", "p95", "max"), [round(float(x), 5) for x in q])))
        out["mean"] = round(float(v[good].mean()), 5)
    return out


def plot_if_available(out: Path, distributions: dict, epi_rows: list, bev_counter: np.ndarray):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return ["Matplotlib not found; charts skipped (install python3-matplotlib)."]
    produced = []
    def save_hist(key, label, unit, number):
        vals = distributions.get(key, [])
        if not vals:
            return
        fig, ax = plt.subplots(figsize=(8,4.5))
        ax.hist(vals, bins=45)
        ax.set(xlabel=unit, ylabel="Sample count", title=label)
        ax.grid(alpha=0.2)
        fig.tight_layout(); name=f"{number}_{key}.png"; fig.savefig(out/name,dpi=140);plt.close(fig);produced.append(name)
    if epi_rows:
        fig, ax = plt.subplots(figsize=(11,4.5))
        ax.bar([v["episode_id"] for v in epi_rows],[v.get("windows") or 0 for v in epi_rows])
        ax.tick_params(axis="x", labelrotation=40)
        ax.set(title="Windows per episode (not independent trials)",ylabel="Window count")
        fig.tight_layout();fig.savefig(out/"01_windows_by_episode.png",dpi=140);plt.close(fig);produced.append("01_windows_by_episode.png")
    for i, key, label, unit in [(2,"vx","Robot executed forward velocity","m/s"), (3,"vy","Robot executed lateral velocity","m/s"), (4,"wz","Robot angular velocity","rad/s"), (5,"lidar_min","Nearest valid LiDAR range","m"), (6,"depth_m","Sampled valid Depth pixel distance","m"), (7,"human_displacement_3s","Human GT 3-second displacement","m")]:
        save_hist(key, label, unit, f"{i:02}")
    totals = bev_counter.sum(axis=1)
    if np.any(totals > 0):
        p = bev_counter / np.maximum(totals[:,None],1)
        fig, ax = plt.subplots(figsize=(8,4.5)); bottom=np.zeros(3)
        for k, name in enumerate(["unknown", "free", "occupied"]):
            ax.bar(["+0.5s", "+1s", "+3s"],p[:,k],bottom=bottom,label=name);bottom+=p[:,k]
        ax.set(title="Future observed BEV class balance (sampled, non-test)",ylabel="Cell fraction",ylim=(0,1))
        ax.legend();fig.tight_layout();fig.savefig(out/"08_future_bev_balance.png",dpi=140);plt.close(fig);produced.append("08_future_bev_balance.png")
    return produced


def main():
    ap=argparse.ArgumentParser(description="LLM-ready detailed profiler for DynamuhSim wm_episode_batch/dataset01")
    ap.add_argument("--root", required=True, type=Path, help="Path ending in wm_episode_batch/generated/dataset01")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--sample-windows", type=int, default=120, help="Max evenly-spaced sample windows per non-test episode")
    ap.add_argument("--depth-frames", type=int, default=12, help="Max depth frames sampled per non-test episode")
    ap.add_argument("--include-test-labels", action="store_true", help="Explicitly UNSEAL test statistics (not recommended before final evaluation)")
    args=ap.parse_args()
    if args.sample_windows<1 or args.depth_frames<0:
        ap.error("--sample-windows must be >=1 and --depth-frames >=0")
    root=args.root.expanduser().resolve(strict=True);out=args.out.expanduser().resolve()
    if root == out or root in out.parents:
        ap.error("--out must be outside dataset root")
    if not (root/"dataset.json").is_file():
        ap.error(f"Cannot find dataset.json in {root}")
    out.mkdir(parents=True,exist_ok=True)
    dataset=json.loads((root/"dataset.json").read_text())
    epi=dataset.get("episodes",[])
    if not isinstance(epi,list):
        ap.error("dataset.json episodes must be a list")
    splits=Counter();scenarios=Counter();navstatuses=Counter()
    rows=[];notes=[];modalities=defaultdict(list)
    dist=defaultdict(list)
    bev_counter=np.zeros((3,3),dtype=np.int64)
    mask_totals=defaultdict(lambda:[0,0])
    causal_good=causal_total=0
    eligible_overall=0
    for e in epi:
        name=e.get("episode_id", "")
        split=e.get("split", "unknown")
        scenario=e.get("scenario", "standing" if "standing" in name else "walking" if "walking" in name else "unknown")
        windows=e.get("windows",0)
        status=e.get("navigation_status","unknown")
        row={"episode_id":name,"split":split,"scenario":scenario,"seed":e.get("seed"),
             "windows":windows,"wm_valid_windows":e.get("wm_observed_bev_eligible_windows"),
             "navigation_status":status,"actual_goal_reached":e.get("actual_goal_reached"),
             "capture_accepted":e.get("capture_accepted"),"sampled_windows":0}
        rows.append(row)
        splits[split]+=1;scenarios[scenario]+=1;navstatuses[status]+=1
        p=root/"episodes"/name
        if not p.is_dir():
            notes.append(f"MISSING episode directory: {p}")
            continue
        if split == "test" and not args.include_test_labels:
            notes.append(f"SEALED test episode {name}: metadata only; no input/label arrays sampled.")
            continue
        try:
            samples_file=p/"inputs/samples_SENSOR_ONLY.npz"
            with np.load(samples_file,allow_pickle=False) as sam:
                n=int(sam["anchor_ns"].shape[0]);idx=np.unique(np.linspace(0,n-1,min(n,args.sample_windows),dtype=np.int64))
                row["sampled_windows"]=len(idx)
                action=sam["action"][idx,-1,:]
                for key,j in [("vx",0),("vy",1),("wz",2)]:
                    dist[key].extend(action[:,j].astype(float).tolist())
                scan=sam["scan_ranges_m"][idx,-1,:]
                valid=sam["scan_ray_valid"][idx,-1,:]
                nearest=np.min(np.where(valid & np.isfinite(scan),scan,np.inf),axis=1)
                dist["lidar_min"].extend(nearest[np.isfinite(nearest)].astype(float).tolist())
                mask_totals["LiDAR ray valid" ][0]+=int(valid.sum())
                mask_totals["LiDAR ray valid" ][1]+=int(valid.size)
                depth_indexes=sam["depth_index"][idx,-1]
                for modality in ["scan","depth","imu","odom","action"]:
                    k=modality+"_valid"
                    if k in sam:
                        m=np.asarray(sam[k][idx])
                        mask_totals[modality+" history valid"][0]+=int(m.sum())
                        mask_totals[modality+" history valid"][1]+=int(m.size)
                        stamp_key=modality+"_stamp_ns";avail_key=modality+"_available_ns"
                        if stamp_key in sam and avail_key in sam:
                            query=sam["history_query_ns"][idx]
                            stamp=sam[stamp_key][idx]
                            avail=sam[avail_key][idx]
                            causal_good+=int(((stamp<=query)&(avail<=query)&(stamp<=avail)&m).sum())
                            causal_total+=int(m.sum())
                if "goal_valid" in sam:
                    g=np.asarray(sam["goal_valid"][idx])
                    mask_totals["goal_valid"][0]+=int(g.sum())
                    mask_totals["goal_valid"][1]+=int(g.size)
            # Depth: numpy memmap, sample ONLY handful of frames, downscale pixels
            if args.depth_frames:
                depth_file=p/"inputs/depth_SENSOR_ONLY.npy"
                depth=np.load(depth_file,mmap_mode="r",allow_pickle=False)
                ids=np.unique(depth_indexes.astype(int))
                sel=np.unique(np.linspace(0,len(ids)-1,min(len(ids),args.depth_frames),dtype=int))
                for ki in ids[sel]:
                    frame=np.asarray(depth[int(ki),::8,::8])
                    m=np.isfinite(frame)&(frame>=.28)&(frame<=5.0)
                    mask_totals["depth valid pixels (sampled)"][0]+=int(m.sum())
                    mask_totals["depth valid pixels (sampled)"][1]+=int(m.size)
                    dist["depth_m"].extend(frame[m].astype(float).tolist())
                del depth
            # BEV and validity masks: never treat unknown as free
            with np.load(p/"labels/task_masks_LABEL_ONLY.npz",allow_pickle=False) as masks:
                for k in masks.files:
                    if k.endswith("_valid"):
                        m=np.asarray(masks[k])
                        mask_totals[k][0]+=int(m.sum());mask_totals[k][1]+=int(m.size)
                eligible=np.asarray(masks["wm_observed_bev_train_valid"])[idx]
                eligible_overall+=int(np.asarray(masks["wm_observed_bev_train_valid"]).sum())
            with np.load(p/"labels/future_observed_bev_LABEL_ONLY.npz",allow_pickle=False) as bevs:
                a=np.asarray(bevs["occupancy"])[idx]
                frame_valid=np.asarray(bevs["frame_valid"])[idx]
                for j in range(min(3,a.shape[1])):
                    values=a[eligible & frame_valid[:,j],j].ravel().astype(np.int16)
                    bev_counter[j]+=np.bincount(values+1,minlength=3)[:3]
            # Human displacement in a frozen robot-anchor coordinate frame is GT motion summary, not measured reaction causality
            with np.load(p/"labels/future_trajectories_actions_LABEL_ONLY.npz",allow_pickle=False) as label:
                initial=np.asarray(label["human_xy_at_anchor_m"])[idx]
                future=np.asarray(label["human_xy_future_m"])[idx,-1]
                movement=np.linalg.norm(future-initial,axis=-1).reshape(-1)
                dist["human_displacement_3s"].extend(movement[np.isfinite(movement)].astype(float).tolist())
                for key in ["standing","walking"]:
                    if scenario==key:dist["human_displacement_3s_"+key].extend(movement[np.isfinite(movement)].astype(float).tolist())
            if row["sampled_windows"] > 0:
                a=np.asarray(action)
                near0=.03; wz0=.10
                row["sampled_action_fraction"]={
                    "stop_translation_below_0.03ms":round(float(np.mean(np.hypot(a[:,0],a[:,1])<near0)),4),
                    "forward_vx_gt_0.03ms":round(float(np.mean(a[:,0]>near0)),4),
                    "reverse_vx_lt_minus_0.03ms":round(float(np.mean(a[:,0]<-near0)),4),
                    "lateral_abs_vy_gt_0.03ms":round(float(np.mean(np.abs(a[:,1])>near0)),4),
                    "turn_abs_wz_gt_0.10rads":round(float(np.mean(np.abs(a[:,2])>wz0)),4),
                }
        except Exception as ex:
            notes.append(f"ERROR profiling {name}: {type(ex).__name__}: {ex}")

    # Field level catalog: read only array headers for input + labels, skip sealed test labels
    schemas=[];schema_errors=[]
    for p in sorted(root.rglob("*.npz")) + sorted(root.rglob("*.npy")):
        rel=p.relative_to(root)
        is_sealed=False
        if not args.include_test_labels and "episodes" in rel.parts:
            for r in rows:
                if r["episode_id"] in rel.parts and r["split"] == "test":
                    is_sealed=True;break
        if is_sealed:
            continue
        try:
            if p.suffix == ".npy":
                with p.open("rb") as fh:
                    sh,dt,fr=npy_header(fh)
                schemas.append({"file":rel.as_posix(),"role":file_role(rel),"key":"<npy-array>","shape":sh,"dtype":dt,"fortran_order":fr,"description":next((x for k,x in FILE_DESCRIPTIONS if k==p.name),"")})
            else:
                with zipfile.ZipFile(p) as z:
                    for member in z.infolist():
                        if member.is_dir() or not member.filename.endswith(".npy"):
                            continue
                        with z.open(member) as fh:
                            sh,dt,fr=npy_header(fh)
                        key=Path(member.filename).name[:-4]
                        info=DESCRIPTIONS.get(key)
                        if info is None and key.endswith(("_stamp_ns","_available_ns","_valid")):
                            info="Timestamp ns / availability ns / validity mask for the named modality."
                        schemas.append({"file":rel.as_posix(),"role":file_role(rel),"key":key,"shape":sh,"dtype":dt,"fortran_order":fr,"description":info or "Undocumented by this script; inspect source or protocol."})
        except Exception as ex:
            schema_errors.append({"file":rel.as_posix(),"error":str(ex)})

    # Aggregate unique patterns rather than repeat every episode's exact same field in prompt
    signatures={}
    for s in schemas:
        key=(s["role"],Path(s["file"]).name,s["key"])
        if key not in signatures:
            signatures[key]={"role":s["role"],"file_basename":key[1],"field":key[2],"dtype":s["dtype"],"example_shape":s["shape"],"description":s["description"],"episodes_with_field":0}
        signatures[key]["episodes_with_field"]+=1
    dictionary=sorted(signatures.values(),key=lambda x:(x["role"],x["file_basename"],x["field"]))
    distributions={key:describe_numeric(v) for key,v in sorted(dist.items())}
    fractions={k:{"valid":int(v[0]),"total":int(v[1]),"valid_fraction":round(v[0]/v[1],5) if v[1] else None} for k,v in sorted(mask_totals.items())}
    ratios=np.divide(bev_counter,np.maximum(bev_counter.sum(axis=1)[:,None],1))
    bev_stats={h:{k:{"cells":int(bev_counter[i,j]),"fraction":round(float(ratios[i,j]),5)} for j,k in enumerate(["unknown","free","occupied"])} for i,h in enumerate(["+0.5s","+1.0s","+3.0s"])}
    profile={
        "dataset_root":str(root),"dataset_metadata":dataset,
        "scope":"Non-test arrays sampled uniformly; test is sealed unless explicitly opted in.",
        "sample_windows_per_episode_max":args.sample_windows,"depth_frames_per_episode_max":args.depth_frames,
        "test_unsealed":args.include_test_labels,
        "episodes":rows,"split_episode_counts":dict(splits),"scenario_episode_counts":dict(scenarios),"navigation_status_counts":dict(navstatuses),
        "numeric_distributions_sampled":distributions,
        "mask_validity":fractions,
        "causal_timestamp_check_sampled":{"passed":causal_good,"eligible":causal_total,"fraction":(causal_good/causal_total if causal_total else None),"criterion":"stamp_ns <= history_query_ns and available_ns <= history_query_ns; only history valid==true"},
        "future_bev_class_balance_sampled":bev_stats,"supported_wm_windows_non_test":eligible_overall,
        "field_dictionary":dictionary,"array_schema_rows":len(schemas),"schema_errors":schema_errors,"warnings":notes,
        "strict_interpretation":{
            "observations":"history sensor/odometry/goal. Never feed human/world GT from LABEL into robot online observation.",
            "action_condition":"recorded FUTURE executed robot commands, not counterfactual interventions, not optimal actions.",
            "future_bev":"observed from future depth, not fully known world occupancy or model output.",
            "missing_labels":"M1 tracked associations, H1-H5 persona and RL reward are not validated in first batch.",
            "unknown_cells":"-1 means unknown, not empty; mask them in loss.",
            "test":"do not explore labels or tune models against test family before final protocol.",
            "statistical_unit":"8 episodes are independent trial units; overlapping windows are strongly correlated.",
        }
    }
    (out/"DATASET_CONTEXT.json").write_text(json.dumps(profile,ensure_ascii=False,indent=2,default=str)+"\n",encoding="utf-8")
    with (out/"FIELD_SCHEMA.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=["file","role","key","shape","dtype","fortran_order","description"]);w.writeheader()
        for item in schemas:w.writerow({**item,"shape":str(item["shape"])})
    with (out/"EPISODE_TABLE.csv").open("w",newline="",encoding="utf-8") as f:
        keys=sorted(set().union(*(r.keys() for r in rows))) if rows else ["episode_id"]
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
    charts=plot_if_available(out,dist,rows,bev_counter)

    # Detailed but compact LLM handoff; all complete per-file paths stay separately in inventory.
    md=["# DYNAMUHSIM WORLD MODEL — DATASET PROFILE FOR AI", "",
        "> Automatically generated from ACTUAL locally available dataset. Episode-level counts come from dataset.json; numerical distributions are SAMPLED and MUST NOT be quoted as full-dataset exhaustive measurements.","",
        f"**Root:** `{root}`",f"**Dataset schema:** `{dataset.get('schema','unknown')}`",f"**Dataset backend:** `{dataset.get('backend','unknown')}`",
        f"**Episode counts:** {len(rows)} (split: {dict(splits)}; scenario: {dict(scenarios)})",f"**Test labels inspected:** {args.include_test_labels}","",
        "## 1. What the dataset represents","",
        "- Real Gazebo-recorded episode traces. The robot is controlled by Nav2/MPPI; humans are simulated by a SocNavGym response worker.",
        "- Observation history: 20 queries at 10Hz, ~2 seconds; LiDAR native 5Hz may be held with age mask; Depth 240×424.",
        "- Candidate future action conditioning in this corpus: 30 *actually executed* robot actions at 10Hz (t..t+2.9s), **not** outcomes for alternative candidate actions.",
        "- Future human GT: 30 steps × 6 IDs × XY in true robot frame at anchor; human detection/tracker association is NOT validated.",
        "- Future observed BEV targets: [3,64,64] at +0.5/+1/+3.0s; 0.1m cells, x forward [0,6.4), y left [-3.2,3.2); -1 unknown, 0 observed free, 1 observed occupied.",
        "- Future BEV comes from recorded future Depth frames (LABEL) and simulator ego-GT transforms, NOT neural predictions or complete 3D world state.",
        "- H1–H5/persona and M1 sensor-track associations are NOT validated; reward is NaN and reward_valid=false; no completed PPO/World Model training.",
        "- The independent experiment unit is EPISODE, NOT overlapping sliding windows. Do not random-split windows.","",
        "## 2. Episode table (metadata; exact)","",
        "| Episode | Split | Scenario | Seed | Windows | WM-valid | Nav2 status | Goal reached | Profiled windows |",
        "|---|---|---|---:|---:|---:|---|---|---:|" ]
    for r in rows:
        md.append(f"| {r['episode_id']} | {r['split']} | {r['scenario']} | {r['seed']} | {r['windows']} | {r['wm_valid_windows']} | {r['navigation_status']} | {r['actual_goal_reached']} | {r['sampled_windows']} |")
    md += ["","## 3. Distribution statistics (sampled non-test; not full census)",""]
    for k,d in distributions.items():
        md.append(f"- **{k}**: {json.dumps(d,ensure_ascii=False)}")
    md += ["","### Mask validity (sampled for sensors; full per-episode masks where available)",""]
    for k,d in fractions.items():
        md.append(f"- **{k}**: {d['valid']}/{d['total']} = {d['valid_fraction']}")
    md += ["", "### Sampled future BEV cell balance", ""]
    for h,d in bev_stats.items():
        md.append(f"- {h}: "+", ".join(f"{k}={v['fraction']}" for k,v in d.items()))
    md += ["", "## 4. Data dictionary (unique file/field definitions)","",
           "| Role | File | Field | dtype | Example shape | Meaning |", "|---|---|---|---|---|---|"]
    for s in dictionary:
        meaning=s["description"].replace("|","/").replace("\n"," ")
        md.append(f"| {s['role']} | `{s['file_basename']}` | `{s['field']}` | `{s['dtype']}` | `{s['example_shape']}` | {meaning} |")
    md += ["", "## 5. File roles", ""]
    for file,desc in FILE_DESCRIPTIONS:
        md.append(f"- `{file}` — {desc}")
    md += ["", "## 6. Important quality / research caveats", "",
           "- `*_SENSOR_ONLY` is the inference-time branch. `*_LABEL_ONLY` contains privileged future/human GT and must NOT be concatenated into online observation features.",
           "- `human_valid` means simulator interpolation supported; not human seen in camera or associated to track ID.",
           "- `wm_observed_bev_train_valid` is task validity and does NOT override the train/validation/test split.",
           "- BEV `-1` is unknown, never free; calculate supervised loss only on `cell_loss_valid`.",
           "- One action sequence per recorded state is observational, not a controlled intervention/counterfactual set. Dataset does not prove robot-action causation of human response.",
           "- A high proportion of standing windows does not establish H4 persona; grouped standing members may sidestep and group lifecycle may change.",
           f"- {len(rows)} accepted episodes in the scanned dataset, a small batch from one route/simulator. Overlapping windows do not increase effective number of independent trials.",
           "- Test is held out by default. This script does not measure World Model prediction accuracy because no model predictions were provided.",
           "- Check provenance when extracting restored data; raw MCAP, compiled plugin and original experimental environment may be absent in a Git clone.",
           "", "## 7. Integrity signals (sampled)", "",
           f"- Causal timestamps valid: {causal_good}/{causal_total} eligible sampled modality-history timestamps.",
           f"- Non-test task-eligible WM windows (sum of explicit masks): {eligible_overall}",
           f"- Array schema field rows: {len(schemas)}; field header errors: {len(schema_errors)}", "",
           "## 8. Outputs & provenance", "",
           "- `DATASET_CONTEXT.json`: machine-readable dataset metadata, episodes, sample stats, full unique field dictionary.",
           "- `FIELD_SCHEMA.csv`: every array key + exact [shape,dtype] by file (excluding sealed test unless opted in).",
           "- `EPISODE_TABLE.csv`: exact canonical metadata episode table.",
           "- `ALL_PATHS.csv` and `DIRECTORY_TREE.txt`: produced by 01_inventory.py, not repeated inside this prompt.",
           "- PNG charts: " + (", ".join(charts) if charts else "none (matplotlib unavailable)"),""]
    if notes or schema_errors:
        md += ["## 9. Warnings / skipped items", ""]
        md.extend("- " + x for x in notes[:80]);md.extend("- " + str(x) for x in schema_errors[:30]);md.append("")
    (out/"DATASET_PROFILE.md").write_text("\n".join(md)+"\n",encoding="utf-8")
    prompt=["# PROMPT — Help me analyze this actual DynamuhSim dataset", "",
        "Read the ATTACHED `DATASET_PROFILE.md` (and optionally `DATASET_CONTEXT.json`, `FIELD_SCHEMA.csv`, `INVENTORY_SUMMARY.md`).", "",
        "1. Restate exactly what is available: sensors, time rates/horizons, coordinate frames, labels, masks, units, split, episodes and navigation outcomes.",
        "2. Distinguish what is directly VERIFIED from the dataset, what is SAMPLED and what is only a research hypothesis. Give precise numeric values only when supported by the attached tables.",
        "3. Judge data quality: missingness, imbalance, train/validation/test integrity, action coverage, moving vs standing human evidence, timestamp consistency and BEV unknown/free/occupied balance.",
        "4. Draw a rigorous boundary between SENSOR_ONLY (online allowed) and LABEL_ONLY (supervision only). DO NOT use simulator GT, test labels or future human trajectories in inference features.",
        "5. Say which tasks can be trained now (observed future BEV) and which cannot be validated yet (M1 tracked trajectories, M3 persona H1-H5, valid RL reward and counterfactual action-response).",
        "6. Recommend evidence-based next dataset experiments, evaluation metrics and ablations, including robot action diversity and paired interventions. Do not invent missing distributions, trained weights or success rates.",
        "7. Provide code only if requested, and use actual `FIELD_SCHEMA.csv` keys and shapes. Treat unknown BEV cells as unknown, not free.",
        "", "Only the analysis outputs are attached, NOT the full raw dataset; ask for a particular slice or raw episode if essential.", ""]
    (out/"PROMPT_FOR_AI.md").write_text("\n".join(prompt),encoding="utf-8")
    print(json.dumps({"root":str(root),"episodes":len(rows),"schema_fields":len(schemas),"unique_fields":len(dictionary),"warnings":len(notes)+len(schema_errors),"test_unsealed":args.include_test_labels,"charts":charts,"out":str(out)},ensure_ascii=False,indent=2))


if __name__=="__main__":main()
