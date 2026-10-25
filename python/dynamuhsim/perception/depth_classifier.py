"""Frozen depth-only proposal classifier; no report, corpus, Torch or ROS access."""
import hashlib
import io
import numpy as np

DEPTH_NAMES = (["log_points", "span_x", "span_y", "span_z"] + [f"z_q{q}" for q in [0,10,25,50,75,90,100]]
               + [f"xy_radius_q{q}" for q in [25,50,75,95]] + ["std_major", "std_middle", "std_minor"]
               + [f"z_band_fraction_{i}" for i in range(6)] + [f"z_band_radius90_{i}" for i in range(6)])
Z_EDGES = np.array([.04,.3,.6,.9,1.2,1.5,1.8])


def depth_features(points_xyz):
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Expected nonempty finite N x 3 sensor points")
    center = np.median(points[:,:2], axis=0)
    local = points.copy();local[:,:2] -= center
    radius = np.linalg.norm(local[:,:2], axis=1)
    std = np.sqrt(np.maximum(np.linalg.eigvalsh(np.cov(points.T, bias=True)),0))[::-1] if len(points)>1 else np.zeros(3)
    band = np.clip(np.searchsorted(Z_EDGES, points[:,2], side="right")-1,0,5)
    fractions = np.bincount(band,minlength=6)/len(points)
    radii = [np.quantile(radius[band==i],.9) if np.any(band==i) else 0. for i in range(6)]
    return np.r_[np.log1p(len(points)),np.ptp(points,axis=0),np.quantile(points[:,2],[0,.1,.25,.5,.75,.9,1]),
                 np.quantile(radius,[.25,.5,.75,.95]),std,fractions,radii].astype(np.float32)


class FrozenDepthClassifier:
    """Classifies already extracted geometry proposals; this is not an image detector."""
    def __init__(self, bundle):
        config = bundle.manifest["classifier"]
        self.metadata = bundle.document(config["metadata"])
        frozen = self.metadata
        if (frozen.get("schema") != "depth_person_selected_model/1" or frozen["candidate"] != "depth_linear"
                or frozen["hidden"] != 0 or frozen["modality"] != "depth" or frozen["feature_names"]["depth"] != DEPTH_NAMES):
            raise ValueError("Expected the unchanged depth-linear pilot and feature order")
        model = bundle.read(config["model"])
        if hashlib.sha256(model).hexdigest() != frozen["model_sha256"]: raise ValueError("Model binding mismatch")
        source = bundle.read(config["source_freeze"])
        if hashlib.sha256(source).hexdigest() != frozen["source_freeze_sha256"]: raise ValueError("Source freeze binding mismatch")
        receipt = bundle.document(config["evaluation_receipt"])
        if receipt["selected_frozen_sha256"] != hashlib.sha256(bundle.read(config["metadata"])).hexdigest():
            raise ValueError("Historical evaluation receipt mismatch")
        for suffix, key in [("training.json", "training_sha256"), ("selected_model.pt", "torch_weights_sha256")]:
            records = [row for path,row in bundle.manifest["provenance_only"].items() if path.endswith("/"+suffix)]
            if len(records)!=1 or records[0]["sha256"]!=frozen[key]: raise ValueError("Provenance binding mismatch")
        with np.load(io.BytesIO(model), allow_pickle=False) as arrays:
            shapes = {"depth_mean":(30,),"depth_std":(30,),"lidar_mean":(8,),"lidar_std":(8,),"depth.0.weight":(1,30),"depth.0.bias":(1,)}
            if set(arrays.files)!=set(shapes): raise ValueError("Unknown model array schema")
            self._arrays = {}
            for name,shape in shapes.items():
                value=arrays[name]
                if value.shape!=shape or value.dtype!=np.float32 or not np.isfinite(value).all(): raise ValueError("Invalid model array: "+name)
                value=value.copy();value.setflags(write=False);self._arrays[name]=value
        if (self._arrays["depth_std"]<=0).any() or (self._arrays["lidar_std"]<=0).any(): raise ValueError("Invalid normalization scale")
        self._threshold = float(frozen["threshold"])
        if not np.isfinite(self._threshold) or not 0<self._threshold<1: raise ValueError("Invalid frozen threshold")
        self.model_sha256 = frozen["model_sha256"]
        self.evaluation_receipt_sha256 = bundle.manifest["files"][config["evaluation_receipt"]]["sha256"]

    @property
    def threshold(self): return self._threshold

    def logits(self, features):
        values=np.asarray(features)
        if values.ndim!=2 or values.shape[1]!=30 or values.dtype.kind!="f" or not np.isfinite(values).all():
            raise ValueError("Expected finite N x 30 floating sensor features")
        try:
            with np.errstate(over="raise",invalid="raise",divide="raise"):
                normalized=((values-self._arrays["depth_mean"])/self._arrays["depth_std"]).astype(np.float32)
                result=(normalized@self._arrays["depth.0.weight"].T+self._arrays["depth.0.bias"])[:,0]
        except FloatingPointError as exc:
            raise ValueError("Features overflow frozen inference") from exc
        if not np.isfinite(result).all(): raise ValueError("Non-finite model output")
        return result

    def predict(self, features):
        logits=self.logits(features)
        score=1/(1+np.exp(-np.clip(logits,-80,80)))
        return dict(score=score,prediction=(score>=self._threshold),threshold=self._threshold)

    def predict_points(self, proposals):
        rows=[depth_features(points) for points in proposals]
        return self.predict(np.stack(rows) if rows else np.empty((0,30),np.float32))
