"""Load unchanged frozen feature/inference code without global module-name collisions."""
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Classifier:
    def __init__(self, folder):
        folder = Path(folder)
        self.config = json.loads((folder / 'selected_frozen.json').read_text())
        weights = folder / 'selected_model.npz'
        if hashlib.sha256(weights.read_bytes()).hexdigest() != self.config['model_sha256']:
            raise ValueError('Frozen classifier weight checksum mismatch')
        with np.load(weights, allow_pickle=False) as arrays:
            self.arrays = dict(arrays)
        self.features = load_file('_depth_person_frozen_features', folder / 'features.py')
        self.inference = load_file('_depth_person_frozen_inference', folder / 'inference.py')

    def predict(self, sample, scan_geometry=None):
        depth, lidar, mask = self.features.extract(sample, scan_geometry)
        data = dict(depth=depth[None, :], lidar=lidar[None, :], lidar_mask=np.array([mask]))
        logit = self.inference.numpy_logits(data, self.arrays, self.config['hidden'], self.config['modality'])[0]
        score = float(1 / (1 + np.exp(-np.clip(logit, -80, 80))))
        return dict(score=score, threshold=self.config['threshold'], experimental=True)
