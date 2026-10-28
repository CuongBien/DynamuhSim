"""Read social_bundle/1 observations; privileged targets are explicit opt-ins."""
import json
from pathlib import Path
import numpy as np

class Episode:
    def __init__(self, path):
        self.root = Path(path).resolve()
        self.metadata = json.loads((self.root / 'bundle.json').read_text())
        assert self.metadata['schema'] == 'social_bundle/1'
        with np.load(self.locate('samples'), allow_pickle=False) as data:
            self.inputs = {key: data[key] for key in data.files}
        assert not any(word in key for key in self.inputs for word in ['human', 'persona', 'backend', 'future', 'world'])
        self.depth = np.load(self.locate('depth'), allow_pickle=False, mmap_mode='r')
        with np.load(self.locate('bev'), allow_pickle=False) as data:
            self.bev = data['occupancy']
            self.bev_index = {int(value): i for i, value in enumerate(data['depth_index'])}
        self.calibration = json.loads(self.locate('calibration').read_text())
        self.labels = None

    def locate(self, name):
        relative = Path(self.metadata['paths'][name])
        assert not relative.is_absolute()
        value = (self.root / relative).resolve()
        assert value.is_relative_to(self.root)
        return value

    def __len__(self):
        return len(self.inputs['anchor_ns'])

    def observation(self, index):
        assert 0 <= index < len(self)
        fields = {key: values[index] for key, values in self.inputs.items()}
        indices = fields['depth_index']
        return dict(observations=fields, depth_history=np.asarray(self.depth[indices]),
                    current_bev=self.bev[self.bev_index[int(indices[-1])]], calibration=self.calibration)

    def supervision(self, index):
        assert 0 <= index < len(self)
        if self.labels is None:
            with np.load(self.locate('labels'), allow_pickle=False) as data:
                self.labels = {key: data[key] for key in data.files}
        return {key: value if key == 'human_ids' else value[index] for key, value in self.labels.items()}

    def task_masks(self, index):
        with np.load(self.locate('task_masks'), allow_pickle=False) as data:
            return {key: data[key][index] for key in data.files}
