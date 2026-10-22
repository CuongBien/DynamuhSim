"""NumPy-only episode reader. Simulator GT is opt-in supervision, never default observations."""
import json
from pathlib import Path
import numpy as np

class Episode:
    def __init__(self,path):
        self.root=Path(path).resolve();self.metadata=json.loads((self.root/'bundle.json').read_text())
        assert self.metadata['schema']=='social_bundle/1'
        def locate(name):
            relative=Path(self.metadata['paths'][name]);assert not relative.is_absolute()
            value=(self.root/relative).resolve();assert value.is_relative_to(self.root);return value
        self.locate=locate
        with np.load(locate('samples'),allow_pickle=False) as a:self.inputs={k:a[k] for k in a.files}
        forbidden=['human','persona','backend','future','world']
        assert not any(any(word in key for word in forbidden) for key in self.inputs)
        self.depth=np.load(locate('depth'),allow_pickle=False,mmap_mode='r')
        with np.load(locate('bev'),allow_pickle=False) as a:
            self.bev=a['occupancy'];self.bev_index={int(v):i for i,v in enumerate(a['depth_index'])}
        self.calibration=json.loads(locate('calibration').read_text());self.labels=None
    def __len__(self):return len(self.inputs['anchor_ns'])
    def observation(self,index):
        assert 0<=index<len(self)
        fields={k:a[index] for k,a in self.inputs.items()};depth_indices=fields['depth_index']
        # Keep original NaN/Inf and explicit history validity. Consumers must use masks.
        return dict(observations=fields,depth_history=np.asarray(self.depth[depth_indices]),
                    current_bev=self.bev[self.bev_index[int(depth_indices[-1])]],calibration=self.calibration)
    def supervision(self,index):
        assert 0<=index<len(self)
        if self.labels is None:
            with np.load(self.locate('labels'),allow_pickle=False) as a:self.labels={k:a[k] for k in a.files}
        return {k:(v if k=='human_ids' else v[index]) for k,v in self.labels.items()}
    def task_masks(self,index):
        with np.load(self.locate('task_masks'),allow_pickle=False) as a:return {k:a[k][index] for k in a.files}
