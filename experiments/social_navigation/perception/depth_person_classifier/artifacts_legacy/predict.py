"""NumPy-only sensor classifier. Experimental same-avatar pilot, no ROS side effects."""
import hashlib,json
from pathlib import Path
import numpy as np
from features import extract
from inference import numpy_logits
class Classifier:
 def __init__(self,folder=None):
  folder=Path(folder) if folder is not None else Path(__file__).resolve().parent;self.config=json.loads((folder/'selected_frozen.json').read_text());file=folder/'selected_model.npz';assert hashlib.sha256(file.read_bytes()).hexdigest()==self.config['model_sha256'];self.arrays=dict(np.load(file,allow_pickle=False))
 def predict(self,sample,scan_geometry):
  depth,lidar,mask=extract(sample,scan_geometry);data=dict(depth=depth[None,:],lidar=lidar[None,:],lidar_mask=np.array([mask]));logit=numpy_logits(data,self.arrays,self.config['hidden'],self.config['modality'])[0];score=float(1/(1+np.exp(-np.clip(logit,-80,80))));return dict(class_index=int(score>=self.config['threshold']),score=score,threshold=self.config['threshold'],experimental=True,model=self.config['candidate'])
