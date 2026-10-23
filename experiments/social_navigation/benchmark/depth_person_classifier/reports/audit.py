#!/usr/bin/env python3
"""Independent counts, model selection and portable post-freeze replay. No retuning."""
import hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent.parent;REPO=HERE.parents[3];RUN=HERE/'reports'
sys.path.insert(0,str(HERE/'artifacts'))
from predict import Classifier
from features import DEPTH_NAMES,LIDAR_NAMES
sys.path.insert(0,str(HERE))
from integrity import verify,sha
from data import load_split,training_weights,CORPUS,loader

def counts(y,scores,t):
 p=np.array(scores)>=t;y=np.asarray(y);tp=int(np.sum((y==1)&p));fn=int(np.sum((y==1)&~p));fp=int(np.sum((y==0)&p));tn=int(np.sum((y==0)&~p));rec=tp/(tp+fn);fpr=fp/(fp+tn);precision=tp/(tp+fp) if tp+fp else 0.;return dict(samples=len(y),tp=tp,fn=fn,fp=fp,tn=tn,recall=rec,fpr=fpr,precision=precision,balanced_accuracy=(rec+1-fpr)/2,accuracy=(tp+tn)/len(y),f1=2*precision*rec/(precision+rec) if precision+rec else 0.)
def independent(data,score,t):
 valid=data['label_valid'];overall=counts(data['y'][valid],score[valid],t);scenes={}
 for sid in sorted({v['scene_id'] for v in data['rows']}):
  keep=valid&np.array([v['scene_id']==sid for v in data['rows']]);scenes[sid]=counts(data['y'][keep],score[keep],t)
 return dict(overall=overall,by_scene=scenes,min_scene_balanced_accuracy=min(v['balanced_accuracy'] for v in scenes.values()),macro_scene_balanced_accuracy=float(np.mean([v['balanced_accuracy'] for v in scenes.values()])))
def passed(m,cfg):
 a=m['overall'];return bool(a['recall']>=cfg['global_recall_min'] and a['fpr']<=cfg['global_fpr_max'] and a['balanced_accuracy']>=cfg['global_balanced_accuracy_min'] and all(v['recall']>=cfg['per_scene_recall_min'] and v['fpr']<=cfg['per_scene_fpr_max'] for v in m['by_scene'].values()))
def check_metrics(actual,expected):
 for k,v in expected.items():
  if isinstance(v,dict):check_metrics(actual[k],v)
  else:assert abs(actual[k]-v)<1e-12,(k,actual[k],v)
def main():
 verify();cfg=json.loads((HERE/'protocol.json').read_text());training=json.loads((RUN/'training.json').read_text());frozen=json.loads((RUN/'selected_frozen.json').read_text());test=json.loads((RUN/'test_result.json').read_text());assert frozen['source_freeze_sha256']==sha(HERE/'frozen.json');assert frozen['training_sha256']==sha(RUN/'training.json');assert frozen['model_sha256']==sha(RUN/'selected_model.npz')==sha(HERE/'artifacts/selected_model.npz');assert sha(HERE/'features.py')==sha(HERE/'artifacts/features.py');assert sha(HERE/'inference.py')==sha(HERE/'artifacts/inference.py');assert test['selected_frozen_sha256']==sha(RUN/'selected_frozen.json');assert test['model_evaluations_on_test']==1 and not test['threshold_tuned_on_test'];assert not training['test_opened'];assert json.loads((RUN/'test_opened.json').read_text())['selected_frozen_sha256']==sha(RUN/'selected_frozen.json')
 train=load_split('train');validation=load_split('validation');weights=np.load(RUN/'train_weights.npy');np.testing.assert_array_equal(weights,training_weights(train));arrays=dict(np.load(HERE/'artifacts/selected_model.npz',allow_pickle=False));valid=train['label_valid'];local=valid&train['lidar_mask']
 for kind,keep in [('depth',valid),('lidar',local)]:
  np.testing.assert_array_equal(arrays[kind+'_mean'],train[kind][keep].mean(0));np.testing.assert_array_equal(arrays[kind+'_std'],np.maximum(train[kind][keep].std(0),1e-3))
 assert arrays['depth.0.weight'].shape==(1,30);assert len(DEPTH_NAMES)==30 and len(LIDAR_NAMES)==8
 # Recompute every predeclared validation threshold count and selection tuple independently.
 rankings=[]
 for order,candidate in enumerate(training['candidates']):
  probability=np.load(RUN/(candidate['name']+'_validation_probabilities.npy'));assert candidate['parameter_device']=='cuda:0' and candidate['gradient_device']=='cuda:0' and candidate['parameter_change_l2']>0
  for t in cfg['thresholds']:
   m=independent(validation,probability,t);stored=next(v for v in training['threshold_grid_scores'] if v['candidate']==candidate['name'] and v['threshold']==t);check_metrics(stored['metrics'],m);a=m['overall'];key=(int(passed(m,cfg['quality_gate'])),m['min_scene_balanced_accuracy'],m['macro_scene_balanced_accuracy'],a['balanced_accuracy'],-a['fpr'],a['recall'],-candidate['parameters'],-abs(t-.5),-order);np.testing.assert_array_equal(key,stored['rank']);rankings.append((key,candidate['name'],t))
 _,winner,threshold=max(rankings,key=lambda x:x[0]);assert winner==frozen['candidate'] and threshold==frozen['threshold'];assert sha(RUN/(winner+'.npz'))==frozen['model_sha256']
 classifier=Classifier();replay_rows=0;maxerror=0.;test_rows=json.loads((RUN/'test_rows_LABEL_REVIEW_ONLY.json').read_text());test_cache=np.load(RUN/'test_predictions_LABEL_REVIEW_ONLY.npz');data_test=dict(y=test_cache['y'],label_valid=test_cache['label_valid'],rows=test_rows);m=independent(data_test,test_cache['score'],threshold);check_metrics(test['test_metrics'],m);assert passed(m,cfg['quality_gate'])==test['pilot_quality_gate'];assert np.array_equal(test_cache['prediction'],test_cache['score']>=threshold)
 for split,data in [('train',train),('validation',validation),('test_integrity_replay',data_test)]:
  reference=np.load(RUN/(winner+'_'+split+'_probabilities.npy')) if split!='test_integrity_replay' else test_cache['score']
  for i,row in enumerate(data['rows']):
   sid=row['scene_id'];sample=loader.load_input(CORPUS,sid,row['sample_index'],allow_test=split=='test_integrity_replay');geometry=json.loads((CORPUS/'inputs'/sid/'lidar_geometry_SENSOR_ONLY.json').read_text());result=classifier.predict(sample,geometry);error=abs(result['score']-float(reference[i]));maxerror=max(maxerror,error);assert error<1e-6 and result['class_index']==int(reference[i]>=threshold);replay_rows+=1
   label=loader.load_label(CORPUS,sid,row['sample_index'],allow_test=split=='test_integrity_replay');assert int(label['class_index'])==int(data['y'][i]) and bool(label['label_valid'])==bool(data['label_valid'][i]);assert not label['persona_valid']
 assert 'torch' not in sys.modules and 'rclpy' not in sys.modules;verify();result=dict(status='PASS',cuda_trained_candidates=len(training['candidates']),validation_threshold_choices_recomputed=len(rankings),normalization_train_only_verified=True,selection_recomputed=True,portable_fixed_model_replay_rows=replay_rows,max_portable_score_abs_error=maxerror,test_replay_is_integrity_only=True,test_models_selected=1,test_threshold_tuning=False,torch_or_ros_imports_in_portable_runtime=False,frozen_source_and_corpus_unchanged=True,pilot_quality_gate=bool(test['pilot_quality_gate']),world_model_training_ready=False)
 (HERE/'reports/audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
