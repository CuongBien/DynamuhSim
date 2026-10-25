import copy,hashlib,importlib.util,json,shutil,subprocess,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.controls.bundle import ControlBundle,registered_bundle,export_bundle,ORIGINS,CLASSIFIER
from dynamuhsim.perception.depth_classifier import FrozenDepthClassifier,depth_features
BUNDLE=ROOT/'control_bundles/sensor_pilot_v1'
DIGEST=json.loads((ROOT/'configs/control_registry.json').read_text())['bundles']['sensor_pilot_v1']['manifest_sha256']
LEGACY=ROOT/'experiments/social_navigation/benchmark/depth_person_classifier'


def load(name):
    spec=importlib.util.spec_from_file_location('_reference_'+name,LEGACY/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class ControlBundleTests(unittest.TestCase):
    def copy_bundle(self,directory):
        target=Path(directory)/'bundle';shutil.copytree(BUNDLE,target);return target

    def rehash(self,root):
        path=root/'manifest.json';manifest=json.loads(path.read_text())
        for name,row in manifest['files'].items():
            data=(root/name).read_bytes();row.update(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
        path.write_text(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def rebind_model(self,root,arrays):
        np.savez_compressed(root/'classifier/model.npz',**arrays)
        metadata=root/'classifier/metadata.json';selected=json.loads(metadata.read_text())
        selected['model_sha256']=hashlib.sha256((root/'classifier/model.npz').read_bytes()).hexdigest()
        metadata.write_text(json.dumps(selected))
        receipt=root/'classifier/evaluation_receipt.json';value=json.loads(receipt.read_text())
        value['selected_frozen_sha256']=hashlib.sha256(metadata.read_bytes()).hexdigest();receipt.write_text(json.dumps(value))
        return self.rehash(root)

    def test_features_match_frozen_legacy_for_degenerate_and_random_geometry(self):
        reference=load('features');rng=np.random.default_rng(731)
        clouds=[np.array([[1.,.2,.8]]),np.repeat([[1.,.2,1.]],8,axis=0)]
        clouds += [rng.normal(size=(n,3)) for n in [2,3,7,31,128,1000]]
        for points in clouds:
            old,_,_=reference.extract(dict(points_xyz=points,scan_valid=False),{})
            self.assertTrue(np.array_equal(depth_features(points),old))

    def test_logits_scores_and_predictions_match_legacy_on_synthetic_features(self):
        bundle=ControlBundle(BUNDLE,DIGEST);model=FrozenDepthClassifier(bundle);old=load('inference');metrics=load('metrics')
        arrays=dict(np.load(BUNDLE/'classifier/model.npz',allow_pickle=False));rng=np.random.default_rng(37)
        for dtype in [np.float32,np.float64]:
            values=rng.normal(size=(127,30)).astype(dtype)
            data=dict(depth=values,lidar=np.zeros((127,8),dtype),lidar_mask=np.zeros(127,bool))
            expected=old.numpy_logits(data,arrays,0,'depth');actual=model.predict(values)
            self.assertTrue(np.array_equal(model.logits(values),expected))
            self.assertTrue(np.array_equal(actual['score'],metrics.sigmoid(expected)))
            self.assertTrue(np.array_equal(actual['prediction'],metrics.sigmoid(expected)>=.5))

    def test_portable_bundle_needs_no_reports_experiments_git_torch_or_corpus(self):
        with tempfile.TemporaryDirectory() as directory:
            root=self.copy_bundle(directory)
            model=FrozenDepthClassifier(ControlBundle(root,DIGEST))
            self.assertEqual(model.predict_points([np.array([[1.,0.,1.]])])['score'].shape,(1,))
            self.assertEqual(model.predict_points([])['score'].shape,(0,))
            self.assertFalse((Path(directory)/'experiments').exists())
            self.assertFalse(list(root.rglob('test_opened.json')))
            self.assertNotIn('torch',sys.modules)

    def test_invalid_manifest_hash_corrupt_file_and_changed_file_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=self.copy_bundle(directory)
            with self.assertRaises(ValueError):ControlBundle(root,'0'*64)
            bundle=ControlBundle(root,DIGEST);path=root/'classifier/model.npz';path.write_bytes(b'corrupt')
            with self.assertRaises(ValueError):ControlBundle(root,DIGEST)
            with self.assertRaises(ValueError):FrozenDepthClassifier(bundle)

    def test_internal_model_or_evaluation_receipt_binding_cannot_be_bypassed_by_rehash(self):
        for field in ['model','receipt']:
            with self.subTest(field=field),tempfile.TemporaryDirectory() as directory:
                root=self.copy_bundle(directory)
                if field=='model':
                    path=root/'classifier/metadata.json';data=json.loads(path.read_text());data['model_sha256']='0'*64
                else:
                    path=root/'classifier/evaluation_receipt.json';data=json.loads(path.read_text());data['selected_frozen_sha256']='0'*64
                path.write_text(json.dumps(data));digest=self.rehash(root)
                with self.assertRaisesRegex(ValueError,'binding|receipt'):FrozenDepthClassifier(ControlBundle(root,digest))

    def test_invalid_array_shape_dtype_normalization_and_nonfinite_weights_rejected(self):
        for kind in ['shape','dtype','zero_std','nan','unknown','object']:
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as directory:
                root=self.copy_bundle(directory);arrays=dict(np.load(root/'classifier/model.npz',allow_pickle=False))
                if kind=='shape':arrays['depth.0.weight']=np.zeros((1,29),np.float32)
                if kind=='dtype':arrays['depth.0.weight']=arrays['depth.0.weight'].astype(np.float64)
                if kind=='zero_std':arrays['depth_std'][0]=0
                if kind=='nan':arrays['depth.0.bias'][0]=np.nan
                if kind=='unknown':arrays['labels']=np.array([1])
                if kind=='object':arrays['depth_mean']=np.array([object()]*30,dtype=object)
                digest=self.rebind_model(root,arrays)
                with self.assertRaises(ValueError):FrozenDepthClassifier(ControlBundle(root,digest))

    def test_invalid_feature_inputs_rejected_and_threshold_has_no_setter(self):
        model=FrozenDepthClassifier(ControlBundle(BUNDLE,DIGEST))
        for values in [np.zeros((1,29),np.float32),np.zeros((1,30),int),np.full((1,30),np.nan),np.full((1,30),np.inf),{'labels':[1]},np.full((1,30),1e308)]:
            with self.subTest(shape=str(type(values))),self.assertRaises(ValueError):model.logits(values)
        with self.assertRaises(AttributeError):model.threshold=.1
        for points in [[],[[1,2]],[[1,np.nan,2]]]:
            with self.assertRaises(ValueError):depth_features(points)

    def test_symlink_and_traversal_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=self.copy_bundle(directory);path=root/'classifier/model.npz';data=path.read_bytes();path.unlink()
            outside=Path(directory)/'outside.npz';outside.write_bytes(data);path.symlink_to(outside)
            with self.assertRaises(ValueError):ControlBundle(root,DIGEST)
            path.unlink();path.write_bytes(data)
            manifest=json.loads((root/'manifest.json').read_text());manifest['files']['../outside.npz']=manifest['files'].pop('classifier/model.npz')
            (root/'manifest.json').write_text(json.dumps(manifest));digest=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()
            with self.assertRaises(ValueError):ControlBundle(root,digest)

    def test_engine_verify_checks_source_hashes_without_executing_source(self):
        with tempfile.TemporaryDirectory() as directory:
            repo=Path(directory)/'repo';repo.mkdir();bundle=ControlBundle(BUNDLE,DIGEST)
            for name,engine in bundle.manifest['engines'].items():
                frozen=bundle.document(engine['metadata'])
                for source in frozen['engine_sha256']:
                    path=repo/engine['source_root']/source;path.parent.mkdir(parents=True,exist_ok=True)
                    path.write_bytes((ROOT/engine['source_root']/source).read_bytes())
                self.assertEqual(bundle.verify_engine(name,repo),frozen)
            source=repo/bundle.manifest['engines']['identity']['source_root']/'guard_tracker.py';source.write_text('raise RuntimeError("not executed")')
            with self.assertRaisesRegex(ValueError,'changed'):bundle.verify_engine('identity',repo)
            self.assertFalse(list(repo.rglob('reports')))

    def test_registry_pins_bundle_and_refuses_path_escape(self):
        self.assertEqual(registered_bundle(ROOT).manifest_sha256,DIGEST)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);registry=root/'registry.json'
            registry.write_text(json.dumps(dict(schema='dynamuhsim_control_registry/1',bundles=dict(sensor_pilot_v1=dict(root='../bundle',manifest_sha256=DIGEST)))))
            with self.assertRaises(ValueError):registered_bundle(root,registry)

    def test_in_memory_model_remains_frozen_when_disk_is_changed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=self.copy_bundle(directory);model=FrozenDepthClassifier(ControlBundle(root,DIGEST));values=np.zeros((2,30),np.float32)
            before=model.logits(values);(root/'classifier/model.npz').write_bytes(b'changed')
            self.assertTrue(np.array_equal(model.logits(values),before))
            with self.assertRaises(ValueError):ControlBundle(root,DIGEST)

    def test_export_reads_only_recorded_git_and_is_repeatable(self):
        with tempfile.TemporaryDirectory() as directory:
            repo=Path(directory)/'repo';repo.mkdir();manifest=json.loads((BUNDLE/'manifest.json').read_text())
            for target,origin in ORIGINS.items():
                path=repo/origin;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((BUNDLE/target).read_bytes())
            for origin in manifest['provenance_only']:
                path=repo/origin;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((ROOT/origin).read_bytes())
            for name,engine in manifest['engines'].items():
                frozen=json.loads((BUNDLE/engine['metadata']).read_text())
                for source in frozen['engine_sha256']:
                    path=repo/engine['source_root']/source;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((ROOT/engine['source_root']/source).read_bytes())
            subprocess.run(['git','init','-q',str(repo)],check=True);subprocess.run(['git','-C',str(repo),'add','.'],check=True)
            subprocess.run(['git','-C',str(repo),'-c','user.name=Bundle Test','-c','user.email=bundle@example.invalid','commit','-qm','controls'],check=True)
            dirty=repo/CLASSIFIER/'reports/selected_model.npz';dirty.write_bytes(b'local edit')
            first=export_bundle(repo,'HEAD',Path(directory)/'one');second=export_bundle(repo,'HEAD',Path(directory)/'two')
            self.assertEqual(first,second);self.assertEqual(dirty.read_bytes(),b'local edit')
            self.assertEqual((Path(directory)/'one/classifier/model.npz').read_bytes(),(BUNDLE/'classifier/model.npz').read_bytes())
            with self.assertRaises(FileExistsError):export_bundle(repo,'HEAD',Path(directory)/'one')

    def test_inference_does_not_modify_bundle_files(self):
        before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in BUNDLE.rglob('*') if p.is_file()}
        model=FrozenDepthClassifier(ControlBundle(BUNDLE,DIGEST));model.predict_points([np.array([[1.,0.,1.],[1.1,.1,1.5]])])
        self.assertEqual(before,{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in BUNDLE.rglob('*') if p.is_file()})

    def test_cli_help_needs_no_ros(self):
        for name in ['export_control_bundle.py','verify_control_bundle.py']:
            p=subprocess.run([sys.executable,str(ROOT/'tools'/name),'--help'],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)


if __name__=='__main__':unittest.main()
