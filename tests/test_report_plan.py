import copy,json,subprocess,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
import test_asset_catalog as fixtures
from dynamuhsim.artifacts.catalog import create_catalog,BENCHMARK
from dynamuhsim.artifacts.history import pack_history
from dynamuhsim.artifacts.report_plan import create_report_plan,python_references,source_pins,dependency_closure,validate_report_plan,hydrate_reports


class ReportPlanTests(unittest.TestCase):
    def fixture(self,directory,freeze=False):
        repo,store,head=fixtures.AssetCatalogTests().fixture(directory)
        (repo/BENCHMARK/'old_stage/reports/snapshot/driver.py').write_text('# frozen producer\n')
        # The catalog fixture has a source pin outside benchmark; optionally keep it.
        if not freeze:
            subprocess.run(['git','-C',str(repo),'rm','-q','protocol/frozen.json'],check=True)
        subprocess.run(['git','-C',str(repo),'add','.'],check=True)
        subprocess.run(['git','-C',str(repo),'-c','user.name=Plan Test','-c','user.email=plan@example.invalid','commit','-qm','plan fixture'],check=True)
        index=pack_history(repo,store,create_catalog(repo,store))
        return repo,store,index

    def test_static_path_aliases_parent_indices_and_dynamic_suffix_are_detected(self):
        text="HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3]\na=BENCH/'target/reports/run01'/args.name\nb=Path('/old/machine/experiments/social_navigation/benchmark/other/reports/x.json')\n"
        refs=python_references(text,BENCHMARK+'/stage/run.py',{'reports:target':BENCHMARK+'/target/reports','reports:other':BENCHMARK+'/other/reports'})
        self.assertEqual({r[0] for r in refs},{'reports:target','reports:other'})

    def test_static_parser_never_calls_arbitrary_code(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'must-not-exist'
            refs=python_references("open("+repr(str(target))+",'w').write('executed')",BENCHMARK+'/stage/run.py',{})
            self.assertEqual(refs,[]);self.assertFalse(target.exists())

    def test_nested_source_maps_are_found(self):
        self.assertEqual(list(source_pins({'x':[{'source_sha256':{'path':'digest'}}]})),[('path','digest')])

    def test_dependency_cycles_resolve_and_unknown_dependencies_fail(self):
        groups={'a':{'dependencies':['b']},'b':{'dependencies':['a','c']},'c':{'dependencies':[]}}
        self.assertEqual(dependency_closure(groups,['a']),['a','b','c'])
        with self.assertRaises(ValueError):dependency_closure(groups,['missing'])
        groups['c']['dependencies']=['unknown']
        with self.assertRaises(ValueError):dependency_closure(groups,['a'])

    def test_catalog_gate_is_retained_and_snapshot_group_can_be_externalized(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index=self.fixture(directory)
            plan=create_report_plan(repo,index)
            self.assertEqual(plan['groups']['reports:socnavgym_nav2_pilot']['action'],'retain')
            old=plan['groups']['reports:old_stage']
            self.assertEqual(old['action'],'externalize_whole_group')
            self.assertEqual(old['categories']['snapshot']['files'],1)
            self.assertEqual(validate_report_plan(index,plan),plan['groups'])
            self.assertEqual(plan,create_report_plan(repo,index))

    def test_source_pin_outside_benchmark_retains_snapshot_group(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index=self.fixture(directory,freeze=True)
            plan=create_report_plan(repo,index)
            self.assertEqual(plan['groups']['reports:old_stage']['action'],'retain')
            self.assertEqual(plan['groups']['reports:old_stage']['source_hash_references'][0]['manifest'],'protocol/frozen.json')

    def test_modified_committed_report_tree_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index=self.fixture(directory)
            (repo/BENCHMARK/'old_stage/reports/changed.txt').write_text('new')
            subprocess.run(['git','-C',str(repo),'add','.'],check=True)
            subprocess.run(['git','-C',str(repo),'-c','user.name=Plan Test','-c','user.email=plan@example.invalid','commit','-qm','report changed'],check=True)
            with self.assertRaisesRegex(ValueError,'differs'):create_report_plan(repo,index)

    def test_plan_rejects_dropped_groups_swapped_object_and_gate_externalization(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index=self.fixture(directory);plan=create_report_plan(repo,index)
            for kind in ['missing','object','gate','index']:
                bad=copy.deepcopy(plan)
                if kind=='missing':bad['groups'].pop('reports:old_stage')
                if kind=='object':bad['groups']['reports:old_stage']['object_id']='0'*64
                if kind=='gate':bad['groups']['reports:socnavgym_nav2_pilot']['action']='externalize_whole_group'
                if kind=='index':bad['history_index_sha256']='0'*64
                with self.subTest(kind=kind),self.assertRaises(ValueError):validate_report_plan(index,bad)

    def test_group_hydration_restores_known_dependencies_and_preserves_existing_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index=self.fixture(directory);plan=create_report_plan(repo,index)
            plan['groups']['reports:old_stage']['dependencies']=['reports:socnavgym_nav2_pilot']
            checkout=Path(directory)/'checkout';checkout.mkdir()
            report=hydrate_reports(checkout,index,plan,['reports:old_stage'],source='archive',store=store)
            self.assertEqual(report['changed_groups'],2)
            self.assertTrue((checkout/BENCHMARK/'old_stage/reports/snapshot/driver.py').exists())
            gate=checkout/BENCHMARK/'socnavgym_nav2_pilot/reports/gate01/validation.json'
            before=gate.read_bytes()
            again=hydrate_reports(checkout,index,plan,['reports:old_stage'],source='archive',store=store)
            self.assertEqual(again['changed_groups'],0);self.assertEqual(gate.read_bytes(),before)

    def test_cli_help_has_no_ros_dependency(self):
        for tool in ['report_dependencies.py','hydrate_reports.py']:
            p=subprocess.run([sys.executable,str(ROOT/'tools'/tool),'--help'],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)


if __name__=='__main__':unittest.main()
