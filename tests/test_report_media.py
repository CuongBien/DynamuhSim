import copy,json,shutil,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
import test_report_plan as fixtures
from dynamuhsim.artifacts.catalog import BENCHMARK,create_catalog
from dynamuhsim.artifacts.history import pack_history
from dynamuhsim.artifacts.report_plan import create_report_plan,hydrate_reports
from dynamuhsim.artifacts.report_media import create_media_plan,validate_media_plan,hydrate_report_media


class ReportMediaTests(unittest.TestCase):
    def fixture(self,directory):
        repo,store,index=fixtures.ReportPlanTests().fixture(directory,freeze=True)
        root=repo/BENCHMARK/'old_stage/reports'
        (root/'gallery').mkdir()
        (root/'preview.png').write_bytes(b'PNG fixture'*100)
        (root/'gallery/animation.gif').write_bytes(b'GIF fixture'*500)
        (root/'test_opened.json').write_text('{"sealed":true}')
        (root/'selected_model.npz').write_bytes(b'frozen weights')
        subprocess.run(['git','-C',str(repo),'add','.'],check=True)
        subprocess.run(['git','-C',str(repo),'-c','user.name=Media Test','-c','user.email=media@example.invalid','commit','-qm','media and controls'],check=True)
        index=pack_history(repo,store,create_catalog(repo,store));plan=create_report_plan(repo,index)
        media=create_media_plan(repo,index,plan)
        checkout=Path(directory)/'checkout';shutil.copytree(repo,checkout,ignore=shutil.ignore_patterns('.git'))
        return repo,store,index,plan,media,checkout

    def remove_media(self,checkout,media):
        group=media['groups']['reports:old_stage']
        for name in group['media']:(checkout/group['root']/name).unlink()
        return checkout/group['root']

    def test_git_and_archive_restore_only_media_and_preserve_model_and_test_seal(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory)
            for source in ['git','archive']:
                root=self.remove_media(checkout,media);seal=(root/'test_opened.json').read_bytes();weights=(root/'selected_model.npz').read_bytes()
                check=hydrate_report_media(checkout,index,media,['reports:old_stage'],source=source,store=store,git_repo=repo,check=True)
                self.assertEqual(check['status'],'MEDIA_MISSING');self.assertEqual(check['changed_files'],0)
                result=hydrate_report_media(checkout,index,media,['reports:old_stage'],source=source,store=store,git_repo=repo)
                self.assertEqual(result['changed_files'],2)
                self.assertEqual((root/'test_opened.json').read_bytes(),seal);self.assertEqual((root/'selected_model.npz').read_bytes(),weights)
                again=hydrate_report_media(checkout,index,media,['reports:old_stage'],source=source,store=store,git_repo=repo)
                self.assertEqual(again['changed_files'],0)

    def test_controls_missing_modified_or_extra_are_never_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory);root=self.remove_media(checkout,media)
            seal=root/'test_opened.json';original=seal.read_bytes();seal.unlink()
            with self.assertRaisesRegex(ValueError,'Missing report control'):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertFalse(seal.exists());self.assertFalse((root/'preview.png').exists())
            seal.write_text('local edit')
            with self.assertRaises(ValueError):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertEqual(seal.read_text(),'local edit')
            seal.write_bytes(original);(root/'extra.json').write_text('keep')
            with self.assertRaisesRegex(ValueError,'Extra report'):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertEqual((root/'extra.json').read_text(),'keep')

    def test_edited_or_symlinked_media_and_symlinked_ancestor_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory);root=checkout/media['groups']['reports:old_stage']['root']
            image=root/'preview.png';original=image.read_bytes();image.write_text('user image')
            with self.assertRaises(ValueError):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertEqual(image.read_text(),'user image')
            image.unlink();image.symlink_to(repo/BENCHMARK/'old_stage/reports/preview.png')
            with self.assertRaises(ValueError):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)

    def test_policy_cannot_select_model_or_seal_or_swap_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory)
            for name in ['test_opened.json','selected_model.npz','snapshot/driver.py','../outside.png']:
                bad=copy.deepcopy(media);g=bad['groups']['reports:old_stage'];g['media'][name]=next(iter(g['media'].values()))
                with self.subTest(name=name),self.assertRaises(ValueError):validate_media_plan(index,bad)
            bad=copy.deepcopy(media);bad['groups']['reports:old_stage']['object_id']='0'*64
            with self.assertRaises(ValueError):validate_media_plan(index,bad)

    def test_changed_media_digest_is_rejected_against_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory);root=self.remove_media(checkout,media)
            media['groups']['reports:old_stage']['media']['preview.png']['sha256']='0'*64
            with self.assertRaisesRegex(ValueError,'inventory'):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertFalse((root/'preview.png').exists())

    def test_publish_race_does_not_overwrite_and_can_resume_valid_partial_recovery(self):
        import os
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory);root=self.remove_media(checkout,media)
            link=os.link;counter=[0]
            def race(src,dst):
                counter[0]+=1
                if counter[0]==2:Path(dst).write_text('concurrent edit')
                return link(src,dst)
            with patch('dynamuhsim.artifacts.report_media.os.link',side_effect=race):
                with self.assertRaises(FileExistsError):hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            raced=[p for p in [root/'preview.png',root/'gallery/animation.gif'] if p.exists() and p.read_bytes()==b'concurrent edit']
            self.assertEqual(len(raced),1);raced[0].unlink()
            result=hydrate_report_media(checkout,index,media,['reports:old_stage'],source='archive',store=store)
            self.assertEqual(result['changed_files'],1)
            self.assertFalse(list(root.parent.glob('.hydrate-media-*')))

    def test_full_report_wrapper_fills_approved_media_gaps_and_counts_them(self):
        with tempfile.TemporaryDirectory() as directory:
            repo,store,index,plan,media,checkout=self.fixture(directory);self.remove_media(checkout,media)
            result=hydrate_reports(checkout,index,plan,['reports:old_stage'],source='archive',store=store,media_plan=media)
            self.assertEqual(result['changed_files'],2);self.assertEqual(result['changed_groups'],1)
            self.assertFalse(result['groups'][0]['changed'])
            again=hydrate_reports(checkout,index,plan,['reports:old_stage'],source='archive',store=store,media_plan=media)
            self.assertEqual(again['changed_groups'],0)

    def test_cli_help_without_ros(self):
        result=subprocess.run([sys.executable,str(ROOT/'tools/report_media.py'),'--help'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)


if __name__=='__main__':unittest.main()
