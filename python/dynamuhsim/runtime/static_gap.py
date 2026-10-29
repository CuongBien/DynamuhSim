"""Run-local static-gap captures through the shared Nav2 runner, with no human worker."""
import datetime,json,shutil,subprocess
from pathlib import Path
from .capture import CaptureContext,capture_sources,validate_config,verify_run,dump,sha,CAMERA_SHA256,capture_live_parameters
from .capture_recording import TOPICS
from .planner_diagnostics import PLANNER_PARAMETERS
from dynamuhsim.simulation.capture import prepare_camera
from dynamuhsim.simulation.robot_geometry import aligned_urdf
from dynamuhsim.simulation.static_gap import fixture,write_assets,configure_world

def prepare(repo,run,case,treatment="baseline"):
    repo=Path(repo).resolve();run=Path(run).resolve();run.relative_to(repo)
    if treatment not in ['baseline','native_cost']:raise ValueError('Unknown static-gap treatment')
    config_path=repo/('configs/capture/geometry_v4_inflation045.json' if treatment=='baseline' else 'configs/capture/geometry_v5_mppi_native_cost.json');config=validate_config(json.loads(config_path.read_text()));spec=fixture(case)
    files=capture_sources(repo)+[config_path,repo/'tools/static_gap_episode.py',repo/'configs/audit/static_gap.json']
    if any(not p.is_file() for p in files):raise FileNotFoundError('Prepare shared native dependencies first')
    run.mkdir(parents=True,exist_ok=False);assets=run/'assets';assets.mkdir();camera=prepare_camera(assets,repo)
    if camera['runtime_sha256']!=CAMERA_SHA256:raise RuntimeError('Native camera bytes changed')
    friction=json.loads(subprocess.check_output([str(repo/'python/dynamuhsim/runtime/native/generated/check_friction'),str(assets/'mecanum_camera.sdf')],text=True))
    if friction['recognized_gz_base_link_frames']!=4:raise RuntimeError('Friction frame contract failed')
    geometry=aligned_urdf(repo/'src/custom_corridor/models/mecanum_robot/model.sdf',repo/'src/custom_corridor/urdf/mecanum_robot.urdf',assets/'mecanum_aligned.urdf');write_assets(assets,spec)
    dump(run/'preflight.json',dict(camera=camera,native_friction=friction,geometry=geometry))
    dump(run/'plan.json',dict(schema='dynamuhsim_capture_plan/1',config=config,scenario='static_gap',seed=0,gui=False,planner_diagnostics=True,fixture=spec,treatment=treatment))
    files+=list(assets.iterdir())+[run/'plan.json',run/'preflight.json']
    dump(run/'capture_freeze.json',dict(schema='dynamuhsim_capture_freeze/2',git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256={str(p.relative_to(repo)):sha(p) for p in sorted(set(files))},historical_manifest_modified=False,retry_policy='Preserve every attempt, no overwrite or automatic retry'))
    return verify_run(repo,run)

class StaticGapCapture(CaptureContext):
    def prepare(self,runtime,result):
        super().prepare(runtime,result)
        result.update(capture_schema='dynamuhsim_static_gap_capture/1',scenario='static_gap',fixture=self.plan['fixture'],response_variant='none_static_obstacles',experiment='Isolated fixed bottleneck; same native Mecanum/NavFn/MPPI treatment, no human worker')
    def configure_world(self,tree):
        super().configure_world(tree);configure_world(tree,self.plan['fixture'])
    def start_bridge(self,runtime,output,env,processes,files):
        super().start_bridge(runtime,output,env,processes,files)
        env['DYNAMUHSIM_CAPTURE_MAP']=str(self.run_dir/'assets/gap.yaml')
    def ready(self,node,wait,output,result,telemetry):
        required=[t for t in TOPICS[:12] if t!='/common/ground_truth']
        if not wait(lambda:self.recorder.ready(required_topics=required) and self.pairs.ready() and self.diagnostics.ready(),30):raise RuntimeError('Static-gap recorder, paired depth or native costmaps missing')
        result['live_parameters']=capture_live_parameters(node,wait,self.config['nav2'],self.config['navigation'],additional=PLANNER_PARAMETERS,controller=self.config.get("controller"))
        dump(output/'live_parameters.json',result['live_parameters'])
        self.recorder.event('ready',round(telemetry.simtime()*1e9),scenario='static_gap',case=self.plan['fixture']['case'])
