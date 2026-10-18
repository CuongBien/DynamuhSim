"""Host-side bridge setup; no CrowdSimPlus imports in ROS Python 3.12."""
import hashlib,json,os,shutil,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
HERE=Path(__file__).resolve().parent
IMAGE='dynamuhsim-crowdsimplus:round2'
SOURCE='d526a07c013c621fa1ad50894b6ba639f61063de'
def prepare_scene(runtime,condition,seed):
    source=HERE.parent/'benchmark/reports'/f'socnavgym_{condition}_s{seed}.json'
    if not source.exists():raise ValueError('Common fixture currently supports pilot seeds 42/43 only')
    baseline=json.loads(source.read_text());(runtime/'scene.json').write_text(json.dumps(baseline['scene'],indent=2))
    assert hashlib.sha256((runtime/'scene.json').read_bytes()).hexdigest()==baseline['scene_sha256']
    shutil.copyfile(HERE.parent/'warehouse/layout.json',runtime/'layout.json')

def convert_world(runtime):
    tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world')
    plugin=world.find("plugin[@name='social_experiment::Adapter']")
    plugin.set('name','social_experiment::CrowdSimAdapter');plugin.set('filename',str(HERE/'build/libcrowdsimplus_adapter.so'))
    for e in world.iter():
        for key,value in list(e.attrib.items()):
            if value.startswith('socnavgym_'):e.set(key,value.replace('socnavgym_','crowdsimplus_',1))
        if e.text and '/socnavgym/observer' in e.text:e.text=e.text.replace('/socnavgym/observer','/crowdsimplus/observer')
    ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode')

def docker_command(runtime,socket_dir,name,invisible=False,human_policy='orca_plus'):
    if human_policy not in ('orca_plus','sfm'):raise ValueError('Unsupported human policy')
    cmd=['docker','run','--rm','--name',name,'--network','none','--read-only','--tmpfs','/tmp:rw,nosuid,nodev,size=256m',
        '--cpus','4','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}',
        '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',f'{HERE}:/work:ro','-v',f'{runtime}:/runtime','-v',f'{socket_dir}:/socket','-w','/work',
        IMAGE,'python','/work/worker.py','--human-policy',human_policy]
    if invisible:cmd.append('--invisible-robot')
    return cmd

def cleanup_container(name):
    if name:subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=20)
