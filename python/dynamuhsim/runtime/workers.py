"""One host/container command builder for the maintained live IPC workers."""
import json, os
from pathlib import Path

def host_worker_command(stage, common, social, runtime, socket_path, model, case):
    return [str(social / '.venv-socnavgym/bin/python'), str(stage / 'worker.py'), '--common', str(common),
            '--runtime', str(runtime), '--socket', str(socket_path), '--model', model, '--case', case]

def worker_command(stage, runtime, socket_dir, model, case, token, *, container_prefix=None):
    stage, runtime, socket_dir = Path(stage), Path(runtime), Path(socket_dir)
    common = stage.parent / 'common_response'
    if container_prefix is None:
        return host_worker_command(stage, common, stage.parent.parent, runtime, socket_dir / 'worker.sock', model, case), None, None
    images = json.loads((common / 'inputs/images.json').read_text())
    image = images['dynamuhsim-crowdsimplus:round2' if model.startswith('crowd_') else 'dynamuhsim-complexitynav:round3']
    container = container_prefix + token[:12]
    command = ['docker', 'run', '--rm', '--name', container, '--network', 'none', '--cpus', '2', '--memory', '4g',
               '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'OMP_NUM_THREADS=1', '-e', 'OPENBLAS_NUM_THREADS=1',
               '-e', 'MKL_NUM_THREADS=1', '-e', 'DYNAMUHSIM_EXPERIMENT_SESSION=' + token, '-v', f'{stage}:/live:ro']
    if stage.name != 'common_gazebo':
        command += ['-v', f'{stage.parent / "common_gazebo"}:/common_gazebo:ro']
    command += ['-v', f'{common}:/common:ro', '-v',
                f'{(common / "reports/run01/protocol.json").resolve()}:/archive/social_navigation/reports/common_response/run01/protocol.json:ro',
                '-v', f'{stage.parent / "complexitynav_geometry"}:/geometry:ro', '-v', f'{runtime}:/runtime',
                '-v', f'{socket_dir}:/socket', image, 'python', '/live/worker.py', '--common', '/common', '--runtime', '/runtime',
                '--socket', '/socket/worker.sock', '--model', model, '--case', case]
    return command, container, image
