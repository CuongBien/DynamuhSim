"""Runtime-only camera metadata/TF repair; retain baseline bytes and diffs."""
import difflib
import hashlib
from pathlib import Path

def prepare_camera(runtime, repo):
    runtime, repo = Path(runtime), Path(repo)
    launch = repo / 'src/custom_corridor/launch/corridor_tb3.launch.py'
    model = repo / 'src/custom_corridor/models/mecanum_robot/model.sdf'
    old_launch, old_model = launch.read_text(), model.read_text()
    assert old_launch.count('        camera_link_tf,\n') == 1
    new_launch = old_launch.replace('        camera_link_tf,\n', '')
    assert old_model.count('<gz_frame_id>camera_link</gz_frame_id>') == 1
    new_model = old_model.replace('<gz_frame_id>camera_link</gz_frame_id>',
                                  '<gz_frame_id>camera_optical_frame</gz_frame_id>')
    result = {}
    for original, before, after, name in [
        (launch, old_launch, new_launch, 'mecanum_camera.launch.py'),
        (model, old_model, new_model, 'mecanum_camera.sdf'),
    ]:
        target = runtime / name
        target.write_text(after)
        (runtime / (name + '.diff')).write_text(''.join(difflib.unified_diff(
            before.splitlines(True), after.splitlines(True), fromfile=str(original), tofile=name)))
        result[name] = dict(original=str(original.relative_to(repo)),
                           original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                           runtime_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    return result
