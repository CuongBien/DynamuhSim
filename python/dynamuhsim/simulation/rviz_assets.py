"""Process-local RViz resource overlay; never patches /opt or changes Gazebo rendering."""
import hashlib
from pathlib import Path
import shutil


def prepare_rviz_overlay(repo, assets):
    from ament_index_python.packages import get_package_share_directory
    source = Path(get_package_share_directory("rviz_rendering"))
    prefix = Path(assets) / "rviz_overlay"
    target = prefix / "share/rviz_rendering"
    shutil.copytree(source, target)
    marker = prefix / "share/ament_index/resource_index/packages/rviz_rendering"
    marker.parent.mkdir(parents=True)
    marker.write_text("")
    fragment = Path("ogre_media/materials/glsl120/indexed_8bit_image.frag")
    original = hashlib.sha256((source / fragment).read_bytes()).hexdigest()
    shutil.copyfile(Path(repo) / "configs/capture/rviz_indexed_8bit_image.frag", target / fragment)
    return dict(scope="RViz child process only; Gazebo and system files unchanged",
                source_share=str(source), original_shader_sha256=original,
                patched_shader_sha256=hashlib.sha256((target / fragment).read_bytes()).hexdigest())
