# Quaternius human visual subsystem

This is a MuJoCo 4.0 animation demo that runs in its own Python environment. It shares the rig and skin code in `src/narrow_nav/visualization/` with the simulator. The external caller owns each actor's XY position, heading, and speed. The subsystem owns animation playback, local skeletal poses, and visual skinning. Its studio preview has one non-colliding floor.

## Sources

The selected assets are the original clothed **Smooth_Male_Casual.fbx** and **Smooth_Female_Casual.fbx**, with Quaternius's CC0 1.0 license. The official Google Drive file listings were inspected. Their binary download endpoints returned quota pages during preparation, so the pinned MRPT/MVSim redistributions supplied the FBX and the original license. The pinned archive and member hashes, original file URLs, and model choices are in [assets/selection.json](assets/selection.json). [Animated Men](https://quaternius.com/packs/animatedmen.html), [Animated Women](https://quaternius.com/packs/animatedwomen.html), [MRPT archive documentation](https://mvsimulator.readthedocs.io/en/latest/world_actors.html)

The selected FBX files contain material colors, original clothing, faces, and hair, with no UV layers or image textures. The rendered bundles use the supplied material colors. Universal Base Characters and the Universal Animation Library were reviewed as candidate sources. The demo uses the selected characters' embedded actions on their own rigs. [Universal Base Characters](https://quaternius.com/packs/universalbasecharacters.html), [Universal Animation Library](https://quaternius.itch.io/universal-animation-library)

## Install and prepare assets

Run from the repository's `humans/` directory. This uses an isolated environment and redirects temporary files and caches into `humans/tmp/`.

```bash
cd /home/haduckien/nav_ws/humans
mkdir -p tmp
export TMPDIR="$PWD/tmp" XDG_CACHE_HOME="$PWD/tmp/cache"
export XDG_CONFIG_HOME="$PWD/tmp/config" XDG_DATA_HOME="$PWD/tmp/data"
export MESA_SHADER_CACHE_DIR="$PWD/tmp/mesa"
export BLENDER_USER_RESOURCES="$PWD/tmp/blender-user"
export BLENDER_USER_CONFIG="$PWD/tmp/blender-config"
export BLENDER_USER_EXTENSIONS="$PWD/tmp/blender-extensions"
unset PYTHONPATH
python3 -m venv .venv
.venv/bin/python -m pip install --cache-dir "$PWD/tmp/pip-cache" -r requirements-dev.txt
.venv/bin/python fetch_assets.py
```

`fetch_assets.py` downloads the pinned CC0 archives when needed and verifies the exact FBX and license files. To use manually downloaded files, place them with the filenames in `assets/source/` from the selection file. The fetch command then verifies them without downloading replacements.

Blender is used only to produce the runtime bundles. The tested offline converter is Blender **4.5.15 LTS**. Use an installed Blender binary or download the portable Linux x64 release inside `humans/`.

```bash
base=https://download.blender.org/release/Blender4.5
archive=blender-4.5.15-linux-x64.tar.xz
curl -fL "$base/$archive" -o "tmp/$archive"
curl -fL "$base/blender-4.5.15.sha256" -o tmp/blender.sha256
(cd tmp && awk -v f="$archive" '$2 == f' blender.sha256 | sha256sum -c -)
tar -xJf "tmp/$archive" -C tmp
tmp/blender-4.5.15-linux-x64/blender -b --factory-startup --disable-autoexec \
  --python-exit-code 1 --python prepare_assets.py
```

The converter reads the skinned mesh, triangulates polygons, preserves UV seams when UVs exist, and saves material parts, normalized weights, hierarchy, rest transforms and 24 Hz source clips. It normalizes bind height to **1.75 m** for the male and **1.65 m** for the female, using a uniform scale per character. All proportions are preserved. Output is `assets/generated/{casual_male,casual_female}/`, containing `manifest.json` and `character.npz`. The converter checks source hashes before loading, and the runtime checks data and texture hashes. When rebuilding, use a new output directory such as `-- --output-dir tmp/bundles-rebuilt` to preserve the existing result.

For different local asset paths, use `-- --selection path/to/selection.json --source-dir /path/to/read-only-assets --output-dir tmp/bundles-custom`. The selection explicitly maps source action names, the motion bone, foot bones, height, facing, and verified license/file hashes. GLB and glTF files with an embedded compatible rig and clips are accepted by the converter path. FBX is the format verified for these two characters.

## Run the 150-second demo

From the repository root, run `./humans/run_demo.sh`. The launcher uses the isolated environment and accepts the same flags as the Python command below. Relative output paths are resolved inside `humans/`.

```bash
export PYTHONPATH="$PWD/../src"  # For direct Python commands run from humans/.
.venv/bin/python -m human_visual.demo --duration 150 --agents 6 --seed 42
```

Use a desktop display for the GUI. Escape closes the window. The following runs execute the same replay without a desktop.

```bash
.venv/bin/python -m human_visual.demo --check-only --duration 150 --agents 6 --seed 42
MUJOCO_GL=egl .venv/bin/python -m human_visual.demo --duration 150 --agents 6 --seed 42 \
  --record-dir reports/frames --record-every 450
```

Recording saves actual MuJoCo PNG frames every N simulation frames inside `humans/`. Use `--record-every 1` for a complete image sequence. Duration, FPS, agent count (1–10) and seed are configurable. The five stages in [replay.json](replay.json) last 30 seconds each at the default duration. They show straight walking, slow and normal walking, external heading changes, stop/idle/start crossfades, and finally the full ensemble. The root tracks are deterministic visual test fixtures.

## Interactive corridor

Run from the repository root.

```bash
./humans/run_demo.sh --scene corridor
./humans/run_demo.sh --scene corridor --agents 10 --duration 45
```

The standalone [MJCF corridor](scenes/corridor.xml) has a tiled floor, a dashed centre line and low walls so that the feet stay visible from most angles. Six actors are active from the first frame by default. They walk at 1.15 m/s along two lanes joined by semicircular turns. The seed sets the initial phase of this deterministic fixture. Turns use the authored walk clip and external heading. Human skins are visual only and follow the scripted paths without physical contact or navigation.

Drag with the left mouse button to orbit and with the right button to pan. Scroll up to zoom in. Press **Space** to pause or resume, **R** to restore the overview and **Esc** to close. Camera controls remain active while paused.

To validate headlessly or record an overview PNG every two simulated seconds, run from the repository root.

```bash
./humans/run_demo.sh --scene corridor --duration 20 --check-only
MUJOCO_GL=egl ./humans/run_demo.sh --scene corridor --duration 20 \
  --record-dir reports/corridor-20s --record-every 60
```

## Isolated API and motion conventions

```python
from pathlib import Path
from human_visual.bundle import load_character
from human_visual.mujoco_skin import build_preview

character = load_character(Path("assets/generated/casual_male"))
visuals = build_preview({"human_01": character})
pose = visuals.update("human_01", 1.0, 2.0, 0.4, 1.15, "walk", 1 / 30)
visuals.forward()
```

For a caller-owned MJCF tree, use `append_visuals(root, actors)`, compile with its returned VFS textures, and bind `MujocoVisuals(model, data, actors)`. `HumanAnimator.update(...)` can also be used on its own with no MuJoCo object. Stable actor IDs are assigned on construction. Updates take metres, radians, nonnegative speed in m/s and nonnegative simulated `dt` in seconds. The basis is right-handed, Z up, heading 0 along +X, and positive heading rotates toward +Y. The FBX facing direction was verified from the foot bones and visible faces.

Normalized clips remove planar translation and yaw of the designated motion root before sampling. The caller's XY and heading apply once. Vertical source motion and relative joints are preserved. Playback uses linear translations and shortest-arc quaternion SLERP, smoothstep crossfades, and loop seam blending only if the sampled source endpoints differ. Playback scales with the measured reference speed, clamped to 0.35–2.5×. The 42-bone parent hierarchy is evaluated in NumPy. MuJoCo receives the resulting transforms through named mocap bodies, and draws skinned triangles through `deformable/skin`. These bodies have no visible bone geometry and no collision proxies. Texture and material resources are reused across instances. [MuJoCo skin reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html#deformable-skin)

Both character assets contain **idle, walk and run**. Slow walk uses the existing walk clip at the requested speed. Turn follows the caller's heading while using walk or idle, and start/stop use crossfades. There are no source turn, slow walk, start, stop or backward-walk clips in the selected FBX files. Backward walk requires a supplied clip and raises an error when absent. The example does not add foot IK, contact mechanics or source retargeting.

## Tests and metrics

```bash
.venv/bin/python -m pytest -q --basetemp "$PWD/tmp/pytest"
for agents in 1 2 5 10; do
  MUJOCO_GL=egl .venv/bin/python -m human_visual.benchmark --agents "$agents" --frames 90
done
```

See [VALIDATION.md](VALIDATION.md) for measured results and evidence. With missing bundles, asset-dependent tests show `BLOCKED (assets)` in the skip reason and the demo exits 2 with instructions. No substitute human is drawn.
