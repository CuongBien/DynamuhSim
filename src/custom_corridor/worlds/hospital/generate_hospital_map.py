"""Build a Nav2 occupancy map from the static geometry in hospital_easy.sdf.

Run after generate_hospital.py; the output replaces maps/hospital/hospital_easy.pgm.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUTPUT = ROOT / "maps" / "hospital"
RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -32.0, -10.5
WIDTH, HEIGHT = 1280, 420  # 64 x 21 m; 2 m margin around the hospital


def fill_box(pixels, x, y, sx, sy, value):
    left = max(0, math.floor((x - sx / 2 - ORIGIN_X) / RESOLUTION))
    right = min(WIDTH, math.ceil((x + sx / 2 - ORIGIN_X) / RESOLUTION))
    bottom = max(0, math.floor((y - sy / 2 - ORIGIN_Y) / RESOLUTION))
    top = min(HEIGHT, math.ceil((y + sy / 2 - ORIGIN_Y) / RESOLUTION))
    for iy in range(bottom, top):
        start = (HEIGHT - 1 - iy) * WIDTH
        pixels[start + left:start + right] = bytes([value]) * (right - left)


def main():
    pixels = bytearray([205]) * (WIDTH * HEIGHT)  # Outside: unknown
    fill_box(pixels, 0, 0, 60, 17, 254)  # Hospital floor: free

    root = ET.parse(HERE / "hospital_easy.sdf").getroot()
    count = 0
    for model in root.findall("./world/model"):
        name = model.get("name", "")
        if name == "hospital_floor" or "marker" in name:
            continue
        if "_door_" in name and name.endswith("_top"):
            continue  # Overhead lintel leaves the doorway open
        if "human" in name:
            continue  # People belong in the live costmap, not the static map
        if "bench" in name and not name.endswith("_seat"):
            continue  # Seat footprint includes the bench legs

        pose = model.findtext("pose")
        size = model.findtext("./link/collision/geometry/box/size")
        if pose is None or size is None:
            continue
        x, y = map(float, pose.split()[:2])
        sx, sy = map(float, size.split()[:2])
        fill_box(pixels, x, y, sx, sy, 0)
        count += 1

    OUTPUT.mkdir(parents=True, exist_ok=True)
    pgm = OUTPUT / "hospital_easy.pgm"
    pgm.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + pixels)
    (OUTPUT / "hospital_easy.yaml").write_text(
        "image: hospital_easy.pgm\n"
        f"resolution: {RESOLUTION:.3f}\n"
        f"origin: [{ORIGIN_X:.3f}, {ORIGIN_Y:.3f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    print(f"Generated {pgm}: {WIDTH}x{HEIGHT}, {count} static obstacles")


if __name__ == "__main__":
    main()
