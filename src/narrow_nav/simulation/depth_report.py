"""Privileged diagnostics of already captured clean/noisy frame pairs."""

from base64 import b64encode
from pathlib import Path
from struct import pack
from zlib import compress, crc32

import numpy as np


def _png_url(pixels: np.ndarray) -> str:
    """Encode an 8-bit RGB image for an SVG without a plotting dependency."""

    def chunk(tag: bytes, data: bytes) -> bytes:
        return pack(">I", len(data)) + tag + data + pack(">I", crc32(tag + data))

    height, width = pixels.shape[:2]
    header = pack(">2I5B", width, height, 8, 2, 0, 0, 0)
    rows = b"".join(b"\0" + row.tobytes() for row in pixels)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
    png += chunk(b"IDAT", compress(rows)) + chunk(b"IEND", b"")
    return "data:image/png;base64," + b64encode(png).decode("ascii")


def _depth_color(depth: np.ndarray, valid: np.ndarray, max_m: float) -> np.ndarray:
    level = np.clip(depth / max_m, 0, 1)
    colors = np.stack((1 - level, 1 - np.abs(2 * level - 1), level), axis=-1)
    return (np.where(valid[..., None], colors, 0) * 255).astype(np.uint8)


def colorize_depth_error(
    error: np.ndarray, valid: np.ndarray, scale_m: float
) -> np.ndarray:
    level = np.clip(error / scale_m, -1, 1)
    colors = np.stack(
        (1 - np.maximum(-level, 0), 1 - np.abs(level), 1 - np.maximum(level, 0)),
        axis=-1,
    )
    return (np.where(valid[..., None], colors, 0) * 255).astype(np.uint8)


def _plot(
    values: list[float], title: str, y: int, upper: float, duration: float
) -> str:
    # Uniform time steps by contract. A single sample is shown as a dot.
    coordinates = [
        (70 + 837 * i / max(1, len(values) - 1), y + 84 - 84 * value / upper)
        for i, value in enumerate(values)
    ]
    marks = " ".join(
        f'<circle cx="{x:.1f}" cy="{row:.1f}" r="1.8" fill="#2563eb"/>'
        for x, row in coordinates
        if np.isfinite(row)
    )
    return (
        f'<text x="70" y="{y - 11}">{title}</text>'
        f'<text x="10" y="{y + 5}">{upper:.3g}</text><text x="40" y="{y + 85}">0</text>'
        f'<path d="M70 {y} V{y + 84} H907" fill="none" stroke="#94a3b8"/>'
        f'<text x="70" y="{y + 105}">0 s</text><text x="907" y="{y + 105}" text-anchor="end">{duration:.3f} s</text>'
        + marks
    )


def write_depth_report(run: Path, count: int, max_depth_m: float) -> None:
    """Show the last captured pair and temporal changes over the episode."""
    fill_rates, temporal_rms = [], []
    previous_error = previous_mask = None
    for index in range(count):
        name = f"depth_{index:08d}.npz"
        with (
            np.load(run / "observations" / name, allow_pickle=False) as observation,
            np.load(run / "ground_truth" / name, allow_pickle=False) as ground_truth,
        ):
            measured, noisy_mask = observation["depth_m"], observation["valid_mask"]
            clean, clean_mask = ground_truth["depth_m"], ground_truth["valid_mask"]
            duration = int(observation["capture_time_ns"]) / 1e9
        valid = clean_mask & noisy_mask
        error = measured - clean
        overlap = (
            np.zeros_like(valid) if previous_mask is None else valid & previous_mask
        )
        delta = (
            np.zeros_like(error) if previous_error is None else error - previous_error
        )
        fill_rates.append(float(noisy_mask.mean()) * 100)
        temporal_rms.append(
            float(np.sqrt(np.mean(delta[overlap] ** 2))) * 1000
            if overlap.any()
            else np.nan
        )
        previous_error, previous_mask = error, valid

    mask_rgb = lambda mask: np.repeat(
        (mask[..., None] * 255).astype(np.uint8), 3, axis=-1
    )
    panels = (
        (
            f"Clean depth 0–{max_depth_m:g} m",
            _depth_color(clean, clean_mask, max_depth_m),
        ),
        (
            f"Noisy depth 0–{max_depth_m:g} m",
            _depth_color(measured, noisy_mask, max_depth_m),
        ),
        ("Spatial error ±100 mm", colorize_depth_error(error, valid, 0.1)),
        ("Clean validity mask", mask_rgb(clean_mask)),
        ("Noisy validity mask", mask_rgb(noisy_mask)),
        ("Temporal error change ±100 mm", colorize_depth_error(delta, overlap, 0.1)),
    )
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="960" height="735" viewBox="0 0 960 735">',
        '<rect width="960" height="735" fill="#f8fafc"/>',
        '<g font-family="sans-serif" font-size="14" fill="#0f172a">',
        f'<text x="20" y="30" font-size="22">D435i depth diagnostics · frame {count - 1} at {duration:.3f} s</text>',
        '<text x="20" y="54">Black = invalid. Error maps: blue negative, white zero, red positive.</text>',
    ]
    for i, (label, pixels) in enumerate(panels):
        x, y = 20 + i % 3 * 310, 82 + i // 3 * 182
        svg.append(f'<text x="{x}" y="{y}">{label}</text>')
        svg.append(
            f'<image x="{x}" y="{y + 9}" width="300" height="155" href="{_png_url(pixels)}"/>'
        )
    svg.append(
        _plot(fill_rates, "Fill rate (%) · valid / all pixels", 465, 100, duration)
    )
    rms_max = max((value for value in temporal_rms if np.isfinite(value)), default=0)
    svg.append(
        _plot(
            temporal_rms,
            "Temporal RMS error change (mm) · consecutive valid overlap",
            598,
            max(1, rms_max),
            duration,
        )
    )
    svg.append("</g></svg>")
    (run / "ground_truth" / "diagnostics.svg").write_text(
        "\n".join(svg), encoding="utf-8"
    )
