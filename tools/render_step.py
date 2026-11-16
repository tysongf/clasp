#!/usr/bin/env python3
"""Headless multi-view shaded Z-buffer renderer for CLASP STEP files.

Renders a 2x2 engineering inspection sheet (Isometric, Front X-Z, Side Y-Z,
Top X-Y) directly from a .step B-Rep file using build123d tessellation and a
vectorized NumPy per-pixel Z-buffer with Phong shading and depth-edge outlines.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from build123d import import_step

# Distinct engineering palette (BGR for OpenCV)
PALETTE_BGR = [
    (205, 198, 185),  # Machined aluminum silver
    (115, 105, 95),   # Titanium slate
    (45, 140, 240),   # Safety orange (jaws / moving parts)
    (210, 145, 55),   # Anodized steel blue (latches)
    (70, 200, 235),   # Gold/amber (core / coils)
    (195, 180, 65),   # Cyan-teal (camera / optics)
    (155, 120, 210),  # Muted magenta-violet
    (120, 205, 130),  # Sage green
]


def view_rotation_matrix(azim_deg: float, elev_deg: float) -> np.ndarray:
    """Return 3x3 world-to-camera rotation matrix (camera looks along -Z_cam, +Y_cam up)."""
    az = math.radians(azim_deg)
    el = math.radians(elev_deg)
    # Forward vector pointing from target to camera
    fx = math.cos(el) * math.cos(az)
    fy = math.cos(el) * math.sin(az)
    fz = math.sin(el)
    forward = np.array([fx, fy, fz], dtype=np.float64)
    up_world = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(np.dot(forward, up_world)) > 0.999:
        up_world = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    right = np.cross(up_world, forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)
    up /= np.linalg.norm(up)
    # Rows map world coords -> (right, up, depth_toward_camera)
    return np.vstack([right, up, forward])


def color_for_solid(solid, idx: int) -> tuple[int, int, int]:
    """Choose consistent semantic color based on solid geometry or index."""
    bb = solid.bounding_box()
    dx = bb.max.X - bb.min.X
    dy = bb.max.Y - bb.min.Y
    # Passive CLASP-M-DFM sits at Z = 0..52.5, diameter 240 mm
    if abs(bb.min.Z) < 0.5 and 238.0 <= dx <= 242.0 and 238.0 <= dy <= 242.0:
        return (210, 204, 195)  # Bright machined aluminum for passive port
    if dx > 250.0 or (140.0 <= dx <= 165.0 and bb.max.Z - bb.min.Z > 30.0):
        return (110, 102, 95)  # Chassis/carriage slate
    return PALETTE_BGR[(idx % (len(PALETTE_BGR) - 2)) + 2]


def tessellate_step(step_path: Path, tol: float = 0.35) -> tuple[list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]], tuple]:
    """Load STEP file and return tessellated meshes [(verts, tris, bgr), ...] and bbox."""
    shape = import_step(str(step_path))
    solids = list(shape.solids())
    if not solids:
        solids = [shape]

    meshes: list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]] = []
    for idx, s in enumerate(solids):
        verts_raw, tris_raw = s.tessellate(tolerance=tol, angular_tolerance=0.25)
        if not verts_raw or not tris_raw:
            continue
        verts = np.array([[v.X, v.Y, v.Z] for v in verts_raw], dtype=np.float64)
        tris = np.array(tris_raw, dtype=np.int32)
        meshes.append((verts, tris, color_for_solid(s, idx)))

    bb = shape.bounding_box()
    return meshes, (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)


def render_orthographic_view(
    meshes: list[tuple[np.ndarray, np.ndarray, tuple[int, int, int]]],
    center: np.ndarray,
    span_mm: float,
    azim_deg: float,
    elev_deg: float,
    width: int = 800,
    height: int = 600,
    title: str = "",
) -> np.ndarray:
    """Render a shaded orthographic view using an exact per-pixel NumPy Z-buffer."""
    R = view_rotation_matrix(azim_deg, elev_deg)
    scale = min(width, height) * 0.80 / max(span_mm, 1.0)

    img = np.full((height, width, 3), 245, dtype=np.uint8)
    zbuf = np.full((height, width), -1e12, dtype=np.float64)

    light1 = np.array([0.45, 0.65, 0.61], dtype=np.float64)
    light1 /= np.linalg.norm(light1)
    light2 = np.array([-0.6, -0.3, 0.74], dtype=np.float64)
    light2 /= np.linalg.norm(light2)

    for verts, tris, base_bgr in meshes:
        v_cam = (verts - center) @ R.T
        sx = width * 0.5 + v_cam[:, 0] * scale
        sy = height * 0.5 - v_cam[:, 1] * scale
        sz = v_cam[:, 2]

        p0 = v_cam[tris[:, 0]]
        p1 = v_cam[tris[:, 1]]
        p2 = v_cam[tris[:, 2]]
        normals = np.cross(p1 - p0, p2 - p0)
        n_norm = np.linalg.norm(normals, axis=1, keepdims=True)
        valid = n_norm[:, 0] > 1e-8
        normals = normals[valid] / n_norm[valid]
        tris_v = tris[valid]

        # Backface culling (normals facing camera have positive Z_cam)
        front = normals[:, 2] > -0.02
        normals = normals[front]
        tris_v = tris_v[front]
        if len(tris_v) == 0:
            continue

        # Two-light + ambient Phong shading
        diff1 = np.clip(normals @ light1, 0.0, 1.0)
        diff2 = np.clip(normals @ light2, 0.0, 1.0)
        # Specular highlight along camera-aligned half vector
        half_v = light1 + np.array([0.0, 0.0, 1.0])
        half_v /= np.linalg.norm(half_v)
        spec = (np.clip(normals @ half_v, 0.0, 1.0) ** 24) * 0.25
        shade = np.clip(0.28 + 0.62 * diff1 + 0.22 * diff2, 0.0, 1.15)

        b0, g0, r0 = base_bgr
        t_sx = sx[tris_v]
        t_sy = sy[tris_v]
        t_sz = sz[tris_v]

        for k in range(len(tris_v)):
            x0, x1, x2 = t_sx[k]
            y0, y1, y2 = t_sy[k]
            z0, z1, z2 = t_sz[k]

            minx = max(0, int(math.floor(min(x0, x1, x2))))
            maxx = min(width - 1, int(math.ceil(max(x0, x1, x2))))
            miny = max(0, int(math.floor(min(y0, y1, y2))))
            maxy = min(height - 1, int(math.ceil(max(y0, y1, y2))))
            if maxx < minx or maxy < miny:
                continue

            area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
            if abs(area) < 1e-4:
                continue

            px = np.arange(minx, maxx + 1, dtype=np.float64)[None, :] + 0.5
            py = np.arange(miny, maxy + 1, dtype=np.float64)[:, None] + 0.5

            w0 = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) / area
            w1 = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) / area
            w2 = 1.0 - w0 - w1

            inside = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
            if not np.any(inside):
                continue

            z_pix = w0 * z0 + w1 * z1 + w2 * z2
            z_sub = zbuf[miny : maxy + 1, minx : maxx + 1]
            closer = inside & (z_pix > z_sub)
            if not np.any(closer):
                continue

            z_sub[closer] = z_pix[closer]
            col = np.array(
                [
                    min(255, int(b0 * shade[k] + 255 * spec[k])),
                    min(255, int(g0 * shade[k] + 255 * spec[k])),
                    min(255, int(r0 * shade[k] + 255 * spec[k])),
                ],
                dtype=np.uint8,
            )
            img_sub = img[miny : maxy + 1, minx : maxx + 1]
            img_sub[closer] = col

    # Subtle depth-discontinuity silhouette lines for crisp engineering readability
    valid_z = zbuf > -1e11
    z_clean = np.where(valid_z, zbuf, np.nan)
    gx = np.abs(np.diff(z_clean, axis=1, prepend=np.nan))
    gy = np.abs(np.diff(z_clean, axis=0, prepend=np.nan))
    edge = np.nan_to_num(gx, nan=0.0) > 4.0
    edge |= np.nan_to_num(gy, nan=0.0) > 4.0
    # Also outline object-vs-background boundary
    bg_edge = valid_z & (
        (~np.roll(valid_z, 1, axis=0))
        | (~np.roll(valid_z, -1, axis=0))
        | (~np.roll(valid_z, 1, axis=1))
        | (~np.roll(valid_z, -1, axis=1))
    )
    img[edge | bg_edge] = (45, 50, 58)

    if title:
        cv2.putText(img, title, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (35, 42, 52), 2, cv2.LINE_AA)
    return img


def render_sheet(step_path: Path, out_png: Path, tol: float = 0.40) -> None:
    """Render 4-view inspection PNG sheet for step_path."""
    meshes, (xmin, xmax, ymin, ymax, zmin, zmax) = tessellate_step(step_path, tol=tol)
    center = np.array([0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax)], dtype=np.float64)
    span = max(xmax - xmin, ymax - ymin, zmax - zmin) * 1.15

    v_iso = render_orthographic_view(meshes, center, span, azim_deg=38.0, elev_deg=26.0, title="ISOMETRIC")
    v_front = render_orthographic_view(meshes, center, span, azim_deg=0.0, elev_deg=0.0, title="SIDE ELEVATION (+X)")
    v_side = render_orthographic_view(meshes, center, span, azim_deg=90.0, elev_deg=0.0, title="SIDE ELEVATION (+Y)")
    v_top = render_orthographic_view(meshes, center, span, azim_deg=0.0, elev_deg=90.0, title="TOP PLAN (+Z)")

    top_row = np.hstack([v_iso, v_front])
    bot_row = np.hstack([v_side, v_top])
    grid = np.vstack([top_row, bot_row])

    # Crisp divider lines and footer info banner
    h, w, _ = grid.shape
    cv2.line(grid, (w // 2, 0), (w // 2, h), (185, 192, 202), 2)
    cv2.line(grid, (0, h // 2), (w, h // 2), (185, 192, 202), 2)

    banner = np.full((52, w, 3), 30, dtype=np.uint8)
    info = (
        f"{step_path.name}  |  Solids: {len(meshes)}  |  "
        f"Envelope: {xmax - xmin:.1f} x {ymax - ymin:.1f} x {zmax - zmin:.1f} mm  "
        f"(Z: {zmin:.1f}..{zmax:.1f} mm)"
    )
    cv2.putText(banner, info, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (235, 240, 246), 2, cv2.LINE_AA)
    out = np.vstack([banner, grid])

    out_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_png), out)
    print(f"Rendered {out_png} ({out.shape[1]}x{out.shape[0]})")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render 4-view shaded PNG inspection sheet from a STEP file.")
    parser.add_argument("step_files", nargs="+", type=Path, help="Input .step file(s)")
    parser.add_argument("--out-dir", type=Path, default=Path("cad/renders"))
    parser.add_argument("--tol", type=float, default=0.40)
    args = parser.parse_args()

    for sp in args.step_files:
        out_png = args.out_dir / f"{sp.stem}_views.png"
        render_sheet(sp, out_png, tol=args.tol)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
