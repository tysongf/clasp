"""Physically grounded synthetic lunar polar-shadow renderer for CLASP ports."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import zxingcpp

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_m import ClaspMSpec


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int = 1920
    height: int = 1440
    fx: float = 680.0
    fy: float = 680.0
    cx: float = 960.0
    cy: float = 720.0

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


def euler_zyx_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return 3x3 rotation matrix R = Rz(roll) @ Ry(pitch) @ Rx(yaw)."""
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cy, -sy], [0.0, sy, cy]])
    return rz @ ry @ rx


def port_model_geometry(spec: ClaspMSpec = ClaspMSpec()) -> dict[str, np.ndarray]:
    """Return 3D port landmarks in the port frame (Z = 0 at mounting plane)."""
    z_collar_top = spec.collar_z0_mm + spec.collar_thickness_mm  # 44.0 mm
    z_fid = z_collar_top + spec.fiducial_boss_height_mm  # 45.5 mm
    fiducials = np.array(
        [
            [
                spec.fiducial_radius_mm * math.cos(math.radians(a)),
                spec.fiducial_radius_mm * math.sin(math.radians(a)),
                z_fid,
            ]
            for a in spec.fiducial_angles_deg
        ],
        dtype=np.float64,
    )

    # [CLASP-141] 18 x 18 mm DataMatrix pad corners in port frame (Z = 45.0 mm)
    dm_rad = math.radians(spec.datamatrix_angle_deg)
    u = np.array([math.cos(dm_rad), math.sin(dm_rad), 0.0])
    v = np.array([-math.sin(dm_rad), math.cos(dm_rad), 0.0])
    dm_center = np.array(
        [
            spec.datamatrix_radius_mm * math.cos(dm_rad),
            spec.datamatrix_radius_mm * math.sin(dm_rad),
            z_collar_top + 1.0,
        ]
    )
    half = 9.0
    # Ordered: TL, TR, BR, BL in local patch coordinates
    dm_corners = np.array(
        [
            dm_center - half * u + half * v,
            dm_center + half * u + half * v,
            dm_center + half * u - half * v,
            dm_center - half * u - half * v,
        ],
        dtype=np.float64,
    )
    return {
        "fiducials": fiducials,
        "dm_corners": dm_corners,
        "z_collar_top": np.array([z_collar_top]),
    }


def make_datamatrix_texture(payload: str, size_px: int = 160) -> np.ndarray:
    """Generate a high-contrast laser-etched DataMatrix texture."""
    bc = zxingcpp.create_barcode(payload, zxingcpp.BarcodeFormat.DataMatrix)
    img = np.asarray(zxingcpp.write_barcode_to_image(bc, scale=8, add_quiet_zones=True), dtype=np.uint8)
    # Dark anodized etch on bare machined aluminum background
    resized = cv2.resize(img, (size_px, size_px), interpolation=cv2.INTER_NEAREST)
    tex = np.where(resized < 128, 28, 210).astype(np.uint8)
    return tex


def render_polar_scene(
    cam: CameraIntrinsics,
    r_port_to_cam: np.ndarray,
    t_port_in_cam_mm: np.ndarray,
    tag_payload: str = "CLASP-F|TANK-LOX-001",
    sun_azimuth_deg: float = 35.0,
    sun_elevation_deg: float = 2.5,
    led_ring_power: float = 1.0,
    occluded_fiducial_idx: int | None = None,
    glint_count: int = 2,
    noise_sigma: float = 3.5,
    seed: int = 0,
    spec: ClaspMSpec = ClaspMSpec(),
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Render a synthetic lunar polar-shadow frame of the CLASP port.

    Returns (bgr_image, ground_truth_info).
    """
    rng = np.random.default_rng(seed)
    K = cam.matrix
    rvec, _ = cv2.Rodrigues(r_port_to_cam)
    geom = port_model_geometry(spec)
    z_top = float(geom["z_collar_top"][0])

    # Sun direction in port frame (grazing 0-5 deg elevation)
    az = math.radians(sun_azimuth_deg)
    el = math.radians(sun_elevation_deg)
    sun_dir_port = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])

    # Background: near-black regolith crater floor with grazing sunlit rim speckles
    frame = np.zeros((cam.height, cam.width), dtype=np.float32)
    yy, xx = np.mgrid[0 : cam.height, 0 : cam.width]
    # Subtle regolith micro-texture only on far sunlit floor strips (pitch black in shadow)
    regolith = rng.normal(8.0, 4.5, (cam.height // 4, cam.width // 4)).astype(np.float32)
    regolith = cv2.resize(regolith, (cam.width, cam.height), interpolation=cv2.INTER_LINEAR)
    sun_mask_bg = ((xx - cam.cx) * math.cos(az) + (yy - cam.cy) * math.sin(az)) > 160
    frame[sun_mask_bg] = np.clip(regolith[sun_mask_bg] * 2.2, 0, 42)

    def proj(pts3d: np.ndarray) -> np.ndarray:
        uv, _ = cv2.projectPoints(pts3d.astype(np.float64), rvec, t_port_in_cam_mm.astype(np.float64), K, None)
        return uv.reshape(-1, 2)

    # Helper to draw 3D circle polygons
    thetas = np.linspace(0.0, 2.0 * math.pi, 96, endpoint=False)

    def ring_pts(radius: float, z: float) -> np.ndarray:
        return np.column_stack([radius * np.cos(thetas), radius * np.sin(thetas), np.full_like(thetas, z)])

    # Base flange shadow body & upper collar under harsh grazing sun + weak active LED fill
    port_normal_cam = r_port_to_cam @ np.array([0.0, 0.0, 1.0])
    view_cos = max(0.15, float(-port_normal_cam[2]))  # camera looks along +Z_cam; port +Z faces -Z_cam
    led_fill = 42.0 * led_ring_power * (300.0 / max(150.0, float(t_port_in_cam_mm[2]))) ** 2 * view_cos
    sun_face = 185.0 * max(0.0, float(sun_dir_port[2]))  # grazing 2.5 deg -> ~8 DN!

    base_pts = np.round(proj(ring_pts(120.0, 0.0))).astype(np.int32)
    cv2.fillPoly(frame, [base_pts], 4.0)  # deep shadow under flange

    collar_outer = np.round(proj(ring_pts(120.0, z_top))).astype(np.int32)
    collar_inner = np.round(proj(ring_pts(74.0, z_top))).astype(np.int32)

    # Sunlit crescent vs black self-shadow across the collar rim
    collar_mask = np.zeros_like(frame, dtype=np.uint8)
    cv2.fillPoly(collar_mask, [collar_outer], 255)
    cv2.fillPoly(collar_mask, [collar_inner], 0)

    # Compute pixel-wise directional shadow cast by the 52.76 mm stepped port rim
    # Sunlit side faces the sun azimuth; shadowed side drops to pure LED fill
    center_uv = proj(np.array([[0.0, 0.0, z_top]]))[0]
    sun_uv_dir = proj(np.array([[sun_dir_port[0] * 40.0, sun_dir_port[1] * 40.0, z_top]]))[0] - center_uv
    sun_uv_norm = sun_uv_dir / max(1e-6, float(np.linalg.norm(sun_uv_dir)))
    signed_proj = (xx - center_uv[0]) * sun_uv_norm[0] + (yy - center_uv[1]) * sun_uv_norm[1]
    collar_intensity = np.where(signed_proj > -15.0, led_fill + sun_face + 18.0, led_fill * 0.65)
    frame[collar_mask > 0] = np.clip(collar_intensity[collar_mask > 0], 0, 255)

    # Central Ø148 mm deep dark cavity (shadow trap)
    bore_wall = np.round(proj(ring_pts(74.0, 0.0))).astype(np.int32)
    cv2.fillPoly(frame, [bore_wall], 2.0)

    # 8 x M8 bolt wrench-access black holes
    for ang in spec.bolt_angles_deg:
        cx_b = 105.0 * math.cos(math.radians(ang))
        cy_b = 105.0 * math.sin(math.radians(ang))
        hole_3d = np.column_stack(
            [
                cx_b + 8.0 * np.cos(thetas[:32]),
                cy_b + 8.0 * np.sin(thetas[:32]),
                np.full(32, z_top),
            ]
        )
        cv2.fillPoly(frame, [np.round(proj(hole_3d)).astype(np.int32)], 3.0)

    # 3 helical cam ramps with grazing solar highlight on sun-facing edge
    for idx, cam_ang in enumerate((0.0, 120.0, 240.0)):
        r_in = spec.cam_primary_r_inner_mm if idx == 0 else spec.cam_r_inner_mm
        r_out = spec.cam_primary_r_outer_mm if idx == 0 else spec.cam_r_outer_mm
        a0, a1 = math.radians(cam_ang - 10.0), math.radians(cam_ang + 10.0)
        arc = np.linspace(a0, a1, 10)
        cam_poly = np.vstack(
            [
                np.column_stack([r_in * np.cos(arc), r_in * np.sin(arc), np.full_like(arc, z_top + 4.0)]),
                np.column_stack([r_out * np.cos(arc[::-1]), r_out * np.sin(arc[::-1]), np.full_like(arc, z_top + 4.0)]),
            ]
        )
        cam_uv = np.round(proj(cam_poly)).astype(np.int32)
        dot_sun = math.cos(math.radians(cam_ang) - az)
        ramp_val = 235.0 if dot_sun > 0.35 else (led_fill * 0.7 if dot_sun > -0.2 else 4.0)
        cv2.fillPoly(frame, [cam_uv], ramp_val)

    # [CLASP-141] DataMatrix plate illuminated by active on-axis LED ring
    dm_tex = make_datamatrix_texture(tag_payload, size_px=160)
    dm_uv = proj(geom["dm_corners"]).astype(np.float32)
    src_dm = np.array([[0, 0], [159, 0], [159, 159], [0, 159]], dtype=np.float32)
    H_dm = cv2.getPerspectiveTransform(src_dm, dm_uv)
    dm_warped = cv2.warpPerspective(dm_tex.astype(np.float32), H_dm, (cam.width, cam.height), flags=cv2.INTER_LINEAR)
    dm_mask = cv2.warpPerspective(np.full_like(dm_tex, 255, dtype=np.uint8), H_dm, (cam.width, cam.height))
    # Modulate DataMatrix contrast by LED illumination (strong even in deep shadow)
    dm_lit = dm_warped * (0.35 + 0.55 * led_ring_power * view_cos)
    frame[dm_mask > 128] = np.clip(dm_lit[dm_mask > 128], 0, 255)

    # [CLASP-140] 5 Asymmetric retroreflective fiducials
    fid_uv = proj(geom["fiducials"])
    # Apparent radius in pixels for 10 mm diameter button
    r_px = 5.0 * cam.fx / max(100.0, float(t_port_in_cam_mm[2]))
    retro_gain = 250.0 * led_ring_power * (view_cos ** 1.2)
    visible_fiducials = np.ones(len(fid_uv), dtype=bool)
    if occluded_fiducial_idx is not None and 0 <= occluded_fiducial_idx < len(fid_uv):
        visible_fiducials[occluded_fiducial_idx] = False

    for i, (u_f, v_f) in enumerate(fid_uv):
        if not visible_fiducials[i]:
            # Dust/shadow-covered fiducial -> dim noise only
            continue
        x0 = max(0, int(math.floor(u_f - 2.5 * r_px)))
        x1 = min(cam.width, int(math.ceil(u_f + 2.5 * r_px)) + 1)
        y0 = max(0, int(math.floor(v_f - 2.5 * r_px)))
        y1 = min(cam.height, int(math.ceil(v_f + 2.5 * r_px)) + 1)
        if x1 <= x0 or y1 <= y0:
            continue
        sub_x = xx[y0:y1, x0:x1] - u_f
        sub_y = yy[y0:y1, x0:x1] - v_f
        dist2 = sub_x ** 2 + sub_y ** 2
        # Flat-top disc + slight optical PSF blur
        disc = np.clip(1.0 - np.maximum(0.0, np.sqrt(dist2) - r_px * 0.75) / max(1.0, 0.4 * r_px), 0.0, 1.0)
        frame[y0:y1, x0:x1] = np.maximum(frame[y0:y1, x0:x1], disc * retro_gain)

    # Inject harsh non-circular solar specular glints on sunlit rim to stress-test blob rejection
    for _ in range(glint_count):
        ang_g = az + rng.uniform(-0.7, 0.7)
        rad_g = rng.uniform(76.0, 118.0)
        glint_3d = np.array([[rad_g * math.cos(ang_g), rad_g * math.sin(ang_g), z_top]])
        gu, gv = proj(glint_3d)[0]
        if 20 <= gu < cam.width - 20 and 20 <= gv < cam.height - 20:
            cv2.ellipse(
                frame,
                (int(round(gu)), int(round(gv))),
                (int(rng.integers(10, 22)), int(rng.integers(2, 4))),
                float(rng.uniform(0, 180)),
                0,
                360,
                255.0,
                -1,
            )

    # Sensor read + shot noise
    frame += rng.normal(0.0, noise_sigma, frame.shape).astype(np.float32)
    u8 = np.clip(np.round(frame), 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(u8, cv2.COLOR_GRAY2BGR)
    return bgr, {
        "fiducials_uv": fid_uv,
        "visible_fiducials": visible_fiducials,
        "dm_corners_uv": dm_uv,
    }

