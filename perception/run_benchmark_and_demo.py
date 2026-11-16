#!/usr/bin/env python3
"""Benchmark CLASP 6-DOF polar-shadow pose + DataMatrix reader and render demo video."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np

from cad.clasp_m import ClaspMSpec
from perception.polar_sim import (
    CameraIntrinsics,
    euler_zyx_to_matrix,
    port_model_geometry,
    render_polar_scene,
)
from perception.pose_and_tag import (
    ClaspLedgerBridge,
    decode_datamatrix_from_pose,
    estimate_port_pose,
)

OUT_DIR = Path(__file__).resolve().parent


def rotation_error_deg(R_est: np.ndarray, R_gt: np.ndarray) -> float:
    trace = np.clip((np.trace(R_est.T @ R_gt) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(trace)))


def run_benchmark(num_trials: int = 100) -> dict:
    """Stress-test 6-DOF pose and DataMatrix decode across the full §5.1 capture envelope."""
    cam = CameraIntrinsics()
    spec = ClaspMSpec()
    rng = np.random.default_rng(42)

    # Nominal camera-to-port facing rotation (port +Z points toward camera -Z_cam)
    R_base = np.diag([1.0, -1.0, -1.0])
    tag_payload = "CLASP-F|TANK-LOX-001"

    pos_errs_mm: list[float] = []
    rot_errs_deg: list[float] = []
    latencies_ms: list[float] = []
    pose_ok = 0
    tag_ok = 0
    occluded_trials = 0
    occluded_pose_ok = 0

    for i in range(num_trials):
        # Include 16 extreme envelope corners in first 16 trials, random across envelope thereafter
        if i < 16:
            sx = 100.0 if (i & 1) else -100.0
            sy = 100.0 if (i & 2) else -100.0
            sz = 350.0 if (i & 4) else 250.0
            pitch = 15.0 if (i & 8) else -15.0
            yaw = 15.0 if ((i >> 1) & 1) else -15.0
            roll = 20.0 if ((i >> 2) & 1) else -20.0
        else:
            sx = float(rng.uniform(-100.0, 100.0))
            sy = float(rng.uniform(-100.0, 100.0))
            sz = float(rng.uniform(250.0, 350.0))
            pitch = float(rng.uniform(-15.0, 15.0))
            yaw = float(rng.uniform(-15.0, 15.0))
            roll = float(rng.uniform(-20.0, 20.0))

        R_gt = euler_zyx_to_matrix(roll, pitch, yaw) @ R_base
        t_gt = np.array([sx, sy, sz], dtype=np.float64)

        # Every 4th random trial (25 total) completely occludes 1 fiducial with dust/shadow
        occ_idx = int(i % 5) if (i >= 16 and i % 4 == 0) else None
        if occ_idx is not None:
            occluded_trials += 1

        bgr, _ = render_polar_scene(
            cam,
            R_gt,
            t_gt,
            tag_payload=tag_payload,
            sun_azimuth_deg=float(rng.uniform(0.0, 360.0)),
            sun_elevation_deg=float(rng.uniform(0.5, 5.0)),
            occluded_fiducial_idx=occ_idx,
            glint_count=2,
            noise_sigma=4.0,
            seed=1000 + i,
            spec=spec,
        )

        pose = estimate_port_pose(bgr, cam, spec)
        latencies_ms.append(pose.latency_ms)

        if pose.success and pose.rvec is not None and pose.tvec_mm is not None:
            R_est, _ = cv2.Rodrigues(pose.rvec)
            t_est = pose.tvec_mm.reshape(3)
            p_err = float(np.linalg.norm(t_est - t_gt))
            r_err = rotation_error_deg(R_est, R_gt)
            if p_err < 10.0 and r_err < 3.0:
                pose_ok += 1
                if occ_idx is not None:
                    occluded_pose_ok += 1
            pos_errs_mm.append(p_err)
            rot_errs_deg.append(r_err)

            tag = decode_datamatrix_from_pose(bgr, cam, pose, spec)
            if tag.decoded and tag.raw_payload == tag_payload:
                tag_ok += 1

    summary = {
        "trials": num_trials,
        "pose_success_rate_pct": round(100.0 * pose_ok / num_trials, 2),
        "occluded_trials": occluded_trials,
        "occluded_success_rate_pct": round(100.0 * occluded_pose_ok / max(1, occluded_trials), 2),
        "datamatrix_decode_rate_pct": round(100.0 * tag_ok / num_trials, 2),
        "pos_err_median_mm": round(float(np.median(pos_errs_mm)), 3) if pos_errs_mm else None,
        "pos_err_p95_mm": round(float(np.percentile(pos_errs_mm, 95)), 3) if pos_errs_mm else None,
        "rot_err_median_deg": round(float(np.median(rot_errs_deg)), 3) if rot_errs_deg else None,
        "rot_err_p95_deg": round(float(np.percentile(rot_errs_deg, 95)), 3) if rot_errs_deg else None,
        "latency_median_ms": round(float(np.median(latencies_ms)), 2),
        "latency_p95_ms": round(float(np.percentile(latencies_ms, 95)), 2),
    }

    out_json = OUT_DIR / "benchmark_results.json"
    out_json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def draw_pose_axes(img: np.ndarray, cam: CameraIntrinsics, rvec: np.ndarray, tvec: np.ndarray) -> None:
    """Draw 6-DOF port coordinate frame (+X red, +Y green, +Z blue) at port center."""
    geom = port_model_geometry()
    z_top = float(geom["z_collar_top"][0])
    pts_3d = np.array(
        [
            [0.0, 0.0, z_top],
            [65.0, 0.0, z_top],
            [0.0, 65.0, z_top],
            [0.0, 0.0, z_top + 65.0],
        ],
        dtype=np.float64,
    )
    uv, _ = cv2.projectPoints(pts_3d, rvec, tvec, cam.matrix, None)
    p0, px, py, pz = [tuple(np.round(p).astype(int)) for p in uv.reshape(-1, 2)]
    cv2.line(img, p0, px, (60, 80, 255), 3, cv2.LINE_AA)   # +X red
    cv2.line(img, p0, py, (80, 235, 110), 3, cv2.LINE_AA)  # +Y green
    cv2.line(img, p0, pz, (255, 190, 50), 3, cv2.LINE_AA)  # +Z blue


def render_demo_video() -> Path:
    """Render 6-second 1280x720 closed-loop lunar polar capture + ledger sync demo."""
    cam = CameraIntrinsics()
    spec = ClaspMSpec()
    R_base = np.diag([1.0, -1.0, -1.0])

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        db_path = tmp / "demo_fleet.db"
        ledger = ClaspLedgerBridge(db_path)
        ledger.init()
        ledger.register_port(
            serial="TANK-LOX-001",
            tier="CLASP-F",
            contents="LOX",
            fill_state="PARTIAL",
            cert="CLASP-CERT-0042",
            fill_fraction=0.38,
            fluid_kg=550.0,
        )
        # Seed 12 prior cycles so telemetry looks realistic
        for c in range(12):
            ledger.record_event(
                serial="TANK-LOX-001",
                effector_id="ARM-SEED",
                event_type="DEMATE",
                contents="LOX",
                fill_state="PARTIAL",
                delta_cycles=1,
                fill_fraction=0.38,
                fluid_kg=550.0,
            )

        port_state = ledger.export_json(with_events=False)["ports"][0]
        tag_payload = "CLASP-F|TANK-LOX-001"

        n_frames = 180  # 6 seconds @ 30 fps
        frames_dir = tmp / "frames"
        frames_dir.mkdir()
        mate_logged = False
        transfer_logged = False
        last_good_tag = None

        for f_idx in range(n_frames):
            # Phase 1 (0..119): Closed-loop approach from extreme §5.1 corner to coaxial alignment
            # Phase 2 (120..149): Hard-latch hold & chilldown verification
            # Phase 3 (150..179): LOX transfer complete & ledger incremented
            u = min(1.0, f_idx / 115.0)
            # Smooth cubic ease-in-out
            s = 3.0 * u * u - 2.0 * u * u * u

            sx = 92.0 * (1.0 - s)
            sy = -78.0 * (1.0 - s)
            sz = 345.0 - 150.0 * s  # 345 mm -> 195 mm standoff
            pitch = 14.0 * (1.0 - s)
            yaw = -12.0 * (1.0 - s)
            roll = 18.0 * (1.0 - s)

            if f_idx == 120 and not mate_logged:
                ledger.record_event(
                    serial="TANK-LOX-001",
                    effector_id="CLASP-A-01",
                    event_type="MATE",
                    contents="LOX",
                    fill_state="CHILLING",
                    delta_cycles=1,
                    fill_fraction=0.38,
                    fluid_kg=550.0,
                    chilldown_k=116.4,
                    notes="Hard latch + active flange chilldown verified",
                )
                port_state = ledger.export_json(with_events=False)["ports"][0]
                mate_logged = True

            if f_idx == 150 and not transfer_logged:
                ledger.record_event(
                    serial="TANK-LOX-001",
                    effector_id="CLASP-A-01",
                    event_type="TRANSFER",
                    contents="LOX",
                    fill_state="FULL",
                    delta_cycles=0,
                    fill_fraction=0.98,
                    fluid_kg=1420.0,
                    chilldown_k=91.2,
                    trapped_loss_g=4.6,
                    notes="2.0 kg/s LOX transfer complete",
                )
                port_state = ledger.export_json(with_events=False)["ports"][0]
                transfer_logged = True

            R_gt = euler_zyx_to_matrix(roll, pitch, yaw) @ R_base
            t_gt = np.array([sx, sy, sz], dtype=np.float64)

            bgr, _ = render_polar_scene(
                cam,
                R_gt,
                t_gt,
                tag_payload=tag_payload,
                sun_azimuth_deg=32.0 + 0.15 * f_idx,
                sun_elevation_deg=2.2,
                occluded_fiducial_idx=(2 if 40 <= f_idx <= 75 else None),  # Show mid-approach dust occlusion resilience
                glint_count=2,
                noise_sigma=3.2,
                seed=f_idx,
                spec=spec,
            )

            pose = estimate_port_pose(bgr, cam, spec)
            tag = decode_datamatrix_from_pose(bgr, cam, pose, spec)
            if tag.decoded:
                last_good_tag = tag

            # Overlay pose markers & DataMatrix bounding quad
            if pose.success and pose.rvec is not None and pose.tvec_mm is not None:
                draw_pose_axes(bgr, cam, pose.rvec, pose.tvec_mm)
                if pose.matched_uv is not None:
                    for u_pt, v_pt in pose.matched_uv:
                        cv2.circle(bgr, (int(round(u_pt)), int(round(v_pt))), 11, (0, 235, 180), 2, cv2.LINE_AA)
                dm_c = port_model_geometry(spec)["dm_corners"]
                dm_uv, _ = cv2.projectPoints(dm_c, pose.rvec, pose.tvec_mm, cam.matrix, None)
                cv2.polylines(bgr, [np.round(dm_uv.reshape(-1, 2)).astype(np.int32)], True, (255, 200, 40), 2, cv2.LINE_AA)

            # Compose 1280x720 HUD: 960x720 camera view + 320x720 telemetry/DataMatrix/ledger panel
            cam_resized = cv2.resize(bgr, (960, 720), interpolation=cv2.INTER_AREA)
            hud = np.zeros((720, 1280, 3), dtype=np.uint8)
            hud[:, :960] = cam_resized
            hud[:, 960:] = (24, 28, 34)
            cv2.line(hud, (960, 0), (960, 720), (65, 75, 90), 2)

            def put(text: str, y: int, color=(230, 235, 242), scale=0.52, thick=1):
                cv2.putText(hud, text, (978, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

            put("CLASP-A VISION + LEDGER", 34, (80, 220, 255), 0.62, 2)
            stage_str = "SOFT CAPTURE APPROACH" if f_idx < 120 else ("HARD LATCH + CHILLDOWN" if f_idx < 150 else "LOX TRANSFER COMPLETE")
            stage_col = (80, 220, 255) if f_idx < 120 else ((80, 235, 140) if f_idx < 150 else (110, 255, 120))
            put(stage_str, 64, stage_col, 0.50, 2)

            if pose.success and pose.tvec_mm is not None and pose.rvec is not None:
                R_est, _ = cv2.Rodrigues(pose.rvec)
                t_est = pose.tvec_mm.reshape(3)
                p_err = float(np.linalg.norm(t_est - t_gt))
                r_err = rotation_error_deg(R_est, R_gt)
                put(f"Pose Lock: {pose.used_fiducials}/5 fiducials", 104, (110, 255, 140))
                put(f"X/Y/Z: {t_est[0]:+5.1f} / {t_est[1]:+5.1f} / {t_est[2]:5.1f} mm", 132)
                put(f"Err  : {p_err:.2f} mm | {r_err:.2f} deg", 158, (185, 220, 255))
                put(f"RMSE : {pose.reproj_rmse_px:.2f} px ({pose.latency_ms:.1f} ms)", 184)
            else:
                put("Pose Lock: SEARCHING", 104, (80, 100, 255))

            if 40 <= f_idx <= 75:
                put("FAULT INJECT: 1 FIDUCIAL OCCLUDED", 214, (70, 180, 255), 0.45, 1)

            # Dewarped [CLASP-141] DataMatrix thumbnail
            put("[CLASP-141] DEWARPED TAG", 252, (80, 220, 255), 0.52, 2)
            patch = tag.rectified_patch if tag.rectified_patch is not None else (last_good_tag.rectified_patch if last_good_tag else None)
            if patch is not None:
                thumb = cv2.cvtColor(cv2.resize(patch, (132, 132), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
                hud[268:400, 980:1112] = thumb
                cv2.rectangle(hud, (980, 268), (1112, 400), (255, 200, 40), 2)
            active_tag = tag if tag.decoded else last_good_tag
            if active_tag and active_tag.decoded:
                cv2.putText(hud, "Decode: OK", (1126, 314), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (110, 255, 140), 1, cv2.LINE_AA)
                cv2.putText(hud, active_tag.port_tier or "", (1126, 344), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (235, 240, 245), 1, cv2.LINE_AA)
                cv2.putText(hud, active_tag.unique_serial or "", (1126, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (235, 240, 245), 1, cv2.LINE_AA)

            # Live C++ SQLite Fleet Ledger readout
            put("C++ FLEET LEDGER (SQLite)", 444, (80, 220, 255), 0.52, 2)
            put(f"Serial  : {port_state['unique_serial']}", 476)
            put(f"Tier    : {port_state['port_tier']}", 504)
            put(f"Contents: {port_state['payload_contents']}", 532)
            fill_pct = int(round((port_state["fill_fraction"] or 0.0) * 100))
            fill_col = (110, 255, 140) if port_state["fill_state"] == "FULL" else (80, 205, 255)
            put(f"Fill    : {port_state['fill_state']} ({fill_pct}%)", 560, fill_col)
            put(f"Mass    : {port_state['fluid_mass_kg']:.0f} kg", 588)
            put(f"Cycles  : {port_state['cycle_count']} / {port_state['rated_cycle_limit']}", 616)
            put(f"Effector: {port_state['last_effector_id']}", 644, (175, 185, 200), 0.46)
            put(f"Sun Elev: 2.2 deg (Polar Shadow)", 684, (165, 175, 190), 0.45)

            frame_path = frames_dir / f"frame_{f_idx:04d}.png"
            cv2.imwrite(str(frame_path), hud)
            if f_idx == 55:
                # Save keyframe showing misaligned approach + 1 occluded fiducial + DataMatrix decode
                cv2.imwrite(str(OUT_DIR / "demo_keyframe.png"), hud)

        mp4_path = OUT_DIR / "clasp_polar_capture_demo.mp4"
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-framerate",
                "30",
                "-i",
                str(frames_dir / "frame_%04d.png"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-crf",
                "20",
                str(mp4_path),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return mp4_path


def main() -> int:
    summary = run_benchmark(num_trials=100)
    print("=== CLASP §5.1 Polar-Shadow Perception & DataMatrix Benchmark (100 trials) ===")
    for k, v in summary.items():
        print(f"  {k:<28s}: {v}")

    mp4_path = render_demo_video()
    print(f"\nDemo video generated : {mp4_path}")
    print(f"Demo keyframe saved  : {OUT_DIR / 'demo_keyframe.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

