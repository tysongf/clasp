"""6-DOF retroreflective fiducial pose estimator, [CLASP-141] DataMatrix decoder, and clasp_ledger bridge."""

from __future__ import annotations

import itertools
import json
import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import zxingcpp

from cad.clasp_m import ClaspMSpec
from perception.polar_sim import CameraIntrinsics, port_model_geometry

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_LEDGER_CLI = REPO_ROOT / "ledger" / "build" / "clasp_ledger_cli"


@dataclass
class PoseEstimate:
    success: bool
    rvec: np.ndarray | None
    tvec_mm: np.ndarray | None
    reproj_rmse_px: float
    detected_candidates: int
    used_fiducials: int
    latency_ms: float
    matched_uv: np.ndarray | None = None


@dataclass
class TagReadResult:
    decoded: bool
    raw_payload: str | None = None
    port_tier: str | None = None
    unique_serial: str | None = None
    rectified_patch: np.ndarray | None = None


def detect_fiducial_candidates(gray: np.ndarray) -> np.ndarray:
    """Extract subpixel centroids of circular retroreflective candidates, rejecting solar glints."""
    # Cap adaptive threshold so bright solar glints cannot mask tilted retroreflective returns
    thresh_val = max(115, min(165, int(np.percentile(gray, 99.2))))
    _, bw = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates: list[tuple[float, float, float]] = []
    h_img, w_img = gray.shape
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 35.0 or area > 2500.0:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim <= 1e-6:
            continue
        circularity = 4.0 * math.pi * area / (perim * perim)
        if circularity < 0.60:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = max(w, h) / max(1, min(w, h))
        if aspect > 1.75:
            continue

        # Intensity-weighted subpixel centroid inside dilated ROI
        x0, x1 = max(0, x - 2), min(w_img, x + w + 2)
        y0, y1 = max(0, y - 2), min(h_img, y + h + 2)
        roi = gray[y0:y1, x0:x1].astype(np.float64)
        bg = np.percentile(roi, 20.0)
        weights = np.maximum(0.0, roi - bg)
        w_sum = float(weights.sum())
        if w_sum <= 1e-6:
            continue
        yy, xx = np.mgrid[y0:y1, x0:x1]
        cx = float((xx * weights).sum() / w_sum)
        cy = float((yy * weights).sum() / w_sum)
        score = float(roi.max()) * circularity
        candidates.append((cx, cy, score))

    # Keep top 8 highest-scoring circular candidates
    candidates.sort(key=lambda item: item[2], reverse=True)
    if not candidates:
        return np.empty((0, 2), dtype=np.float64)
    return np.array([[c[0], c[1]] for c in candidates[:8]], dtype=np.float64)


def _cyclic_order_indices(pts2d: np.ndarray) -> np.ndarray:
    """Sort 2D points in clockwise image-plane angular order around their centroid."""
    center = pts2d.mean(axis=0)
    angles = np.arctan2(pts2d[:, 1] - center[1], pts2d[:, 0] - center[0])
    return np.argsort(angles)


def _eval_pnp_hypothesis(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    K: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray] | None:
    """Solve PnP and validate physical depth + front-facing normal + reprojection RMSE."""
    flag = cv2.SOLVEPNP_SQPNP if len(obj_pts) >= 4 else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, None, flags=flag)
    if not ok:
        return None
    tz = float(tvec[2, 0])
    if tz < 120.0 or tz > 700.0:
        return None
    R, _ = cv2.Rodrigues(rvec)
    # Port +Z must face toward camera (-Z_cam)
    if (R @ np.array([0.0, 0.0, 1.0]))[2] > -0.25:
        return None

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, None)
    err = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    if rmse > 3.5:
        return None
    return rmse, rvec, tvec


def estimate_port_pose(
    bgr: np.ndarray,
    cam: CameraIntrinsics,
    spec: ClaspMSpec = ClaspMSpec(),
) -> PoseEstimate:
    """Estimate 6-DOF pose (rvec, tvec_mm) of the CLASP port from [CLASP-140] fiducials."""
    t0 = time.perf_counter()
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    K = cam.matrix
    model_pts = port_model_geometry(spec)["fiducials"]  # N = 5 points ordered CCW in port XY
    n_model = len(model_pts)

    cands = detect_fiducial_candidates(gray)
    if len(cands) < 4:
        return PoseEstimate(False, None, None, math.inf, len(cands), 0, (time.perf_counter() - t0) * 1e3)

    best: tuple[float, np.ndarray, np.ndarray, np.ndarray, int] | None = None

    # First try 5-point matches (all 5 fiducials visible)
    if len(cands) >= n_model:
        for subset in itertools.combinations(range(len(cands)), n_model):
            pts2d = cands[list(subset)]
            order = _cyclic_order_indices(pts2d)
            ordered_2d = pts2d[order]
            # Port +Z faces -Z_cam -> CCW port points appear CW in image (or mirrored if rolled);
            # check cyclic shifts of model_pts (and reversed order just in case)
            for seq in (model_pts, model_pts[::-1]):
                for shift in range(n_model):
                    shifted = np.roll(seq, shift, axis=0)
                    res = _eval_pnp_hypothesis(shifted, ordered_2d, K)
                    if res is not None:
                        rmse, rvec, tvec = res
                        if best is None or rmse < best[0]:
                            best = (rmse, rvec, tvec, ordered_2d, n_model)
            if best is not None and best[0] < 1.2:
                break

    # Fallback to 4-point matches (tolerates 1 occluded/shadowed fiducial)
    if best is None:
        for model_sub in itertools.combinations(range(n_model), 4):
            sub_3d = model_pts[list(model_sub)]
            for cand_sub in itertools.combinations(range(len(cands)), 4):
                pts2d = cands[list(cand_sub)]
                order = _cyclic_order_indices(pts2d)
                ordered_2d = pts2d[order]
                for seq in (sub_3d, sub_3d[::-1]):
                    for shift in range(4):
                        shifted = np.roll(seq, shift, axis=0)
                        res = _eval_pnp_hypothesis(shifted, ordered_2d, K)
                        if res is not None:
                            rmse, rvec, tvec = res
                            if best is None or rmse < best[0]:
                                best = (rmse, rvec, tvec, ordered_2d, 4)

    if best is None:
        return PoseEstimate(False, None, None, math.inf, len(cands), 0, (time.perf_counter() - t0) * 1e3)

    rmse, rvec, tvec, matched_uv, used_n = best
    # Iterative Levenberg-Marquardt refinement using the matched correspondence
    # Re-project all 5 model points to confirm consistency and refine
    proj_all, _ = cv2.projectPoints(model_pts, rvec, tvec, K, None)
    proj_all_2d = proj_all.reshape(-1, 2)
    matched_3d = []
    matched_2d = []
    for i in range(n_model):
        dists = np.linalg.norm(cands - proj_all_2d[i], axis=1)
        j = int(np.argmin(dists))
        if dists[j] < 6.0:
            matched_3d.append(model_pts[i])
            matched_2d.append(cands[j])
    if len(matched_3d) >= 4:
        obj_ref = np.array(matched_3d, dtype=np.float64)
        img_ref = np.array(matched_2d, dtype=np.float64)
        ok_ref, rvec_ref, tvec_ref = cv2.solvePnP(
            obj_ref, img_ref, K, None, rvec=rvec, tvec=tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if ok_ref:
            rvec, tvec = rvec_ref, tvec_ref
            proj2, _ = cv2.projectPoints(obj_ref, rvec, tvec, K, None)
            rmse = float(np.sqrt(np.mean(np.sum((proj2.reshape(-1, 2) - img_ref) ** 2, axis=1))))
            used_n = len(obj_ref)
            matched_uv = img_ref

    return PoseEstimate(
        True,
        rvec,
        tvec,
        rmse,
        len(cands),
        used_n,
        (time.perf_counter() - t0) * 1e3,
        matched_uv=matched_uv,
    )


def decode_datamatrix_from_pose(
    bgr: np.ndarray,
    cam: CameraIntrinsics,
    pose: PoseEstimate,
    spec: ClaspMSpec = ClaspMSpec(),
) -> TagReadResult:
    """Warp the [CLASP-141] DataMatrix pad to frontal view using estimated 6-DOF pose and decode."""
    if not pose.success or pose.rvec is None or pose.tvec_mm is None:
        return TagReadResult(False)

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    dm_corners = port_model_geometry(spec)["dm_corners"]
    uv, _ = cv2.projectPoints(dm_corners, pose.rvec, pose.tvec_mm, cam.matrix, None)
    src_quad = uv.reshape(-1, 2).astype(np.float32)

    patch_px = 160
    dst_quad = np.array([[0, 0], [patch_px - 1, 0], [patch_px - 1, patch_px - 1], [0, patch_px - 1]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src_quad, dst_quad)
    rectified = cv2.warpPerspective(gray, H, (patch_px, patch_px), flags=cv2.INTER_CUBIC)

    # Normalize contrast & add quiet-zone border matching bright anodized background
    p5, p95 = np.percentile(rectified, (5, 95))
    if p95 - p5 > 8.0:
        stretched = np.clip((rectified.astype(np.float32) - p5) * (255.0 / (p95 - p5)), 0, 255).astype(np.uint8)
    else:
        stretched = rectified

    bg_val = int(np.percentile(stretched, 85))
    padded = cv2.copyMakeBorder(stretched, 24, 24, 24, 24, cv2.BORDER_CONSTANT, value=bg_val)
    _, otsu = cv2.threshold(padded, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    for candidate_img in (padded, otsu):
        hits = zxingcpp.read_barcodes(candidate_img, formats=zxingcpp.BarcodeFormat.DataMatrix)
        if hits:
            text = hits[0].text
            if "|" in text:
                tier, serial = text.split("|", 1)
                return TagReadResult(True, text, tier, serial, rectified_patch=stretched)
            return TagReadResult(True, text, rectified_patch=stretched)

    return TagReadResult(False, rectified_patch=stretched)


class ClaspLedgerBridge:
    """Subprocess bridge to the compiled C++20 clasp_ledger_cli binary."""

    def __init__(self, db_path: Path, cli_path: Path = DEFAULT_LEDGER_CLI) -> None:
        self.db_path = db_path
        self.cli_path = cli_path

    def _run(self, *args: str) -> str:
        proc = subprocess.run(
            [str(self.cli_path), *args],
            check=True,
            capture_output=True,
            text=True,
        )
        return proc.stdout.strip()

    def init(self) -> None:
        self._run("init", str(self.db_path))

    def parse_tag(self, payload: str) -> str:
        return self._run("parse-tag", payload)

    def register_port(
        self,
        serial: str,
        tier: str,
        contents: str,
        fill_state: str,
        cert: str = "CLASP-CERT-0001",
        fill_fraction: float | None = None,
        fluid_kg: float | None = None,
    ) -> str:
        cmd = ["register", str(self.db_path), serial, tier, contents, fill_state, "--cert", cert]
        if fill_fraction is not None:
            cmd += ["--fill-fraction", f"{fill_fraction:.3f}"]
        if fluid_kg is not None:
            cmd += ["--fluid-kg", f"{fluid_kg:.1f}"]
        return self._run(*cmd)

    def record_event(
        self,
        serial: str,
        effector_id: str,
        event_type: str,
        contents: str,
        fill_state: str,
        delta_cycles: int = 0,
        fill_fraction: float | None = None,
        fluid_kg: float | None = None,
        chilldown_k: float | None = None,
        trapped_loss_g: float | None = None,
        notes: str = "",
    ) -> str:
        cmd = [
            "record-event",
            str(self.db_path),
            serial,
            effector_id,
            event_type,
            contents,
            fill_state,
            "--delta-cycles",
            str(delta_cycles),
        ]
        if fill_fraction is not None:
            cmd += ["--fill-fraction", f"{fill_fraction:.3f}"]
        if fluid_kg is not None:
            cmd += ["--fluid-kg", f"{fluid_kg:.1f}"]
        if chilldown_k is not None:
            cmd += ["--chilldown-k", f"{chilldown_k:.1f}"]
        if trapped_loss_g is not None:
            cmd += ["--trapped-loss-g", f"{trapped_loss_g:.2f}"]
        if notes:
            cmd += ["--notes", notes]
        return self._run(*cmd)

    def export_json(self, with_events: bool = True) -> dict:
        args = ["export-json", str(self.db_path)]
        if with_events:
            args.append("--with-events")
        return json.loads(self._run(*args))

