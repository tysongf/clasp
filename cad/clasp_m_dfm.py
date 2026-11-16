#!/usr/bin/env python3
"""Lathe + 3-Axis Mill DFM variant of CLASP-M for <= CAD 2,000 quoting.

Design-for-Manufacturing changes vs. cad/clasp_m.py:
  1. Eliminates all 7 radial side-entry waist windows (removes 4th/5th-axis indexing);
     replaces weight savings by turning the continuous axisymmetric waist down to
     172 mm OD (12 mm cylindrical wall, R = 74..86 mm).
  2. Entire OD/ID profile (240 mm flange, 172 mm waist, 240 mm collar, 148 mm bore,
     and 2 mm chamfers/groove radii) is turnable in a single CNC lathe setup.
  3. Replaces twisted 3D-lofted helical cams with planar 15.0 deg inclined ramp
     chords bounded by the cam annulus (3-axis top-down milling only).
  4. Retains 100% interface compatibility with CLASP-STD-001, CLASP-FP packaging,
     8 x M8 clocked bolts, 2 x Ø6 dowels, 5 fiducials, and 18 x 18 mm DataMatrix pad.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Cylinder,
    Locations,
    Mode,
    Plane,
    Polygon,
    Rectangle,
    chamfer,
    export_step,
    extrude,
    fillet,
)

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_m import AL_7075_T6_DENSITY_G_CM3, ClaspMSpec, verify_part


@dataclass(frozen=True)
class ClaspMDFMSpec(ClaspMSpec):
    """DFM-optimized parameters preserving 100% CLASP-STD-001 interface compatibility."""

    waist_od_mm: float = 172.0  # Continuous lathe-turned 12 mm wall (no radial cutouts)
    groove_fillet_mm: float = 2.0  # Standard lathe grooving tool nose radius
    cam_chord_len_mm: float = 28.0  # Planar 15 deg ramp length
    cam_primary_r_inner_mm: float = 75.0  # 1.0 mm land clear of Ø148 mm bore seam


def build_planar_cam_ramp(
    r_inner: float,
    r_outer: float,
    z_base: float,
    angle_deg: float,
    chord_len_mm: float,
    lead_angle_deg: float,
):
    """Build a 3-axis-friendly planar 15 deg inclined ramp bounded to [r_inner, r_outer]."""
    r_mid = 0.5 * (r_inner + r_outer)
    rise = chord_len_mm * math.tan(math.radians(lead_angle_deg))
    rad = math.radians(angle_deg)
    tx, ty = -math.sin(rad), math.cos(rad)
    rx, ry = math.cos(rad), math.sin(rad)

    # Embed 0.5 mm into collar top (z_base - 0.5) so boolean fuse has clean solid overlap
    z_embed = z_base - 0.5
    h_low = 1.5   # 1.0 mm above z_base
    h_high = h_low + rise
    half_c = 0.5 * chord_len_mm

    profile_plane = Plane(
        origin=(r_mid * rx, r_mid * ry, z_embed),
        x_dir=(tx, ty, 0.0),
        z_dir=(rx, ry, 0.0),
    )

    with BuildPart() as ramp_part:
        with BuildSketch(profile_plane):
            Polygon(
                (-half_c, 0.0),
                (half_c, 0.0),
                (half_c, h_high),
                (-half_c, h_low),
                align=None,
            )
        extrude(amount=(r_outer - r_inner) + 6.0, both=True)

        # Bound cleanly to concentric cam annulus [r_inner, r_outer]
        Cylinder(radius=r_outer, height=z_base + 30.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.INTERSECT)
        Cylinder(radius=r_inner, height=z_base + 30.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    return ramp_part.part


def build_clasp_m_dfm(spec: ClaspMDFMSpec = ClaspMDFMSpec()):
    """Construct the 2-setup (Lathe + 3-Axis Mill) DFM CLASP-M passive port."""
    r_outer = spec.outer_diameter_mm / 2.0
    r_inner = spec.inner_bore_diameter_mm / 2.0
    r_waist = spec.waist_od_mm / 2.0
    r_bcd = spec.bolt_circle_diameter_mm / 2.0
    waist_h = spec.collar_z0_mm - spec.base_flange_thickness_mm
    collar_top_z = spec.collar_z0_mm + spec.collar_thickness_mm

    with BuildPart() as part:
        # --- SETUP 1: Single CNC Lathe Turning Envelope ---
        Cylinder(radius=r_outer, height=spec.base_flange_thickness_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((0, 0, spec.base_flange_thickness_mm)):
            Cylinder(radius=r_waist, height=waist_h, align=(Align.CENTER, Align.CENTER, Align.MIN))
        with Locations((0, 0, spec.collar_z0_mm)):
            Cylinder(radius=r_outer, height=spec.collar_thickness_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Standard lathe grooving tool root fillets at waist corners (Z = 10 & Z = 32)
        root_edges = (
            part.edges()
            .filter_by(lambda e: abs(e.center().Z - spec.base_flange_thickness_mm) < 0.05 or abs(e.center().Z - spec.collar_z0_mm) < 0.05)
            .filter_by(lambda e: abs(math.hypot(e.center().X, e.center().Y) - r_waist) < 0.5)
        )
        fillet(root_edges, radius=spec.groove_fillet_mm)

        # [CLASP-117] Lathe-turned 2.0 mm chamfers on outer grapple/latch collar
        collar_edges = (
            part.edges()
            .filter_by(lambda e: abs(e.center().Z - spec.collar_z0_mm) < 0.01 or abs(e.center().Z - collar_top_z) < 0.01)
            .filter_by(lambda e: math.hypot(e.center().X, e.center().Y) > r_outer - 1.0)
        )
        chamfer(collar_edges, length=spec.collar_chamfer_mm)

        # Central bore turned in same lathe setup
        Cylinder(radius=r_inner, height=spec.max_height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # --- SETUP 2: Purely Top-Down (-Z) 3-Axis Milling / Drilling ---
        # [CLASP-011] 8 x M8 clocked mounting holes + top-collar wrench access holes
        for ang in spec.bolt_angles_deg:
            x = r_bcd * math.cos(math.radians(ang))
            y = r_bcd * math.sin(math.radians(ang))
            with Locations((x, y, 0)):
                Cylinder(
                    radius=spec.bolt_clearance_diameter_mm / 2.0,
                    height=spec.base_flange_thickness_mm + 2.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )
            with Locations((x, y, spec.collar_z0_mm - 1.0)):
                Cylinder(
                    radius=spec.bolt_head_access_diameter_mm / 2.0,
                    height=spec.collar_thickness_mm + 2.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )

        # 2 x Ø6 mm locating dowel pin holes
        for ang in spec.dowel_angles_deg:
            x = r_bcd * math.cos(math.radians(ang))
            y = r_bcd * math.sin(math.radians(ang))
            with Locations((x, y, 0)):
                Cylinder(
                    radius=spec.dowel_diameter_mm / 2.0,
                    height=spec.base_flange_thickness_mm + 1.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )

        # [CLASP-021] - [CLASP-023] 3 planar 15 deg cam ramps at 0, 120, 240 deg
        for idx, cam_ang in enumerate((0.0, 120.0, 240.0)):
            r_in = spec.cam_primary_r_inner_mm if idx == 0 else spec.cam_r_inner_mm
            r_out = spec.cam_primary_r_outer_mm if idx == 0 else spec.cam_r_outer_mm
            ramp = build_planar_cam_ramp(
                r_inner=r_in,
                r_outer=r_out,
                z_base=spec.cam_base_z_mm,
                angle_deg=cam_ang,
                chord_len_mm=spec.cam_chord_len_mm,
                lead_angle_deg=spec.cam_lead_angle_deg,
            )
            part.part = part.part.fuse(ramp)

        # [CLASP-023] Distinct clocking key boss on primary +X ramp (radially clear of annulus seam)
        with Locations((0.5 * (spec.cam_primary_r_inner_mm + spec.cam_primary_r_outer_mm), 0.0, spec.cam_base_z_mm - 0.5)):
            Box(
                (spec.cam_primary_r_outer_mm - spec.cam_primary_r_inner_mm) - 2.0,
                6.0,
                3.5,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )

        # [CLASP-140] 5 asymmetric fiducial bosses & [CLASP-141] 18 x 18 mm DataMatrix pad
        for ang in spec.fiducial_angles_deg:
            x = spec.fiducial_radius_mm * math.cos(math.radians(ang))
            y = spec.fiducial_radius_mm * math.sin(math.radians(ang))
            with Locations((x, y, collar_top_z)):
                Cylinder(
                    radius=spec.fiducial_boss_diameter_mm / 2.0,
                    height=spec.fiducial_boss_height_mm,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )

        dm_rad = math.radians(spec.datamatrix_angle_deg)
        dm_plane = Plane(
            origin=(
                spec.datamatrix_radius_mm * math.cos(dm_rad),
                spec.datamatrix_radius_mm * math.sin(dm_rad),
                collar_top_z,
            ),
            x_dir=(math.cos(dm_rad), math.sin(dm_rad), 0.0),
            z_dir=(0.0, 0.0, 1.0),
        )
        with BuildSketch(dm_plane):
            Rectangle(18.0, 18.0)
        extrude(amount=1.0)

    return part.part


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and verify DFM CLASP-M STEP (Lathe + 3-Axis Mill).")
    parser.add_argument("--step", type=Path, default=Path(__file__).resolve().parent / "CLASP-M-DFM.step")
    args = parser.parse_args()

    spec = ClaspMDFMSpec()
    solid = build_clasp_m_dfm(spec)
    metrics = verify_part(solid, spec)

    args.step.parent.mkdir(parents=True, exist_ok=True)
    export_step(solid, str(args.step))

    print("CLASP-M-DFM (Lathe + 3-Axis Mill) Verification: PASS")
    print(f"  Footprint : {metrics['diameter_x_mm']:.2f} x {metrics['diameter_y_mm']:.2f} mm (limit <= {spec.outer_diameter_mm:.1f} mm)")
    print(f"  Height    : {metrics['height_mm']:.2f} mm (limit <= {spec.max_height_mm:.1f} mm)")
    print(f"  Volume    : {metrics['volume_cm3']:.2f} cm^3")
    print(f"  Mass      : {metrics['mass_7075_t6_kg']:.3f} kg in Al 7075-T6 (limit <= {spec.mass_limit_kg:.2f} kg, margin +{metrics['mass_margin_kg']:.3f} kg)")
    print(f"  CoM       : ({metrics['com_x_mm']:.2f}, {metrics['com_y_mm']:.2f}, {metrics['com_z_mm']:.2f}) mm")
    print(f"  Exported  : {args.step}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
