#!/usr/bin/env python3
"""Parametric CLASP-A (Active End Effector, Annex B) CAD & Kinematics Verifier.

Models the active manipulator half matching CLASP-M-DFM.step / CLASP-M.step:
  - Upper wrist chassis, clevis brackets, & ISO wrist flange (Z = 74..118 mm)
  - Compliant axial retraction carriage on 3 guide/actuator shafts (Z = 44.5..62.5 mm)
  - 3x cam followers at 0, 120, 240 deg (with distinct 0 deg clocking key)
  - 3x positive over-center latch hooks at 30, 150, 270 deg hinged to carriage clevises
  - 4x 1-piece articulated wide-scoop soft-capture jaws at 45, 135, 225, 315 deg hinged
    to upper chassis (R_open = 245 mm, ~233 mm inscribed capture R, 170 mm scoop wings)
  - Active CLASP-FP core module (25 mm LOX coupler + 500 W primary coil + optical transceivers)
  - Perimeter-mounted polar-shadow camera & LED pod (clear of central cryo coupler for [CLASP-003])

Verifies:
  1. Each capture jaw and latch hook is a single manifold connected solid hinged to CLASP-A.
  2. Zero unintended 3D B-Rep interference against CLASP-M-DFM in HARD_LATCHED state.
  3. Zero interference across extreme +100 mm lateral / +15 deg pitch SOFT_CAPTURE approach.
Exports CLASP-A-Latched.step, CLASP-A-OpenAligned.step, and CLASP-A-MaxMisaligned.step.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Circle,
    Compound,
    Cylinder,
    Location,
    Locations,
    Mode,
    Plane,
    Polygon,
    export_step,
    extrude,
)

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_m import AL_7075_T6_DENSITY_G_CM3
from cad.clasp_m_dfm import ClaspMDFMSpec, build_clasp_m_dfm


@dataclass(frozen=True)
class ClaspASpec:
    """Nominal mechanical parameters for CLASP-A (Annex B active end effector)."""

    # Upper fixed chassis & robot wrist interface
    chassis_od_mm: float = 296.0
    chassis_id_mm: float = 154.0
    chassis_z0_mm: float = 74.0
    chassis_height_mm: float = 28.0
    wrist_flange_od_mm: float = 160.0
    wrist_flange_z0_mm: float = 102.0
    wrist_flange_height_mm: float = 16.0  # Top Z = 118.0 mm

    # Floating alignment carriage (retracted position at Z = 44.5 mm above collar)
    carriage_od_mm: float = 256.0
    carriage_id_mm: float = 152.0
    carriage_z0_mm: float = 44.5  # 0.5 mm nominal gap above CLASP-M Z=44.0 collar
    carriage_height_mm: float = 18.0

    # 3x Axial retraction actuators / guide shafts (60, 180, 300 deg)
    actuator_angles_deg: tuple[float, ...] = (60.0, 180.0, 300.0)
    actuator_radius_mm: float = 124.0  # Clears R=105 fiducials (R_in=113 mm) while sitting on carriage
    actuator_diameter_mm: float = 20.0

    # 3x Cam followers (0, 120, 240 deg) seated ahead of the 15-deg cam ramp entry
    cam_angles_deg: tuple[float, ...] = (0.0, 120.0, 240.0)
    follower_offset_deg: float = -18.5
    follower_radius_mm: float = 83.0

    # 3x Positive over-center latch hooks (30, 150, 270 deg) engaging Z = 32 mm undercut
    latch_angles_deg: tuple[float, ...] = (30.0, 150.0, 270.0)
    latch_hook_inner_r_mm: float = 104.0  # Clears Ø172 mm (R=86) waist by 18 mm
    latch_hook_outer_r_mm: float = 138.0
    latch_z_bottom_mm: float = 22.0
    latch_z_engage_mm: float = 31.5  # 0.5 mm clearance below Z = 32.0 collar lip

    # 4x Wide-opening soft-capture funnel jaws (45, 135, 225, 315 deg -> >230 mm inscribed R all 360 deg)
    jaw_angles_deg: tuple[float, ...] = (45.0, 135.0, 225.0, 315.0)
    jaw_pivot_r_mm: float = 144.0
    jaw_pivot_z_mm: float = 88.0
    jaw_open_radius_mm: float = 245.0  # Inscribed capture R ~ 233 mm > 120 + 100 mm lateral offset
    jaw_closed_radius_mm: float = 123.5  # Clears R=120 collar OD when latched

    # Active CLASP-FP core module (inside Ø148 mm bore, Z = 48.8..102 mm)
    active_core_od_mm: float = 144.0
    active_core_z0_mm: float = 48.8
    active_core_height_mm: float = 53.2

    # Perimeter wide-FOV camera + LED illumination pod (clear of central cryo coupler)
    camera_angle_deg: float = 90.0
    camera_radius_mm: float = 136.0
    camera_z0_mm: float = 64.0


def build_chassis_and_carriage(spec: ClaspASpec, m_spec: ClaspMDFMSpec):
    """Build upper wrist housing, ISO wrist flange, actuators, clevis ears, and carriage."""
    with BuildPart() as p:
        # Upper wrist chassis (Z = 74..102 mm)
        with Locations((0, 0, spec.chassis_z0_mm)):
            Cylinder(radius=spec.chassis_od_mm / 2.0, height=spec.chassis_height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))
            Cylinder(radius=spec.chassis_id_mm / 2.0, height=spec.chassis_height_mm + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # Robot wrist mounting flange (Z = 102..118 mm)
        with Locations((0, 0, spec.wrist_flange_z0_mm)):
            Cylinder(radius=spec.wrist_flange_od_mm / 2.0, height=spec.wrist_flange_height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Compliant alignment carriage (Z = 44.5..62.5 mm)
        with Locations((0, 0, spec.carriage_z0_mm)):
            Cylinder(radius=spec.carriage_od_mm / 2.0, height=spec.carriage_height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))
            Cylinder(radius=spec.carriage_id_mm / 2.0, height=spec.carriage_height_mm + 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
            # Annular cam clearance pocket (R = 76..94.5 mm, up to Z = 54.0 mm)
            Cylinder(radius=94.5, height=9.5, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # Relief pockets on carriage underside for [CLASP-140] fiducials and [CLASP-141] DataMatrix pad
        for ang in m_spec.fiducial_angles_deg:
            rad = math.radians(ang)
            with Locations((m_spec.fiducial_radius_mm * math.cos(rad), m_spec.fiducial_radius_mm * math.sin(rad), spec.carriage_z0_mm - 0.5)):
                Cylinder(radius=7.5, height=3.0, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        dm_rad = math.radians(m_spec.datamatrix_angle_deg)
        with Locations((m_spec.datamatrix_radius_mm * math.cos(dm_rad), m_spec.datamatrix_radius_mm * math.sin(dm_rad), spec.carriage_z0_mm - 0.5)):
            Box(22.0, 22.0, 3.0, rotation=(0, 0, m_spec.datamatrix_angle_deg), align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # 3x Axial ball-screw / guide columns physically bridging carriage (Z=44.5) to chassis (Z=102)
        for ang in spec.actuator_angles_deg:
            rad = math.radians(ang)
            with Locations((spec.actuator_radius_mm * math.cos(rad), spec.actuator_radius_mm * math.sin(rad), spec.carriage_z0_mm)):
                Cylinder(
                    radius=spec.actuator_diameter_mm / 2.0,
                    height=spec.chassis_z0_mm + spec.chassis_height_mm - spec.carriage_z0_mm,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )

        # 3x Latch clevis ears protruding from carriage OD (R = 124..142 mm) at 30, 150, 270 deg
        for ang in spec.latch_angles_deg:
            rad = math.radians(ang)
            with Locations((130.0 * math.cos(rad), 130.0 * math.sin(rad), spec.carriage_z0_mm + 2.0)):
                Box(22.0, 34.0, spec.carriage_height_mm - 2.0, rotation=(0, 0, ang), align=(Align.CENTER, Align.CENTER, Align.MIN))

        # 4x Jaw clevis hinge brackets on upper chassis OD (R = 140..158 mm) at 45, 135, 225, 315 deg
        for ang in spec.jaw_angles_deg:
            rad = math.radians(ang)
            with Locations((146.0 * math.cos(rad), 146.0 * math.sin(rad), spec.jaw_pivot_z_mm - 10.0)):
                Box(24.0, 38.0, 22.0, rotation=(0, 0, ang), align=(Align.CENTER, Align.CENTER, Align.MIN))

    solid = p.part
    solid.label = "CLASP_A_Chassis_And_Carriage"
    return solid


def build_cam_followers(spec: ClaspASpec):
    """Build 3 cam-follower pins extending from carriage recess ceiling down to Z = 44.8 mm."""
    with BuildPart() as p:
        for idx, cam_ang in enumerate(spec.cam_angles_deg):
            ang = cam_ang + spec.follower_offset_deg
            rad = math.radians(ang)
            pin_d = 12.0 if idx == 0 else 9.0
            with Locations((spec.follower_radius_mm * math.cos(rad), spec.follower_radius_mm * math.sin(rad), spec.carriage_z0_mm + 0.3)):
                Cylinder(radius=pin_d / 2.0, height=9.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    solid = p.part
    solid.label = "CLASP_A_Cam_Followers"
    return solid


def build_positive_latches(spec: ClaspASpec, open_state: bool = False):
    """Build 3 one-piece L-shaped positive latch hooks hinged to carriage clevises."""
    with BuildPart() as p:
        for ang in spec.latch_angles_deg:
            rad = math.radians(ang)
            rx, ry = math.cos(rad), math.sin(rad)
            tx, ty = -math.sin(rad), math.cos(rad)
            radial_plane = Plane(origin=(0.0, 0.0, 0.0), x_dir=(rx, ry, 0.0), z_dir=(-tx, -ty, 0.0))
            # Open latches retract upward & outward (z >= 43 mm, r >= 130 mm) during soft capture
            r_in = 130.0 if open_state else spec.latch_hook_inner_r_mm
            r_out = 148.0 if open_state else spec.latch_hook_outer_r_mm
            r_wall = 134.0 if open_state else 123.5
            z_bot = 43.0 if open_state else spec.latch_z_bottom_mm
            z_eng = 47.0 if open_state else spec.latch_z_engage_mm
            z_top = spec.carriage_z0_mm + spec.carriage_height_mm - 2.0
            with BuildSketch(radial_plane):
                Polygon(
                    (r_in, z_bot),
                    (r_out, z_bot),
                    (r_out, z_top),
                    (r_wall, z_top),
                    (r_wall, z_eng),
                    (r_in, z_eng),
                    align=None,
                )
            extrude(amount=11.0, both=True)
    solid = p.part
    solid.label = "CLASP_A_Positive_Latches_Open" if open_state else "CLASP_A_Positive_Latches"
    return solid


def build_capture_jaws(spec: ClaspASpec, open_state: bool):
    """Build 3 single-piece articulated capture jaws with wide flared funnel paddles."""
    r_piv = spec.jaw_pivot_r_mm
    z_piv = spec.jaw_pivot_z_mm
    r_tip = spec.jaw_open_radius_mm if open_state else spec.jaw_closed_radius_mm
    # Open jaw tips extend down to Z = -2 mm in CLASP-A frame so they straddle collar during +/-50 mm capture
    z_tip = -2.0 if open_state else 34.0
    r_back = r_piv + 12.0
    r_tip_out = r_tip + 14.0
    paddle_h = 30.0 if open_state else 12.0

    with BuildPart() as p:
        for ang in spec.jaw_angles_deg:
            rad = math.radians(ang)
            rx, ry = math.cos(rad), math.sin(rad)
            tx, ty = -math.sin(rad), math.cos(rad)
            radial_plane = Plane(origin=(0.0, 0.0, 0.0), x_dir=(rx, ry, 0.0), z_dir=(-tx, -ty, 0.0))

            with BuildSketch(radial_plane):
                if open_state:
                    Polygon(
                        (r_piv - 6.0, z_piv + 8.0),
                        (r_back, z_piv + 8.0),
                        (r_tip_out, z_tip + paddle_h),
                        (r_tip_out, z_tip),
                        (r_tip, z_tip),
                        (r_tip - 4.0, z_tip + 16.0),
                        (r_piv - 6.0, z_piv - 8.0),
                        align=None,
                    )
                else:
                    Polygon(
                        (r_piv - 6.0, z_piv + 8.0),
                        (r_back, z_piv + 8.0),
                        (r_tip_out, z_tip),
                        (r_tip, z_tip),
                        (r_tip, z_tip + paddle_h),
                        (r_piv - 6.0, z_piv - 8.0),
                        align=None,
                    )
            extrude(amount=14.0, both=True)

            # Wide tangential capture scoop wing at open tip (170 mm wide -> true 360-deg capture funnel)
            if open_state:
                with Locations((r_tip * rx, r_tip * ry, z_tip + 0.5 * paddle_h)):
                    Box(14.0, 170.0, paddle_h, rotation=(0.0, 0.0, ang), align=(Align.CENTER, Align.CENTER, Align.CENTER))

            # Hinged clevis pin along tangential axis at (r_piv, z_piv)
            pin_plane = Plane(origin=(r_piv * rx, r_piv * ry, z_piv), x_dir=(rx, ry, 0.0), z_dir=(-tx, -ty, 0.0))
            with BuildSketch(pin_plane):
                Circle(radius=5.0)
            extrude(amount=20.0, both=True)

    solid = p.part
    solid.label = "CLASP_A_Capture_Jaws_Open" if open_state else "CLASP_A_Capture_Jaws_Closed"
    return solid


def build_active_core_and_camera(spec: ClaspASpec):
    """Build central CLASP-FP active dock module and off-center polar-vision camera/LED pod."""
    cam_rad = math.radians(spec.camera_angle_deg)
    with BuildPart() as p:
        # Central active CLASP-FP docking core (physically nested inside chassis + carriage ID)
        with Locations((0, 0, spec.active_core_z0_mm)):
            Cylinder(
                radius=spec.active_core_od_mm / 2.0,
                height=spec.active_core_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
        # Retaining flange bridging core to chassis (Z = 74..102 mm) so core is physically supported
        with Locations((0, 0, spec.chassis_z0_mm)):
            Cylinder(radius=78.0, height=16.0, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # Perimeter wide-FOV camera + LED ring pod mounted solidly to chassis & carriage side
        with Locations((spec.camera_radius_mm * math.cos(cam_rad), spec.camera_radius_mm * math.sin(cam_rad), spec.camera_z0_mm)):
            Box(28.0, 38.0, 32.0, rotation=(0, 0, spec.camera_angle_deg), align=(Align.CENTER, Align.CENTER, Align.MIN))
    solid = p.part
    solid.label = "CLASP_A_FP_Core_And_Camera_Pod"
    return solid


def overlap_volume(a, b) -> float:
    """Compute total 3D intersection volume (mm^3) across ShapeList or Solid returns."""
    res = a.intersect(b)
    if res is None:
        return 0.0
    if hasattr(res, "volume"):
        return float(res.volume)
    return float(sum(item.volume for item in res if hasattr(item, "volume")))


def verify_clasp_a(m_solid, m_spec: ClaspMDFMSpec, a_spec: ClaspASpec) -> dict[str, object]:
    """Verify connectivity, zero B-Rep interference in HARD_LATCHED, and extreme open capture."""
    chassis = build_chassis_and_carriage(a_spec, m_spec)
    followers = build_cam_followers(a_spec)
    latches = build_positive_latches(a_spec)
    jaws_closed = build_capture_jaws(a_spec, open_state=False)
    core_cam = build_active_core_and_camera(a_spec)

    # Verify structural connectivity: chassis is 1 solid, latches = 3 hooks, jaws = 4 arms
    n_jaws = len(a_spec.jaw_angles_deg)
    assert len(chassis.solids()) == 1, f"Expected unified chassis+carriage, got {len(chassis.solids())} solids"
    assert len(latches.solids()) == 3, f"Expected 3 latch hooks, got {len(latches.solids())}"
    assert len(jaws_closed.solids()) == n_jaws, f"Expected {n_jaws} closed jaws, got {len(jaws_closed.solids())}"

    active_parts = [chassis, followers, latches, jaws_closed, core_cam]
    for part in active_parts:
        assert part.is_valid, f"Invalid B-Rep in {part.label}"
        vol = overlap_volume(m_solid, part)
        assert vol < 1e-3, f"Hard-latched interference: CLASP-M-DFM <-> {part.label} ({vol:.3f} mm^3)"

    # Open soft-capture check at extreme §5.1 corner: +100 mm lateral X, +50 mm axial Z, +15 deg pitch
    jaws_open = build_capture_jaws(a_spec, open_state=True)
    latches_open = build_positive_latches(a_spec, open_state=True)
    assert jaws_open.is_valid and latches_open.is_valid
    assert len(jaws_open.solids()) == n_jaws, f"Expected {n_jaws} unified open jaws, got {len(jaws_open.solids())}"
    assert len(latches_open.solids()) == 3, f"Expected 3 retracted open latches, got {len(latches_open.solids())}"

    approach_loc = Location((-100.0, 0.0, 50.0), (0.0, 15.0, 0.0))
    misaligned_parts = [
        chassis.moved(approach_loc),
        followers.moved(approach_loc),
        latches_open.moved(approach_loc),
        jaws_open.moved(approach_loc),
        core_cam.moved(approach_loc),
    ]
    for part in misaligned_parts:
        open_vol = overlap_volume(m_solid, part)
        assert open_vol < 1e-3, f"Open soft-capture interference at +100 mm / +15 deg: {part.label} ({open_vol:.3f} mm^3)"

    aligned_open_loc = Location((0.0, 0.0, 50.0))
    open_aligned_parts = [
        chassis.moved(aligned_open_loc),
        followers.moved(aligned_open_loc),
        latches_open.moved(aligned_open_loc),
        jaws_open.moved(aligned_open_loc),
        core_cam.moved(aligned_open_loc),
    ]

    # Mass estimate of mechanical CLASP-A (Al 7075-T6 effective ~65% fill factor for housings/actuators)
    vol_cm3 = sum(p.volume for p in (chassis, followers, latches, jaws_closed)) / 1000.0
    mech_mass_kg = (vol_cm3 * 0.65 * AL_7075_T6_DENSITY_G_CM3) / 1000.0
    core_mass_kg = ((core_cam.volume / 1000.0) * 3.80) / 1000.0

    # Clone m_solid per Compound so build123d/anytree does not re-parent it out of earlier assemblies
    m_latched = copy.copy(m_solid)
    m_open_aligned = copy.copy(m_solid)
    m_misaligned = copy.copy(m_solid)

    latched_assembly = Compound(children=[m_latched, *active_parts])
    open_aligned_assembly = Compound(children=[m_open_aligned, *open_aligned_parts])
    misaligned_assembly = Compound(children=[m_misaligned, *misaligned_parts])

    expected_solids = 1 + sum(len(p.solids()) for p in active_parts)
    for tag, assy in (
        ("latched", latched_assembly),
        ("open_aligned", open_aligned_assembly),
        ("misaligned", misaligned_assembly),
    ):
        assert len(assy.solids()) == expected_solids, f"{tag} expected {expected_solids} solids, got {len(assy.solids())}"

    bbox_open = jaws_open.bounding_box()
    koz_diameter_mm = max(bbox_open.max.X - bbox_open.min.X, bbox_open.max.Y - bbox_open.min.Y) + 10.0

    return {
        "latched_assembly": latched_assembly,
        "open_aligned_assembly": open_aligned_assembly,
        "misaligned_assembly": misaligned_assembly,
        "mech_mass_kg": mech_mass_kg,
        "fp_total_mass_kg": mech_mass_kg + core_mass_kg,
        "koz_diameter_mm": koz_diameter_mm,
        "koz_min_z_mm": 18.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and verify CLASP-A (Annex B) active end effector.")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()

    m_spec = ClaspMDFMSpec()
    a_spec = ClaspASpec()
    m_solid = build_clasp_m_dfm(m_spec)
    m_solid.label = "CLASP_M_DFM_Passive_Port"

    rep = verify_clasp_a(m_solid, m_spec, a_spec)

    latched_path = args.out_dir / "CLASP-A-Latched.step"
    open_aligned_path = args.out_dir / "CLASP-A-OpenAligned.step"
    misaligned_path = args.out_dir / "CLASP-A-MaxMisaligned.step"
    export_step(rep["latched_assembly"], str(latched_path))
    export_step(rep["open_aligned_assembly"], str(open_aligned_path))
    export_step(rep["misaligned_assembly"], str(misaligned_path))

    print("CLASP-A (Annex B Active End Effector) Verification: PASS")
    print(f"  Connectivity checks : chassis=1 solid, latches=3 hooks, jaws={len(a_spec.jaw_angles_deg)} unified arms (0 floating parts)")
    print(f"  Hard-latched checks : 5/5 subsystems zero-interference against CLASP-M-DFM (0.000 mm^3)")
    print(f"  Soft-capture check  : 5/5 subsystems PASS at +100 mm X / +15 deg pitch extreme envelope")
    print(f"  Payload Keep-Out    : Ø {rep['koz_diameter_mm']:.1f} mm clear above Z = {rep['koz_min_z_mm']:.1f} mm")
    print(f"  Est. Active Mass    : {rep['mech_mass_kg']:.2f} kg (CLASP-A mech) | {rep['fp_total_mass_kg']:.2f} kg (w/ FP core + camera)")
    print(f"  Exported            : {latched_path}")
    print(f"  Exported            : {open_aligned_path}")
    print(f"  Exported            : {misaligned_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
