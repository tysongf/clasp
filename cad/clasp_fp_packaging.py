#!/usr/bin/env python3
"""Parametric CLASP-FP internal packaging envelope model & collision verifier.

Places all CLASP-F / CLASP-FP subsystems inside the Ø 148 mm central bore of
CLASP-M and mathematically verifies:
  1. Zero 3D B-Rep interference against the CLASP-M structural port.
  2. Zero pairwise 3D B-Rep interference between internal subsystems.
  3. Compliance with [CLASP-010] (<= 240 mm OD, <= 55 mm height).
  4. Compliance with [CLASP-012] mass limits (CLASP-F <= 6.0 kg, CLASP-FP <= 7.5 kg).
Exports a multi-solid assembly STEP file for CAD inspection.
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
    Compound,
    Cylinder,
    Locations,
    Mode,
    Plane,
    Polygon,
    export_step,
    extrude,
)

# Allow direct execution as `python cad/clasp_fp_packaging.py`
if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_m import AL_7075_T6_DENSITY_G_CM3, ClaspMSpec, build_clasp_m


@dataclass(frozen=True)
class ClaspFPPackagingSpec:
    """Internal packaging geometry inside the Ø 148 mm (R = 74.0 mm) CLASP-M bore."""

    bore_clearance_radius_mm: float = 73.5  # 0.5 mm radial clearance inside 74.0 mm bore

    # [CLASP-060] - [CLASP-102] Central 25 mm dual-poppet LOX fluid coupling
    fluid_bore_diameter_mm: float = 25.0
    fluid_body_od_mm: float = 56.0  # R = 28.0 mm (poppets + 2 mm bypass + 5 MPa wall)
    fluid_flange_od_mm: float = 68.0  # R = 34.0 mm mounting flange
    fluid_flange_z0_mm: float = 2.0
    fluid_flange_height_mm: float = 12.0
    fluid_mating_z_mm: float = 48.0  # <= 55.0 mm

    # Structural cartridge ring + 4 Ti-6Al-4V tangential flexure thermal-break spokes ([CLASP-151])
    cartridge_id_mm: float = 138.0  # R = 69.0 mm
    cartridge_od_mm: float = 147.0  # R = 73.5 mm
    cartridge_z0_mm: float = 0.0
    cartridge_height_mm: float = 28.0
    spoke_angles_deg: tuple[float, ...] = (45.0, 135.0, 225.0, 315.0)
    spoke_width_mm: float = 2.6  # Slender tangential flexure thickness (radially compliant)
    spoke_flex_radius_mm: float = 50.0
    spoke_flex_sweep_deg: float = 48.0
    spoke_z0_mm: float = 4.0
    spoke_height_mm: float = 10.0

    # [CLASP-120] - [CLASP-121] 500 W inductive power coil + ferrite shield cup
    coil_id_mm: float = 78.0  # R = 39.0 mm (5.0 mm thermal gap outside R = 34 mm fluid flange)
    coil_od_mm: float = 116.0  # R = 58.0 mm
    coil_z0_mm: float = 29.0
    coil_height_mm: float = 18.0  # Top at Z = 47.0 mm

    # [CLASP-130] - [CLASP-131] Dual redundant short-range optical transceivers (0 & 180 deg)
    optic_radius_mm: float = 65.5  # Spans R = 59.5 .. 71.5 mm
    optic_diameter_mm: float = 12.0
    optic_z0_mm: float = 29.0
    optic_height_mm: float = 18.0
    optic_angles_deg: tuple[float, ...] = (0.0, 180.0)

    # [CLASP-122] Dual passive magnetic / Hall latch interlock targets (90 & 270 deg)
    interlock_radius_mm: float = 65.5
    interlock_radial_mm: float = 10.0
    interlock_tangential_mm: float = 12.0
    interlock_z0_mm: float = 34.0
    interlock_height_mm: float = 13.0
    interlock_angles_deg: tuple[float, ...] = (90.0, 270.0)

    # [CLASP-111] - [CLASP-113] Potted EDS HV driver electronics bays (lower annulus)
    eds_driver_radius_mm: float = 53.0  # Spans R = 42 .. 64 mm
    eds_driver_radial_mm: float = 20.0
    eds_driver_tangential_mm: float = 32.0
    eds_driver_z0_mm: float = 15.0
    eds_driver_height_mm: float = 12.0  # Top at Z = 27.0 mm (clears coil Z = 29.0 mm & spokes Z <= 14 mm)
    eds_driver_angles_deg: tuple[float, ...] = (0.0, 180.0)

    # [CLASP-111] Conformal EDS electrode face ring
    eds_face_id_mm: float = 76.0  # R = 38.0 mm
    eds_face_od_mm: float = 145.0  # R = 72.5 mm
    eds_face_z0_mm: float = 47.2  # 0.2 mm dielectric gap above coil/optics
    eds_face_height_mm: float = 1.0  # Top at Z = 48.2 mm


def build_fluid_coupling(spec: ClaspFPPackagingSpec):
    """Build central 25 mm coaxial dual-poppet LOX coupling envelope (hollow bore)."""
    with BuildPart() as p:
        Cylinder(
            radius=spec.fluid_body_od_mm / 2.0,
            height=spec.fluid_mating_z_mm,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        with Locations((0, 0, spec.fluid_flange_z0_mm)):
            Cylinder(
                radius=spec.fluid_flange_od_mm / 2.0,
                height=spec.fluid_flange_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
        Cylinder(
            radius=spec.fluid_bore_diameter_mm / 2.0,
            height=spec.fluid_mating_z_mm + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
            mode=Mode.SUBTRACT,
        )
    solid = p.part
    solid.label = "CLASP_FP_Fluid_Coupling_25mm"
    return solid


def build_thermal_cartridge(spec: ClaspFPPackagingSpec):
    """Build outer bore-mounted cartridge sleeve and 4 tangential Ti-6Al-4V flexure spokes."""
    r_inner_spoke = spec.fluid_flange_od_mm / 2.0 - 0.5
    r_outer_spoke = spec.cartridge_id_mm / 2.0 + 0.5
    r_flex = spec.spoke_flex_radius_mm
    half_t = spec.spoke_width_mm / 2.0
    r_flex_in = r_flex - half_t
    r_flex_out = r_flex + half_t
    stub_w = max(spec.spoke_width_mm + 0.6, 3.2)

    with BuildPart() as p:
        with Locations((0, 0, spec.cartridge_z0_mm)):
            Cylinder(
                radius=spec.cartridge_od_mm / 2.0,
                height=spec.cartridge_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
            Cylinder(
                radius=spec.cartridge_id_mm / 2.0,
                height=spec.cartridge_height_mm + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT,
            )
        z_plane = Plane.XY.offset(spec.spoke_z0_mm)
        for ang in spec.spoke_angles_deg:
            ang_out = ang + spec.spoke_flex_sweep_deg
            r_in_mid = 0.5 * (r_inner_spoke + (r_flex_out - 0.3))
            r_in_len = (r_flex_out - 0.3) - r_inner_spoke
            r_out_mid = 0.5 * ((r_flex_in + 0.3) + r_outer_spoke)
            r_out_len = r_outer_spoke - (r_flex_in + 0.3)

            rad_in = math.radians(ang)
            with Locations((r_in_mid * math.cos(rad_in), r_in_mid * math.sin(rad_in), spec.spoke_z0_mm)):
                Box(r_in_len, stub_w, spec.spoke_height_mm, rotation=(0, 0, ang), align=(Align.CENTER, Align.CENTER, Align.MIN))

            rad_out = math.radians(ang_out)
            with Locations((r_out_mid * math.cos(rad_out), r_out_mid * math.sin(rad_out), spec.spoke_z0_mm)):
                Box(r_out_len, stub_w, spec.spoke_height_mm, rotation=(0, 0, ang_out), align=(Align.CENTER, Align.CENTER, Align.MIN))

            n_pts = 18
            a0 = math.radians(ang - 1.5)
            a1 = math.radians(ang_out + 1.5)
            outer_pts = [
                (r_flex_out * math.cos(a0 + (a1 - a0) * i / n_pts), r_flex_out * math.sin(a0 + (a1 - a0) * i / n_pts))
                for i in range(n_pts + 1)
            ]
            inner_pts = [
                (r_flex_in * math.cos(a1 - (a1 - a0) * i / n_pts), r_flex_in * math.sin(a1 - (a1 - a0) * i / n_pts))
                for i in range(n_pts + 1)
            ]
            with BuildSketch(z_plane):
                Polygon(*(outer_pts + inner_pts), align=None)
            extrude(amount=spec.spoke_height_mm)

        # Ensure clean subtraction of fluid flange boundary so zero overlap exists
        with Locations((0, 0, spec.fluid_flange_z0_mm)):
            Cylinder(
                radius=spec.fluid_flange_od_mm / 2.0,
                height=spec.fluid_flange_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT,
            )
    solid = p.part
    solid.label = "CLASP_FP_Thermal_Cartridge"
    return solid


def build_inductive_coil(spec: ClaspFPPackagingSpec):
    """Build 500 W inductive power coil + ferrite core annulus."""
    with BuildPart() as p:
        with Locations((0, 0, spec.coil_z0_mm)):
            Cylinder(
                radius=spec.coil_od_mm / 2.0,
                height=spec.coil_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
            Cylinder(
                radius=spec.coil_id_mm / 2.0,
                height=spec.coil_height_mm + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT,
            )
    solid = p.part
    solid.label = "CLASP_FP_Inductive_Coil_500W"
    return solid


def build_optical_transceivers(spec: ClaspFPPackagingSpec):
    """Build dual redundant 10 Mbit/s optical transceiver housings."""
    with BuildPart() as p:
        for ang in spec.optic_angles_deg:
            rad = math.radians(ang)
            with Locations((spec.optic_radius_mm * math.cos(rad), spec.optic_radius_mm * math.sin(rad), spec.optic_z0_mm)):
                Cylinder(
                    radius=spec.optic_diameter_mm / 2.0,
                    height=spec.optic_height_mm,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )
    solid = p.part
    solid.label = "CLASP_FP_Optical_Transceivers"
    return solid


def build_magnetic_interlocks(spec: ClaspFPPackagingSpec):
    """Build dual passive magnetic/Hall latch interlock targets."""
    with BuildPart() as p:
        for ang in spec.interlock_angles_deg:
            rad = math.radians(ang)
            with Locations(
                (spec.interlock_radius_mm * math.cos(rad), spec.interlock_radius_mm * math.sin(rad), spec.interlock_z0_mm)
            ):
                Box(
                    spec.interlock_radial_mm,
                    spec.interlock_tangential_mm,
                    spec.interlock_height_mm,
                    rotation=(0, 0, ang),
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )
    solid = p.part
    solid.label = "CLASP_FP_Magnetic_Interlocks"
    return solid


def build_eds_driver_bays(spec: ClaspFPPackagingSpec):
    """Build potted 1 kV 4-phase EDS conditioning electronics modules."""
    with BuildPart() as p:
        for ang in spec.eds_driver_angles_deg:
            rad = math.radians(ang)
            with Locations(
                (spec.eds_driver_radius_mm * math.cos(rad), spec.eds_driver_radius_mm * math.sin(rad), spec.eds_driver_z0_mm)
            ):
                Box(
                    spec.eds_driver_radial_mm,
                    spec.eds_driver_tangential_mm,
                    spec.eds_driver_height_mm,
                    rotation=(0, 0, ang),
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )
    solid = p.part
    solid.label = "CLASP_FP_EDS_Driver_Bays"
    return solid


def build_eds_face(spec: ClaspFPPackagingSpec):
    """Build conformal dielectric EDS electrode array face ring with optical apertures."""
    with BuildPart() as p:
        with Locations((0, 0, spec.eds_face_z0_mm)):
            Cylinder(
                radius=spec.eds_face_od_mm / 2.0,
                height=spec.eds_face_height_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
            Cylinder(
                radius=spec.eds_face_id_mm / 2.0,
                height=spec.eds_face_height_mm + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
                mode=Mode.SUBTRACT,
            )
        # Clear line-of-sight windows for the dual 10 Mbit/s optical transceivers
        for ang in spec.optic_angles_deg:
            rad = math.radians(ang)
            with Locations(
                (spec.optic_radius_mm * math.cos(rad), spec.optic_radius_mm * math.sin(rad), spec.eds_face_z0_mm - 0.5)
            ):
                Cylinder(
                    radius=(spec.optic_diameter_mm + 0.5) / 2.0,
                    height=spec.eds_face_height_mm + 1.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                    mode=Mode.SUBTRACT,
                )
    solid = p.part
    solid.label = "CLASP_FP_EDS_Electrode_Face"
    return solid


def verify_packaging(
    clasp_m,
    subsystems: dict[str, tuple[object, float]],
    m_spec: ClaspMSpec,
) -> dict[str, object]:
    """Run exact B-Rep collision checks, envelope checks, and tiered mass roll-up."""
    names = list(subsystems.keys())

    # 1. Zero interference against CLASP-M structural port
    for name in names:
        sub, _ = subsystems[name]
        assert sub.is_valid, f"Subsystem {name} B-Rep is invalid"
        overlap = clasp_m.intersect(sub)
        overlap_vol = overlap.volume if overlap is not None else 0.0
        assert overlap_vol < 1e-3, f"Collision detected: CLASP-M <-> {name} ({overlap_vol:.3f} mm^3)"

    # 2. Zero pairwise interference between all CLASP-FP internal subsystems
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a_name, (a_solid, _) = names[i], subsystems[names[i]]
            b_name, (b_solid, _) = names[j], subsystems[names[j]]
            overlap = a_solid.intersect(b_solid)
            overlap_vol = overlap.volume if overlap is not None else 0.0
            assert overlap_vol < 1e-3, f"Collision detected: {a_name} <-> {b_name} ({overlap_vol:.3f} mm^3)"

    # 3. Bounding envelope check ([CLASP-010])
    all_parts = [clasp_m] + [subsystems[n][0] for n in names]
    assembly = Compound(children=all_parts)
    bbox = assembly.bounding_box()
    diam_x = bbox.max.X - bbox.min.X
    diam_y = bbox.max.Y - bbox.min.Y
    height_z = bbox.max.Z - bbox.min.Z
    assert diam_x <= m_spec.outer_diameter_mm + 1e-4
    assert diam_y <= m_spec.outer_diameter_mm + 1e-4
    assert bbox.min.Z >= -1e-4 and height_z <= m_spec.max_height_mm + 1e-4

    # 4. Mass roll-up ([CLASP-012])
    clasp_m_kg = ((clasp_m.volume / 1000.0) * AL_7075_T6_DENSITY_G_CM3) / 1000.0
    sub_masses_kg: dict[str, float] = {}
    for name, (solid, eff_density_g_cm3) in subsystems.items():
        vol_cm3 = solid.volume / 1000.0
        sub_masses_kg[name] = (vol_cm3 * eff_density_g_cm3) / 1000.0

    clasp_f_keys = ("fluid_coupling", "thermal_cartridge", "eds_driver_bays", "eds_face")
    clasp_f_kg = clasp_m_kg + sum(sub_masses_kg[k] for k in clasp_f_keys)
    clasp_fp_kg = clasp_m_kg + sum(sub_masses_kg.values())

    assert clasp_m_kg <= 3.50, f"CLASP-M mass {clasp_m_kg:.3f} kg > 3.50 kg"
    assert clasp_f_kg <= 6.00, f"CLASP-F mass {clasp_f_kg:.3f} kg > 6.00 kg"
    assert clasp_fp_kg <= 7.50, f"CLASP-FP mass {clasp_fp_kg:.3f} kg > 7.50 kg"

    return {
        "assembly": assembly,
        "diam_x_mm": diam_x,
        "diam_y_mm": diam_y,
        "height_mm": height_z,
        "clasp_m_kg": clasp_m_kg,
        "clasp_f_kg": clasp_f_kg,
        "clasp_fp_kg": clasp_fp_kg,
        "sub_masses_kg": sub_masses_kg,
        "pair_checks": len(names) + (len(names) * (len(names) - 1)) // 2,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify CLASP-FP internal packaging inside CLASP-M bore.")
    parser.add_argument(
        "--step",
        type=Path,
        default=Path(__file__).resolve().parent / "CLASP-FP-Packaging.step",
    )
    args = parser.parse_args()

    m_spec = ClaspMSpec()
    fp_spec = ClaspFPPackagingSpec()

    clasp_m = build_clasp_m(m_spec)
    clasp_m.label = "CLASP_M_Structural_Port"

    # (solid, realistic effective subsystem bulk density in g/cm^3 accounting for internal voids/potting)
    subsystems = {
        "fluid_coupling": (build_fluid_coupling(fp_spec), 6.20),  # Inconel/316L body w/ internal poppet/spring cavity
        "thermal_cartridge": (build_thermal_cartridge(fp_spec), 4.43),  # Ti-6Al-4V thermal break & bore sleeve
        "inductive_coil": (build_inductive_coil(fp_spec), 4.80),  # Cu Litz + ferrite core + epoxy potting
        "optical_transceivers": (build_optical_transceivers(fp_spec), 2.40),  # Al housing + sapphire window + optoelectronics
        "magnetic_interlocks": (build_magnetic_interlocks(fp_spec), 7.50),  # SmCo/NdFeB + cryo stainless sleeve
        "eds_driver_bays": (build_eds_driver_bays(fp_spec), 2.20),  # Potted HV PCB / transformer module
        "eds_face": (build_eds_face(fp_spec), 2.50),  # Alumina/polyimide electrode ring
    }

    report = verify_packaging(clasp_m, subsystems, m_spec)

    args.step.parent.mkdir(parents=True, exist_ok=True)
    export_step(report["assembly"], str(args.step))

    print("CLASP-FP Packaging Envelope & Collision Verification: PASS")
    print(f"  Collision checks : {report['pair_checks']}/{report['pair_checks']} zero-overlap (0.000 mm^3)")
    print(f"  Envelope         : {report['diam_x_mm']:.2f} x {report['diam_y_mm']:.2f} x {report['height_mm']:.2f} mm (limit <= 240.0 x 240.0 x 55.0 mm)")
    print(f"  CLASP-M mass     : {report['clasp_m_kg']:.3f} kg (limit <= 3.50 kg)")
    print(f"  CLASP-F mass     : {report['clasp_f_kg']:.3f} kg (limit <= 6.00 kg)")
    print(f"  CLASP-FP mass    : {report['clasp_fp_kg']:.3f} kg (limit <= 7.50 kg, margin +{7.50 - report['clasp_fp_kg']:.3f} kg)")
    print("  Subsystem breakdown:")
    for name, kg in report["sub_masses_kg"].items():
        print(f"    - {name:<22s} : {kg:.3f} kg")
    print(f"  Exported         : {args.step}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
