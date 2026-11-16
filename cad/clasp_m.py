#!/usr/bin/env python3
"""Parametric CLASP-M passive port CAD generator and mass/envelope verifier.

Generates the CLASP-M (Mechanical tier) passive port solid model from
CLASP-STD-001 Revision A parameters, verifies the <= 240 mm OD, <= 55 mm height,
and <= 3.50 kg mass requirements in Al 7075-T6 (2.81 g/cm^3), and exports STEP.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
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
    Rectangle,
    chamfer,
    export_step,
    export_stl,
    extrude,
    loft,
)

AL_7075_T6_DENSITY_G_CM3 = 2.81


@dataclass(frozen=True)
class ClaspMSpec:
    """Nominal geometric parameters traced to CLASP-STD-001."""

    # [CLASP-010] Envelope
    outer_diameter_mm: float = 240.0
    max_height_mm: float = 55.0
    mass_limit_kg: float = 3.50

    # [CLASP-011] Mounting pattern: 8 x M8 on 210 mm BCD with 15 deg clocking offset
    bolt_circle_diameter_mm: float = 210.0
    bolt_clearance_diameter_mm: float = 9.0
    bolt_head_access_diameter_mm: float = 16.0
    bolt_angles_deg: tuple[float, ...] = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 330.0)
    dowel_diameter_mm: float = 6.0
    dowel_angles_deg: tuple[float, ...] = (22.5, 202.5)

    # Structural body & [CLASP-020] external perimeter grapple/latch collar
    base_flange_thickness_mm: float = 10.0
    inner_bore_diameter_mm: float = 148.0
    waist_od_mm: float = 182.0
    collar_z0_mm: float = 32.0
    collar_thickness_mm: float = 12.0  # Z = 32..44 mm
    collar_chamfer_mm: float = 2.0

    # [CLASP-021] - [CLASP-023] Helical cam ramps (15 deg lead angle at 120 deg spacing)
    cam_lead_angle_deg: float = 15.0
    cam_span_deg: float = 20.0
    cam_r_inner_mm: float = 76.0
    cam_r_outer_mm: float = 90.0
    cam_primary_r_inner_mm: float = 74.0  # [CLASP-023] Distinct clocking ramp at 0 deg
    cam_primary_r_outer_mm: float = 92.0
    cam_base_z_mm: float = 44.0

    # [CLASP-140] - [CLASP-141] Asymmetric fiducial bosses & DataMatrix pad
    fiducial_angles_deg: tuple[float, ...] = (18.0, 68.0, 112.0, 196.0, 300.0)
    fiducial_radius_mm: float = 105.0
    fiducial_boss_diameter_mm: float = 10.0
    fiducial_boss_height_mm: float = 1.5
    datamatrix_angle_deg: float = 247.5
    datamatrix_radius_mm: float = 105.0

    # Waist lightening windows (cleared away from bolts)
    window_angles_deg: tuple[float, ...] = field(
        default_factory=lambda: (22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 297.5)
    )


def build_cam_ramp(
    r_inner: float,
    r_outer: float,
    z_base: float,
    angle_start_deg: float,
    span_deg: float,
    lead_angle_deg: float,
    steps: int = 6,
) -> Compound:
    """Build a helical cam ramp section rising at lead_angle_deg along mean radius."""
    r_mid = 0.5 * (r_inner + r_outer)
    radial_width = r_outer - r_inner
    total_rise = r_mid * math.radians(span_deg) * math.tan(math.radians(lead_angle_deg))

    with BuildPart() as cam_part:
        sections = []
        for i in range(steps + 1):
            frac = i / steps
            theta = angle_start_deg + frac * span_deg
            # Keep minimum 1.0 mm thickness at leading edge so loft is manifold
            height = 1.0 + frac * total_rise
            rad = math.radians(theta)
            normal = (math.sin(rad), -math.cos(rad), 0.0)
            origin = (r_mid * math.cos(rad), r_mid * math.sin(rad), z_base)
            plane = Plane(origin=origin, x_dir=(math.cos(rad), math.sin(rad), 0.0), z_dir=normal)
            with BuildSketch(plane) as sk:
                Rectangle(radial_width, height, align=(Align.CENTER, Align.MIN))
            sections.append(sk.sketch)
        loft(sections, ruled=True)
    return cam_part.part


def build_clasp_m(spec: ClaspMSpec = ClaspMSpec()):
    """Construct the parametric CLASP-M passive port B-Rep solid."""
    r_outer = spec.outer_diameter_mm / 2.0
    r_inner = spec.inner_bore_diameter_mm / 2.0
    r_waist = spec.waist_od_mm / 2.0
    r_bcd = spec.bolt_circle_diameter_mm / 2.0

    with BuildPart() as part:
        # 1. Base mounting flange (Z = 0 .. base_flange_thickness)
        Cylinder(radius=r_outer, height=spec.base_flange_thickness_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # 2. Structural waist wall (Z = base_flange_thickness .. collar_z0)
        waist_h = spec.collar_z0_mm - spec.base_flange_thickness_mm
        with Locations((0, 0, spec.base_flange_thickness_mm)):
            Cylinder(radius=r_waist, height=waist_h, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # 3. Upper external perimeter grapple & positive-latch collar (Z = 32 .. 44 mm)
        with Locations((0, 0, spec.collar_z0_mm)):
            Cylinder(radius=r_outer, height=spec.collar_thickness_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))

        # 4. [CLASP-117] Top & bottom chamfers on the external grapple/latch collar
        collar_top_z = spec.collar_z0_mm + spec.collar_thickness_mm
        collar_edges = (
            part.edges()
            .filter_by(lambda e: abs(e.center().Z - spec.collar_z0_mm) < 0.01 or abs(e.center().Z - collar_top_z) < 0.01)
            .filter_by(lambda e: math.hypot(e.center().X, e.center().Y) > r_outer - 1.0)
        )
        chamfer(collar_edges, length=spec.collar_chamfer_mm)

        # 5. Central bore (reserves common internal envelope for CLASP-F/-FP)
        Cylinder(radius=r_inner, height=spec.max_height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

        # 6. [CLASP-011] 8 x M8 clocked mounting holes + top-collar wrench access holes
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

        # 7. Locating shear/torsion dowel pin holes on 210 mm BCD
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

        # 8. Waist lightening pockets (radial cutouts leaving stiffening ribs between bolt zones)
        window_z = spec.base_flange_thickness_mm + waist_h * 0.5
        for ang in spec.window_angles_deg:
            rad = math.radians(ang)
            r_mid_wall = 0.5 * (r_inner + r_waist)
            p_plane = Plane(
                origin=(r_mid_wall * math.cos(rad), r_mid_wall * math.sin(rad), window_z),
                z_dir=(math.cos(rad), math.sin(rad), 0.0),
                x_dir=(-math.sin(rad), math.cos(rad), 0.0),
            )
            with BuildSketch(p_plane):
                Rectangle(22.0, 12.0)
            extrude(amount=25.0, both=True, mode=Mode.SUBTRACT)

        # 9. [CLASP-021] - [CLASP-023] Three helical cam ramps at 0, 120, 240 deg
        for idx, cam_ang in enumerate((0.0, 120.0, 240.0)):
            r_in = spec.cam_primary_r_inner_mm if idx == 0 else spec.cam_r_inner_mm
            r_out = spec.cam_primary_r_outer_mm if idx == 0 else spec.cam_r_outer_mm
            ramp = build_cam_ramp(
                r_inner=r_in,
                r_outer=r_out,
                z_base=spec.cam_base_z_mm,
                angle_start_deg=cam_ang - spec.cam_span_deg * 0.5,
                span_deg=spec.cam_span_deg,
                lead_angle_deg=spec.cam_lead_angle_deg,
            )
            part.part = part.part.fuse(ramp)

        # [CLASP-023] Distinct clocking key boss on primary +X ramp
        with Locations((0.5 * (spec.cam_primary_r_inner_mm + spec.cam_primary_r_outer_mm), 0.0, spec.cam_base_z_mm)):
            Box(
                spec.cam_primary_r_outer_mm - spec.cam_primary_r_inner_mm,
                6.0,
                3.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )

        # 10. [CLASP-140] Asymmetric retroreflective fiducial pads + [CLASP-141] DataMatrix pad
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


def verify_part(solid, spec: ClaspMSpec) -> dict[str, float]:
    """Verify CLASP-M envelope, manifoldness, and mass requirements against CLASP-STD-001."""
    assert solid.is_valid, "Generated B-Rep solid is not valid"
    assert len(solid.solids()) == 1, f"Expected single connected solid, got {len(solid.solids())}"

    bbox = solid.bounding_box()
    diameter_x = bbox.max.X - bbox.min.X
    diameter_y = bbox.max.Y - bbox.min.Y
    height_z = bbox.max.Z - bbox.min.Z

    volume_cm3 = solid.volume / 1000.0
    mass_kg = (volume_cm3 * AL_7075_T6_DENSITY_G_CM3) / 1000.0
    com = solid.center()

    assert diameter_x <= spec.outer_diameter_mm + 1e-4, f"X span {diameter_x:.2f} mm > {spec.outer_diameter_mm}"
    assert diameter_y <= spec.outer_diameter_mm + 1e-4, f"Y span {diameter_y:.2f} mm > {spec.outer_diameter_mm}"
    assert bbox.min.Z >= -1e-4 and height_z <= spec.max_height_mm + 1e-4, f"Height {height_z:.2f} mm > {spec.max_height_mm}"
    assert mass_kg <= spec.mass_limit_kg, f"7075-T6 mass {mass_kg:.3f} kg > {spec.mass_limit_kg:.2f} kg"

    return {
        "diameter_x_mm": diameter_x,
        "diameter_y_mm": diameter_y,
        "height_mm": height_z,
        "volume_cm3": volume_cm3,
        "mass_7075_t6_kg": mass_kg,
        "mass_margin_kg": spec.mass_limit_kg - mass_kg,
        "com_x_mm": com.X,
        "com_y_mm": com.Y,
        "com_z_mm": com.Z,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and verify CLASP-M parametric STEP model.")
    parser.add_argument("--step", type=Path, default=Path(__file__).resolve().parent / "CLASP-M.step")
    parser.add_argument("--stl", type=Path, default=None)
    args = parser.parse_args()

    spec = ClaspMSpec()
    solid = build_clasp_m(spec)
    metrics = verify_part(solid, spec)

    args.step.parent.mkdir(parents=True, exist_ok=True)
    export_step(solid, str(args.step))
    if args.stl is not None:
        export_stl(solid, str(args.stl))

    print("CLASP-M Parametric CAD Verification: PASS")
    print(f"  Footprint : {metrics['diameter_x_mm']:.2f} x {metrics['diameter_y_mm']:.2f} mm (limit <= {spec.outer_diameter_mm:.1f} mm)")
    print(f"  Height    : {metrics['height_mm']:.2f} mm (limit <= {spec.max_height_mm:.1f} mm)")
    print(f"  Volume    : {metrics['volume_cm3']:.2f} cm^3")
    print(f"  Mass      : {metrics['mass_7075_t6_kg']:.3f} kg in Al 7075-T6 (limit <= {spec.mass_limit_kg:.2f} kg, margin +{metrics['mass_margin_kg']:.3f} kg)")
    print(f"  CoM       : ({metrics['com_x_mm']:.2f}, {metrics['com_y_mm']:.2f}, {metrics['com_z_mm']:.2f}) mm")
    print(f"  Exported  : {args.step}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

