#!/usr/bin/env python3
"""3D Quadratic (Tet10) FEA of CLASP-A Active End Effector components (cad/clasp_a.py):
  1. Single positive over-center latch hook under worst-case [CLASP-030] clocked uplift
  2. Single articulated 4-jaw wide-scoop arm under §5.1 capture impact + stall load (centered & wing-tip)
  3. Double-shear clevis pin & 3x Ø20 mm actuator column sizing checks
Evaluates Hybrid Flight materials (17-4PH H900, Ti-6Al-4V, Al 7075-T6, Al 6061-T6).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.sparse.linalg import spsolve
from skfem import condense
from skfem.models.elasticity import lame_parameters, linear_elasticity

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_a import ClaspASpec, build_capture_jaw_solid, build_latch_hook_solid
from fea.fea_common import (
    MATERIALS,
    MaterialProps,
    build_traction_rhs,
    build_vector_basis,
    check_resultant,
    compute_element_von_mises,
    margin_of_safety,
    mesh_shape_to_skfem,
)


def analyze_latch_hook(spec: ClaspASpec, worst_hook_load_n: float = 14_186.0, order: int = 2) -> None:
    """Solve 3D FEA of single latch hook under worst-case clocked [CLASP-030] limit load."""
    hook = build_latch_hook_solid(spec, 0.0, open_state=False)
    mesh = mesh_shape_to_skfem(hook, lc_min=1.8, lc_max=3.2)
    basis = build_vector_basis(mesh, order=order)
    coords = basis.doflocs[:, ::3].T

    bfacets = mesh.boundary_facets()
    fc = mesh.p[:, mesh.facets[:, bfacets]].mean(axis=1)
    fbx, fby, fbz = fc[0], fc[1], fc[2]

    # Boundary conditions:
    # Supported by carriage clevis pin/housing across upper spine (r >= 123.5, z >= 46.5 mm)
    def fixed_clevis(x: np.ndarray) -> np.ndarray:
        return (x[0] >= 123.3) & (x[2] >= 46.5)

    D = basis.get_dofs(fixed_clevis).flatten()

    # Load: collar undercut bearing reaction on nose top face (r in [104, 120], z = 31.5 mm) pushing -Z
    nose_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    nose_mask[bfacets] = (np.abs(fbz - spec.latch_z_engage_mm) < 0.35) & (fbx >= 103.5) & (fbx <= 120.5)
    f_raw = build_traction_rhs(basis, mesh, nose_mask, lambda x, y, z: (0.0 * x, 0.0 * y, -np.ones_like(z)))
    nf, _ = check_resultant(coords, f_raw)
    f_hook = f_raw * (worst_hook_load_n / (-nf[2]))

    # Mask outside immediate Dirichlet clamp edge to separate body throat stress from clamp edge
    tet_c = mesh.p[:, mesh.t].mean(axis=1)
    throat_mask = ~((tet_c[0] >= 122.0) & (tet_c[2] >= 45.0))

    print("=== CLASP-A Component 1: Positive Latch Hook 3D FEA ===")
    print(f"Mesh     : {mesh.nelements:,} tets, TetP{order}, {basis.N:,} DOFs")
    print(f"Load Case: Worst clocked hook reaction = {worst_hook_load_n/1e3:.2f} kN @ 1.0x Limit ({worst_hook_load_n*1.4/1e3:.2f} kN @ 1.4x Ult)")

    # Reference solve (scale stresses linearly per E_ref/E_mat -> displacements inverse to E)
    ref_mat = MATERIALS["17-4PH H900"]
    lam, mu = lame_parameters(ref_mat.e_mpa, ref_mat.nu)
    K = linear_elasticity(lam, mu).assemble(basis)
    K_f, b_f, _, I = condense(K, f_hook, D=D)
    u_ref = np.zeros(basis.N)
    u_ref[I] = spsolve(K_f, b_f)

    # Note: stress is practically independent of E (depends slightly on nu); solve exact nu per alloy
    print(f"{'Alloy':<12} | {'Nose Disp':>9} | {'Body vM':>9} | {'99.9% vM':>9} | {'+120C MoS_y':>11} | {'+120C MoS_u':>11}")
    print("-" * 76)
    for name in ("17-4PH H900", "Ti-6Al-4V", "Al 7075-T6"):
        mat: MaterialProps = MATERIALS[name]
        lam_m, mu_m = lame_parameters(mat.e_mpa, mat.nu)
        K_m = linear_elasticity(lam_m, mu_m).assemble(basis)
        Km_f, bm_f, _, Im = condense(K_m, f_hook, D=D)
        u_m = np.zeros(basis.N)
        u_m[Im] = spsolve(Km_f, bm_f)

        disp_mm = float(np.linalg.norm(u_m.reshape(-1, 3), axis=1).max())
        vm = compute_element_von_mises(basis, u_m, mat)
        body_vm = float(vm[throat_mask].max())
        p999_vm = float(np.percentile(vm, 99.9))
        # Use 99.9th percentile for unfilleted CAD corner alongside body peak for real throat stress
        mos_y = margin_of_safety(body_vm, mat.fty_120c_mpa, 1.25)
        mos_u = margin_of_safety(body_vm, mat.ftu_120c_mpa, 1.40)
        mos_p999_y = margin_of_safety(p999_vm, mat.fty_120c_mpa, 1.25)
        print(
            f"{mat.name:<12} | {disp_mm:7.3f} mm | {body_vm:7.1f} MPa | {p999_vm:7.1f} MPa | "
            f"{mos_y:+5.2f} ({mos_p999_y:+.2f}) | {mos_u:+11.2f}"
        )
    print("  *(Includes R3.5 mm inner throat fillet at (r=123.5, z=31.5); parenthesis = 99.9th-pct nominal throat MoS_y)*\n")


def analyze_capture_jaw(spec: ClaspASpec, order: int = 2) -> None:
    """Solve 3D FEA of one open 4-jaw scoop arm under §5.1 capture impact + stall loads."""
    jaw = build_capture_jaw_solid(spec, 0.0, open_state=True)
    mesh = mesh_shape_to_skfem(jaw, lc_min=2.4, lc_max=4.8)
    basis = build_vector_basis(mesh, order=order)
    coords = basis.doflocs[:, ::3].T

    bfacets = mesh.boundary_facets()
    fc = mesh.p[:, mesh.facets[:, bfacets]].mean(axis=1)
    fbx, fby, fbz = fc[0], fc[1], fc[2]

    # Supports: protruding Ø10 mm hinge pin ends (|y| >= 14.5) + rotary actuator drive face on back of pivot head
    def fixed_pivot_and_actuator(x: np.ndarray) -> np.ndarray:
        pin_ends = (np.hypot(x[0] - spec.jaw_pivot_r_mm, x[2] - spec.jaw_pivot_z_mm) <= 5.5) & (np.abs(x[1]) >= 14.2)
        act_stop = (x[0] >= spec.jaw_pivot_r_mm + 9.0) & (x[2] >= spec.jaw_pivot_z_mm - 8.5) & (x[2] <= spec.jaw_pivot_z_mm + 8.5)
        return pin_ends | act_stop

    D = basis.get_dofs(fixed_pivot_and_actuator).flatten()

    # Load 1: Centered scoop capture impact (1.5 kN bump + 3.0 kN stall = 4.5 kN +X, +1.5 kN +Z)
    paddle_inner = np.zeros(mesh.facets.shape[1], dtype=bool)
    paddle_inner[bfacets] = (fbx >= spec.jaw_open_radius_mm - 7.5) & (fbx <= spec.jaw_open_radius_mm - 6.5) & (fbz <= 28.5)
    f_center_x = build_traction_rhs(basis, mesh, paddle_inner, lambda x, y, z: (np.ones_like(x), 0.0 * y, 0.0 * z))
    f_center_z = build_traction_rhs(basis, mesh, paddle_inner, lambda x, y, z: (0.0 * x, 0.0 * y, np.ones_like(z)))
    nfx, _ = check_resultant(coords, f_center_x)
    nfz, _ = check_resultant(coords, f_center_z)
    f_centered = f_center_x * (4_500.0 / nfx[0]) + f_center_z * (1_500.0 / nfz[2])

    # Load 2: Asymmetric wing-tip strike (1.5 kN outward +X on outer +Y wing tip y in [45, 85] mm)
    wing_tip = np.zeros(mesh.facets.shape[1], dtype=bool)
    wing_tip[bfacets] = (fbx >= spec.jaw_open_radius_mm - 7.5) & (fbx <= spec.jaw_open_radius_mm - 6.5) & (fby >= 45.0)
    f_wing_raw = build_traction_rhs(basis, mesh, wing_tip, lambda x, y, z: (np.ones_like(x), 0.0 * y, 0.0 * z))
    nfw, _ = check_resultant(coords, f_wing_raw)
    f_wingtip = f_wing_raw * (1_500.0 / nfw[0])

    tet_c = mesh.p[:, mesh.t].mean(axis=1)
    away_from_pin = np.hypot(tet_c[0] - spec.jaw_pivot_r_mm, tet_c[2] - spec.jaw_pivot_z_mm) > 10.0

    print("=== CLASP-A Component 2: Open Wide-Scoop Capture Jaw 3D FEA ===")
    print(f"Mesh     : {mesh.nelements:,} tets, TetP{order}, {basis.N:,} DOFs")

    for mat_name in ("Al 7075-T6", "Al 6061-T6", "Ti-6Al-4V"):
        mat: MaterialProps = MATERIALS[mat_name]
        lam, mu = lame_parameters(mat.e_mpa, mat.nu)
        K = linear_elasticity(lam, mu).assemble(basis)
        K_f, _, _, I = condense(K, f_centered, D=D)

        print(f"\nMaterial: {mat.name} (20 C Fty={mat.fty_20c_mpa:.0f}, +120 C Fty={mat.fty_120c_mpa:.0f} MPa)")
        for case_name, f_vec in (
            ("Centered Bump+Stall (4.5 kN X + 1.5 kN Z)", f_centered),
            ("Wing-Tip Oblique Strike (1.5 kN @ y=+65 mm)", f_wingtip),
        ):
            u = np.zeros(basis.N)
            u[I] = spsolve(K_f, f_vec[I])
            tip_disp = float(np.linalg.norm(u.reshape(-1, 3), axis=1).max())
            vm = compute_element_von_mises(basis, u, mat)
            arm_vm = float(vm[away_from_pin].max())
            p999_vm = float(np.percentile(vm, 99.9))
            mos_y = margin_of_safety(arm_vm, mat.fty_120c_mpa, 1.25)
            mos_u = margin_of_safety(arm_vm, mat.ftu_120c_mpa, 1.40)
            print(
                f"  {case_name:<45} | Tip={tip_disp:5.2f} mm | Arm Peak={arm_vm:6.1f} MPa "
                f"(99.9%={p999_vm:5.1f}) | +120C MoS_y={mos_y:+.2f}, MoS_u={mos_u:+.2f}"
            )


def analyze_pins_and_columns(spec: ClaspASpec, worst_hook_load_n: float = 14_186.0) -> None:
    """Check Ø10 mm clevis pins (double shear) and 3x Ø20 mm actuator guide shafts."""
    print("\n=== CLASP-A Component 3: Pins & Actuator Columns Sizing ===")
    pin_area_mm2 = math.pi * (5.0 ** 2)  # Ø10 mm pin

    # 1. Latch clevis pin (double shear under worst_hook_load_n, tau_allow ~ 0.577 * Ftu/Fty)
    tau_latch_mpa = (worst_hook_load_n / (2.0 * pin_area_mm2))
    print(f"Latch Clevis Pin (Ø10 mm double shear @ {worst_hook_load_n/1e3:.2f} kN): tau = {tau_latch_mpa:.1f} MPa")
    for name in ("17-4PH H900", "Ti-6Al-4V", "Al 7075-T6"):
        mat = MATERIALS[name]
        tau_y = 0.577 * mat.fty_120c_mpa
        tau_u = 0.577 * mat.ftu_120c_mpa
        print(
            f"  {name:<12}: +120C MoS_y={margin_of_safety(tau_latch_mpa, tau_y, 1.25):+.2f}, "
            f"MoS_u={margin_of_safety(tau_latch_mpa, tau_u, 1.40):+.2f}"
        )

    # 2. Jaw hinge pin (double shear under 4.5 kN radial + 1.5 kN axial = 4.74 kN)
    f_jaw_pin = math.hypot(4_500.0, 1_500.0)
    tau_jaw_mpa = f_jaw_pin / (2.0 * pin_area_mm2)
    print(f"Jaw Hinge Pin    (Ø10 mm double shear @ {f_jaw_pin/1e3:.2f} kN): tau = {tau_jaw_mpa:.1f} MPa")
    for name in ("17-4PH H900", "Ti-6Al-4V", "Al 7075-T6"):
        mat = MATERIALS[name]
        print(
            f"  {name:<12}: +120C MoS_y={margin_of_safety(tau_jaw_mpa, 0.577 * mat.fty_120c_mpa, 1.25):+.2f}, "
            f"MoS_u={margin_of_safety(tau_jaw_mpa, 0.577 * mat.ftu_120c_mpa, 1.40):+.2f}"
        )

    # 3. 3x Ø20 mm actuator guide columns at R = 124 mm (60, 180, 300 deg)
    r_act = spec.actuator_radius_mm
    d_act = spec.actuator_diameter_mm
    a_act = math.pi * (d_act / 2.0) ** 2  # 314.16 mm^2
    # Maximum single-column axial load under 10 kN + 2.0 kN·m moment across 3 columns at 120 deg
    f_col_max_n = (10_000.0 / 3.0) + (2.0e6 * r_act) / (1.5 * r_act * r_act)
    sig_col_mpa = f_col_max_n / a_act
    print(f"3x Ø20 mm Columns (R={r_act:.0f} mm): Peak column axial = {f_col_max_n/1e3:.2f} kN -> sigma = {sig_col_mpa:.1f} MPa")
    for name in ("17-4PH H900", "Ti-6Al-4V", "Al 7075-T6"):
        mat = MATERIALS[name]
        print(
            f"  {name:<12}: +120C MoS_y={margin_of_safety(sig_col_mpa, mat.fty_120c_mpa, 1.25):+.2f}, "
            f"MoS_u={margin_of_safety(sig_col_mpa, mat.ftu_120c_mpa, 1.40):+.2f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="CLASP-A active mechanism 3D FEA.")
    parser.add_argument("--order", type=int, choices=(1, 2), default=2)
    parser.add_argument("--worst-hook-n", type=float, default=14_186.0)
    args = parser.parse_args()

    t0 = time.perf_counter()
    spec = ClaspASpec()
    analyze_latch_hook(spec, worst_hook_load_n=args.worst_hook_n, order=args.order)
    analyze_capture_jaw(spec, order=args.order)
    analyze_pins_and_columns(spec, worst_hook_load_n=args.worst_hook_n)
    print(f"\nTotal CLASP-A FEA runtime: {time.perf_counter() - t0:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

