#!/usr/bin/env python3
"""[CLASP-151] Coupled 3D Heat-Conduction + Thermoelastic FEA:
  - 90 K LOX coupling bore vs 83 K lunar night / +120 C (393 K) sunlit housing
  - Evaluates Ti-6Al-4V vs 316L SS cryo cartridge/spokes inside Al 7075-T6 vs Al 6061-T6
  - Checks radial bore clearance (vs 0.500 mm nominal gap), constrained spoke thermal stress,
    conductive heat leak (W), and Z=44 mm cam/seal datum axial flatness warp.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from build123d import Align, BuildPart, Cylinder, Mode
from scipy.sparse.linalg import splu, spsolve
from skfem import Basis, BilinearForm, ElementTetP1, LinearForm, condense
from skfem.helpers import div, dot, grad
from skfem.models.elasticity import lame_parameters, linear_elasticity

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cad.clasp_fp_packaging import ClaspFPPackagingSpec, build_fluid_coupling, build_thermal_cartridge
from fea.fea_common import (
    MATERIALS,
    MaterialProps,
    build_vector_basis,
    compute_element_von_mises,
    margin_of_safety,
    mesh_shape_to_skfem,
    mesh_step_to_skfem,
)

T_REF_K = 293.15   # +20 C assembly reference
T_COLD_K = 83.15   # -190 C lunar night survival ([CLASP-150])
T_LOX_K = 90.0     # 90 K liquid oxygen service ([CLASP-050])
T_HOT_K = 393.15   # +120 C max operational shell


def build_fused_cryo_core(spec: ClaspFPPackagingSpec):
    """Fuse central 25 mm LOX coupling with thermal-break spokes & outer cartridge sleeve."""
    fluid = build_fluid_coupling(spec)
    cart = build_thermal_cartridge(spec)
    return fluid + cart


def solve_steady_heat_conduction(mesh, mat: MaterialProps, t_inner_k: float, t_outer_k: float, r_outer_mm: float = 73.5) -> tuple[np.ndarray, float]:
    """Solve Laplace div(k grad T) = 0 (P1 scalar) and return nodal temperatures (K) & heat leak (W)."""
    sbasis = Basis(mesh, ElementTetP1())

    @BilinearForm
    def a_form(u, v, w):
        # k [W/(m*K)] = 1e-3 * k [W/(mm*K)] in mm coordinates -> scalar ratio gives exact flux scaling
        return (mat.k_w_mk * 1e-3) * dot(grad(u), grad(v))

    K_th = a_form.assemble(sbasis)

    def dirichlet_mask(x: np.ndarray) -> np.ndarray:
        r = np.hypot(x[0], x[1])
        return (r <= 12.75) | (r >= r_outer_mm - 0.25)

    dofs_inner = sbasis.get_dofs(lambda x: np.hypot(x[0], x[1]) <= 12.75).flatten()
    dofs_outer = sbasis.get_dofs(lambda x: np.hypot(x[0], x[1]) >= r_outer_mm - 0.25).flatten()
    D = np.unique(np.concatenate([dofs_inner, dofs_outer]))

    T_bc = np.zeros(sbasis.N, dtype=np.float64)
    T_bc[dofs_inner] = t_inner_k
    T_bc[dofs_outer] = t_outer_k

    K_f, b_f, x0, I = condense(K_th, np.zeros(sbasis.N), x=T_bc, D=D)
    T = x0.copy()
    T[I] = spsolve(K_f, b_f)

    # Conductive heat leak (W) flowing into inner 90 K wall: reaction sum at inner Dirichlet DOFs
    q_nodal = K_th @ T
    heat_leak_w = float(-q_nodal[dofs_inner].sum())
    return T, heat_leak_w


def build_thermoelastic_rhs(basis: Basis, T_nodes_p1: np.ndarray, mat: MaterialProps, t_ref_k: float) -> np.ndarray:
    """Assemble exact continuum thermoelastic load RHS sum_e int (3*lam + 2*mu)*alpha*dT * div(v) dV."""
    lam, mu = lame_parameters(mat.e_mpa, mat.nu)
    beta_ref = 3.0 * lam + 2.0 * mu
    sbasis = basis.with_element(ElementTetP1())
    T_field = sbasis.interpolate(T_nodes_p1)

    @LinearForm
    def rhs_form(v, w):
        t_val = w.t_field.value
        # Apply cryo secant CTE where T < T_ref and hot CTE where T > T_ref
        alpha = np.where(t_val < t_ref_k, mat.alpha_cold_per_k, mat.alpha_hot_per_k)
        dT = t_val - t_ref_k
        return (beta_ref * alpha * dT) * div(v)

    return rhs_form.assemble(basis, t_field=T_field)


def analyze_housing_thermal_shift(mesh, basis, shell_mat: MaterialProps, delta_t_k: float) -> tuple[float, float]:
    """Compute exact 3D thermoelastic bore radial shift (mm) & Z=44 axial warp (mm) for CLASP-M-DFM."""
    lam, mu = lame_parameters(shell_mat.e_mpa, shell_mat.nu)
    K = linear_elasticity(lam, mu).assemble(basis)

    T_nodes = np.full(mesh.nvertices, T_REF_K + delta_t_k, dtype=np.float64)
    f_th = build_thermoelastic_rhs(basis, T_nodes, shell_mat, T_REF_K)

    # Pin Z=0 plane axially + dowel X/Y to prevent rigid-body modes cleanly
    D_z = basis.get_dofs(lambda x: np.abs(x[2]) < 0.25).all(["u^3"])
    D_pin1 = basis.get_dofs(lambda x: (np.abs(x[2]) < 0.25) & (np.hypot(x[0] - 105.0 * math.cos(math.radians(22.5)), x[1] - 105.0 * math.sin(math.radians(22.5))) <= 3.3)).all(["u^1", "u^2"])
    D_pin2 = basis.get_dofs(lambda x: (np.abs(x[2]) < 0.25) & (np.hypot(x[0] - 105.0 * math.cos(math.radians(202.5)), x[1] - 105.0 * math.sin(math.radians(202.5))) <= 3.3)).all(["u^2"])
    D_rb = np.unique(np.concatenate([D_z, D_pin1, D_pin2]))

    K_f, b_f, _, I = condense(K, f_th, D=D_rb)
    u = np.zeros(basis.N)
    u[I] = spsolve(K_f, b_f)

    coords = mesh.p.T
    u_xyz = np.column_stack([u[basis.nodal_dofs[0]], u[basis.nodal_dofs[1]], u[basis.nodal_dofs[2]]])
    r = np.hypot(coords[:, 0], coords[:, 1])
    bore_mask = (np.abs(r - 74.0) < 0.5) & (coords[:, 2] >= 5.0) & (coords[:, 2] <= 40.0)
    ur_bore = (u_xyz[bore_mask, 0] * coords[bore_mask, 0] + u_xyz[bore_mask, 1] * coords[bore_mask, 1]) / r[bore_mask]
    top_mask = (np.abs(coords[:, 2] - 44.0) < 0.3) & (r >= 92.0)
    z_warp = float(u_xyz[top_mask, 2].max() - u_xyz[top_mask, 2].min())
    return float(ur_bore.mean()), z_warp


def analyze_core_pair_cached(
    mesh,
    basis: Basis,
    spec: ClaspFPPackagingSpec,
    core_mat: MaterialProps,
    shell_mats: tuple[MaterialProps, ...],
    cases: tuple[tuple[float, str], ...],
) -> None:
    """Factor stiffness once per core_mat and solve all thermal + shell CTE pairs."""
    lam, mu = lame_parameters(core_mat.e_mpa, core_mat.nu)
    K = linear_elasticity(lam, mu).assemble(basis)

    coords = mesh.p.T
    r_v = np.hypot(coords[:, 0], coords[:, 1])
    D_free_z = basis.get_dofs(lambda x: (np.abs(x[2]) < 0.25) & (np.hypot(x[0], x[1]) >= 71.0)).all(["u^3"])
    D_free_x = basis.get_dofs(lambda x: (np.abs(x[2]) < 0.25) & (np.abs(x[0]) < 3.0) & (x[1] > 70.0)).all(["u^1"])
    D_free_y = basis.get_dofs(lambda x: (np.abs(x[2]) < 0.25) & (np.abs(x[1]) < 3.0) & (x[0] > 70.0)).all(["u^2"])
    D_free = np.unique(np.concatenate([D_free_z, D_free_x, D_free_y]))
    Kf, _, _, If = condense(K, np.zeros(basis.N), D=D_free)
    Kf_lu = splu(Kf.tocsc())

    D_rim = basis.get_dofs(lambda x: np.hypot(x[0], x[1]) >= (spec.cartridge_od_mm / 2.0 - 0.25)).flatten()
    Kc, _, _, Ic = condense(K, np.zeros(basis.N), D=D_rim)
    Kc_lu = splu(Kc.tocsc())

    # Pre-solve steady conduction + free thermal deformation once per thermal case (shell-independent)
    rim_mask = r_v >= (spec.cartridge_od_mm / 2.0 - 0.3)
    top_lox = coords[:, 2] >= (spec.fluid_mating_z_mm - 0.5)
    tet_c = mesh.p[:, mesh.t].mean(axis=1)
    r_el = np.hypot(tet_c[0], tet_c[1])
    spoke_mask = (r_el >= 34.0) & (r_el <= 68.5)
    dof_locs = basis.doflocs

    thermal_cache = []
    for t_outer_k, label in cases:
        T_nodes, q_w = solve_steady_heat_conduction(mesh, core_mat, T_LOX_K, t_outer_k, r_outer_mm=spec.cartridge_od_mm / 2.0)
        dT_elem = T_nodes[mesh.t].mean(axis=0) - T_REF_K
        f_th = build_thermoelastic_rhs(basis, T_nodes, core_mat, T_REF_K)
        u_free = np.zeros(basis.N)
        u_free[If] = Kf_lu.solve(f_th[If])
        u_free_xyz = np.column_stack([u_free[basis.nodal_dofs[0]], u_free[basis.nodal_dofs[1]], u_free[basis.nodal_dofs[2]]])
        dr_core_free = float(((u_free_xyz[rim_mask, 0] * coords[rim_mask, 0] + u_free_xyz[rim_mask, 1] * coords[rim_mask, 1]) / r_v[rim_mask]).mean())
        dz_lox = float(u_free_xyz[top_lox, 2].mean())
        thermal_cache.append((t_outer_k, label, q_w, dT_elem, f_th, dr_core_free, dz_lox))

    for shell_mat in shell_mats:
        print(f"\nPair: {shell_mat.name} Housing + {core_mat.name} Cryo Core (nominal 20 C gap = 0.500 mm):")
        for t_outer_k, label, q_w, dT_elem, f_th, dr_core_free, dz_lox in thermal_cache:
            alpha_al = shell_mat.alpha_cold_per_k if t_outer_k < T_REF_K else shell_mat.alpha_hot_per_k
            dr_al_bore = alpha_al * (t_outer_k - T_REF_K) * 74.0
            radial_gap_mm = 0.500 + dr_al_bore - dr_core_free

            u_bc = np.zeros(basis.N)
            scale = alpha_al * (t_outer_k - T_REF_K)
            u_bc[0::3] = scale * dof_locs[0, 0::3]
            u_bc[1::3] = scale * dof_locs[1, 1::3]
            u_bc[2::3] = scale * dof_locs[2, 2::3]
            x_d = np.zeros(basis.N)
            x_d[D_rim] = u_bc[D_rim]
            rhs_const = (f_th - K @ x_d)[Ic]
            u_const = x_d.copy()
            u_const[Ic] = Kc_lu.solve(rhs_const)

            vm_const = compute_element_von_mises(basis, u_const, core_mat, delta_t_elem=dT_elem, t_is_cold=True)
            spoke_peak = float(vm_const[spoke_mask].max())
            fty_crit = core_mat.fty_120c_mpa if t_outer_k > T_REF_K else core_mat.fty_20c_mpa
            mos_y = margin_of_safety(spoke_peak, fty_crit, 1.25)
            gap_status = "CLEAR" if radial_gap_mm > 0.05 else "INTERFERENCE/PINCH RISK"
            print(
                f"  {label:<28} | Heat={q_w:5.1f} W | Al={dr_al_bore:+.3f} Core={dr_core_free:+.3f} "
                f"-> Gap={radial_gap_mm:.3f} mm ({gap_status}) | LOX Z={dz_lox:+.3f} mm | "
                f"Spoke Peak={spoke_peak:5.1f} MPa (MoS_y={mos_y:+.2f})"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="[CLASP-151] Cryo/Hot coupled thermoelastic FEA.")
    parser.add_argument("--step", type=Path, default=Path(__file__).resolve().parent.parent / "cad" / "CLASP-M-DFM.step")
    args = parser.parse_args()

    t0 = time.perf_counter()
    spec = ClaspFPPackagingSpec()
    core_shape = build_fused_cryo_core(spec)

    housing_mesh = mesh_step_to_skfem(args.step, lc_min=4.0, lc_max=8.0)
    housing_basis = build_vector_basis(housing_mesh, order=1)
    core_mesh = mesh_shape_to_skfem(core_shape, lc_min=1.8, lc_max=3.8)
    core_basis = build_vector_basis(core_mesh, order=2)

    print("=== [CLASP-151] Outer Housing CLASP-M-DFM Thermal Bore & Cam Flatness ===")
    for shell_name in ("Al 7075-T6", "Al 6061-T6"):
        mat = MATERIALS[shell_name]
        dr_cold, warp_cold = analyze_housing_thermal_shift(housing_mesh, housing_basis, mat, T_COLD_K - T_REF_K)
        dr_hot, warp_hot = analyze_housing_thermal_shift(housing_mesh, housing_basis, mat, T_HOT_K - T_REF_K)
        print(
            f"  {shell_name:<12}: @ 83 K -> Bore dR={dr_cold:+.3f} mm, Cam Z-warp={warp_cold:.4f} mm | "
            f"@ +120 C -> Bore dR={dr_hot:+.3f} mm, Cam Z-warp={warp_hot:.4f} mm"
        )

    print(f"\n=== [CLASP-151] Inner 90 K LOX Core ({core_mesh.nelements:,} tets, {core_basis.N:,} DOFs) vs Outer Al Bore ===")
    shell_mats = (MATERIALS["Al 7075-T6"], MATERIALS["Al 6061-T6"])
    cases = (
        (T_COLD_K, "Uniform Cold (83 K / -190 C)"),
        (T_LOX_K,  "Uniform LOX  (90 K / -183 C)"),
        (T_HOT_K,  "Sunlit Grd   (90K->+120C)"),
    )
    for core_name in ("Ti-6Al-4V", "316L SS"):
        analyze_core_pair_cached(core_mesh, core_basis, spec, MATERIALS[core_name], shell_mats, cases)

    print(f"\nTotal thermoelastic FEA runtime: {time.perf_counter() - t0:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
