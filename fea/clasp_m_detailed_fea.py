#!/usr/bin/env python3
"""Detailed CLASP-M-DFM 3D FEA: discrete 3-latch/3-cam loads, clocking sweep,
compression-only Z=0 deck contact with preloaded M8 spring-bolts, and Al 7075-T6 vs 6061-T6."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import csc_matrix
from scipy.sparse.linalg import spsolve
from skfem import BilinearForm, FacetBasis, MeshTet, condense
from skfem.helpers import dot
from skfem.models.elasticity import lame_parameters, linear_elasticity

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fea.fea_common import (
    MATERIALS,
    MaterialProps,
    angular_diff_deg,
    build_traction_rhs,
    build_vector_basis,
    check_resultant,
    compute_element_von_mises,
    margin_of_safety,
    mesh_step_to_skfem,
)

BOLT_ANGLES_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 330.0)
DOWEL_ANGLES_DEG = (22.5, 202.5)
LATCH_ANGLES_DEG = (30.0, 150.0, 270.0)
CAM_ANGLES_DEG = (0.0, 120.0, 240.0)
R_BCD_MM = 105.0


def solve_discrete_hook_loads(
    tension_n: float,
    moment_knm: float,
    moment_dir_deg: float,
) -> tuple[np.ndarray, float, float]:
    """Solve unilateral 3-hook + collar-compression static equilibrium (N).

    Bending vector puts axial tension at `moment_dir_deg` and collar compression
    opposite at `moment_dir_deg + 180 deg` (effective collar R_c = 105 mm, hooks R_h = 110 mm).
    Returns (hook_forces_n[3], collar_comp_n, comp_angle_deg).
    """
    r_hook = 110.0
    r_comp = 105.0
    comp_deg = (moment_dir_deg + 180.0) % 360.0

    # Target moment M_xy from +Z uplift along moment_dir_deg:
    # r x (Fz e_z) = (y Fz, -x Fz, 0)
    th = math.radians(moment_dir_deg)
    mx_tgt = (moment_knm * 1e6) * math.sin(th)
    my_tgt = -(moment_knm * 1e6) * math.cos(th)

    # Unknowns x = [F_30, F_150, F_270, C_comp] >= 0
    A = np.zeros((3, 4), dtype=np.float64)
    for idx, ang in enumerate(LATCH_ANGLES_DEG):
        rad = math.radians(ang)
        xh, yh = r_hook * math.cos(rad), r_hook * math.sin(rad)
        A[0, idx] = 1.0
        A[1, idx] = yh
        A[2, idx] = -xh

    crad = math.radians(comp_deg)
    xc, yc = r_comp * math.cos(crad), r_comp * math.sin(crad)
    A[0, 3] = -1.0
    A[1, 3] = -yc
    A[2, 3] = xc

    b = np.array([tension_n, mx_tgt, my_tgt], dtype=np.float64)

    # Slight regularization prefers minimum total clamping/prying while matching exact equilibrium
    reg = 1e-6 * np.eye(4)
    A_aug = np.vstack([A, reg])
    b_aug = np.concatenate([b, np.zeros(4)])
    res = lsq_linear(A_aug, b_aug, bounds=(0.0, np.inf), tol=1e-12)
    sol = res.x
    return sol[:3], float(sol[3]), comp_deg


def build_robin_z_matrix(basis, mesh: MeshTet, facet_mask: np.ndarray, k_n_mm3: float) -> csc_matrix:
    """Assemble vertical-only surface spring matrix sum(k * u_z * v_z dS)."""
    facets = np.flatnonzero(facet_mask)
    if len(facets) == 0:
        return csc_matrix((basis.N, basis.N), dtype=np.float64)
    fbasis = FacetBasis(mesh, basis.elem, facets=facets)

    @BilinearForm
    def form(u, v, w):
        return k_n_mm3 * u[2] * v[2]

    return form.assemble(fbasis)


def build_bolt_shear_matrix(basis, mesh: MeshTet, facet_mask: np.ndarray, k_xy_n_mm3: float) -> csc_matrix:
    """Assemble weak in-plane washer friction/bolt shear matrix."""
    facets = np.flatnonzero(facet_mask)
    if len(facets) == 0:
        return csc_matrix((basis.N, basis.N), dtype=np.float64)
    fbasis = FacetBasis(mesh, basis.elem, facets=facets)

    @BilinearForm
    def form(u, v, w):
        return k_xy_n_mm3 * (u[0] * v[0] + u[1] * v[1])

    return form.assemble(fbasis)


def assemble_clocked_load_vector(
    basis,
    mesh: MeshTet,
    coords: np.ndarray,
    bfacets: np.ndarray,
    fbz: np.ndarray,
    fbr: np.ndarray,
    fbx: np.ndarray,
    fby: np.ndarray,
    clock_deg: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Assemble [CLASP-030] limit load using discrete 3-latch, 3-cam, and perimeter shear patches."""
    ang_deg = np.degrees(np.arctan2(fby, fbx)) % 360.0

    # 1. Discrete 3x latch hook bearing patches on Z=32 collar undercut (22 mm width -> +/-11 mm)
    hook_forces, comp_force, comp_deg = solve_discrete_hook_loads(10_000.0, 2.0, clock_deg)
    f_total = np.zeros(basis.N, dtype=np.float64)

    for idx, hang in enumerate(LATCH_ANGLES_DEG):
        f_req = float(hook_forces[idx])
        if f_req <= 1e-3:
            continue
        rad = math.radians(hang)
        tx_vec = -fby * math.cos(rad) + fbx * math.sin(rad)  # signed tangential distance
        patch = np.zeros(mesh.facets.shape[1], dtype=bool)
        patch[bfacets] = (
            (np.abs(fbz - 32.0) < 0.35)
            & (fbr >= 103.5)
            & (fbr <= 118.2)
            & (np.abs(angular_diff_deg(ang_deg, hang)) < 12.0)
            & (np.abs(tx_vec) <= 12.5)
        )
        f_raw = build_traction_rhs(basis, mesh, patch, lambda x, y, z: (0.0 * x, 0.0 * y, np.ones_like(z)))
        nf, _ = check_resultant(coords, f_raw)
        f_total += f_raw * (f_req / nf[2])

    # 2. Collar top compression crescent on Z=44 mm centered at comp_deg (+/-35 deg contact arc)
    if comp_force > 1e-3:
        comp_patch = np.zeros(mesh.facets.shape[1], dtype=bool)
        comp_patch[bfacets] = (
            (np.abs(fbz - 44.0) < 0.35)
            & (fbr >= 92.5)
            & (fbr <= 117.5)
            & (np.abs(angular_diff_deg(ang_deg, comp_deg)) <= 35.0)
        )
        f_raw = build_traction_rhs(basis, mesh, comp_patch, lambda x, y, z: (0.0 * x, 0.0 * y, -np.ones_like(z)))
        nf, _ = check_resultant(coords, f_raw)
        f_total += f_raw * (comp_force / (-nf[2]))

    # 3. Radial shear 5.0 kN along clock_deg over 180-deg bearing arc on outer collar R=120 mm
    sh_rad = math.radians(clock_deg)
    ux, uy = math.cos(sh_rad), math.sin(sh_rad)
    shear_patch = np.zeros(mesh.facets.shape[1], dtype=bool)
    shear_patch[bfacets] = (
        (fbr >= 119.3)
        & (fbz >= 34.0)
        & (fbz <= 42.0)
        & ((fbx * ux + fby * uy) > 0.0)
    )
    f_sraw = build_traction_rhs(
        basis,
        mesh,
        shear_patch,
        lambda x, y, z: (
            (x * ux + y * uy) * ux / np.maximum(np.hypot(x, y), 1e-6),
            (x * ux + y * uy) * uy / np.maximum(np.hypot(x, y), 1e-6),
            0.0 * z,
        ),
    )
    nf, _ = check_resultant(coords, f_sraw)
    f_shear_proj = float(nf[0] * ux + nf[1] * uy)
    f_total += f_sraw * (5_000.0 / f_shear_proj)

    # 4. Discrete 3x cam torsion pockets (1.0 kN·m split across 0, 120, 240 deg pockets)
    cam_patch = np.zeros(mesh.facets.shape[1], dtype=bool)
    for cang in CAM_ANGLES_DEG:
        cam_patch[bfacets] |= (
            (fbz >= 43.8)
            & (fbz <= 50.5)
            & (fbr >= 74.0)
            & (fbr <= 92.2)
            & (np.abs(angular_diff_deg(ang_deg, cang)) <= 16.0)
        )
    f_traw = build_traction_rhs(basis, mesh, cam_patch, lambda x, y, z: (-y, x, 0.0 * z))
    _, nm = check_resultant(coords, f_traw)
    f_total += f_traw * (1.0e6 / nm[2])

    return f_total, hook_forces, comp_force


def solve_preloaded_nonlinear_deck(
    basis,
    mesh: MeshTet,
    K_elastic: csc_matrix,
    coords: np.ndarray,
    bfacets: np.ndarray,
    fbx: np.ndarray,
    fby: np.ndarray,
    fbz: np.ndarray,
    f_ext: np.ndarray,
    bolt_preload_n: float = 15_000.0,
    max_iters: int = 8,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Solve active-set compression-only Z=0 deck contact + preloaded M8 spring-bolts."""
    # Fixed Dirichlet constraints: 2 x Ø6 mm dowel bearing walls (X/Y shear reaction, Z free)
    dowel_centers = np.array(
        [[R_BCD_MM * math.cos(math.radians(a)), R_BCD_MM * math.sin(math.radians(a))] for a in DOWEL_ANGLES_DEG]
    )

    def dowel_xy_mask(x: np.ndarray) -> np.ndarray:
        nx, ny, nz = x[0], x[1], x[2]
        m = np.zeros(len(nx), dtype=bool)
        in_flange = nz <= 10.2
        for cx, cy in dowel_centers:
            m |= in_flange & (np.hypot(nx - cx, ny - cy) <= 3.3)
        return m

    D_xy = basis.get_dofs(dowel_xy_mask).all(["u^1", "u^2"])

    # Washer pads (8 x M8 on Z=0, annulus r_w in [4.5, 9.0] mm -> area ~ 190.85 mm^2)
    bolt_centers = np.array(
        [[R_BCD_MM * math.cos(math.radians(a)), R_BCD_MM * math.sin(math.radians(a))] for a in BOLT_ANGLES_DEG]
    )
    bolt_masks = []
    all_washer_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    on_deck = np.abs(fbz) < 0.25
    for cx, cy in bolt_centers:
        d = np.hypot(fbx - cx, fby - cy)
        bm = np.zeros(mesh.facets.shape[1], dtype=bool)
        bm[bfacets] = on_deck & (d >= 4.2) & (d <= 9.5)
        bolt_masks.append(bm)
        all_washer_mask |= bm

    # M8 bolt axial stiffness ~ 450 kN/mm over ~190 mm^2 -> k_bolt = 2370 N/mm^3
    k_bolt_n_mm3 = 2370.0
    K_bolts = build_robin_z_matrix(basis, mesh, all_washer_mask, k_bolt_n_mm3)
    K_bolt_shear = build_bolt_shear_matrix(basis, mesh, all_washer_mask, 450.0)

    # Downward clamping preload on each bolt pad (15 kN/bolt, normalized exactly)
    f_preload = np.zeros(basis.N, dtype=np.float64)
    for bm in bolt_masks:
        fv = build_traction_rhs(basis, mesh, bm, lambda x, y, z: (0.0 * x, 0.0 * y, -np.ones_like(z)))
        nf, _ = check_resultant(coords, fv)
        f_preload += fv * (bolt_preload_n / (-nf[2]))

    # Elastic compression-only foundation across Z=0 underside (k_deck = 15,000 N/mm^3)
    k_deck_n_mm3 = 15_000.0
    deck_all = np.zeros(mesh.facets.shape[1], dtype=bool)
    deck_all[bfacets] = on_deck
    active_deck = deck_all.copy()

    rhs_total = f_ext + f_preload
    K_base = K_elastic + K_bolts + K_bolt_shear

    u = np.zeros(basis.N, dtype=np.float64)
    uz_vertex_dofs = basis.nodal_dofs[2]
    for _ in range(max_iters):
        K_deck = build_robin_z_matrix(basis, mesh, active_deck, k_deck_n_mm3)
        K_sys = K_base + K_deck
        K_free, b_free, _, I = condense(K_sys, rhs_total, D=D_xy)
        u_next = np.zeros(basis.N, dtype=np.float64)
        u_next[I] = spsolve(K_free, b_free)
        uz_verts = u_next[uz_vertex_dofs]
        facet_uz = uz_verts[mesh.facets[:, bfacets]].mean(axis=0)
        new_active = np.zeros(mesh.facets.shape[1], dtype=bool)
        new_active[bfacets] = on_deck & (facet_uz <= 1e-6)
        u = u_next
        if np.array_equal(new_active, active_deck):
            break
        active_deck = new_active

    # Compute individual M8 bolt axial reactions: F_bolt_i = F_preload + K_bolt_i * u_z
    bolt_forces_kn = np.zeros(len(BOLT_ANGLES_DEG), dtype=np.float64)
    for i, bm in enumerate(bolt_masks):
        Kb_i = build_robin_z_matrix(basis, mesh, bm, k_bolt_n_mm3)
        delta_f_n = float((Kb_i @ u)[2::3].sum())
        bolt_forces_kn[i] = (bolt_preload_n + delta_f_n) / 1000.0

    # Max Z=0 flange lift-off gap
    deck_verts = np.unique(mesh.facets[:, deck_all])
    uz_verts = u[uz_vertex_dofs]
    max_liftoff_mm = float(np.maximum(uz_verts[deck_verts], 0.0).max())
    return u, bolt_forces_kn, max_liftoff_mm, float(uz_verts[deck_verts].min())


def run_detailed_fea(
    step_path: Path,
    lc_min: float = 3.5,
    lc_max: float = 7.5,
    order: int = 2,
) -> int:
    t0 = time.perf_counter()
    mesh: MeshTet = mesh_step_to_skfem(step_path, lc_min=lc_min, lc_max=lc_max)

    # Phase 1: Linear-element 12-angle clocking sweep (finds worst hook/bolt clock angle)
    basis_p1 = build_vector_basis(mesh, order=1)
    coords_p1 = basis_p1.doflocs[:, ::3].T
    bfacets = mesh.boundary_facets()
    fc = mesh.p[:, mesh.facets[:, bfacets]].mean(axis=1)
    fbx, fby, fbz = fc[0], fc[1], fc[2]
    fbr = np.hypot(fbx, fby)

    mat_7075 = MATERIALS["Al 7075-T6"]
    lam, mu = lame_parameters(mat_7075.e_mpa, mat_7075.nu)
    K_p1 = linear_elasticity(lam, mu).assemble(basis_p1)

    tet_centers = mesh.p[:, mesh.t].mean(axis=1)
    near_bc = np.zeros(mesh.nelements, dtype=bool)
    for cx, cy in [
        (R_BCD_MM * math.cos(math.radians(a)), R_BCD_MM * math.sin(math.radians(a)))
        for a in BOLT_ANGLES_DEG + DOWEL_ANGLES_DEG
    ]:
        near_bc |= (tet_centers[2] < 3.0) & (np.hypot(tet_centers[0] - cx, tet_centers[1] - cy) < 12.0)
    interior_mask = ~near_bc

    print(f"=== Detailed CLASP-M-DFM Clocking & Bolt-Prying Sweep ({step_path.name}) ===")
    print(f"Mesh: {mesh.nelements:,} tets | Sweep order: TetP1 -> Final worst-case: TetP{order}")
    print("Bending + Shear clocking sweep (0..330 deg, discrete 3-latch/3-cam + Z=0 nonlinear deck, Al 7075-T6):")
    print(f"{'Clock':>5} | {'Max Hook':>10} | {'Body vM':>9} | {'Peak vM':>9} | {'Max Bolt':>9} | {'Z=0 Lift':>9}")
    print("-" * 68)

    worst_deg = 30.0
    worst_body_vm = -1.0
    for deg in range(0, 360, 30):
        f_ext, h_forces, _ = assemble_clocked_load_vector(
            basis_p1, mesh, coords_p1, bfacets, fbz, fbr, fbx, fby, float(deg)
        )
        u_sw, bolts_kn, liftoff_mm, _ = solve_preloaded_nonlinear_deck(
            basis_p1, mesh, K_p1, coords_p1, bfacets, fbx, fby, fbz, f_ext
        )
        vm = compute_element_von_mises(basis_p1, u_sw, mat_7075)
        body_vm = float(vm[interior_mask].max())
        peak_vm = float(vm.max())
        if body_vm > worst_body_vm:
            worst_body_vm = body_vm
            worst_deg = float(deg)
        print(
            f"{deg:4d}° | {h_forces.max()/1000.0:7.2f} kN | {body_vm:7.1f} MPa | "
            f"{peak_vm:7.1f} MPa | {bolts_kn.max():7.2f} kN | {liftoff_mm:7.4f} mm"
        )

    # Phase 2: Quadratic Tet10 final evaluation at worst clocking angle (Al 7075-T6 vs Al 6061-T6)
    print(f"\n=== High-Fidelity TetP{order} Worst-Case Verification at Clocking = {worst_deg:.0f}° ===")
    basis_hi = build_vector_basis(mesh, order=order)
    coords_hi = basis_hi.doflocs[:, ::3].T
    f_ext_hi, hook_f_hi, comp_f_hi = assemble_clocked_load_vector(
        basis_hi, mesh, coords_hi, bfacets, fbz, fbr, fbx, fby, worst_deg
    )
    nf_chk, nm_chk = check_resultant(coords_hi, f_ext_hi)
    print(
        f"Discrete Hook Reactions: 30°={hook_f_hi[0]/1e3:.2f} kN, 150°={hook_f_hi[1]/1e3:.2f} kN, "
        f"270°={hook_f_hi[2]/1e3:.2f} kN | Collar Comp={comp_f_hi/1e3:.2f} kN"
    )
    print(
        f"Resultant check: F=({nf_chk[0]:.1f}, {nf_chk[1]:.1f}, {nf_chk[2]:.1f}) N, "
        f"|M_xy|={math.hypot(nm_chk[0], nm_chk[1])/1e6:.3f} kN·m, Tz={nm_chk[2]/1e6:.3f} kN·m"
    )

    for mat_name in ("Al 7075-T6", "Al 6061-T6"):
        mat: MaterialProps = MATERIALS[mat_name]
        lam_m, mu_m = lame_parameters(mat.e_mpa, mat.nu)
        K_hi = linear_elasticity(lam_m, mu_m).assemble(basis_hi)
        u_hi, bolts_kn, liftoff_mm, min_z = solve_preloaded_nonlinear_deck(
            basis_hi, mesh, K_hi, coords_hi, bfacets, fbx, fby, fbz, f_ext_hi
        )
        vm = compute_element_von_mises(basis_hi, u_hi, mat)
        max_disp = float(np.linalg.norm(u_hi.reshape(-1, 3), axis=1).max())
        body_vm = float(vm[interior_mask].max())
        p999_vm = float(np.percentile(vm, 99.9))
        peak_vm = float(vm.max())

        # M8 Class 10.9 (proof ~ 30.4 kN) & A286 CRES (proof ~ 25.0 kN) bolt check
        max_bolt_14x = 15.0 + 1.40 * (float(bolts_kn.max()) - 15.0)
        mos_bolt_a286 = margin_of_safety(max_bolt_14x, 25.0, 1.0)

        print(f"\nMaterial: {mat.name} (E={mat.e_mpa:,.0f} MPa, {basis_hi.N:,} DOFs)")
        print(
            f"  1.00x Limit : Max Disp={max_disp:.4f} mm | Body vM={body_vm:.1f} MPa | "
            f"99.9%={p999_vm:.1f} MPa | Peak={peak_vm:.1f} MPa"
        )
        print(
            f"  Bolts & Deck: Max Bolt={bolts_kn.max():.2f} kN (15.0 kN preload, 1.4x Ult={max_bolt_14x:.2f} kN, "
            f"A286 MoS={mos_bolt_a286:+.2f}) | Z=0 Lift-Off={liftoff_mm:.4f} mm"
        )
        for label, fty, ftu in (
            ("20 C", mat.fty_20c_mpa, mat.ftu_20c_mpa),
            ("+120 C", mat.fty_120c_mpa, mat.ftu_120c_mpa),
        ):
            print(
                f"  [CLASP-031] {label:<6}: Body MoS_y={margin_of_safety(body_vm, fty, 1.25):+.2f}, "
                f"MoS_u={margin_of_safety(body_vm, ftu, 1.40):+.2f} | "
                f"Peak MoS_y={margin_of_safety(peak_vm, fty, 1.25):+.2f}, "
                f"MoS_u={margin_of_safety(peak_vm, ftu, 1.40):+.2f}"
            )

    print(f"\nTotal runtime: {time.perf_counter() - t0:.2f}s")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Detailed CLASP-M-DFM discrete/nonlinear FEA.")
    parser.add_argument("--step", type=Path, default=Path(__file__).resolve().parent.parent / "cad" / "CLASP-M-DFM.step")
    parser.add_argument("--order", type=int, choices=(1, 2), default=2)
    parser.add_argument("--lc-min", type=float, default=3.5)
    parser.add_argument("--lc-max", type=float, default=7.5)
    args = parser.parse_args()
    return run_detailed_fea(args.step, lc_min=args.lc_min, lc_max=args.lc_max, order=args.order)


if __name__ == "__main__":
    raise SystemExit(main())
