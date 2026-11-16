#!/usr/bin/env python3
"""Preliminary 3D linear-elastic FEA of cad/CLASP-M.step under [CLASP-030] and [CLASP-031].

Solves 3D tetrahedral elasticity (Al 7075-T6) using gmsh + scikit-fem:
  - Fixed Dirichlet BCs on the 8 x M8 bolted washer pads (Z = 0) and 2 x Ø6 dowel walls
  - Exact statically normalized traction distributions for:
      * Axial tension (+Z = 10 kN) and compression (-Z = 10 kN)
      * Shear (+X = 5 kN, +Y = 5 kN)
      * Bending moment (M_y = 2.0 kN·m)
      * Torsion (T_z = 1.0 kN·m)
  - Superposes individual 1.0x limit cases and combined worst-case envelopes at
    1.0x Limit, 1.25x Yield FoS, and 1.40x Ultimate FoS ([CLASP-031]).
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import gmsh
import numpy as np
from scipy.sparse.linalg import spsolve
from skfem import (
    Basis,
    ElementTetP1,
    ElementTetP2,
    ElementVector,
    FacetBasis,
    LinearForm,
    MeshTet,
    condense,
)
from skfem.models.elasticity import lame_parameters, linear_elasticity

# Al 7075-T651 MMPDS-01 room-temperature baseline & conservative +120 C hot-soak knockdown (0.80x)
E_MPA = 71_700.0
NU = 0.33
FTY_20C_MPA = 462.0
FTU_20C_MPA = 538.0
FTY_120C_MPA = 370.0  # Conservative 0.80x thermal knockdown at [CLASP-150] +120 C max op temp
FTU_120C_MPA = 430.0

BOLT_ANGLES_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 330.0)
DOWEL_ANGLES_DEG = (22.5, 202.5)
R_BCD_MM = 105.0


def mesh_step_to_skfem(step_path: Path, lc_min: float, lc_max: float) -> MeshTet:
    """Import STEP geometry in gmsh and generate a 3D tetrahedral MeshTet."""
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.importShapes(str(step_path))
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", lc_min)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", lc_max)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.model.mesh.generate(3)

    ntags, ncoords, _ = gmsh.model.mesh.getNodes()
    _, enodes = gmsh.model.mesh.getElementsByType(4)  # 4-node linear tetrahedra
    gmsh.finalize()

    tag_to_idx = np.empty(int(ntags.max()) + 1, dtype=np.int64)
    tag_to_idx[ntags.astype(np.int64)] = np.arange(len(ntags))
    pts = np.asarray(ncoords, dtype=np.float64).reshape(-1, 3).T
    tets = tag_to_idx[np.asarray(enodes, dtype=np.int64).reshape(-1, 4)].T
    return MeshTet(pts, tets)


def build_traction_rhs(basis: Basis, mesh: MeshTet, facet_mask: np.ndarray, traction_fn) -> np.ndarray:
    """Assemble consistent nodal load vector from a facet traction function t(x, y, z) -> (3, ...)."""
    facets = np.flatnonzero(facet_mask)
    if len(facets) == 0:
        raise RuntimeError("Empty boundary facet set for load application")
    fbasis = FacetBasis(mesh, basis.elem, facets=facets)

    @LinearForm
    def form(v, w):
        x, y, z = w.x
        tx, ty, tz = traction_fn(x, y, z)
        return tx * v[0] + ty * v[1] + tz * v[2]

    return form.assemble(fbasis)


def check_resultant(coords: np.ndarray, f: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute net force (N) and moment about origin (N·mm) from nodal force vector."""
    f3 = f.reshape(-1, 3)
    net_f = f3.sum(axis=0)
    net_m = np.cross(coords, f3).sum(axis=0)
    return net_f, net_m


def compute_element_von_mises(basis: Basis, u: np.ndarray, lam: float, mu: float) -> np.ndarray:
    """Evaluate von Mises stress (MPa) at element centroids from displacement vector u."""
    uh = basis.interpolate(u)
    grad_u = uh.grad  # shape (3, 3, n_elems, n_qp)
    eps = 0.5 * (grad_u + np.swapaxes(grad_u, 0, 1))
    trace_eps = eps[0, 0] + eps[1, 1] + eps[2, 2]

    # Stress tensor sigma = lam * tr(eps) * I + 2 * mu * eps
    sig = 2.0 * mu * eps
    for i in range(3):
        sig[i, i] += lam * trace_eps

    # Average over quadrature points per element to avoid artificial single-QP spikes
    sig_e = sig.mean(axis=-1)  # (3, 3, n_elems)
    sxx, syy, szz = sig_e[0, 0], sig_e[1, 1], sig_e[2, 2]
    sxy, syz, szx = sig_e[0, 1], sig_e[1, 2], sig_e[2, 0]

    vm = np.sqrt(
        0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
        + 3.0 * (sxy ** 2 + syz ** 2 + szx ** 2)
    )
    return vm


def margin_of_safety(stress_mpa: float, allowable_mpa: float, fos: float) -> float:
    if stress_mpa <= 0.0:
        return math.inf
    return (allowable_mpa / (fos * stress_mpa)) - 1.0


def run_fea(step_path: Path, order: int = 2, lc_min: float = 5.0, lc_max: float = 8.5) -> int:
    t0 = time.perf_counter()
    mesh = mesh_step_to_skfem(step_path, lc_min=lc_min, lc_max=lc_max)
    elem = ElementVector(ElementTetP2() if order == 2 else ElementTetP1())
    basis = Basis(mesh, elem)
    coords = basis.doflocs[:, ::3].T  # (n_nodes, 3)

    lam, mu = lame_parameters(E_MPA, NU)
    K = linear_elasticity(lam, mu).assemble(basis)

    # Identify boundary facet centroids
    bfacets = mesh.boundary_facets()
    fc = mesh.p[:, mesh.facets[:, bfacets]].mean(axis=1)  # (3, n_bfacets)
    fx, fy, fz = fc[0], fc[1], fc[2]
    fr = np.hypot(fx, fy)

    # 1. Fixed Dirichlet BCs: 8 x M8 washer pads (Z = 0) + 2 x Ø6 dowel pin walls (Z = 0..10)
    bolt_centers = np.array(
        [[R_BCD_MM * math.cos(math.radians(a)), R_BCD_MM * math.sin(math.radians(a))] for a in BOLT_ANGLES_DEG]
    )
    dowel_centers = np.array(
        [[R_BCD_MM * math.cos(math.radians(a)), R_BCD_MM * math.sin(math.radians(a))] for a in DOWEL_ANGLES_DEG]
    )

    def fixed_nodes_mask(x: np.ndarray) -> np.ndarray:
        nx, ny, nz = x[0], x[1], x[2]
        mask = np.zeros(len(nx), dtype=bool)
        # Clamped M8 washer pads at Z = 0 (4.2 <= dist <= 10.5 mm)
        on_base = np.abs(nz) < 0.25
        for cx, cy in bolt_centers:
            d = np.hypot(nx - cx, ny - cy)
            mask |= on_base & (d >= 4.2) & (d <= 10.5)
        # Dowel pin bearing hole walls (Z <= 10.2 mm, d <= 3.3 mm)
        in_flange = nz <= 10.2
        for cx, cy in dowel_centers:
            d = np.hypot(nx - cx, ny - cy)
            mask |= in_flange & (d <= 3.3)
        return mask

    D = basis.get_dofs(nodes=mesh.nodes_satisfying(fixed_nodes_mask) if order == 1 else fixed_nodes_mask).flatten()
    if len(D) == 0:
        raise RuntimeError("No Dirichlet DOFs selected on bolt/dowel pads")

    # 2. Load surface facet masks
    # Bottom undercut shoulder of upper collar (Z = 32.0 mm, R = 91.5..117.5 mm)
    latch_shoulder_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    latch_shoulder_mask[bfacets] = (np.abs(fz - 32.0) < 0.35) & (fr >= 91.5) & (fr <= 117.5)

    # Top face of upper collar outside cams (Z = 44.0 mm, R = 92.5..117.5 mm)
    collar_top_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    collar_top_mask[bfacets] = (np.abs(fz - 44.0) < 0.35) & (fr >= 92.5) & (fr <= 117.5)

    # Outer cylindrical shear perimeter (+X bearing half on R = 120 mm, Z = 34.2..41.8 mm)
    shear_x_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    shear_x_mask[bfacets] = (fr >= 119.5) & (fz >= 34.2) & (fz <= 41.8) & (fx > 0.0)

    shear_y_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    shear_y_mask[bfacets] = (fr >= 119.5) & (fz >= 34.2) & (fz <= 41.8) & (fy > 0.0)

    # Bending split: compression (-Z) on +X collar top, tension (+Z) on -X latch shoulder
    bend_comp_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    bend_comp_mask[bfacets] = (np.abs(fz - 44.0) < 0.35) & (fr >= 92.5) & (fr <= 117.5) & (fx > 0.0)
    bend_tens_mask = np.zeros(mesh.facets.shape[1], dtype=bool)
    bend_tens_mask[bfacets] = (np.abs(fz - 32.0) < 0.35) & (fr >= 91.5) & (fr <= 117.5) & (fx < 0.0)

    # 3. Assemble & normalize exact unit/limit load vectors
    # (a) Axial Tension: +10,000 N along +Z
    f_tens_raw = build_traction_rhs(basis, mesh, latch_shoulder_mask, lambda x, y, z: (0.0 * x, 0.0 * y, np.ones_like(z)))
    net_f, _ = check_resultant(coords, f_tens_raw)
    f_tension = f_tens_raw * (10_000.0 / net_f[2])

    # (b) Axial Compression: -10,000 N along -Z
    f_comp_raw = build_traction_rhs(basis, mesh, collar_top_mask, lambda x, y, z: (0.0 * x, 0.0 * y, -np.ones_like(z)))
    net_f, _ = check_resultant(coords, f_comp_raw)
    f_compression = f_comp_raw * (10_000.0 / (-net_f[2]))

    # (c) Shear +X: +5,000 N along +X (cosine bearing on +X collar rim)
    f_sx_raw = build_traction_rhs(
        basis, mesh, shear_x_mask, lambda x, y, z: (x / np.maximum(np.hypot(x, y), 1e-6), 0.0 * y, 0.0 * z)
    )
    net_f, _ = check_resultant(coords, f_sx_raw)
    f_shear_x = f_sx_raw * (5_000.0 / net_f[0])

    # (d) Shear +Y: +5,000 N along +Y
    f_sy_raw = build_traction_rhs(
        basis, mesh, shear_y_mask, lambda x, y, z: (0.0 * x, y / np.maximum(np.hypot(x, y), 1e-6), 0.0 * z)
    )
    net_f, _ = check_resultant(coords, f_sy_raw)
    f_shear_y = f_sy_raw * (5_000.0 / net_f[1])

    # (e) Bending Moment My = 2.0 kN·m = 2.0e6 N·mm (exact zero net axial force)
    f_bc = build_traction_rhs(basis, mesh, bend_comp_mask, lambda x, y, z: (0.0 * x, 0.0 * y, -x))
    f_bt = build_traction_rhs(basis, mesh, bend_tens_mask, lambda x, y, z: (0.0 * x, 0.0 * y, -x))
    fc_net, _ = check_resultant(coords, f_bc)
    ft_net, _ = check_resultant(coords, f_bt)
    # Balance net Fz = 0, then scale to exact My = +2.0e6 N·mm (note: r x F_z has M_y = -x * F_z)
    f_bend_bal = f_bc + (-fc_net[2] / ft_net[2]) * f_bt
    _, bm_net = check_resultant(coords, f_bend_bal)
    f_bending = f_bend_bal * (2.0e6 / bm_net[1])

    # (f) Torsion Tz = 1.0 kN·m = 1.0e6 N·mm about +Z
    f_tor_raw = build_traction_rhs(basis, mesh, collar_top_mask, lambda x, y, z: (-y, x, 0.0 * z))
    _, tm_net = check_resultant(coords, f_tor_raw)
    f_torsion = f_tor_raw * (1.0e6 / tm_net[2])

    # 4. Condense & solve all 6 basis load vectors
    basis_loads = {
        "Axial Tension (10 kN)": f_tension,
        "Axial Compression (10 kN)": f_compression,
        "Shear +X (5 kN)": f_shear_x,
        "Shear +Y (5 kN)": f_shear_y,
        "Bending My (2.0 kN·m)": f_bending,
        "Torsion Tz (1.0 kN·m)": f_torsion,
    }

    K_free, _, _, I = condense(K, np.zeros(basis.N), D=D)
    F_mat = np.column_stack([f[I] for f in basis_loads.values()])
    t_solve0 = time.perf_counter()
    U_free = spsolve(K_free, F_mat)
    t_solve1 = time.perf_counter()

    displacements: dict[str, np.ndarray] = {}
    for col, name in enumerate(basis_loads):
        u_full = np.zeros(basis.N)
        u_full[I] = U_free[:, col]
        displacements[name] = u_full

    # Combined worst-case limit load: Tension + Shear +X + Bending My + Torsion Tz
    u_comb_limit = (
        displacements["Axial Tension (10 kN)"]
        + displacements["Shear +X (5 kN)"]
        + displacements["Bending My (2.0 kN·m)"]
        + displacements["Torsion Tz (1.0 kN·m)"]
    )
    f_comb_limit = f_tension + f_shear_x + f_bending + f_torsion
    comb_f, comb_m = check_resultant(coords, f_comb_limit)

    # Identify elements away from immediate BC singularity (Z > 2.5 mm or outside clamping pads)
    tet_centers = mesh.p[:, mesh.t].mean(axis=1)  # (3, n_tets)
    near_bc = np.zeros(mesh.nelements, dtype=bool)
    in_bc_z = tet_centers[2] < 3.0
    for cx, cy in np.vstack([bolt_centers, dowel_centers]):
        d = np.hypot(tet_centers[0] - cx, tet_centers[1] - cy)
        near_bc |= in_bc_z & (d < 12.5)
    interior_mask = ~near_bc

    print("=== CLASP-M Preliminary 3D FEA Report (Al 7075-T6) ===")
    print(f"Mesh     : {mesh.nelements:,} tets, TetP{order}, {basis.N:,} DOFs ({len(D):,} fixed)")
    print(f"Solve    : {t_solve1 - t_solve0:.2f}s (total {time.perf_counter() - t0:.2f}s)")
    print(f"Material : E = {E_MPA:,.0f} MPa, nu = {NU}")
    print(f"Allowable: 20 C (Fty={FTY_20C_MPA:.0f}, Ftu={FTU_20C_MPA:.0f} MPa) | +120 C hot-soak (Fty={FTY_120C_MPA:.0f}, Ftu={FTU_120C_MPA:.0f} MPa)")
    print(f"Combined resultant check: F=({comb_f[0]:.1f}, {comb_f[1]:.1f}, {comb_f[2]:.1f}) N, "
          f"M=({comb_m[0]/1e6:.3f}, {comb_m[1]/1e6:.3f}, {comb_m[2]/1e6:.3f}) kN·m\n")

    header = f"{'Load Case':<30} | {'Max Disp':>9} | {'Peak vM':>9} | {'99.9% vM':>9} | {'Body Peak':>10}"
    print(header)
    print("-" * len(header))

    def report_case(label: str, u: np.ndarray) -> tuple[float, float]:
        u3 = u.reshape(-1, 3)
        max_disp = float(np.linalg.norm(u3, axis=1).max())
        vm = compute_element_von_mises(basis, u, lam, mu)
        peak_all = float(vm.max())
        p999 = float(np.percentile(vm, 99.9))
        peak_body = float(vm[interior_mask].max())
        print(f"{label:<30} | {max_disp:7.4f} mm | {peak_all:7.1f} MPa | {p999:7.1f} MPa | {peak_body:8.1f} MPa")
        return peak_all, peak_body

    for name, u in displacements.items():
        report_case(name, u)
    print("-" * len(header))
    peak_1x, body_1x = report_case("Combined Limit (1.00x)", u_comb_limit)
    report_case("Combined Yield (1.25x)", 1.25 * u_comb_limit)
    report_case("Combined Ultimate (1.40x)", 1.40 * u_comb_limit)

    # Compute structural margins per [CLASP-031] on combined worst-case
    print("\n=== [CLASP-031] Combined Worst-Case Margins of Safety ===")
    for temp_label, fty, ftu in (
        ("20 C (MMPDS-01)", FTY_20C_MPA, FTU_20C_MPA),
        ("+120 C (0.80x hot)", FTY_120C_MPA, FTU_120C_MPA),
    ):
        mos_y_body = margin_of_safety(body_1x, fty, 1.25)
        mos_u_body = margin_of_safety(body_1x, ftu, 1.40)
        mos_y_peak = margin_of_safety(peak_1x, fty, 1.25)
        mos_u_peak = margin_of_safety(peak_1x, ftu, 1.40)
        print(
            f"  {temp_label:<18}: "
            f"Body MoS_y={mos_y_body:+.2f}, MoS_u={mos_u_body:+.2f} | "
            f"Global Peak MoS_y={mos_y_peak:+.2f}, MoS_u={mos_u_peak:+.2f}"
        )

    # Joint rotational & axial stiffness estimates
    u_ax = displacements["Axial Tension (10 kN)"].reshape(-1, 3)
    collar_nodes = np.abs(coords[:, 2] - 44.0) < 0.5
    mean_uz_mm = float(u_ax[collar_nodes, 2].mean())
    k_axial_kn_mm = 10.0 / max(mean_uz_mm, 1e-9)

    u_bend = displacements["Bending My (2.0 kN·m)"].reshape(-1, 3)
    # Fit theta_y ~ -u_z / x across collar top ring
    x_c = coords[collar_nodes, 0]
    uz_c = u_bend[collar_nodes, 2]
    nz_x = np.abs(x_c) > 20.0
    theta_y_rad = float(-np.sum(x_c[nz_x] * uz_c[nz_x]) / np.sum(x_c[nz_x] ** 2))
    k_rot_knm_mrad = 2.0 / max(theta_y_rad * 1e3, 1e-9)
    print(f"\nEstimated Joint Stiffness: k_axial = {k_axial_kn_mm:.1f} kN/mm | k_bending = {k_rot_knm_mrad:.2f} kN·m/mrad")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Preliminary 3D FEA of CLASP-M under [CLASP-030]/[CLASP-031].")
    parser.add_argument("--step", type=Path, default=Path(__file__).resolve().parent.parent / "cad" / "CLASP-M.step")
    parser.add_argument("--order", type=int, choices=(1, 2), default=2)
    parser.add_argument("--lc-min", type=float, default=5.0)
    parser.add_argument("--lc-max", type=float, default=8.5)
    args = parser.parse_args()
    return run_fea(args.step, order=args.order, lc_min=args.lc_min, lc_max=args.lc_max)


if __name__ == "__main__":
    raise SystemExit(main())

