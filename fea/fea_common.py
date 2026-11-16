#!/usr/bin/env python3
"""Shared material allowables, gmsh STEP/B-Rep meshing, and scikit-fem elasticity helpers."""

from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

import gmsh
import numpy as np
from build123d import export_step
from skfem import (
    Basis,
    ElementTetP1,
    ElementTetP2,
    ElementVector,
    FacetBasis,
    LinearForm,
    MeshTet,
)
from skfem.models.elasticity import lame_parameters


@dataclass(frozen=True)
class MaterialProps:
    """Linear-elastic, thermal, and temperature-dependent strength allowables."""

    name: str
    e_mpa: float
    nu: float
    fty_20c_mpa: float
    ftu_20c_mpa: float
    fty_120c_mpa: float
    ftu_120c_mpa: float
    alpha_hot_per_k: float   # 293 K -> 393 K (+120 C)
    alpha_cold_per_k: float  # 293 K -> 90 K secant (NIST cryogenic average)
    k_w_mk: float            # Thermal conductivity


MATERIALS: dict[str, MaterialProps] = {
    "Al 7075-T6": MaterialProps(
        name="Al 7075-T6",
        e_mpa=71_700.0,
        nu=0.33,
        fty_20c_mpa=462.0,
        ftu_20c_mpa=538.0,
        fty_120c_mpa=370.0,   # Conservative 0.80x hot-soak knockdown (+120 C)
        ftu_120c_mpa=430.0,
        alpha_hot_per_k=23.6e-6,
        alpha_cold_per_k=19.8e-6,
        k_w_mk=130.0,
    ),
    "Al 6061-T6": MaterialProps(
        name="Al 6061-T6",
        e_mpa=68_900.0,
        nu=0.33,
        fty_20c_mpa=276.0,
        ftu_20c_mpa=310.0,
        fty_120c_mpa=210.0,   # Conservative ~0.76x hot-soak knockdown (+120 C)
        ftu_120c_mpa=248.0,
        alpha_hot_per_k=23.6e-6,
        alpha_cold_per_k=19.7e-6,
        k_w_mk=167.0,
    ),
    "Ti-6Al-4V": MaterialProps(
        name="Ti-6Al-4V",
        e_mpa=113_800.0,
        nu=0.34,
        fty_20c_mpa=880.0,
        ftu_20c_mpa=950.0,
        fty_120c_mpa=740.0,
        ftu_120c_mpa=810.0,
        alpha_hot_per_k=9.0e-6,
        alpha_cold_per_k=8.4e-6,
        k_w_mk=6.7,
    ),
    "17-4PH H900": MaterialProps(
        name="17-4PH H900",
        e_mpa=196_500.0,
        nu=0.27,
        fty_20c_mpa=1170.0,
        ftu_20c_mpa=1310.0,
        fty_120c_mpa=1050.0,
        ftu_120c_mpa=1180.0,
        alpha_hot_per_k=10.8e-6,
        alpha_cold_per_k=9.8e-6,
        k_w_mk=18.3,
    ),
    "316L SS": MaterialProps(
        name="316L SS",
        e_mpa=193_000.0,
        nu=0.30,
        fty_20c_mpa=205.0,
        ftu_20c_mpa=515.0,
        fty_120c_mpa=170.0,
        ftu_120c_mpa=440.0,
        alpha_hot_per_k=16.2e-6,
        alpha_cold_per_k=14.3e-6,
        k_w_mk=16.3,
    ),
}


def mesh_step_to_skfem(step_path: Path, lc_min: float, lc_max: float) -> MeshTet:
    """Mesh a STEP file in gmsh and return a scikit-fem MeshTet."""
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        gmsh.model.occ.importShapes(str(step_path))
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", lc_min)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", lc_max)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.model.mesh.generate(3)

        ntags, ncoords, _ = gmsh.model.mesh.getNodes()
        _, enodes = gmsh.model.mesh.getElementsByType(4)
    finally:
        gmsh.finalize()

    tag_to_idx = np.empty(int(ntags.max()) + 1, dtype=np.int64)
    tag_to_idx[ntags.astype(np.int64)] = np.arange(len(ntags))
    pts = np.asarray(ncoords, dtype=np.float64).reshape(-1, 3).T
    tets = tag_to_idx[np.asarray(enodes, dtype=np.int64).reshape(-1, 4)].T
    return MeshTet(pts, tets)


def mesh_shape_to_skfem(shape, lc_min: float, lc_max: float) -> MeshTet:
    """Export an in-memory build123d solid/compound to a temp STEP and mesh it."""
    with tempfile.NamedTemporaryFile(suffix=".step", delete=True) as tmp:
        export_step(shape, tmp.name)
        return mesh_step_to_skfem(Path(tmp.name), lc_min=lc_min, lc_max=lc_max)


def build_vector_basis(mesh: MeshTet, order: int = 2) -> Basis:
    """Create a 3D vector P1 or P2 finite-element basis."""
    elem = ElementVector(ElementTetP2() if order == 2 else ElementTetP1())
    return Basis(mesh, elem)


def build_traction_rhs(basis: Basis, mesh: MeshTet, facet_mask: np.ndarray, traction_fn) -> np.ndarray:
    """Assemble consistent nodal load vector from a facet traction function t(x, y, z) -> (tx, ty, tz)."""
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


def check_resultant(coords: np.ndarray, f: np.ndarray, origin: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[np.ndarray, np.ndarray]:
    """Return net force (N) and moment about origin (N·mm) from a nodal force vector."""
    f3 = f.reshape(-1, 3)
    rel = coords - np.asarray(origin, dtype=np.float64)
    net_f = f3.sum(axis=0)
    net_m = np.cross(rel, f3).sum(axis=0)
    return net_f, net_m


def compute_element_von_mises(
    basis: Basis,
    u: np.ndarray,
    mat: MaterialProps,
    delta_t_elem: np.ndarray | None = None,
    t_is_cold: bool = False,
) -> np.ndarray:
    """Evaluate QP-averaged element von Mises stress (MPa), optionally including thermal strain."""
    lam, mu = lame_parameters(mat.e_mpa, mat.nu)
    uh = basis.interpolate(u)
    grad_u = uh.grad
    eps = 0.5 * (grad_u + np.swapaxes(grad_u, 0, 1))
    trace_eps = eps[0, 0] + eps[1, 1] + eps[2, 2]

    sig = 2.0 * mu * eps
    for i in range(3):
        sig[i, i] += lam * trace_eps

    if delta_t_elem is not None:
        alpha = np.where(delta_t_elem < 0.0, mat.alpha_cold_per_k, mat.alpha_hot_per_k) if t_is_cold else mat.alpha_hot_per_k
        thermal_iso = (3.0 * lam + 2.0 * mu) * alpha[:, None] * delta_t_elem[:, None]
        for i in range(3):
            sig[i, i] -= thermal_iso

    sig_e = sig.mean(axis=-1)
    sxx, syy, szz = sig_e[0, 0], sig_e[1, 1], sig_e[2, 2]
    sxy, syz, szx = sig_e[0, 1], sig_e[1, 2], sig_e[2, 0]
    return np.sqrt(
        0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
        + 3.0 * (sxy ** 2 + syz ** 2 + szx ** 2)
    )


def margin_of_safety(stress_mpa: float, allowable_mpa: float, fos: float) -> float:
    """Compute aerospace Margin of Safety: allowable / (FoS * stress) - 1."""
    if stress_mpa <= 1e-12:
        return math.inf
    return (allowable_mpa / (fos * stress_mpa)) - 1.0


def angular_diff_deg(a_deg: np.ndarray | float, b_deg: float) -> np.ndarray | float:
    """Smallest signed angular separation wrapped into [-180, 180] degrees."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0

