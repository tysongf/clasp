#!/usr/bin/env python3
"""Spec linter and first-principles engineering sanity check for CLASP-STD-001."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

DOC_PATH = Path(__file__).resolve().parent.parent / "CLASP-STD-001_RevA_Working_Draft.md"

RFC2119_RE = re.compile(r"\*\*(shall not|shall|should not|should|may)\*\*")
REQ_RE = re.compile(r"^`\[(CLASP-\d{3})\]`\s+(.*)$")
HEADING_RE = re.compile(r"^#{2,3}\s+(\d+(?:\.\d+)?)\.?\s+(.+)$")
SECTION_REF_RE = re.compile(r"§(\d+(?:\.\d+)?)")


def extract_sections(lines: list[str]) -> set[str]:
    """Return all numbered section identifiers (e.g. '1', '1.2', '10.2')."""
    sections: set[str] = set()
    for line in lines:
        m = HEADING_RE.match(line.strip())
        if m:
            sections.add(m.group(1))
    return sections


def extract_requirements(lines: list[str]) -> list[tuple[int, str, str]]:
    """Return (line_no, req_id, text) tuples for all [CLASP-nnn] requirements."""
    reqs: list[tuple[int, str, str]] = []
    for idx, line in enumerate(lines, start=1):
        m = REQ_RE.match(line.strip())
        if m:
            reqs.append((idx, m.group(1), m.group(2)))
    return reqs


def lint_spec(text: str) -> list[str]:
    """Run structural, cross-reference, and RFC 2119 checks on the spec."""
    errors: list[str] = []
    lines = text.splitlines()
    sections = extract_sections(lines)
    reqs = extract_requirements(lines)

    # 1. Check broken section references
    for idx, line in enumerate(lines, start=1):
        for ref in SECTION_REF_RE.findall(line):
            if ref not in sections:
                errors.append(f"Line {idx}: Broken section reference §{ref}")

    # 2. Check requirement IDs: uniqueness, ordering, and RFC 2119 keywords
    seen: dict[str, int] = {}
    prev_num = -1
    for idx, req_id, body in reqs:
        num = int(req_id.split("-")[1])
        if req_id in seen:
            errors.append(f"Line {idx}: Duplicate requirement ID {req_id} (first at line {seen[req_id]})")
        seen[req_id] = idx
        if num <= prev_num:
            errors.append(f"Line {idx}: Non-monotonic requirement ID {req_id} after {prev_num:03d}")
        prev_num = num
        if not RFC2119_RE.search(body):
            errors.append(f"Line {idx}: {req_id} has no bold RFC 2119 keyword (**shall**, **should**, **may**)")

    # 3. Check normative references in §2 are cited in body (excluding informative & RFC 2119)
    in_refs = False
    normative_refs: list[str] = []
    body_after_refs: list[str] = []
    for line in lines:
        if line.strip().startswith("## 2. Normative references"):
            in_refs = True
            continue
        if in_refs and line.strip().startswith("---"):
            in_refs = False
            continue
        if in_refs and line.startswith("|") and not line.startswith("|---") and not line.startswith("| Reference"):
            parts = [p.strip() for p in line.strip("|").split("|")]
            if len(parts) >= 2 and "*(informative)*" not in parts[1] and parts[0] != "RFC 2119":
                normative_refs.append(parts[0])
        elif not in_refs:
            body_after_refs.append(line)

    body_blob = "\n".join(body_after_refs)
    for ref in normative_refs:
        if ref not in body_blob:
            errors.append(f"§2 Normative reference '{ref}' is not cited anywhere in the body")

    return errors


def engineering_sanity_checks() -> None:
    """Compute and print first-principles physical checks for CLASP-STD-001 parameters."""
    print("=== Engineering Sanity Checks ===")

    # 1. Lunar gravity & load sizing (§5.3)
    mass_kg = 2000.0
    g_lunar = 1.625  # m/s^2
    weight_kn = (mass_kg * g_lunar) / 1000.0
    daf = 3.0
    dynamic_kn = weight_kn * daf
    axial_limit_kn = 10.0
    bending_limit_knm = 2.0
    cg_offset_at_limit_m = bending_limit_knm / axial_limit_kn
    cg_offset_at_static_m = bending_limit_knm / weight_kn
    print(f"Lunar static weight (2,000 kg): {weight_kn:.2f} kN | 3.0x DAF: {dynamic_kn:.2f} kN (limit {axial_limit_kn:.1f} kN)")
    print(f"Max CG offset for 2.0 kN·m bending: {cg_offset_at_limit_m:.3f} m @ 10 kN | {cg_offset_at_static_m:.3f} m @ 3.25 kN")

    # 2. Bolt pattern load check (8 x M8 on 210 mm BCD)
    n_bolts = 8
    bcd_r_m = 0.210 / 2.0
    sum_y2 = (n_bolts / 2.0) * (bcd_r_m ** 2)
    max_bolt_tension_kn = (axial_limit_kn / n_bolts) + (bending_limit_knm * bcd_r_m / sum_y2)
    shear_kn = 5.0
    torsion_knm = 1.0
    max_bolt_shear_kn = (shear_kn / n_bolts) + (torsion_knm / (n_bolts * bcd_r_m))
    print(
        f"Bolt limit load (8xM8, 210 mm BCD): "
        f"{max_bolt_tension_kn:.2f} kN tension/bolt ({max_bolt_tension_kn * 1.4:.2f} kN ult), "
        f"{max_bolt_shear_kn:.2f} kN shear/bolt"
    )

    # 3. Cam ramp alignment/retraction efficiency (§5.2: 15° ± 2°; preload held by positive latch)
    for deg in (13.0, 15.0, 17.0):
        alpha = math.radians(deg)
        mu_lock = math.tan(alpha)
        # Retraction mechanical efficiency at nominal sliding friction mu = 0.20 (lubricated/coated UHV)
        mu_slide = 0.20
        eta_slide = math.tan(alpha) / math.tan(alpha + math.atan(mu_slide))
        print(
            f"Cam angle {deg:.0f}°: retraction eff @ mu={mu_slide:.2f} is {eta_slide * 100:.1f}% "
            f"(ref: friction-lock threshold mu={mu_lock:.3f}; preload held by positive latch)"
        )

    # 4. LOX flow velocity & trapped volume (§6.2, §6.3)
    bore_m = 0.025
    area_m2 = math.pi * (bore_m / 2.0) ** 2
    mdot_kg_s = 2.0
    rho_lox = 1141.0  # kg/m^3 at 90 K
    vel_m_s = mdot_kg_s / (rho_lox * area_m2)
    trapped_vol_m3 = 5.0e-6  # 5 cm^3
    equiv_plug_mm = (trapped_vol_m3 / area_m2) * 1000.0
    print(f"LOX flow (2.0 kg/s, 25 mm bore): velocity = {vel_m_s:.2f} m/s | 5 cm³ plug length = {equiv_plug_mm:.1f} mm")

    # 5. CLASP-M solid cylinder vs pocketed shell mass reference (240 mm OD x 55 mm H)
    od_m = 0.240
    h_m = 0.055
    solid_vol_cm3 = math.pi * (od_m / 2.0) ** 2 * h_m * 1e6
    for name, rho_g_cm3 in (("Al 7075-T6", 2.81), ("Ti-6Al-4V", 4.43), ("316L SS", 8.00)):
        solid_kg = (solid_vol_cm3 * rho_g_cm3) / 1000.0
        # Assume ~55% material removal by pocketing + central bore for CLASP-M
        pocketed_kg = solid_kg * 0.45
        print(f"CLASP-M envelope ({name}): solid = {solid_kg:.2f} kg | 55%-pocketed estimate = {pocketed_kg:.2f} kg (limit 3.50 kg)")


def main() -> int:
    text = DOC_PATH.read_text(encoding="utf-8")
    reqs = extract_requirements(text.splitlines())
    sections = extract_sections(text.splitlines())
    errors = lint_spec(text)

    if "--print-reqs" in sys.argv:
        for _, req_id, body in reqs:
            print(f"{req_id}\t{body}")
        return 0

    print(f"Parsed {len(sections)} numbered sections/subsections and {len(reqs)} [CLASP-nnn] requirements.")
    if errors:
        print("\nLINT ERRORS:")
        for err in errors:
            print(f"  - {err}")
    else:
        print("Spec lint: PASS (0 broken section refs, 0 ID/RFC-2119/reference errors).")

    print()
    engineering_sanity_checks()
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
