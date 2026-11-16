# CLASP-STD-001

## Common Lunar Attachment and Servicing Port

### Interface Definition Document

| | |
|---|---|
| **Document** | CLASP-STD-001 |
| **Revision** | A (Working Draft) |
| **Date** | 2026-11-15 |
| **Status** | For comment. Not for design release. |
| **Issuing body** | *[TBD - see §11]* |
| **Distribution** | Unrestricted |

---

## Foreword

This document defines a common physical interface for attaching, handling, and servicing payloads on the lunar surface and in cislunar space.

Its design philosophy is borrowed from ISO 1161. The corner fitting of a freight container is a hole in a steel block. It is passive, costs almost nothing, and performs three functions at once: lifting point, lashing point, and stacking point. Nobody was persuaded to adopt it. It was cheaper to comply than to argue.

CLASP applies the same logic. All cost, actuation, sensing, and intelligence reside in the active end effector. The passive port is dumb metal. One geometry serves as grapple point, fluid port, and tie-down, because a payload integrator will accept one unfamiliar feature but not three.

**The governing requirement of this standard is economic, not mechanical.** If a CLASP-M port cannot be added to an arbitrary payload for under CAD 2,000 in recurring cost, this standard has failed regardless of its technical merit.

---

## 1. Scope

### 1.1 Applicability

This standard applies to mechanical capture, structural attachment, cryogenic fluid transfer, and contactless power and data exchange between:

- a robotic manipulator or docking mechanism (the **active side**), and
- a payload, tank, pallet, rover, or fixed structure (the **passive side**).

It covers operations on the lunar surface, in lunar orbit, and at cislunar libration points.

### 1.2 Exclusions

This revision does not cover:

- Crewed docking or pressurised mating. See IDSS IDD.
- Liquid hydrogen service. See §6.1 for rationale.
- Payloads exceeding 2,000 kg mass under lunar surface gravity (1.625 m/s², 3.25 kN static weight) on a single port. Multi-port and larger size classes are reserved for future revisions.
- Hypergolic or toxic propellant transfer.

### 1.3 Normative language

The key words **shall**, **shall not**, **should**, **should not**, and **may** are to be interpreted as described in RFC 2119. Requirements are identified by tags of the form `[CLASP-nnn]` and are individually verifiable per §10.

---

## 2. Normative references

| Reference | Title |
|---|---|
| IDSS IDD Rev E | International Docking System Standard Interface Definition Document *(informative)* |
| NASA-STD-6001 | Flammability, Offgassing and Compatibility Requirements |
| ANSI/AIAA S-080 | Space Systems: Metallic Pressure Vessels, Pressurized Structures, and Pressure Components |
| ASTM E595 | Total Mass Loss and Collected Volatile Condensable Materials from Outgassing |
| IEST-STD-CC1246 | Product Cleanliness Levels |
| ISO 1161 | Series 1 Freight Containers: Corner and Intermediate Fittings *(informative)* |
| RFC 2119 | Key Words for Use in RFCs to Indicate Requirement Levels |

---

## 3. Terms and definitions

**Active side** — The manipulator-mounted half of the interface. Contains all actuators, sensors, and control logic.

**Passive side (port)** — The payload-mounted half. Contains no actuators and requires no power.

**Soft capture** — Initial compliant engagement within the misalignment envelope of §5.1. Loads are low and the connection is not structural.

**Hard latch** — Preloaded rigid engagement. The interface is structural and load-bearing.

**Mating cycle** — One complete soft capture, alignment, hard latch, and release sequence.

**Trapped volume** — Fluid contained between the two poppet seats at the moment of demate. Lost to vacuum on separation.

**Chilldown** — Controlled pre-cooling of receiving hardware before full-rate cryogenic flow, to limit flash boiling.

---

## 4. Interface tiers

### 4.1 Tier definitions

`[CLASP-001]` Three tiers **shall** be defined:

| Tier | Capability | Typical user |
|---|---|---|
| **CLASP-M** | Mechanical capture and structural load path | Cargo pallets, regolith bins, structural elements, tie-downs |
| **CLASP-F** | CLASP-M plus cryogenic fluid coupling | Tanks, depots, landers, ISRU plant |
| **CLASP-FP** | CLASP-F plus contactless power and data | Rovers, instruments, powered payloads |

`[CLASP-002]` All tiers **shall** share an identical external footprint, bolt pattern, capture geometry, and alignment features.

`[CLASP-003]` A single active end effector **shall** be capable of capturing any tier without reconfiguration.

**Rationale.** Tiering exists so that low-value bulk cargo is not taxed for capability it does not need. A regolith bin should carry a CLASP-M port costing well under CAD 2,000. Common geometry preserves fleet interoperability.

### 4.2 Physical envelope

`[CLASP-010]` The passive port **shall** occupy a circular footprint of 240 mm diameter and **shall not** protrude more than 55 mm above the payload mounting plane.

`[CLASP-011]` The mounting pattern **shall** be 8 × M8 fasteners on a 210 mm bolt circle, with one position offset by 15° to enforce clocking.

`[CLASP-012]` Passive port mass **shall not** exceed:

| Tier | Mass limit |
|---|---|
| CLASP-M | 3.5 kg |
| CLASP-F | 6.0 kg |
| CLASP-FP | 7.5 kg |

### 4.3 Coordinate frame

The port frame origin lies at the centre of the mating plane *(working CAD baseline: `44.0 mm` axial offset from the `Z = 0` payload mounting flange to the collar cam datum, with `Ø 486.6 mm` active-grapple keep-out clear above `Z = 18.0 mm` in `cad/clasp_a.py`; formal datum and KOZ drawing to be frozen in Revision B)*.

- **+Z** — along the mating axis, positive outward from the payload
- **+X** — in the mating plane, aligned with the primary keying feature
- **+Y** — completes the right-handed set

---

## 5. Mechanical interface

### 5.1 Capture envelope

`[CLASP-020]` The interface **shall** achieve soft capture across the following misalignment envelope via an external perimeter grapple feature *(working CAD baseline: `240 mm` OD `Z = 32–44 mm` collar undercut in `cad/clasp_m_dfm.py` and `4`-jaw `CLASP-A` funnel with `Ø 486.6 mm` keep-out above `Z = 18.0 mm` in `cad/clasp_a.py`; formal groove tolerances TBD in Revision B)*:

| Parameter | Tolerance |
|---|---|
| Lateral offset (X, Y) | ±100 mm |
| Axial offset (Z) | ±50 mm |
| Angular misalignment (pitch, yaw) | ±15° |
| Roll misalignment | ±20° |
| Closing rate | 0 to 30 mm/s |
| Lateral rate | ≤ 15 mm/s |
| Angular rate | ≤ 1.0 °/s |

**Rationale.** This envelope is deliberately generous. Lunar illumination is the worst in the solar system for machine vision: no atmosphere means no fill light, shadows are effectively black, and at polar sites the sun sits permanently near the horizon producing kilometre-long shadows that sweep as the site rotates. Margin purchased in the mechanism is cheaper and more reliable than margin demanded from the perception stack.

### 5.2 Alignment

`[CLASP-021]` Alignment **shall** be achieved by three helical cam ramps spaced at 120°, drawing the interface into coaxial alignment as the active side retracts.

`[CLASP-022]` Cam lead angle **shall** be 15° ± 2° for alignment and retraction; structural preload and retention under the loads of §5.3 **shall** be carried by a positive mechanical latch rather than cam friction.

`[CLASP-023]` One ramp **shall** be geometrically distinct to establish unambiguous clocking.

`[CLASP-024]` Alignment **shall** be achieved by passive geometry. No powered actuation on the passive side is permitted.

### 5.3 Structural capability

`[CLASP-030]` When hard-latched, the interface **shall** carry the following limit loads without permanent deformation *(combined vs. independent load cases, joint stiffness, and hard-latched alignment tolerances TBD in Revision B)*:

| Load case | Limit |
|---|---|
| Axial tension / compression (Z) | 10 kN |
| Shear (X, Y) | 5 kN |
| Bending moment | 2.0 kN·m |
| Torsion about Z | 1.0 kN·m |

`[CLASP-031]` Yield factor of safety **shall** be ≥ 1.25 and ultimate ≥ 1.4 on limit load.

`[CLASP-032]` A positive mechanical latch **shall** maintain sufficient preload to prevent gapping under limit load without relying on cam friction.

**Sizing basis.** A 2,000 kg payload in lunar gravity weighs 3.25 kN. A dynamic amplification factor of 3.0 for robotic handling transients gives 9.75 kN, rounded to 10 kN. *(Note: the 2.0 kN·m bending limit corresponds to a 0.20 m centre-of-gravity offset at 10 kN transient load; shear/torsion dowel pins TBD.)*

### 5.4 Sequence

`[CLASP-040]` The mating sequence **shall** proceed in the following order, with each stage confirmed before the next begins:

1. Approach and soft capture
2. Dust mitigation cycle (§7)
3. Alignment and retraction
4. Hard latch and preload
5. Latch confirmation
6. Fluid or power connection, if applicable

`[CLASP-041]` Fluid poppets **shall not** open before hard latch confirmation. Mechanical integrity precedes fluid connection without exception.

`[CLASP-042]` The active side **shall** provide a mechanical release path recoverable without power or pressure.

---

## 6. Fluid interface (CLASP-F and CLASP-FP)

### 6.1 Service fluids

`[CLASP-050]` The interface **shall** be qualified for liquid oxygen at 90 K as the primary service fluid.

`[CLASP-051]` The interface **should** be qualified for liquid methane at 112 K as a secondary service fluid.

`[CLASP-052]` Liquid hydrogen service **shall not** be supported in Revision A.

**Rationale.** At an O/F ratio of 3.6, oxygen is 78% of propellant mass in a methalox stack and is the substantially easier cryogen to hold. Storing 100 t of LOX requires roughly 1 kW of cryocooler power against a sunshielded environment; the same mass of hydrogen occupies sixteen times the volume and needs approximately twenty times the lift power at 20 K. Hydrogen is where depot programmes go to die. Excluding it is a feature of this standard, not a limitation.

### 6.2 Coupling architecture

`[CLASP-060]` The coupling **shall** be of coaxial dual-poppet configuration, with opposed spring-loaded poppets that mutually displace on mating.

`[CLASP-061]` Nominal flow bore **shall** be 25 mm.

`[CLASP-062]` The coupling **shall** support a design flow rate of 2.0 kg/s LOX.

`[CLASP-063]` The coupling **shall** be located on the Z axis, centred in the port face.

### 6.3 Spill and leakage

`[CLASP-070]` Trapped volume **shall not** exceed 5 cm³.

`[CLASP-071]` External leakage when mated **shall not** exceed 1 × 10⁻⁴ scc/s helium.

`[CLASP-072]` Each poppet **shall** seal independently to the same rate when demated.

**Loss budget.** 5 cm³ of LOX at 1,141 kg/m³ is 5.7 g per demate. One thousand mating cycles costs 5.7 kg. This is negligible against a 100 t depot inventory and is the reason the dual-poppet architecture is mandated rather than recommended.

### 6.4 Sealing

`[CLASP-080]` Dynamic seats **shall** be PCTFE, or metallic C-seals where cycle life permits.

`[CLASP-081]` Elastomeric seals **shall not** be used in any cryogenic sealing function.

`[CLASP-082]` All non-metallic materials **shall** meet ASTM E595 with TML ≤ 1.0% and CVCM ≤ 0.10%.

`[CLASP-083]` All wetted materials **shall** be LOX-compatible per NASA-STD-6001 and cleaned to IEST-STD-CC1246 Level 100A or better.

**Rationale.** Elastomers embrittle at 90 K and degrade under unattenuated UV and charged-particle exposure. This interface must function after five years parked unattended in vacuum.

### 6.5 Chilldown

`[CLASP-090]` The coupling **shall** incorporate an integral bypass orifice of 2.0 mm nominal diameter, permitting pilot flow before main poppet actuation.

`[CLASP-091]` The chilldown sequence **shall** reduce receiving-side line and coupling body temperature to ≤ 120 K before main poppet opening.

`[CLASP-092]` Chilldown state (≤ 120 K) **shall** be verified by the active side before full-rate transfer is authorised: via active-side contact thermometry on the mated passive flange and bypass pressure/boiloff decay on `CLASP-F` (which remains sensor-free), or additionally via optical telemetry on `CLASP-FP`. *(Open items: define how the 2.0 mm bypass orifice seals when demated per [CLASP-072], and how two-phase boiloff vents during chilldown; see §12.)*

**Rationale.** Introducing 90 K liquid into warm receiving hardware causes flash boiling with typical losses of 5 to 10% of transferred mass. On the ground this is managed by crew procedure. Here it must be a sequence embedded in the interface itself, which places it in the standard rather than in each operator's operations manual. This requirement is the single largest determinant of whether propellant logistics closes economically.

### 6.6 Pressure

`[CLASP-100]` Maximum expected operating pressure **shall** be 2.0 MPa.

`[CLASP-101]` Proof pressure **shall** be 1.5 × MEOP (3.0 MPa).

`[CLASP-102]` Burst pressure **shall** be 2.5 × MEOP (5.0 MPa), verified per ANSI/AIAA S-080.

---

## 7. Dust mitigation

Lunar regolith is unweathered, electrostatically charged, and mechanically abrasive, with a median grain size near 70 μm and a significant sub-10 μm fraction. Apollo experience with seizing connectors and abraded seals is the governing precedent. No single mitigation is sufficient.

`[CLASP-110]` All tiers **shall** incorporate passive mechanical dust exclusion (§7.3); fluid tiers (`CLASP-F` and `CLASP-FP`) **shall** additionally incorporate electrodynamic dust mitigation (§7.1) and active-side gas purge (§7.2).

### 7.1 Electrodynamic dust shield (CLASP-F and CLASP-FP)

`[CLASP-111]` On `CLASP-F` and `CLASP-FP`, the sealing face **shall** incorporate an interdigitated electrode array capable of generating a travelling electrostatic wave.

`[CLASP-112]` The array **shall** be driven at 1.0 kV peak-to-peak minimum, 4-phase, 10 Hz nominal, drawing ≤ 5 W, and **shall** be de-energised prior to any purge gas admission or fluid transfer.

`[CLASP-113]` Drive power **shall** be supplied inductively from the active side. The passive port **shall** require no stored or wired power.

`[CLASP-114]` The system **shall** demonstrate ≥ 90% removal of particles ≥ 10 μm from the sealing face *(measurement basis—mass, area, or count—and initial dust loading TBD in Revision B)*.

### 7.2 Purge (CLASP-F and CLASP-FP)

`[CLASP-115]` On `CLASP-F` and `CLASP-FP`, the active side **shall** deliver a dry inert or service-matched gas purge pulse across the mating face immediately before contact, after EDS de-energisation.

`[CLASP-116]` Purge consumption **shall not** exceed 0.5 g per mating cycle, delivered over 0.5 s.

**Rationale.** Keeping `CLASP-M` strictly mechanical preserves its `< CAD 2,000` dumb-metal cost target and avoids forcing non-fluid robotic handlers to carry purge gas. For fluid depots, boiloff or inert purge gas is readily available and prevents dust contamination of cryogenic seals without energising high-voltage electrodes in a gas plume.

### 7.3 Mechanical exclusion (All tiers)

`[CLASP-117]` Lead-in chamfers **shall** be configured to mechanically wipe the sealing surfaces during the mating stroke.

`[CLASP-118]` The port **shall** accept a passive cover, removable and replaceable by the active end effector.

`[CLASP-119]` Where payload geometry permits, ports **should** be oriented face-down or vertical so that settling and levitating dust does not accumulate on the sealing face.

---

## 8. Power, data, and identification (CLASP-FP; §8.3 applies to all tiers)

### 8.1 Power (CLASP-FP)

`[CLASP-120]` Power transfer **shall** be inductive. Exposed electrical contacts **shall not** be permitted on any tier.

`[CLASP-121]` The interface **shall** deliver ≥ 500 W at ≥ 85% end-to-end efficiency.

`[CLASP-122]` Hardware safe/arm and inhibit functions **shall** be strictly non-contact: a mechanical latch-seat interlock inhibiting fluid poppet actuation (`[CLASP-041]`) on `CLASP-F` and `CLASP-FP`, and a passive magnetic or Hall-effect latch interlock inhibiting inductive power enable on `CLASP-FP`.

**Rationale.** Eliminating all exposed electrical contacts prevents pin damage, contact abrasion, dust bridging, and cryogenic contact fretting across 1,000 mating cycles, at a cost of roughly 10–15% power transfer efficiency. Mechanical and magnetic latch interlocks preserve hardware inhibits without relying on the optical data link.

### 8.2 Data (CLASP-FP)

`[CLASP-130]` Data transfer **shall** be by short-range optical link, ≥ 10 Mbit/s full duplex.

`[CLASP-131]` The link **shall** operate across the full hard-latched alignment tolerance *(tolerance values TBD in §5.3)*.

### 8.3 Passive identification (All tiers)

`[CLASP-140]` Every port, all tiers, **shall** carry passive retroreflective fiducials in an asymmetric pattern sufficient for six-degree-of-freedom pose estimation *(working CAD baseline: `5 × Ø 10 mm` bosses on `R = 105 mm` at `18°, 68°, 112°, 196°, 300°`; optical coating spec TBD in Revision B)*.

`[CLASP-141]` Every port, all tiers, **shall** carry a permanent, machine-readable static optical DataMatrix tag encoding, at minimum: port tier and unique serial number *(working CAD baseline: `18 × 18 mm` pad at `247.5°, R = 105 mm`; physical tolerance and laser-etch contrast specification TBD in Revision B)*. Dynamic operational state (payload contents, fill state, and accumulated cycle count) **shall** be maintained in active-side and fleet ledger records keyed by unique serial.

`[CLASP-142]` Fiducials and DataMatrix tag **shall** be readable with no power on the passive side using the active-side pose-estimation imaging system.

**Rationale.** An unpowered tank or regolith bin parked in permanent shadow for two years must remain fully addressable by an arriving manipulator. Keeping port identification purely optical and static keeps `CLASP-M` cost negligible and eliminates RF/battery survival risk across 5 years of radiation and thermal cycling.

---

## 9. Environment

`[CLASP-150]` The passive port **shall** survive and operate across the following environment:

| Parameter | Requirement |
|---|---|
| Operational temperature | −170 °C to +120 °C (ambient structure); down to 90 K (−183 °C) on cryogenic wetted surfaces during transfer |
| Survival temperature | −190 °C (83 K) to +130 °C *(bounds 90 K LOX service; 30–50 K PSR survival TBD)* |
| Thermal cycles | 500 full lunar day/night cycles minimum |
| Pressure | 10⁻¹⁰ to 10⁻¹² torr |
| Dust exposure | Per §10.2 simulant campaign |
| Storage, unattended | 5 years in lunar surface environment *(radiation/UV dose limits TBD)* |
| Mating cycles (CLASP-M) | 500 minimum |
| Mating cycles (CLASP-F, -FP) | 1,000 minimum |

`[CLASP-151]` Differential thermal expansion across the mated interface **shall not** compromise seal integrity or latch preload across the full operational range *(allowable material/CTE bounds TBD)*.

*(Open items for §9: launch vibration/shock loads, UHV anti-galling/cold-welding controls, and electrical bonding/ESD grounding resistance across the mated interface; see §12.)*

---

## 10. Verification and conformance

### 10.1 Verification methods

Each requirement is verified by Test (T), Analysis (A), Inspection (I), or Demonstration (D). The full verification cross-reference matrix is Annex A *(to be issued with Revision B)*.

### 10.2 Conformance campaign

`[CLASP-160]` Certification **shall** require successful completion of all of the following:

1. **Dust immersion** — Full cycle life in lunar regolith simulant (LHS-1 or JSC-1A) at representative charging state, with no degradation beyond allowables *(charging state, deposition density, and post-dust leak/functional allowables TBD)*.
2. **Thermal vacuum cycling** — 500 cycles across the full survival range at ≤ 10⁻⁶ torr.
3. **Cryogenic mate/demate** — Full rated cycle life with LOX, verifying trapped volume and leak rate at beginning and end of life.
4. **Misalignment matrix** — Capture demonstrated at extreme bounding corners of the §5.1 6-DOF envelope plus nominal *(representative test matrix TBD)*.
5. **Structural** — Limit and ultimate load application per §5.3.
6. **Interoperability** — Successful mate with the reference active end effector and with at least one independently manufactured port.

`[CLASP-161]` Certification testing **shall** be performed at a CLASP-accredited conformance facility.

`[CLASP-162]` Certified ports **shall** be marked with the CLASP conformance mark, tier designation, and certificate number, and **shall** be entered in the public conformance registry.

---

## 11. Licensing and governance

### 11.1 Position

The passive port specification is published **free of charge and free of encumbrance**. Any party may manufacture, sell, or integrate a conforming passive port with no royalty, licence fee, or permission.

This is deliberate. The standard earns nothing from the port and is not intended to. Universal adoption of the passive side is the entire objective.

### 11.2 Scope of open grant

The royalty-free grant in §11.1 covers the passive port interface specification and conformance geometry. The following remain independent of the passive specification grant:

1. **Active end effector implementations.** Proprietary manipulator mechanisms and reference hardware designs.
2. **Autonomy and pose-estimation software.** Capture, perception, and control stacks operating under lunar illumination and dust constraints.
3. **Accredited conformance testing and mark administration.** Qualification campaigns and use of the CLASP conformance mark are governed through accredited test facilities under §10 and §11.3 to preserve fleet-wide interoperability.

### 11.3 Governance

`[CLASP-170]` The standard **shall** be administered by a governing body with published membership, an open comment process, and a documented change-control procedure.

`[CLASP-171]` Backward compatibility of the passive port geometry **shall** be maintained across all future revisions.

*Constitution of the governing body is open. This is the principal item for resolution before Revision B.*

### 11.4 Adoption path

Early adoption targets nationally funded and commercial lunar surface hardware so that an initial multi-vendor installed base and verified interoperability record exist prior to Revision B release.

---

## 12. Open items for Revision B

| # | Item |
|---|---|
| 1 | Constitution and membership of the governing body (§11.3) |
| 2 | Annex A verification cross-reference matrix |
| 3 | Larger size class for payloads above 2,000 kg |
| 4 | Multi-port load-sharing rules and tolerance stack-up |
| 5 | Confirm inductive power efficiency achievable at 500 W under dust loading |
| 6 | EDS electrode durability across 1,000 mate cycles with mechanical wiping (§7.1 and §7.3 may be in tension) |
| 7 | Reference active end effector interface control document *(draft 4-jaw `CLASP-A` CAD/kinematics modeled in `cad/clasp_a.py`; formal ICD & mechanism tolerances TBD)* |
| 8 | Cost model validating the CAD 2,000 recurring target for `CLASP-M` *(initial `CLASP-M-DFM.step` commercial `Al 6061-T6` Qty 1 economy quote well below target; `Al 7075-T6` lot/inspection pricing TBD)* |
| 9 | Position on liquid hydrogen for a future CLASP-FH tier |
| 10 | Cross-compatibility assessment against IDSS and any emerging cargo standards |
| 11 | Confirm inductive EDS packaging for `CLASP-F`/`CLASP-FP` (passive sealing face vs. active-side sweep) and dust-removal metric basis (`[CLASP-114]`) |
| 12 | Validate safety-panel fault-tree compliance for zero-pin mechanical + magnetic/Hall inhibits (`[CLASP-122]`) |
| 13 | Define 2.0 mm bypass orifice demated sealing (`[CLASP-072]`), two-phase chilldown boiloff venting, and active-side flange contact thermometry accuracy (`[CLASP-092]`) |
| 14 | Freeze optical DataMatrix physical tolerances, laser-etch contrast, and fiducial coating specs (`[CLASP-140]`, `[CLASP-141]`) *(reference `18 × 18 mm` pad & 5-fiducial polar-shadow benchmark in `cad/clasp_m.py` & `perception/`)* |
| 15 | Freeze external perimeter grapple groove geometry, positive latch tolerances, keep-out zones (`Ø 486.6 mm` above `Z = 18.0 mm` baseline in `cad/clasp_a.py`), `44.0 mm` datum offset, passive cover interface, and latched tolerances |
| 16 | Review 2.0 kN·m bending moment limit (`[CLASP-030]`) against representative payload CG offsets and add shear/torsion locating dowel pins |
| 17 | Add launch vibration/shock, 30–50 K PSR survival, radiation/UV dose, UHV anti-galling/cold-welding, and ESD electrical bonding requirements (§9) |

---

## Annex A — Verification cross-reference matrix

*Working draft covering all `62` `[CLASP-nnn]` requirements is maintained in `Annex_A_Verification_Matrix_Draft.md` and parity-checked by `tools/clasp_check.py`. Formal baseline to be frozen with Revision B.*

## Annex B — Reference active end effector ICD

*Working parametric CAD and capture-kinematics baseline (`CLASP-A`: `4`-jaw scoop funnel, `3` cam followers, `3` over-center positive latches, `4.50 kg` mechanism / `8.09 kg` with active `CLASP-FP` core) is maintained in `cad/clasp_a.py` and `cad/CLASP-A-*.step`. Formal ICD drawing to be frozen with Revision B.*

---

*End of CLASP-STD-001 Revision A working draft. Comments to the issuing body.*
