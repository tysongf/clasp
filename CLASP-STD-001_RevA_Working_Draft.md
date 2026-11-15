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

The port frame origin lies at the centre of the mating plane *(axial datum offset from the payload mounting plane, plus payload keep-out zones, to be defined in Revision B)*.

- **+Z** — along the mating axis, positive outward from the payload
- **+X** — in the mating plane, aligned with the primary keying feature
- **+Y** — completes the right-handed set

---

## 5. Mechanical interface

### 5.1 Capture envelope

`[CLASP-020]` The interface **shall** achieve soft capture across the following misalignment envelope:

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

`[CLASP-022]` Cam lead angle **shall** be 15° ± 2°, selected to be non-backdriving under the loads of §5.3. *(Open item: verify whether friction-locked cams are viable across UHV/dust friction variations vs. a positive mechanical latch; see §12.)*

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

`[CLASP-032]` Latch preload **shall** be sufficient to prevent gapping under limit load.

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

`[CLASP-092]` Chilldown state **shall** be reported to the active side before full-rate transfer is authorised. *(Open items: define how CLASP-F—which lacks a power/data link—reports chilldown state, how the 2.0 mm bypass orifice seals when demated per [CLASP-072], and how two-phase boiloff vents during chilldown; see §12.)*

**Rationale.** Introducing 90 K liquid into warm receiving hardware causes flash boiling with typical losses of 5 to 10% of transferred mass. On the ground this is managed by crew procedure. Here it must be a sequence embedded in the interface itself, which places it in the standard rather than in each operator's operations manual. This requirement is the single largest determinant of whether propellant logistics closes economically.

### 6.6 Pressure

`[CLASP-100]` Maximum expected operating pressure **shall** be 2.0 MPa.

`[CLASP-101]` Proof pressure **shall** be 1.5 × MEOP (3.0 MPa).

`[CLASP-102]` Burst pressure **shall** be 2.5 × MEOP (5.0 MPa), verified per ANSI/AIAA S-080.

---

## 7. Dust mitigation

Lunar regolith is unweathered, electrostatically charged, and mechanically abrasive, with a median grain size near 70 μm and a significant sub-10 μm fraction. Apollo experience with seizing connectors and abraded seals is the governing precedent. No single mitigation is sufficient.

*(Editorial note for Revision B: Tier applicability of §7.1 and §7.2 is open. Placing an inductively powered 1 kV EDS array on CLASP-M conflicts with the "dumb metal" / CAD 2,000 target, requiring a gaseous oxygen purge on every active arm penalises purely mechanical handlers, and energising 1 kV electrodes in a GOX purge presents a discharge/ignition hazard; see §12.)*

`[CLASP-110]` Three independent dust mitigations **shall** be provided.

### 7.1 Electrodynamic dust shield

`[CLASP-111]` The passive port sealing face **shall** incorporate an interdigitated electrode array capable of generating a travelling electrostatic wave.

`[CLASP-112]` The array **shall** be driven at 1.0 kV peak-to-peak minimum, 4-phase, 10 Hz nominal, drawing ≤ 5 W.

`[CLASP-113]` Drive power **shall** be supplied inductively from the active side. The passive port **shall** require no stored or wired power.

`[CLASP-114]` The system **shall** demonstrate ≥ 90% removal of particles ≥ 10 μm from the sealing face *(measurement basis—mass, area, or count—and initial dust loading TBD in Revision B)*.

### 7.2 Purge

`[CLASP-115]` The active side **shall** deliver a gaseous oxygen purge pulse across the mating face immediately before contact.

`[CLASP-116]` Purge consumption **shall not** exceed 0.5 g per mating cycle, delivered over 0.5 s.

**Rationale.** Boiloff gas is available at any depot and would otherwise be vented. Purge is effectively free.

### 7.3 Mechanical exclusion

`[CLASP-117]` Lead-in chamfers **shall** be configured to mechanically wipe the sealing surfaces during the mating stroke.

`[CLASP-118]` The port **shall** accept a passive cover, removable and replaceable by the active end effector.

`[CLASP-119]` Where payload geometry permits, ports **should** be oriented face-down or vertical so that settling and levitating dust does not accumulate on the sealing face.

---

## 8. Power, data, and identification (CLASP-FP)

### 8.1 Power

`[CLASP-120]` Power transfer **shall** be inductive. No exposed electrical contacts are permitted in the primary power path.

`[CLASP-121]` The interface **shall** deliver ≥ 500 W at ≥ 85% end-to-end efficiency.

`[CLASP-122]` A single hardwired redundant discrete pair **shall** be provided for safe/arm and inhibit functions. *(Open item: reconcile physical contacts with [CLASP-120], define connector/pinout/dust wiping, and address inhibit coverage for CLASP-F; see §12.)*

**Rationale.** Contactless power eliminates pin damage, contact abrasion, and dust bridging, at a cost of roughly 10–15% efficiency. Safety-critical inhibits are the sole exception: they do not ride on a link that can be optically obscured.

### 8.2 Data

`[CLASP-130]` Data transfer **shall** be by short-range optical link, ≥ 10 Mbit/s full duplex.

`[CLASP-131]` The link **shall** operate across the full hard-latched alignment tolerance *(tolerance values TBD in §5.3)*.

`[CLASP-132]` A defined low-rate fallback **shall** be available over the discrete pair of `[CLASP-122]` *(bit rate, protocol, and safety isolation from inhibit circuits TBD in Revision B)*.

### 8.3 Passive identification

`[CLASP-140]` Every port, all tiers, **shall** carry passive retroreflective fiducials in an asymmetric pattern sufficient for six-degree-of-freedom pose estimation.

`[CLASP-141]` Every port **shall** carry a machine-readable passive tag encoding, at minimum: port tier, unique serial, payload contents identifier, fill state, and accumulated cycle count. *(Open item: separate static identity from active-written dynamic fields—fill state and cycle count—and specify the RF/optical tag protocol; see §12.)*

`[CLASP-142]` Fiducials and tag **shall** be readable with no power on the passive side.

**Rationale.** An unpowered tank parked in permanent shadow for two years must remain fully addressable by an arriving manipulator. Everything required to identify, locate, and grasp a payload is passive by design.

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
| 7 | Reference active end effector interface control document |
| 8 | Cost model validating the CAD 2,000 recurring target for CLASP-M |
| 9 | Position on liquid hydrogen for a future CLASP-FH tier |
| 10 | Cross-compatibility assessment against IDSS and any emerging cargo standards |
| 11 | Tier scoping of §7 dust mitigation: reconcile 1 kV EDS + GOX purge with CLASP-M "dumb metal" / CAD 2,000 target and evaluate 1 kV/GOX ignition hazard |
| 12 | Reconcile hardwired discrete safe/arm contacts (`[CLASP-122]`) with contactless philosophy (`[CLASP-120]`), data isolation (`[CLASP-132]`), and CLASP-F interlocks |
| 13 | Define how CLASP-F senses and reports ≤ 120 K chilldown state (`[CLASP-092]`) without power or data, plus bypass orifice demated sealing and chilldown boiloff venting |
| 14 | Specify passive tag standard/protocol and separate static identity from active-written dynamic state (`[CLASP-141]`) |
| 15 | Define soft-capture grapple geometry, hard-latch engagement features, payload keep-out zones, mounting datum offset, passive cover interface, and hard-latched alignment tolerances |
| 16 | Evaluate non-backdriving 15° ± 2° cam friction lock vs. positive mechanical latch under UHV/dust friction variations (`[CLASP-022]`) and review 2.0 kN·m bending margin (`[CLASP-030]`) |
| 17 | Add launch vibration/shock, 30–50 K PSR survival, radiation/UV dose, UHV anti-galling/cold-welding, and ESD electrical bonding requirements (§9) |

---

## Annex A — Verification cross-reference matrix

*To be issued with Revision B.*

## Annex B — Reference active end effector ICD

*To be issued with Revision B.*

---

*End of CLASP-STD-001 Revision A working draft. Comments to the issuing body.*
