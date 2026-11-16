# CLASP-STD-001 — Annex A: Verification Cross-Reference Matrix (Working Draft)

| | |
|---|---|
| **Parent Document** | [CLASP-STD-001_RevA_Working_Draft.md](CLASP-STD-001_RevA_Working_Draft.md) |
| **Revision** | A (Working Draft Supplement) |
| **Date** | 2026-11-15 |
| **Status** | Draft VCRM for Revision B preparation |

**Verification Method Key (§10.1):** **T** = Test | **A** = Analysis | **I** = Inspection | **D** = Demonstration

---

## Verification Cross-Reference Matrix (62 Requirements)

| ID | Section | Requirement Summary | T | A | I | D | Verification Notes / Rev B Dependencies |
|---|---|---|---|---|---|---|---|
| `CLASP-001` | §4.1 | Three tiers defined (`CLASP-M`, `CLASP-F`, `CLASP-FP`) | | | X | | Design documentation and tier drawing audit |
| `CLASP-002` | §4.1 | Shared external footprint, bolt pattern, capture, and alignment geometry | | | X | | Drawing/CAD overlay comparison across all tiers |
| `CLASP-003` | §4.1 | Single active end effector captures any tier without reconfiguration | | | | X | Multi-tier mating demo with reference end effector |
| `CLASP-010` | §4.2 | 240 mm circular footprint, ≤ 55 mm protrusion above mounting plane | | | X | | Dimensional CMM / gauge inspection |
| `CLASP-011` | §4.2 | 8 × M8 on 210 mm BCD, 15° clocking offset | | | X | | Dimensional inspection and fastener fit gauge |
| `CLASP-012` | §4.2 | Passive port mass limits (3.5 kg / 6.0 kg / 7.5 kg) | | | X | | Calibrated scale measurement of flight-representative hardware |
| `CLASP-020` | §5.1 | Soft capture via external perimeter grapple across 6-DOF envelope | X | X | | | Dynamic 6-DOF capture rig across extreme corners (§10.2 #4); preliminary CAD funnel/KOZ (`Ø 486.6 mm` above `Z = 18 mm`) in `cad/clasp_a.py`, formal spec Open Item #15 |
| `CLASP-021` | §5.2 | Alignment via three 120° helical cam ramps during retraction | | | X | X | Geometry inspection + kinematic retraction demo |
| `CLASP-022` | §5.2 | Cam lead angle 15° ± 2° for alignment/retraction; positive latch retention | X | X | X | | Angle inspection; positive latch retention test under §5.3 loads (Open Item #15) |
| `CLASP-023` | §5.2 | One ramp geometrically distinct for unambiguous clocking | | | X | X | Inspection + intentional 120°/240° mis-clock rejection test |
| `CLASP-024` | §5.2 | Passive alignment geometry; no powered actuation on passive side | | | X | | Schematic and BOM audit |
| `CLASP-030` | §5.3 | Limit loads (10 kN axial, 5 kN shear, 2.0 kN·m bend, 1.0 kN·m torsion) | X | X | | | Static load frame test + discrete/clocked `Tet10` FEA (`fea/clasp_m_detailed_fea.py`, `fea/clasp_a_fea.py`); combined/stiffness/dowel spec TBD (Open Item #15/#16) |
| `CLASP-031` | §5.3 | Yield FoS ≥ 1.25 and ultimate FoS ≥ 1.4 on limit load | X | X | | | FEA stress report (`Al 7075-T6` port/jaws + `17-4PH H900` hooks verified `+0.51..+0.77` hot) + coupon/assembly load test to 1.4× limit |
| `CLASP-032` | §5.3 | Positive mechanical latch maintains preload without gapping | X | X | | | Displacement/gap sensor check under combined limit loading |
| `CLASP-040` | §5.4 | Confirmed 6-stage mating sequence | | | | X | Active end effector state-machine telemetry log |
| `CLASP-041` | §5.4 | Fluid poppets prohibited from opening before hard latch confirmation | X | X | | | Mechanical/software interlock fault-injection test |
| `CLASP-042` | §5.4 | Active side manual/unpowered mechanical release path | | | | X | Unpowered/depressurized emergency release demo |
| `CLASP-050` | §6.1 | Qualified for LOX at 90 K (primary service fluid) | X | | | | Cryogenic flow & thermal soak campaign (§10.2 #3) |
| `CLASP-051` | §6.1 | Qualified for LCH₄ at 112 K (secondary service fluid) *(should)* | X | X | | | Cryogenic compatibility & flow verification (recommended) |
| `CLASP-052` | §6.1 | LH₂ service not supported in Revision A | | | X | | Scope and ICD compliance audit |
| `CLASP-060` | §6.2 | Coaxial dual-poppet configuration, mutual displacement | | | X | X | Cross-section drawing audit + poppet stroke actuation check |
| `CLASP-061` | §6.2 | Nominal flow bore 25 mm | | | X | | Bore gauge inspection |
| `CLASP-062` | §6.2 | Design flow rate 2.0 kg/s LOX | X | | | | Cryogenic flow loop test measuring mass flow and ΔP |
| `CLASP-063` | §6.2 | Coupling centred on Z axis | | | X | | Concentricity inspection relative to port datum |
| `CLASP-070` | §6.3 | Trapped volume ≤ 5 cm³ | X | X | | | CAD volumetric analysis + demate capture/weigh test at BOL/EOL |
| `CLASP-071` | §6.3 | Mated external leakage ≤ 1 × 10⁻⁴ scc/s helium | X | | | | Helium mass-spectrometer leak test at ambient & cryo BOL/EOL |
| `CLASP-072` | §6.3 | Independent poppet seal ≤ 1 × 10⁻⁴ scc/s helium when demated | X | | | | Individual half helium leak test after mate/demate cycling |
| `CLASP-080` | §6.4 | Dynamic seats PCTFE or metallic C-seals | | | X | | Material certification & drawing inspection |
| `CLASP-081` | §6.4 | No elastomeric seals in cryogenic sealing functions | | | X | | BOM and wetted-part material audit |
| `CLASP-082` | §6.4 | Non-metallic materials meet ASTM E595 (TML ≤ 1.0%, CVCM ≤ 0.10%) | X | | X | | Material batch test reports per ASTM E595 |
| `CLASP-083` | §6.4 | LOX compatibility (NASA-STD-6001) & IEST-STD-CC1246 Level 100A | X | | X | | Batch LOX impact/compatibility certs + precision cleaning cert |
| `CLASP-090` | §6.5 | Integral 2.0 mm nominal bypass orifice for pilot chilldown | | | X | X | Orifice inspection + pilot flow test prior to main stroke |
| `CLASP-091` | §6.5 | Chilldown reduces receiving line/body to ≤ 120 K before main open | X | X | | | Cryo chilldown thermal test and transient thermal model |
| `CLASP-092` | §6.5 | Active-side verification of ≤ 120 K chilldown prior to main open | X | | | X | Contact thermometry + bypass decay test on sensor-free `CLASP-F` (Open Item #13) |
| `CLASP-100` | §6.6 | MEOP = 2.0 MPa | X | X | | | Functional pressure test at 2.0 MPa (cryo and ambient) |
| `CLASP-101` | §6.6 | Proof pressure = 1.5 × MEOP (3.0 MPa) | X | | | | Hydrostatic/pneumatic proof test without permanent set |
| `CLASP-102` | §6.6 | Burst pressure = 2.5 × MEOP (5.0 MPa) per ANSI/AIAA S-080 | X | X | | | Qualification unit burst test and stress analysis |
| `CLASP-110` | §7 | Tiered dust mitigations: mechanical (§7.3 all), EDS + purge (§7.1–§7.2 `F`/`FP`) | | | X | | System design and tier compliance audit |
| `CLASP-111` | §7.1 | Interdigitated EDS electrode array on `CLASP-F`/`-FP` sealing face | | | X | | Fabrication drawing and visual/electrical inspection (Open Item #11) |
| `CLASP-112` | §7.1 | EDS ≥ 1.0 kV p-p, 4-phase, 10 Hz, ≤ 5 W; de-energised before purge/fluid | X | | | | High-voltage waveform, power draw, and de-energisation interlock test |
| `CLASP-113` | §7.1 | EDS power inductive from active side; no passive stored/wired power | X | | X | | Schematic audit + unpowered passive port activation test |
| `CLASP-114` | §7.1 | ≥ 90% removal of particles ≥ 10 μm from sealing face | X | | | | Simulant dust-clearing test (metric basis TBD — Open Item #11) |
| `CLASP-115` | §7.2 | Inert or service-matched dry gas purge on `CLASP-F`/`-FP` after EDS off | | | | X | Purge valve timing, gas compatibility, and EDS-off interlock demo |
| `CLASP-116` | §7.2 | Purge consumption ≤ 0.5 g over 0.5 s per cycle | X | | | | Mass-flow / pressure-decay calibration test |
| `CLASP-117` | §7.3 | Lead-in chamfers wipe sealing surfaces during mating stroke | | | X | X | CAD contact interference inspection + dust-wipe demo |
| `CLASP-118` | §7.3 | Passive dust cover removable/replaceable by active end effector | | | | X | Robotic remove/stow/replace demonstration |
| `CLASP-119` | §7.3 | Face-down or vertical port orientation where geometry permits *(should)* | | | X | | Payload integration drawing review |
| `CLASP-120` | §8.1 | Inductive power transfer; exposed electrical contacts prohibited on all tiers | | | X | X | Hardware inspection + contactless power transfer test |
| `CLASP-121` | §8.1 | ≥ 500 W delivered at ≥ 85% end-to-end efficiency | X | | | | Loaded power efficiency test under dust & alignment extremes (Open Item #5) |
| `CLASP-122` | §8.1 | Non-contact safe/arm & inhibits (mechanical fluid + magnetic/Hall power) | X | | X | | Fault-injection and latch-seat inhibit test (Open Item #12) |
| `CLASP-130` | §8.2 | Short-range optical link ≥ 10 Mbit/s full duplex | X | | | | Bit-error-rate & throughput test over optical link |
| `CLASP-131` | §8.2 | Optical link operates across full hard-latched alignment tolerance | X | | | | Optical link margin test at worst-case hard-latched offsets |
| `CLASP-140` | §8.3 | Asymmetric retroreflective fiducials on all tiers for 6-DOF pose | X | X | X | | Pose estimation test under low-angle lunar lighting simulation (`cad/clasp_m.py` 5-boss geometry & `perception/` benchmark) |
| `CLASP-141` | §8.3 | Static optical DataMatrix tag (tier, serial); dynamic state in fleet ledger | | | X | X | Optical read test (`18 × 18 mm` pad) under polar lighting + `ledger/` C++20 SQLite sync check (Open Item #14) |
| `CLASP-142` | §8.3 | Fiducials and tag readable with zero passive-side power | | | | X | Cold/unpowered target acquisition & read demonstration |
| `CLASP-150` | §9 | Survive/operate across thermal, vacuum, dust, 5-yr storage, cycle life | X | X | | | TVAC (500 cycles), dust campaign, radiation/launch analyses (Open Item #17) |
| `CLASP-151` | §9 | Differential CTE does not compromise seal integrity or latch preload | X | X | | | Mated dissimilar-material cryo/hot thermal test + coupled `90 K / +120 °C` thermoelastic FEA (`fea/clasp_thermoelastic_fea.py`, `Ti-6Al-4V` tangential flexure spokes `MoS_y >= +3.28`) |
| `CLASP-160` | §10.2 | Full 6-part qualification campaign completed | X | | | X | Witnessed test reports covering §10.2 items 1–6 |
| `CLASP-161` | §10.2 | Testing performed at CLASP-accredited conformance facility | | | X | | Facility accreditation certificate verification |
| `CLASP-162` | §10.2 | Certified ports marked and entered in public registry | | | X | | Marking inspection and registry record audit |
| `CLASP-170` | §11.3 | Governing body membership, open comment, and change control | | | X | | Governance charter audit (Open Item #1) |
| `CLASP-171` | §11.3 | Backward compatibility of passive port geometry maintained | | | X | | Revision-over-revision ICD compatibility audit |

