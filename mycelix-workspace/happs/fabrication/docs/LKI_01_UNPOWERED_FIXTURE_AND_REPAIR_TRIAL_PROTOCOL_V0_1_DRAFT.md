# LKI-01 Unpowered Fixture and Repair-Trial Protocol v0.1

**Status:** Proposed test plan; no fixture fabricated or inspected  
**Configuration:** Candidate-independent, unpowered surrogate only  
**Companion artifacts:** [Repair-trial record schema](./LKI_01_REPAIR_TRIAL_RECORD_SCHEMA_V0_1_DRAFT.json), [illustrative planned record](./LKI_01_REPAIR_TRIAL_RECORD_EXAMPLE_V0_1_DRAFT.json), [v0.4 repairability benchmark extension](./LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json)

## 1. Purpose and claim ceiling

The next physical engineering increment is an adjustable, full-scale **unpowered mockup** that tests geometry, reach/access, opening/closing, surrogate-module insertion/removal, handling and the documented service path. It is not a functional appliance, load-rating certificate, thermal model, electrical test, food-safety test or proof of long-term reliability.

The first mockup must contain no energized appliance, mains connection, refrigerant/process fluid, heating element or live appliance electronics. Use inert dimensional surrogates only. Any ballast must be secured, its placement and mass documented, and its load cases reviewed by a competent structural engineer before use. Do not use unreviewed heavy loads to “see if it holds.”

## 2. Paradigm shift: prototype the service lifecycle before the appliance

Build a fixture that can be reconfigured as appliance candidates change, rather than making a finished cabinet around an unverified model. Separate four envelopes:

1. **Physical envelope:** exact external dimensions of the surrogate/module, with source and revision.
2. **Installation envelope:** manufacturer-required cavity, clearances, brackets, ducts and utility/service holes for one identified model.
3. **Operational envelope:** drawer/door travel and any panel, handle or adjacent-surface interference.
4. **Removal/service envelope:** the swept path and technician access needed to remove the module without destructive dismantling.

A rectangular box fitting inside another box checks only the first envelope. It does not show that the real drawer opens, a fastener is reachable, the module can be lifted or slid out, or a required exhaust path is preserved.

## 3. Dimension control and fixture definition

### 3.1 Baseline datums

The controlled drawing must declare units and a Cartesian datum system before fabrication. Suggested convention:

- **Datum A:** level reference plane supporting the fixture.
- **Datum B:** front face plane of the principal cabinet frame.
- **Datum C:** left interior reference plane.
- Record every dimension as a measured value, a controlled-source value, a target or an unknown. Never mix them into one unlabeled CAD parameter.
- Record measuring instrument ID, calibration/check status, resolution if known, operator, date/time and measurement uncertainty or tolerance source where applicable.
- Photograph the measurement points and retain the drawing revision and SHA-256 digest of the source drawing/report.

The 1800 × 900 × 900 mm example envelope in the illustrative JSON is a **concept target only**, not a decided product size and not a fit claim. Do not cut final material from it.

### 3.2 Adjustable surrogate bay

Use adjustable rails/panels or replaceable spacers to accommodate candidate envelopes. The fixture drawing should separately dimension:
- external mockup envelope;
- internal clear bay after rails, fasteners, runners and liners;
- datum locations and mounting features;
- module surrogate, mass and centre-of-gravity representation if known;
- drawer/door swept volume;
- install/remove swept volume;
- user and technician access zones;
- ventilation/duct and utility routes as *reserved zones* only when derived from the exact candidate's controlled documents.

Do not infer a universal refrigerator opening. Candidate screening previously found that Fisher & Paykel's RB90S64MKIW1 planning guide lists an 855 mm chassis width and 864 mm minimum inside cabinetry width, with model-specific installation requirements. Those values are a reference example only, not a selection or a valid dimension set for this generic fixture. The associated installation instructions require service removability and accessible electrical isolation. Confirm the current model-specific manual and local market support before basing any build on it.

## 4. Controlled stages and hard stops

| Stage | Work | Gate / retained evidence |
|---|---|---|
| F0 — design freeze for mockup | Set revision IDs, datums, task list, exact surrogate envelope, material and fabrication drawing | Design review signed; source/target/unknown fields explicit |
| F1 — fabricated, not yet inspected | Build inert fixture and dummy module only | Fabrication record; dimensional inspection pending; no participant trial yet |
| F2 — dimensional inspection | Measure external envelope, bay clear envelope, diagonals/squareness where relevant, mounting datums and access openings | Measurement sheet with instrument/operator/revision and discrepancies logged |
| F3 — static handling/access screening | With no unreviewed load, inspect access and move the unloaded or reviewed-mass surrogate along the declared path | Stop on instability, sharp-edge/pinch hazard, unplanned interference or need for destructive disassembly |
| F4 — repeatable removal task | Trained personnel execute a written removal/reinstallation procedure on a surrogate; preserve failed attempts and damage | Trial records, video/photos where appropriate, tools, separate timing phases, post-check |
| F5 — design iteration | Modify one controlled feature, increment revision and rerun affected checks | Change record links old/new geometry, reason, risks and regression tasks |
| F6 — candidate-specific review | Only after exact candidate and current installation documents are identified, evaluate candidate installation/service constraints | Engineering review of compatibility; no energizing authorized by this protocol |

**Immediate stop conditions:** any appliance becomes energized or connected to mains; refrigerant/process fluid or a heating element is introduced; ballast is loose; the fixture tips or shifts unexpectedly; a structural load is outside its reviewed case; a pinch/sharp-edge hazard emerges; the test requires an unqualified hazardous procedure; or a dimension/source/revision mismatch is discovered. Record a stop as an outcome, not as a missing or successful attempt.

## 5. Trial design and data capture

Use one [repair-trial record](./LKI_01_REPAIR_TRIAL_RECORD_SCHEMA_V0_1_DRAFT.json) per attempt. It is deliberately revision-bound and uses explicit time-phase states. An absent value must not silently become zero.

Every performed attempt should identify:
- product/BOM/geometry/interface revisions and fixture drawing revision;
- task and procedure revision, task start condition and stop condition;
- role and relevant training/familiarity (use pseudonymous participant tokens, no names);
- tools, surrogate mass/configuration, dimensional basis and environmental conditions as needed;
- outcome: success, failure, stopped for safety or incomplete;
- any permanent mockup damage, interference, unplanned steps, failed attempts or rework;
- diagnosis, hands-on labour, parts/tool wait, verification, rework and total downtime as separate measured/not-applicable/not-recorded/not-started phases;
- post-check status, limitations and source-evidence digests.

For the initial formative engineering pilot, a practical starting plan is 3 trained operators × 3 repeated attempts per defined module-removal task, where access to operators permits. This is for finding geometry/process defects and repeatability problems, not a universal statistically powered sample size. Repetitions by the same operator are not independent participants. Before a confirmatory comparison, pre-register the baseline, primary metric, repeat structure, sample-size/power-or-precision rationale, task order/counterbalancing, exclusions, failure handling and analysis.

Human-facing studies should begin only after the mockup has passed inspection and the task is non-hazardous. Do not recruit intended users to perform appliance opening, electrical diagnosis, refrigerant work or other tasks intended for qualified technicians.

## 6. Initial acceptance checks and benchmark targets

These are proposed engineering acceptance targets, not completed measurements.

| Check | Proposed target | Fail-closed rule |
|---|---|---|
| Dimensional traceability | 100% of acceptance-critical dimensions carry a source/basis, unit, revision and status | Any unknown critical dimension blocks the fit claim |
| Surrogate fit | 100% of measured envelope + stated allowances fits the candidate-independent test bay for the declared test configuration | Any axis exceeds clear bay: fail; do not round the failure away |
| Motion and removal | 100% of planned insert/remove path completes without unplanned disassembly or permanent mockup damage in the declared task | Any unresolved interference, safety stop or damage blocks the serviceability gate |
| Repeatability pilot | 3 × 3 attempts where practical; retain all attempts and report observed range and failures | Missing runs are not successes; any design change requires revision and impact review |
| Time accounting | Every trial has an explicit state for all six phases | Missing phase cannot be reported as zero or used in a complete burden claim |
| Post-check | Every successful surrogate replacement has a declared post-check and result | No post-check: attempt not counted as verified restoration |
| Evidence | 100% of evidence used for a measured claim links to a file/source digest and exact revision scope | Filename/URL or status label alone is not provenance |
| Safety boundary | Zero energized or hazardous systems in F0–F5 | Any violation stops the session; this protocol does not authorize live testing |

The “3 × 3” pilot is a formative starting point, not a statistical certification. Do not set a physical pass rate of 100% by dropping difficult attempts; the target is complete observability and compliance with the predeclared hard gates.

## 7. Design implications for LKI-01

- Keep the appliance choice open until written target-market availability, the exact current installation manual, local service channel and parts terms are established.
- Preserve manufacturer-mandated removal paths, ventilation/duct routes and accessible isolation. A clean visual design cannot override installation constraints.
- Separate refrigerator and cooking-module interfaces and risk reviews. This fixture must not be used to authorize a combined energized build.
- Prefer reversible fasteners and replaceable panels/runners/wear parts, but demonstrate their access and service choreography on the mockup rather than assuming the CAD intent is achievable.
- Use the Mycelix product/BOM and repair/restoration owners as the authoritative record owners; the trial JSON is an evidence capture contract, not a competing parts, inventory or work-order system.
- Symtropy may check geometric motion and surrogate paths; these checks do not replace structural FEA, airflow/thermal analysis, electrical safety testing or qualified appliance assessment.

## 8. Standards and manufacturer research anchors

- [ISO/TS 10303-4439:2025, Product life cycle support](https://www.iso.org/standard/91589.html) describes product assemblies, through-life configuration changes, support tasks, resource/skill/location information and product activity/configuration history. The proposed Mycelix record is a narrow working contract, not a claim of ISO 10303 conformance.
- [ISO 10303-239:2024, Product life cycle support](https://committee.iso.org/standard/78832.html) is a related product-life-cycle data model reference. Treat it as a future interoperability crosswalk opportunity, not a reason to invent a second system of record.
- [IEC 60335-1:2020+AMD1:2025](https://webstore.iec.ch/en/publication/61880) is the current consolidated general household-appliance safety publication surfaced by IEC. The relevant Part 2 and local adoption/applicability must be identified for each appliance; this protocol is not a conformity assessment.
- [ISO 14040:2006](https://www.iso.org/standard/37456.html), confirmed current with Amendment 1, defines the LCA principles/framework including goal/scope, inventory, impact assessment, interpretation, reporting and review. Do not claim whole-life environmental superiority from material choice or component replacement alone.
- [Fisher & Paykel RB36S CoolDrawer installation guide](https://www.fisherpaykel.com/on/demandware.static/-/Sites-fpa-master-catalog/default/dw4568cc40/InstallationManuals-FisherPaykelCA/FP-InstallGuide-en-RB36S-CoolDrawer-0-866529A-US-CA.pdf) recommends accessible electrical connection outside the cavity and requires the supplied bracket/fittings to withstand the model's declared 100 kg load case. This is a model-specific installation instruction, not a generic design load for LKI-01.

Sources are research anchors. Obtain the exact current model, installation guide, locally applicable requirements and competent review before candidate-specific design or any energized test.

## 9. Completion state

- Repair-trial JSON schema: drafted.
- Example record: planned-only fixture trial; not evidence of a performed test.
- Fixture plan: drafted.
- Physical fixture fabricated: no.
- Dimensions measured: no.
- Candidate appliance selected/locally qualified: no.
- Appliance safety or conformity established: no.
- Long-life performance demonstrated: no.
