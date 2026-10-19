# Lifetime Product Engineering Contract

**Status:** Proposed engineering contract  
**First demonstrator:** LKI-01 Lifetime Kitchen Island  
**Product family:** Modular, repairable household furniture and appliances  
**Owner:** Luminous Dynamics  
**Last researched:** 2026-10-18

## 1. Purpose

This contract defines how a long-lived physical product moves from requirements to design, simulation, sourcing, assembly, validation, repair, and later revisions. It is an engineering interface between product design, fabrication, manufacturing, supply-chain provenance, and evidence systems.

The central proposition is that **the durable structure should outlive replaceable technology**. The product is not one inseparable object; it is a versioned system of permanent structure, serviceable modules, wear parts, and documented interfaces.

This document is a design baseline, not a certification, a completed CAD package, evidence of supplier availability, or proof that a 50-year life has already been achieved.

## 2. Paradigm shifts

### 2.1 From product lifetime to structure/module lifetime

Traditional integrated cabinetry is often designed around an appliance's exact envelope. A replacement can trigger expensive cabinet or countertop work. Instead, define explicit replaceable-module envelopes, mounting interfaces, service access, thermal boundaries, utility isolation, and validated compatibility. The cabinet remains; a failed or obsolete module can be replaced.

### 2.2 From sealed integration to reversible integration

Integration should mean coordinated fit, usability, safety, and appearance—not permanent bonding. Prefer reversible fasteners and replaceable subassemblies. Adhesives and permanent joints remain appropriate where technically required, but should not block predictable service.

### 2.3 From a single optimum to a constrained Pareto frontier

Do not optimise only price, appearance, or energy. Compare designs across purchase cost, lifetime cost, energy, repair time, material impact, ergonomics, local manufacturability, noise, and resilience. Safety and legal constraints are gates, not weighted preferences: no score can compensate for a failed safety requirement.

### 2.4 From claimed durability to a falsifiable reliability case

A 50-year design target cannot be proven by a short simulation or a single prototype. Build a case from material and joint selection, component service histories, fatigue and cycling tests, accelerated ageing with stated limits, supplier continuity, replaceability, field feedback, and uncertainty. Separate observed results from extrapolations.

### 2.5 From a static bill of materials to a living product passport

Every released product revision should resolve to a versioned BOM, approved materials, supplier and batch evidence where available, assembly instructions, inspection results, appliance/module identities, service procedures, compatible replacements, and known limitations. Sensitive supplier pricing and personal service data should not be public by default; provenance can be verifiable without publishing every commercial detail.

### 2.6 From unit-sale economics to lifetime service economics

Evaluate total cost over 20–30 years, including energy, maintenance, repairs, parts, transport, and the cost of replacing a whole product when a module fails. The business model should reward repair and continued service, not create avoidable dependency on proprietary consumables or cloud services.

### 2.7 From one global design to local production with a stable interface

Keep critical interface dimensions, safety requirements, and performance tests stable while allowing validated local material and supplier substitutions. A local cabinetmaker may produce the frame while a certified appliance supplier provides the cold module. Substitutions must pass equivalence checks; geographic availability alone is not proof of quality.

## 3. Reference product: LKI-01

LKI-01 is a modular kitchen island with:
- a permanent load-bearing furniture structure;
- a replaceable worktop and replaceable wear surfaces;
- one or more certified refrigeration or freezer modules;
- an optional, separately certified induction cooking module;
- accessible service routes, ventilation and utility isolation;
- documented fasteners, access envelopes, maintenance clearances, and assembly sequence;
- a product passport, BOM, parts list, and repair history.

The first prototype should use proven certified appliance modules. Do not invent a compressor circuit, refrigerant system, mains power electronics, or induction assembly as part of the first cabinet prototype.

The refrigeration and cooking modules must be thermally separated and independently ventilated according to their manufacturers' instructions. The enclosure must not obstruct airflow, trap heat, compromise flammable-refrigerant safety, or create an unsafe spill/electrical path. If those requirements cannot be met, remove the cooking feature from the first prototype rather than relaxing the safety envelope.

## 4. Canonical product data contract

The product definition should be represented in a machine-readable, versioned schema. Each field must distinguish a target, a measured result, an externally sourced fact, and an unverified assumption.

### Product identity

- Product ID: stable identity across revisions.
- Revision: immutable released revision identifier.
- Lifecycle state: concept, prototype, under-test, released, superseded, or withdrawn.
- Requirements revision: exact requirement set used for the design review.
- Geometry revision: exact CAD/geometry revision and units.
- BOM revision: exact material and component list.
- Evidence manifest: links or hashes for test results, calculations, supplier evidence, and approvals.
- Change record: what changed, why, affected interfaces, risk review, and required regression tests.

### Component identity

Each component should record part ID, revision, quantity, unit, material/specification, critical dimensions and tolerance, mass if known, finish, expected environment, supplier candidates, substitution rules, service class, failure modes, replacement instructions, safety classification, and evidence status.

For purchased components, record manufacturer/model, exact datasheet revision, declared certification or conformity evidence, purchase/source URL, date checked, quote validity and lead time where known. Never represent a search result or an old quote as current inventory.

### Interface contract

Every replaceable module must declare:
- mechanical envelope and datum scheme;
- mounting points, fasteners, torque requirements where specified, and load path;
- required installation/removal direction and service access envelope;
- ventilation and thermal boundary conditions;
- electrical, plumbing, drainage, and control connections if applicable;
- isolation procedure and safe removal state;
- environmental sealing and spill boundaries;
- acceptable mass and centre-of-gravity envelope;
- supported replacement modules and evidence for compatibility.

Interface compatibility is not merely matching dimensions. It includes thermal performance, electrical ratings, ventilation, safety, control behaviour, stability, service access, and manufacturer's conditions of use.

### Draft machine-readable artifacts

- [LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json](./LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json) defines a proposed aggregate product record, component/supplier evidence snapshots, module interfaces, metrics, release gates, and evidence references.
- [LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json](./LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json) is an illustrative LKI-01 record with three placeholder BOM entries, two provisional interfaces, and five unpassed release gates.
- [LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_1_DRAFT.json](./LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_1_DRAFT.json) defines 14 adversarial known-answer scenarios, nine hard invariants, and five proposed software-evaluator metrics.
- [LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_2_DRAFT.json](./LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_2_DRAFT.json) preserves v0.1 and adds a module-envelope/required-fit-clearance negative control, for 15 known-answer cases.
- [LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json](./LKI_01_LIFETIME_BENCHMARK_CORPUS_V0_3_DRAFT.json) adds two evidence-backed interface verification cases: status strings with missing evidence must block; measured interface claims require resolvable, digest-bound source and test records.
- [LIFETIME_PRODUCT_OWNER_CROSSWALK_DRAFT.md](./LIFETIME_PRODUCT_OWNER_CROSSWALK_DRAFT.md) maps draft fields to existing Mycelix Manufacturing/Supply Chain/product identity/repair owners, plus Symtropy, Symthaea and Sol-Atlas issue owners.
- [tools/lifetime-product-audit/README.md](../tools/lifetime-product-audit/README.md) documents a standalone Rust audit tool; [src/main.rs](../tools/lifetime-product-audit/src/main.rs) includes schema validation, draft-specific semantic checks, scenario-oracle checks and unit tests.

All artifacts are drafts, not canonical wire contracts or approved manufacturing data. The example has no supplier records or physical-test evidence; dimensions and performance targets are provisional. The BOM and supplier fields are derived snapshots for offline audit, not replacements for Mycelix Manufacturing or Supply Chain. The Rust tool and its 12 authored unit tests have **not been compiled or executed in the authoring environment**; run it on a pinned toolchain and commit a generated lockfile before treating its source as verified. A passing audit cannot certify or qualify a physical product.

## 5. Acceptance targets for the first design programme

Targets below are proposed programme goals; they are not measured results or universal standards. Each target must have an owner, test method, sample count, conditions, acceptance rule, and retained evidence before it can become a release gate.

| Measure | Proposed target | Evidence required |
|---|---|---|
| Structure design horizon | 50-year design intent | Material/joint rationale, replaceable-wear strategy, lifecycle risk review; not a claim of 50-year demonstrated life |
| Major module replacement | No destructive disassembly of permanent structure | Timed technician trial, photos/video, tool list, post-repair inspection |
| Planned module service time | Under 60 minutes for defined major-module replacement | Repeated timed trials by trained technicians; report median, range, and failed attempts |
| Routine access | Maintenance parts accessible without removing unrelated permanent structure | Serviceability walkthrough and access-envelope check |
| Spare-parts commitment | Aim for 20 years after last sale, with compatible successor strategy | Written supplier/internal parts policy; distinguish commitment from supplier guarantee |
| Cabinet wear parts | Replaceable independently | Demonstrated drawer-runner, seal, hinge, foot, finish, and worktop repair trials as applicable |
| Energy | Beat a size/capacity/use-matched certified baseline by 15% if a defensible comparison is possible | Standardised test conditions, calibrated measurement, baseline datasheet and uncertainty |
| Cooling | Maintain required food-storage temperatures within the selected appliance's declared operating conditions | Instrumented chamber/load/door-cycle/recovery tests; independent review |
| Noise | No worse than the matched baseline at defined seating/room positions | Repeatable sound-pressure test with operating states and room conditions recorded |
| Structural performance | Meet a documented static, cyclic, stability, and foreseeable-misuse load case | Calculations, fixture definition, measured deflection, pass/fail criteria, residual damage check |
| Repairability | Common tools and documented non-destructive replacement for designated service parts | Independent repair trial and published instructions |
| Lifetime cost | Lower 20–30-year total cost than a comparable conventional alternative | Transparent assumptions, energy scenarios, part prices, repair labour, replacement schedule and sensitivity analysis |
| Environmental impact | Compare lifecycle impacts rather than claiming sustainability from material choice alone | Goal/scope, inventory, impact method, and limitations consistent with LCA principles |

Do not claim a target is met from a simulation alone. Every metric has an explicit evidence status: **proposed**, **analytically estimated**, **simulated**, **bench-tested**, **independently verified**, or **field-observed**.

## 6. Hard gates and release rules

The following are non-compensable gates:
1. Applicable safety requirements and manufacturer installation conditions are identified and reviewed.
2. No known critical electrical, thermal, fire, stability, refrigerant, food-storage, or entrapment issue remains unresolved.
3. Released dimensions, tolerances, materials, BOM, assembly process, and service instructions agree with the tested build.
4. Material or supplier substitutions that affect safety, thermal behaviour, structure, or conformity trigger revalidation.
5. Claims are supported by evidence tied to the exact product revision; a queued CI job or a test of a different revision is not a pass.
6. The product can be safely isolated and serviced by the intended qualified person.
7. Remaining hazards, assumptions, and unverified claims are documented and accepted by the appropriate responsible reviewer.

The gate owner must record pass/fail/not-tested. “Not tested” is never silently converted into “pass.” Marketing language must be no stronger than the actual evidence.

## 7. Test architecture: simulation plus physical evidence

### Digital checks

- Geometry: units, dimensions, tolerance envelope, interference, access, drawer/door motion.
- Assembly: tool access, fastener sequence, module removal path, and dependency ordering.
- BOM: positive quantities, valid units, resolved subassemblies, no cycles, declared optional components, and revision consistency.
- Interfaces: candidate module must fit the mechanical, thermal, utility, service, and safety contract.
- Lifecycle: each expected wear/failure mode maps to an inspection or replacement procedure.
- Reproducibility: test fixture, test input, program version, data, and product revision are retained and hash-linked.

Symtropy may support motion and rigid-body interaction checks. It is not a finite-element stress solver or computational fluid dynamics solver by virtue of being a physics engine. Use suitable CAD/FEA/thermal methods and measurements where the question requires them.

### Physical tests

Create fixtures and procedures for static load, cyclic loading of drawers/doors, fastener retention, tipping/stability, spill containment where relevant, cleaning chemicals, humidity/temperature exposure, finish wear, transport vibration, module replacement, noise, and energy consumption. Test scope and severity must follow the use case and competent engineering review.

For refrigeration, test the integrated enclosure—not only the appliance alone—because surrounding cabinetry can alter intake/exhaust airflow, heat rejection, condensation, and access. Use an independent lab/qualified assessor for applicable electrical and appliance safety verification before any consumer sale.

Accelerated ageing can identify weaknesses but cannot automatically be converted into a 50-year life claim. The acceleration model and its limits must be stated.

## 8. Responsibilities across the Luminous Dynamics engineering stack

### Mycelix Fabrication and Manufacturing

Own product identity, revision history, canonical BOM, subassembly expansion, design files, repair manifest, approved supplier evidence, manufacturing orders, inspection evidence, change records, and product passport. Extend existing Fabrication and BOM concepts instead of creating competing schemas.

### Symtropy

Own the deterministic rigid-body digital-twin slice: cabinet geometry imported or represented in a defined coordinate system, moving parts, joint limits, collision/access checks, module envelopes, and repeatable regression scenarios. Explicitly label what is outside its validated solver capabilities.

### Symthaea

Own decision support and experiment planning: candidate generation, requirement-aware multi-objective search, scenario selection, uncertainty tracking, anomaly triage, and explanation of trade-offs. A hard safety constraint must stay outside any weighted objective and be independently checked.

### Sol-Atlas

Own sourcing and service-network views: candidate suppliers, manufacturing/repair capability, material origin where verified, shipping routes, lead times, grid/service context, and alternative sourcing. Every time-sensitive assertion must have source, date checked, geographic scope, confidence, and expiration/re-check policy.

### Human engineers and qualified assessors

Own engineering approval, choice of applicable standards, safety analysis, supplier qualification, physical test design, interpretation of test results, conformity assessment, and release decisions. Automation supports these responsibilities but does not replace them.

## 8A. Repairability qualification protocol

Repairability is a measured workflow outcome, not a design adjective or a single score. Use the existing Symtropy repair-task graph owner (including [symtropy#1773](https://github.com/Luminous-Dynamics/symtropy/issues/1773) and its draft parent work) for procedure/task authority, and Mycelix's existing [ProductiveLoop repair/restoration owner #3313](https://github.com/Luminous-Dynamics/mycelix/issues/3313) and [validator #3318](https://github.com/Luminous-Dynamics/mycelix/issues/3318) for the repair-outcome evidence composition. This section owns only the benchmark protocol; it does not define a competing repair ledger or task-graph schema.

### Endpoints that must remain separate

For each pre-registered task, record with a monotonic clock where appropriate:
- **Diagnosis time:** task start until a supported fault class and next action are recorded; keep initial wrong hypotheses and additional diagnostic loops.
- **Hands-on labour:** active person-minutes, including tool setup and repeated assembly/disassembly.
- **Bay elapsed time:** arrival/acceptance through post-repair verification and release, including diagnosis and repair.
- **Parts/tool wait:** time blocked on unavailable tools, consumables, approved replacement parts or documentation.
- **Verification time:** time to execute the required post-repair test and document its exact profile, instrument/calibration, result and residual limitations.
- **Rework and failed attempts:** repeat work, damage, rejected parts, unsuccessful repairs and follow-up visits.
- **Total capability downtime:** from loss of the specified capability to verified restoration, including queueing/transport/parts delays whenever observable.
- **Restoration correctness:** whether the exact required function was restored under the declared test conditions.
- **Recurrence:** a separate, bounded follow-up outcome whose observation duration and operating exposure are specified before the trial.

Do not collapse these endpoints into one “repair time.” A quicker hands-on intervention can still produce worse outcomes if parts wait, post-checks or rework increase. Do not infer restored capability from a closed work order, a control command, a replacement part having been planned, or a single nominal reading.

### Baseline and candidate comparison

Freeze the exact physical unit/configuration, BOM revision, service-procedure/task-graph revision, known fault profile, tool and parts availability, technician qualification criteria, safety prerequisites, test instruments/calibration, and pass/fail oracle before collecting confirmatory observations. Compare the baseline and candidate at matched tasks and starting conditions. A change in procedure, geometry, substitute part or required safety check is a new configuration or declared deviation, not invisible test noise.

Begin with a feasibility pilot of **at least 3 qualified technicians × 3 repetitions per task where practical**, consistent with the existing Symtropy repairability protocol. Treat that pilot as instrumentation/protocol calibration, not proof of superiority. Plan confirmatory trials using **5 qualified technicians × 6 repetitions per baseline/candidate/task cell (30 observations per cell)** as an initial planning default, then refine sample size and analysis via a pre-registered power/precision analysis appropriate to technician, unit and task effects. These are research design defaults, not a claim that every task needs the same fixed number of trials or that 30 samples automatically provide statistical power.

Report per-task median and P90 for diagnosis time, hands-on labour, bay time and total downtime, plus spread/uncertainty, technician/unit effects, missing observations, exclusions, failures, rework, residual defects and recurrence status. Preserve raw observations and procedure/source rights. Never exclude a failed attempt from the denominator without a visible, justified rule. Never credit faster service if isolation, safety checks, compatibility, performance verification or workmanship requirements were skipped.

### Human-facing repairability

Technical disassembly scores alone are not enough. A 2025 observational study of 26 participants found misalignment between calculated repairability scores and user-perceived repairability for two small appliances; difficulty opening the product and identifying faults were salient barriers. This is evidence that user-facing access, instructions and fault legibility should be measured alongside professional service performance—not evidence for a universal numerical correction factor. See [The need for incorporating user perception into the repairability indices (2025)](https://doi.org/10.1016/j.spc.2025.05.024).

For consumer-facing tasks, test documentation comprehension, correct identification of user-serviceable parts, safe stop/escalation behaviour and task completion with representative users. Keep user tasks separate from qualified-service tasks. Do not ask participants to perform mains, refrigerant, high-temperature or otherwise hazardous work simply to score usability; use simulations, de-energized specimens or qualified technicians as appropriate. The design research should also include physical disassembly on real specimens where safe and relevant; a 2025 household-appliance study applied EN 45554 alongside manual disassembly tests across five appliance categories. See [Design for repair: insights to develop functional modules in household appliances (2025)](https://doi.org/10.1016/j.procir.2024.12.004).

### Proposed repairability release gates

- Exact product unit, BOM, module, service-procedure and test-profile revisions are bound to every observation.
- Required safety isolation and post-repair tests are completed; missing prerequisites block the task.
- Parts planned and parts actually consumed/replaced remain distinct.
- The repair result is “restored,” “not restored,” “partially restored,” or “inconclusive” under the declared profile; it cannot mint certification, governance standing, compensation, or authority.
- Recurrence-free status cannot be asserted until the predeclared follow-up window and operating exposure have been completed.
- Baseline and candidate performance claims are withheld until the pre-registered analysis, exclusions and uncertainty have been reviewed.

## 9. Product changes and compatibility

Every design change must identify:
- changed part, interface, or requirement;
- reason and supporting failure, cost, usability, or performance evidence;
- downstream affected components and documents;
- safety and conformity impact;
- required regression and physical tests;
- compatibility with previous revisions;
- spare-part and service-instruction impact;
- reviewer and release decision.

Never silently change a part under an unchanged revision. Never label an alternative as drop-in compatible until the complete interface contract has been checked.

## 10. Research baseline and standards to review

This is a shortlist for competent review, not a declaration of compliance. Applicability depends on product configuration, market, installation, and current adopted national editions.

- **IEC 60335-1** — general safety requirements for household and similar electrical appliances. Check the current adopted edition and amendments for the destination market.
- **IEC 60335-2-24** — particular safety requirements for refrigerating appliances, ice-cream appliances, and ice makers; the IEC catalogue lists a 2025 edition.
- **EN 45554:2020** — generic methods to assess repair, reuse, and upgrade capability of energy-related products.
- **EN 45552** — general method for assessing durability of energy-related products; review alongside the repairability assessment.
- **IEC 62430:2019** — environmentally conscious design principles and process requirements.
- **ISO 14040:2006** — life-cycle assessment principles and framework. ISO reports it was reviewed and confirmed in 2022.
- **Destination-market building, electrical, fire, worktop, materials, and appliance installation requirements**, identified with qualified local professionals.

For ordinary household refrigerating appliances, Commission Regulation (EU) 2019/2019 is the relevant EU ecodesign starting point. Its listed spare-part periods are component-specific: at least seven years for thermostats, temperature sensors, printed circuit boards and light sources for professional repairers; at least seven years for handles, hinges, trays and baskets for repairers/end-users; and at least ten years for door gaskets. Listed parts must be replaceable with commonly available tools without permanent damage. This is not an eight-year universal guarantee for every component. Regulation (EU) 2019/2024 instead concerns refrigerating appliances with a direct-sales function, a different product scope. LKI-01's proposed 20-year parts-support target is an aspirational product-policy goal, not a current supplier commitment or general legal minimum.

As of 19 October 2026, Directive (EU) 2024/1799 is in force, with its repair rules applying from 31 July 2026. Articles 5 and 6 apply only to goods and to the extent that repairability requirements are provided by legal acts listed in Annex II; that Annex includes household washing machines/washer-dryers, household dishwashers and refrigerating appliances, among other groups. Exact duties depend on product scope, the applicable product-specific requirements and national implementation. The directive is an EU-market reference, not automatically a South African requirement and not a substitute for a jurisdiction-specific legal review.

## 11. First implementation increments

1. Keep the existing draft record as an aggregate offline engineering view until the [owner crosswalk](./LIFETIME_PRODUCT_OWNER_CROSSWALK_DRAFT.md) is accepted; do not promote a duplicate BOM, product identity, supplier ledger, repair log, or qualification authority.
2. Review and reconcile the draft product/component/interface schema against Mycelix's existing design revision, Manufacturing BOM, physical-unit/qualification and ProductiveLoop/repair owners.
3. Run the standalone tool at [tools/lifetime-product-audit](../tools/lifetime-product-audit/README.md) on a pinned Rust toolchain, fix compiler/test findings, generate and commit Cargo.lock, and retain exact-head output digests. The tool is not yet compiled or executed.
4. Extend the known-answer suite with deliberate mutations for unresolved references, stale supplier evidence, gate-without-evidence, incompatible substitutes, stale revisions, missing calibration and false lifetime claims. Test through the exact JSON Schema implementation, not only a hand-rolled structural check.
5. Reuse the existing Mycelix Manufacturing BOM expansion and existing repair/task graph validators; do not duplicate BOM cycle validation, work-order states, repair-task authority, or physical-unit identity.
6. Build Symtropy regression scenarios for cabinet/module envelopes, movement limits, tool/access clearance, module extraction paths and disassembly from versioned geometry. Keep structural stress, airflow, and thermal transfer in qualified engineering tools.
7. Establish a Symthaea candidate-comparison fixture with hard safety constraints, a multi-objective frontier, uncertainty and a reproducible baseline. Treat generated candidates as hypotheses until inspected/tested.
8. Add Sol-Atlas sourcing/context references only through existing material and regional repair-workcell owners; preserve source date, license, uncertainty, spatial scope and stale-data status.
9. Choose a low-risk first physical validation slice that follows Symtropy's existing appliance pilot sequencing. Do not make the integrated refrigeration-plus-induction kitchen island the first powered prototype.
10. For a later LKI-01 prototype, document the as-built configuration, qualified appliance installation, module-replacement trials and instrumented energy/temperature/noise tests. Publish only claims supported by exact-revision evidence.

## 12. Sources

- IEC 60335-1 catalogue: https://webstore.iec.ch/en/publication/61880
- IEC 60335-2-24:2025 catalogue: https://webstore.iec.ch/en/publication/75938
- EN 45554:2020 overview, NEN: https://www.nen.nl/en/nen-en-45554-2020-en-270440
- IEC 62430:2019 catalogue: https://webstore.iec.ch/en/publication/30879
- ISO 14040:2006 overview: https://www.iso.org/standard/37456.html
- Commission Regulation (EU) 2019/2019, ecodesign for ordinary refrigerating appliances: https://eur-lex.europa.eu/eli/reg/2019/2019/oj/eng
- Commission Regulation (EU) 2019/2024, ecodesign for refrigerating appliances with a direct-sales function: https://eur-lex.europa.eu/eli/reg/2019/2024/oj/eng
- Directive (EU) 2024/1799, consolidated 31 July 2026: https://eur-lex.europa.eu/eli/dir/2024/1799/2026-07-31

Standards are often copyrighted and full text may require purchase. The links establish the research starting point; they do not substitute for acquiring the applicable standards, reviewing their normative clauses, or obtaining independent conformity advice.
