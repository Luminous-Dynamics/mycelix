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

- [LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json](./LIFETIME_PRODUCT_RECORD_SCHEMA_V1_DRAFT.json) defines the proposed product record, component/supplier evidence, module interfaces, metrics, release gates, and lifecycle evidence.
- [LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json](./LKI_01_PRODUCT_RECORD_EXAMPLE_DRAFT.json) is an illustrative LKI-01 record with three placeholder BOM entries, two provisional interfaces, and five unpassed release gates.

Both artifacts are drafts for review, not canonical wire contracts or approved manufacturing data. The example intentionally has no supplier records or test evidence; its dimensions and target values are provisional. Schema adoption must include validator tests, migration/version policy, and ownership review against the existing Fabrication/BOM types.

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

The EU refrigerating-appliance ecodesign regulation (EU) 2019/2024 is a useful reference: it specifies at least eight years of availability after the last unit is placed on the market for listed refrigerator components and requires listed parts to be replaceable with commonly available tools without permanent damage. LKI-01's proposed 20-year support target intentionally aims above that reference where feasible, but legal applicability and current consolidated text must be checked for the market and product.

## 11. First implementation increments

1. Adopt this contract as a proposed design baseline; do not claim that it is already implemented end-to-end.
2. Define the first machine-readable product and component schema, including units, revisions, evidence status, interface requirements, and safety gate status.
3. Add a small LKI-01 example BOM with explicit illustrative-only components and quantities; do not insert unverified supplier model numbers as approved parts.
4. Implement deterministic BOM validation: non-empty IDs, positive quantities, valid units, acyclic subassembly expansion, and reproducible output.
5. Build Symtropy regression scenarios for clearances, movement limits, access, and removal paths from a versioned geometry fixture.
6. Establish a Symthaea candidate-comparison fixture with hard constraints, multi-objective outputs, uncertainty, and baseline reproduction.
7. Add Sol-Atlas sourcing-record schema and stale-evidence checks before live supplier data is treated as procurement-ready.
8. Assemble one physical prototype using certified, off-the-shelf appliance modules and record the as-built configuration.
9. Run a repair-time trial and instrumented thermal/energy test; publish only the claims supported by the results.

## 12. Sources

- IEC 60335-1 catalogue: https://webstore.iec.ch/en/publication/61880
- IEC 60335-2-24:2025 catalogue: https://webstore.iec.ch/en/publication/75938
- EN 45554:2020 overview, NEN: https://www.nen.nl/en/nen-en-45554-2020-en-270440
- IEC 62430:2019 catalogue: https://webstore.iec.ch/en/publication/30879
- ISO 14040:2006 overview: https://www.iso.org/standard/37456.html
- Commission Regulation (EU) 2019/2024, refrigerating appliance ecodesign requirements: https://eur-lex.europa.eu/legal-content/en/ALL/?uri=CELEX:32019R2024

Standards are often copyrighted and full text may require purchase. The links establish the research starting point; they do not substitute for acquiring the applicable standards, reviewing their normative clauses, or obtaining independent conformity advice.
