# Modular Desk Prototype — Qualification Plan

**Status:** Proposed test plan; not a certificate, test report, or approval to manufacture  
**Date:** 2026-10-13  
**Applies to:** `modular_desk_prototype`, family revision 1, only after geometry, joinery, hardware, and materials are frozen  
**Owner:** Product engineering  
**Release rule:** No production release until applicable requirements, acceptance criteria, test evidence, and authorized review are bound to the exact design revision.

## 1. Purpose and claim ceiling

This plan defines the evidence needed before a generated desk configuration can move from a software prototype to a physical product qualification process. It does not reproduce paid standard text, prescribe unverified load limits, or claim regulatory compliance.

The current compiler only emits rectangular panel envelopes and candidate process steps. Its dimensions are schema bounds, not ergonomic recommendations or safety limits. The generated packet is permanently release-blocked. Do not manufacture, sell, or advertise a packet as qualified based on the compiler output alone.

## 2. Product configuration must be frozen first

Before physical testing, engineering must release a detailed assembly model containing at least:

- dimensioned parts with material grade, thickness, grain/face orientation, edge finish, tolerances, and supplier specification;
- connector/fastener part numbers, count, locations, required pilot-hole/drilling geometry, tool access, and installation torque/limits where applicable;
- structural load paths, joint assumptions, tabletop support conditions, tipping/stability considerations, and permitted use envelope;
- all brackets, feet, glides, cross-members, aprons, cable openings, and accessories included in the tested configuration;
- assembly steps, required tools, tightening sequence, reassembly policy, and prohibited substitutions;
- packaging layout, part protection, package dimensions/mass, and handling assumptions.

A product with missing joinery or fastener definitions is not a testable/released design. Any later change to material, thickness, joint, hardware, support geometry, panel span, or manufacturing tolerance triggers a documented engineering change assessment and can require requalification.

## 3. Standards and source review

Use the current edition applicable to the chosen market and intended use. Obtain and review complete standards through authorized sources; the summaries below are only orientation.

### Table/desk stability, strength, and durability

- [ISO 19682:2023 — Furniture — Tables — Test methods for the determination of stability, strength and durability](https://www.iso.org/standard/73590.html). ISO describes this as applying to the structure of tables and desks. This is the current published successor to **ISO 21016:2007**, which ISO marks withdrawn. Do not cite the withdrawn edition as the active desk test-method standard.
- [ANSI/BIFMA X5.5-2021 — Desk and Table Products](https://www.bifma.org/page/standardsshortdesc) covers desk/table products aimed at commercial office, institutional, and educational environments, including product types called out in the standard.
- [ANSI/BIFMA X6.5-2022 — Home Office and Occasional-Use Desk, Table and Storage Products](https://www.bifma.org/page/standardsshortdesc) may be relevant if the intended claim/use is home-office or occasional-use furniture. The current BIFMA descriptions page also lists X5.5-2021 for commercial/institutional desk and table products and X10.1-2024 for computer-use ergonomic dimensions.

Choose one applicable qualification route with a competent test engineer or laboratory. Do not mix convenient tests from different standards and then claim conformance to either standard. Confirm edition/status again before product release.

### Ergonomic design intent

- [ISO 9241-5:2024 — Workstation layout and postural requirements](https://www.iso.org/standard/86222.html) gives ergonomic principles and requirements for workstation equipment used with visual-display systems. Use it as a design-review reference for workstation layout, fit, adjustability, postural change, support surfaces, and user information where applicable. It is not a claim that one fixed desk height fits every person or that the furniture alone ensures ergonomic health outcomes.
- [ANSI/BIFMA X10.1-2024 — Ergonomics Requirements for Furniture Designed for Computer Use](https://www.bifma.org/page/standardsshortdesc) establishes dimensional requirements applying relevant measurable ergonomics principles to computer-use furniture. Evaluate it alongside ISO 9241-5 when the market and intended use make it applicable; neither standard replaces structural stability/strength/durability testing.

### Storage modules, if introduced

- [ISO 7170:2021 — Storage units — Test methods for strength, durability and stability](https://www.iso.org/standard/76864.html) applies to assembled storage units and their movable/non-movable parts. ISO currently lists it as published and under systematic review; recheck its lifecycle before test-plan approval. Add it to the plan when drawers, cupboards, or storage units enter scope. It is not a substitute for table/desk-specific testing.

### Market and material obligations

Confirm market-specific legal obligations before sale in South Africa or any export market. Review supplier documentation and applicable requirements for panel emissions/adhesives, coatings, chemical restrictions, packaging, consumer product safety, labeling, warranty, and waste handling. This plan does not determine which SANS or other legal requirements apply; that requires market/product-specific review.

## 4. Required test and inspection work

| Work item | Required record | Acceptance criteria source |
|---|---|---|
| Dimensional inspection | Measured part/assembly dimensions, squareness/diagonals, flatness, levelness, critical hole/edge locations, measurement equipment and calibration status | Released drawing tolerances and measurement plan approved before testing |
| Assembly usability | Assembly time, tool count, ambiguous steps, incorrect-assembly opportunities, missing-part detection, fastener accessibility, damage from assembly | Approved usability requirements and observation protocol |
| Joint and connection integrity | Fastener/joint configuration, installation method, inspected failures, looseness, damage, and deviations | Engineering calculations and chosen standard/lab protocol |
| Stability and tipping | Exact product setup, support/floor condition, test fixture, load application point/direction, measured displacement, tipping/sliding observations | Applicable standard plus product-specific acceptance requirement approved in advance |
| Static strength and deflection | Applied load steps, duration, instrumentation, permanent deformation, damage, joint movement, and recovery | Applicable standard, use case, structural review, and predeclared acceptance criteria |
| Durability/cyclic use | Cycle count, force/loading method, interruptions, looseness, wear, failures, and post-test measurements | Applicable standard and predeclared intended-life/performance criteria |
| Surface and edge quality | Visual/physical defects, edge-band adhesion/finish, splinter/sharp-edge check, spill/cleaning procedure if claimed | Released quality specification and relevant material/finish requirements |
| Production capability | Supplier/machine profile, measured process variation, rejected-part/rework rates, traceable first-article inspection | Drawing tolerances and approved process-capability plan |
| Packaging and delivery | Packed configuration, package mass/dimensions, observed handling damage, delivery incident rate, protection and unboxing review | Defined logistics/packaging protocol and allowed damage rate |
| Repair/reassembly | Part replacement time, compatible spare parts, damage from disassembly/reassembly, instructions, post-repair inspection | Declared repairability claims and a predeclared test cycle/procedure |

The table intentionally avoids generic kilogram limits and cycle counts. Those values must be chosen from the applicable standard and intended use, then reviewed by a qualified person and frozen before the physical test. Passing one test is evidence only for the configuration and scope actually tested.

## 5. Variant and family qualification

A parameterized family does not inherit qualification automatically.

Initially qualify one fixed configuration. For further variants, create an engineering equivalence/risk assessment covering at least:
- maximum unsupported spans and width/depth changes;
- top thickness, material density/grade, grain direction, moisture/finish, and stiffness differences;
- changed joints/fasteners, hole spacing, edge distances, and connector access;
- foot/support spacing and centre-of-mass changes;
- loads and intended use, including monitors/accessories if allowed;
- manufacturing tolerance, tooling, and supplier-process differences.

Require additional calculations or physical testing whenever equivalence cannot be justified with traceable engineering evidence. The software must not infer that all dimensions inside current input bounds are structurally safe.

## 6. Evidence packet required for a qualified configuration

Bind a qualification record to the immutable design and manufacturing packet. Minimum fields:

- design family, family revision, full configuration, design digest, BOM digest, routing digest, and material/finish specifications;
- test plan ID/revision, applicable standard and edition, clauses/test-method references, lab/procedure IDs, preapproved acceptance criteria, and rationale for applicability;
- sample identity, build lot, supplier and process profile, assembly instructions/revision, fastener/adhesive lots where relevant;
- test engineer and reviewer identities/roles, equipment identifiers and calibration status, raw data and image/file digests, results, deviations, and dispositions;
- exact tested configuration, any justified representative-variant scope, limitations, re-test triggers, and expiration/review policy;
- independent approval receipt binding the above evidence to the exact design digest.

A result without an applicable acceptance criterion is not a pass. A report that cannot identify the tested design revision is not qualification evidence for a later revision. Bind the `family_ruleset_version` and `geometry_ruleset_id` as well as the packet schema/digest profile; matching customer dimensions alone does not establish equivalence if the generator rules changed.

## 7. Release state machine

Qualification must be distinct from geometry/schema validation.

`NotAssessed -> TestPlanApproved -> Tested -> QualifiedForScope`

Failure paths include `Failed`, `NeedsRework`, `Suspended`, and `ExpiredOrReviewRequired`. A new material, joint, critical dimension, supplier process, or standard edition may require reassessment. No Symthaea response, simulation result, unit test, or compiler digest may itself authorize `QualifiedForScope`.

The production release gate remains closed unless the required records exist, are internally consistent, are current, and are signed/approved by the designated authority. If evidence is missing, contradictory, stale, or bound to another digest, fail closed.

## 8. Pilot metrics and no-go rules

Record observed data rather than generated estimates:
- first-pass yield, rework, scrap, and usable-offcut recovery;
- manufacturing/assembly labor minutes and tooling/setup time;
- packaging failures, delivery damage, missing parts, and support contacts;
- actual delivered-and-assembled cost reconciled to supplier invoices and labor;
- assembly comprehension, error rate, and repair/replacement turnaround.

No-go conditions include structural/stability test failure, unidentified critical parts, unapproved substitutions, any safety-related unresolved defect, cost model missing significant variable expenses, untraceable qualification evidence, or an order packet whose design/BOM/routing/quote revisions disagree.

The proposed business target of at least 10% lower all-in cost than a comparable alternative remains a hypothesis. It should be evaluated only after equivalent product scope, delivered price, assembly, quality, returns, and warranty/support assumptions have been frozen.

## 9. Change-control and ongoing review

Every drawing, material, joint, fastener, toolpath, machine, packaging, instruction, family-ruleset, geometry-ruleset, or packet-schema change gets a change record identifying affected artifacts and required rechecks. A design-digest, ruleset, geometry-rule, or packet-schema change invalidates any release/qualification association until an accountable reviewer determines the required retesting or equivalence evidence. Version bumps alone do not prove equivalence.

Preserve failed tests and their evidence. Supersede them with a linked record; do not erase or relabel failures. Recheck applicable standards and market requirements before a release, because this document is a dated plan, not a continuously updated compliance determination.

## 10. References

- ISO 19682:2023: https://www.iso.org/standard/73590.html
- ISO 21016:2007 withdrawal and successor record: https://www.iso.org/cms/%20render/live/en/sites/isoorg/contents/data/standard/03/56/35609.html
- ANSI/BIFMA standard descriptions and editions: https://www.bifma.org/page/standardsshortdesc
- ISO 9241-5:2024: https://www.iso.org/standard/86222.html
- ISO 7170:2021: https://www.iso.org/standard/76864.html
