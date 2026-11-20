# AeroCommons component build packet — v1

Status: synthetic research verifier; non-flight-critical bench fixture only  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6345

## Purpose

The verifier checks whether a component's design definition, requirements, process plan, material batch, machine/tooling, as-run observation, inspection and test evidence are structurally linked to one exact component configuration. It is the first executable slice of the AeroCommons design → build → inspect → test → evidence lineage.

## Paradigm shift

Treat the physical article as a configuration-bound digital thread rather than a CAD file accompanied by a folder of unrelated PDFs. A design revision, material substitution, process change, machine/tool change, inspection-method change or calibration expiry should be capable of triggering targeted evidence review. The present v1 verifier checks the initial packet joins; it does not implement change-impact propagation or evaluate physical adequacy.

## Interoperability boundaries

- STEP AP242 (ISO 10303-242:2025) is the product-definition and configuration exchange anchor.
- QIF (ISO 23952:2020) is the quality/metrology exchange anchor.
- ISO 23247-5:2026 is the manufacturing digital-thread interoperability anchor.

The packet stores external artefact references and metadata only. No standards parser, normative schema validation or conformance claim is implemented.

## Required lineage

1. Component definition, serialized physical-unit identity, exact configuration and requirement identities.
2. Manufacturing process-plan artifact, material batch/lot, machine and tooling.
3. Active as-run build observation bound to the serialized unit and exact configuration.
4. Inspection plan, instrument, calibration evidence valid at the inspection event time, QIF result artifact and inspection evidence.
5. Test plan, instrument, calibration evidence valid at the test event time, test result artifact and test evidence.
6. Exact evidence references joining build → inspection → test, with test time not preceding inspection time.

Primary evidence must use the same component and configuration. Calibration evidence is bound to its exact instrument and must be active, but is scoped to its own calibration configuration.

## Structural invariants

- Unknown and missing packet, reference, artifact and nested evidence fields are rejected at the strict JSON admission boundary.
- Artifact references must include expected kind, nonzero byte length, media type, format identifier and a lowercase SHA-256-shaped identifier whose sha256 reference matches the declared digest.
- Structural packet validation checks digest syntax. The separate whole-packet payload verifier recomputes SHA-256 and byte length for all four content artifacts. The checked-in synthetic payloads prove this exact-byte replay path; they are not valid normative AP242/QIF files.
- Build evidence must reference design, process, material, requirements, machine and tooling.
- Inspection evidence must reference build evidence, inspection plan/instrument, instrument calibration evidence and a QIF artifact.
- Test evidence must reference build and inspection evidence, test plan/instrument, instrument calibration evidence and a test result artifact.
- Primary evidence and calibration evidence must be active and pass the existing typed evidence structural validator.
- Calibration validity uses the half-open interval [valid_from, valid_until); the calibration record must be the exact referenced evidence record and must cover the declared measurement time.
- The serialized physical unit is distinct from the component definition. Build, inspection and test claims bind to that unit; the build record also links back to its component definition.
- The recorded times are replay metadata only; this packet does not establish clock accuracy or synchronization.
- Unknown, contradicted or missing records never become positive completeness.

## Standards baseline and interoperability gap

As of 20 November 2026, ISO 10303-242:2025 Edition 4 is published (August 2025) and ISO lists Edition 5 as under development. ISO 23952:2020 (QIF) was reviewed and confirmed current in June 2026. ISO 23247-5:2026 was published in June 2026 for manufacturing digital-thread principles and methods. The pilot references these standards but does not implement normative parsers or claim conformance.

NIST's Digital Thread for Manufacturing programme explicitly identifies remaining gaps in semantic product/manufacturing information, GUID linkage, hybrid geometry and composite representation, as well as the need for conformance testing and trust/cybersecurity. This is the next interoperability benchmark target: measure semantic preservation across AP242 → process planning → QIF → evidence and unit identity, not merely whether files parse or hashes match.

Official references:
- ISO 10303-242:2025: https://www.iso.org/standard/84300.html
- ISO 23952:2020: https://www.iso.org/standard/77461.html
- ISO 23247-5:2026: https://www.iso.org/standard/87425.html
- NIST Digital Thread for Manufacturing: https://www.nist.gov/programs-projects/digital-thread-manufacturing

## Matched evaluation

Compare with a manually assembled folder/checklist workflow using the same frozen packet and adversarial mutations. Measure missing-link detection, wrong-configuration detection, time to trace a change, duplicate/contradictory artifact handling, and reviewer reconstruction effort. Compare format interoperability with AP242/QIF/ISO 23247 profiles, not with claims that this verifier replaces PLM, QMS, MES or a manufacturing digital twin.

## Acceptance targets

These are software targets, not manufacturing acceptance criteria:

- 100% agreement with an independent golden corpus.
- Zero positive completeness outcomes for negative controls.
- Deterministic result under record and input permutation.
- Zero cross-configuration evidence joins.
- Explicit structural-only result semantics.

## Claim ceiling

STRUCTURALLY_COMPLETE means only that the submitted synthetic packet is internally complete under this verifier's rules. It is not a finding of material suitability, process capability, dimensional conformity, metrology traceability, test adequacy, part acceptance, certification, airworthiness or permission to fabricate, install, operate or fly.

## Standards version notes (checked 20 November 2026)

- ISO 10303-242:2025 Edition 4 remains the published AP242 baseline. ISO/CD 10303-242 Edition 5 is under development; its committee-draft consultation closed 22 April 2026 and the draft was referred back to the working group. Do not silently upgrade the packet's declared design format until the exact edition/profile is selected and fixture-tested.
- ISO 23952:2020 (QIF) was confirmed current by ISO on 14 June 2026.
- ISO 23247-5:2026 was published in June 2026 and provides principles/methods for the manufacturing digital thread across design, planning, production and testing.

This packet uses those as interoperability anchors. It does not claim conformance to their normative schemas or profiles.
