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

1. Design definition and requirement identities.
2. Manufacturing process-plan artifact, material batch/lot, machine and tooling.
3. Active as-run build observation bound to the component and exact configuration.
4. Inspection plan, instrument, active calibration evidence, QIF result artifact and inspection evidence.
5. Test plan, instrument, active calibration evidence, test result artifact and test evidence.
6. Exact evidence references joining build → inspection → test.

Primary evidence must use the same component and configuration. Calibration evidence is bound to its exact instrument and must be active, but is scoped to its own calibration configuration.

## Structural invariants

- Unknown packet fields are rejected.
- Artifact references must include expected kind, nonzero byte length, media type, format identifier and a lowercase SHA-256-shaped identifier whose sha256 reference matches the declared digest.
- The digest is syntax-checked only; v1 does not read artifact bytes to recompute it.
- Build evidence must reference design, process, material, requirements, machine and tooling.
- Inspection evidence must reference build evidence, inspection plan/instrument, instrument calibration evidence and a QIF artifact.
- Test evidence must reference build and inspection evidence, test plan/instrument, instrument calibration evidence and a test result artifact.
- Primary evidence and calibration evidence must be active and pass the existing typed evidence structural validator.
- Unknown, contradicted or missing records never become positive completeness.

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
