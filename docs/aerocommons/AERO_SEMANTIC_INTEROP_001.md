# AeroCommons semantic interoperability benchmark — v1

Status: research contract; synthetic authored cases plus external reference-case pointers  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6348

## Why this benchmark exists

A file can be syntactically valid, schema-valid and byte-identical while losing the engineering meaning it was supposed to carry. A nominal dimension may survive while its tolerance type is lost; a measurement result may survive while its persistent identifier no longer points to the correct feature; a datum may be rendered graphically while the semantic datum relationship disappears.

This benchmark therefore evaluates **semantic preservation across a manufacturing lifecycle**, not parser success alone.

## External reference baseline

Use the exact publicly published versions and licensing terms of:

- CAx-IF / MBx Interoperability Forum, **Cross-Domain Exchange for Downstream Uses**, joint AP242/QIF guidance with trace information; published test cases 7 and 8 (16 December 2025): https://www.mbx-if.org/home/cax/recpractices/
- NIST STEP File Analyzer and Viewer, for independently checking AP242/PMI support and conformance findings: https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer and https://github.com/usnistgov/SFA
- NIST Digital Thread for Manufacturing, including documented semantic PMI, persistent identifier, hybrid geometry and composite data gaps: https://www.nist.gov/programs-projects/digital-thread-manufacturing
- NIST GCR 24-050, spatial and temporal association across STEP AP242, NC, MTConnect and QIF: https://www.nist.gov/publications/spatial-and-temporal-data-alignment-disparate-sources-feature-association
- ISO 10303-242:2025 (Edition 4; Edition 5 is under development), ISO 23952:2020 (QIF; confirmed current in 2026), and ISO 23247-5:2026.

These links are research references, not blanket rights to copy or redistribute source artifacts. Each external case must have an exact version, acquisition record, rights disposition and byte digest before it enters a runnable corpus.

## Pipeline under test

`AP242 product definition + semantic PMI`
→ `manufacturing planning / NC / machine execution`
→ `QIF inspection plan + results`
→ `Mycelix component / serialized unit / evidence lineage`

The benchmark does not require every participant to use the same internal database. It requires each stage to preserve and expose the agreed semantic assertions and trace links.

## Semantic dimensions

The v1 assertion profile includes:

- product, part, assembly, component and serialized physical-unit identity;
- persistent identifiers and source locators;
- requirement and feature/shape-aspect association;
- nominal dimensions, units, tolerances, datum references and GD&T meaning;
- coordinate frame and transformation lineage;
- material and process-critical properties, including composite layup where the source profile supports it;
- process operation, setup, tool/machine and as-executed observations;
- inspection characteristic, measurement method, instrument and calibration evidence;
- measurement value, unit, uncertainty, event-time scope and QIF result identity;
- links from inspection result back to the exact design requirement, feature, configuration and serial unit.

A field not represented by a tested profile is `UNSUPPORTED` or `UNKNOWN`, never silently treated as preserved.

## Evaluation modes

1. **Structural conformance:** syntax/schema/profile requirements.
2. **Semantic preservation:** equality or explicitly justified equivalence of assertions before/after each transformation.
3. **Trace preservation:** source-to-target identity and provenance edges remain correct.
4. **Spatial/temporal alignment:** measured feature and execution event bind to the correct geometry and event interval.
5. **Mutation resistance:** malformed, ambiguous, stale or cross-subject references do not produce affirmative preservation.
6. **Independent replay:** a second implementation reproduces the assertion and disposition results from frozen inputs.

## Required scorecard

Report each dimension separately:

- supported assertion recall / preservation rate;
- silent semantic loss count (hard gate);
- wrong-subject / wrong-feature / wrong-revision join count (hard gate);
- identifier collision and unresolved-link counts;
- unsupported/ambiguous semantic coverage;
- spatial and temporal association error where applicable;
- round-trip semantic drift;
- byte-integrity and source/toolchain completeness;
- parser and independent-verifier disagreement;
- p50/p95 runtime and memory.

Proposed research gate: 100% preservation of all declared supported assertions on the frozen corpus, zero silent semantic loss, zero false-positive joins, 100% source-byte/toolchain binding, and explicit accounting for every unsupported field. These are targets, not measured results or regulatory requirements.

## Matched comparison

Run the same source files and assertions through:

- a documented manual engineering/inspection checklist;
- NIST SFA and its declared supported checks;
- the CAx-IF reference cases and guidance;
- rights-cleared commercial CAD/CAM/QMS/PLM exporters/importers where the relevant functions can be instrumented;
- the AeroCommons adapters.

Do not equate one vendor's successful import/export with full interoperability. Measure semantic coverage, unsupported fields, identity preservation, trace reconstruction effort, and time to identify a changed requirement's affected inspection results.

## Initial synthetic corpus

`fixtures/aero-semantic-interop/reference-cases-v1.json` includes authored, explicitly synthetic assertion cases for linear dimensions/tolerances, datum references, persistent identifiers, unit conversion and QIF result linkage. Its cases are not substitutes for CAx-IF Test Cases 7/8 and are not valid AP242 or QIF instance documents.

## Ownership

- Symtropy #4165 owns inspectability/NDE and measurement-observability engineering.
- Symtropy #2878/#3921/#5853 own aircraft engineering, manufacturability and whole-aircraft comparisons.
- Symthaea #7652 owns factory/process reasoning and workforce loops.
- Mycelix/AeroCommons owns cross-stage identity, evidence, provenance and change-impact linkage.
- NIST SFA and CAx-IF retain their role as independent standards/reference tooling and test sources.

## Claim ceiling

A pass establishes preservation only for the frozen assertions, exact artifact versions, selected standards profiles, tested transformation path and toolchain. It does not prove full AP242/QIF/ISO 23247 conformance, physical measurement validity, process capability, part acceptance, certification or airworthiness.
