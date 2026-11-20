# AEROCOMMONS-INTEROP-BENCH-001 — adjudicated QIF/AP242 differential benchmark

Issue: https://github.com/Luminous-Dynamics/mycelix/issues/6388  
Crate: `tools/aero-interop-benchmark`  
Schema: `docs/aerocommons/AERO_INTEROP_BENCHMARK_V1.schema.json`

## Industry baseline (20 November 2026)

The benchmark is designed to compose with specialist systems rather than claim to replace them:

| Existing capability | Publicly documented baseline | What AeroCommons should do differently |
|---|---|---|
| NIST STEP File Analyzer and Viewer | Version 5.41; AP242 and other STEP protocols; entity/attribute reports; semantic PMI, graphic PMI and validation-property analysis against CAx-IF recommended practices | Treat NIST output as a pinned independent observation, preserve report bytes, correlate its PMI entities with QIF trace outputs, and surface disagreement by semantic field |
| QIF community | QIF 3.0 C++, C# and Python bindings; sample instances; separate XSD/schema and XSLT checking tools | Reuse normative validation; pin schema/check dependencies; qualify extraction separately from schema validity |
| CAx-IF | Cross-Domain Exchange for Downstream Uses v1.0 (29 January 2026); Test Cases 7 and 8 (16 December 2025); persistent-ID practice v1.8 (13 March 2026) | Build an adjudicated, replayable Test 7/8 corpus and include adversarial mutations beyond the published happy path |
| PTC Windchill | Enterprise PLM, BOM/configuration/change management, manufacturing process planning and connected lifecycle workflows; current Windchill 13.1.2 material | Do not replace PLM. Provide a vendor-neutral evidence/identity/qualification sidecar across PLM, QMS, MES, CMM and suppliers |
| Hexagon PC-DMIS | QIF 3.0 import into measurement routines; CMM programming, simulation and measurement workflow | Test exact metadata/PMI/trace preservation across import/export and failed measurement cases; do not claim to replace CMM programming |
| ZEISS PiWeb | Quality-data management and reporting across measuring technologies, statistical analyses, measurement history and open API integrations; PiWeb 2026 release stream | Connect exact metrology evidence and source lineage to a vendor-neutral, independently replayable digital thread |

Sources:
- NIST STEP File Analyzer and Viewer 5.41: https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer
- QIF community: https://github.com/QualityInformationFramework/qif-community
- QIF validation tools: https://github.com/QualityInformationFramework/qif-validation-tools
- CAx-IF recommended practices: https://www.mbx-if.org/home/cax/recpractices/
- PTC Windchill: https://www.ptc.com/en/products/windchill
- PC-DMIS QIF import documentation: https://docs.hexagonmi.com/pcdmis/2023.1/en/helpcenter/mergedprojects/core/05_file_adv_topics/Importing_a_QIF_File.htm
- ZEISS PiWeb: https://www.zeiss.com/metrology/en/software/zeiss-piweb.html

## Standards/version baselines

- ISO 10303-242:2025 Edition 4 was published 25 August 2025. Edition 5 remains a committee draft under development; ISO/CD 10303-242 was referred back to the working group after the comment period closed 22 April 2026.
- ISO 23952:2020 QIF was reviewed and confirmed current on 14 June 2026.
- ISO 23247-5:2026 digital thread for manufacturing was published 24 June 2026.

These versions are reference identities for corpus profiles, not claims of certification or conformance.

## Paradigm shift

The field has many strong specialist engines. The missing capability is a neutral means to compare them without silently changing the corpus, losing failure outcomes or letting a vendor output define its own correctness.

The benchmark treats interoperability as a set of independently adjudicated propositions with exact provenance:

`Frozen Inputs → Pinned Tool/Runtime → Run Receipt → Exact Outputs → Oracle Comparison → Adjudication → Reproducible Scorecard`

No majority vote, model-generated ground truth or aggregate trust score. If two tools disagree, retain the disagreement and identify the semantic field/trace edge responsible.

## Evaluation model

The first Rust increment implements:
- closed, versioned benchmark manifest and deterministic digest;
- rights-cleared input and independent-oracle artifacts;
- explicit oracle method/profile version;
- tool registry with category, version and executable digest;
- case-level source artifacts and per-claim labels;
- exact corpus and oracle payload-byte verification before scoring;
- reuse of `AeroAdapterRunReceiptV1` for each executed tool × case;
- exact receipt binding to benchmark, case, registered tool and declared input identities;
- complete tool × case grid, including Blocked and NotRun slots;
- exactly one observation per claim for every successful run;
- separate Failed/TimedOut receipts, which never yield semantic observations;
- scorecards by QIF structure, QIF normative checks, AP242 PMI, extraction, identity, semantic preservation, result-status retention, reproducibility and execution security.

The scorecard reports TP/FP/TN/FN, critical false accepts, oracle-unadjudicated claims, tool Unknown/Unsupported/Error, NotRun and Blocked. It also reports offered and adjudicated denominators. There is no weighted or aggregate "quality" score.

## Corpus plan

Initial source campaign:
1. CAx-IF Test 7: QIF Results with copied/expanded mapping and nominal information.
2. CAx-IF Test 8: abbreviated Results with external xId links into Plan across MeasurementResultsSets.
3. Synthetic positive/negative controls for IDs, QPIds, external references, PMI, tolerance semantics and result-status trace.
4. Metamorphic/adversarial cases: QPId mismatch, wrong external reference ID, same xId in a different Plan, Plan digest mutation under unchanged QPId, wrong-type xId, missing/ambiguous EntityExternalId, STEP UUID mismatch, unit/tolerance changes, graphic-only PMI, dropped trace edge, deleted failed measurement, missing source locator, duplicate IDs, unsupported constructs, valid-XSD/invalid-XSLT, URI traversal and resource exhaustion.

The present unit-test fixture is synthetic and does not claim to reproduce official Test 7/8 files. CAx-IF source package rights must be checked per artifact before repository redistribution. A private-use corpus may be evaluated under different terms from the public CI corpus.

## Oracle governance

- The oracle artifact, rights status, method and profile version are part of the case identity.
- Each claim names a stable source locator/mutation, dimension, expected Accept/Reject/Unadjudicated label, rationale code and safety criticality.
- Unadjudicated is not coerced to Accept or Reject and is excluded from confusion-matrix counts.
- Oracle coverage gaps remain visible.
- LLMs may propose candidate mappings for human review but cannot author ground truth and then grade their own output.
- At least two independent reviewers/tools should adjudicate high-risk claims; disputed labels remain Unadjudicated until evidence resolves them.

## Prospective targets (not achieved)

| Dimension | Target |
|---|---|
| Trace-critical oracle agreement | 100% on frozen positive/negative controls |
| Critical false accepts | 0 on wrong-QPId, wrong-subject, stale-revision, dropped-field and namespace-collision controls |
| Namespace integrity | 0 QIF local-ID/xId/STEP UUID conflations |
| Result retention | 100% trace retention for Passed, Failed, NotEvaluated and Unknown measurement records |
| Corpus coverage | 100% exact tool × case slots; no silent omission |
| Claim coverage | 100% claim observations for successful runs |
| Reproducibility | 100% exact digest replay for source, tool, receipt, report and output artifacts |
| Execution security | 0 unmanifested file/network access in hostile-input worker tests |

Performance scorecards must publish p50/p95 latency, peak memory/CPU, bytes processed, throughput and total cost. Every claim needs numerator, denominator, corpus digest, tool/runtime/config digest, exclusion policy and matched hardware. Budget accounting includes setup, tuning, aborted runs, failures, audit and acquisition overhead.

## Relationship to the rest of AeroCommons

- #6349 semantic assertion comparator: per-output semantic comparison primitive.
- #6355 adapter-run receipt: exact one-run context and payload binding.
- #6360 replay benchmark: same-context reproducibility and context-difference classification.
- #6371 QIF xId trace resolver: normalized cross-document identity resolution.
- #6372 QIF ingest qualification: source-stage reports and extraction mapping evidence.
- #6388 this benchmark: corpus, oracle, adjudication, denominator policy and multi-tool scorecards.

## Claim ceiling

The benchmark evaluator scores supplied, receipt-verified observations against supplied, rights-cleared oracle artifacts. It does not execute vendor tools, parse STEP/QIF, authenticate a tool publisher, establish the oracle's physical truth, validate measurements or authorize product release. No SOTA, performance, standards-conformance, metrology, certification or airworthiness claim is made by the first synthetic increment.
