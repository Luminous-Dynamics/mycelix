# AeroCommons cross-artifact evidence join v0.1

**Tracker:** [AEROCOMMONS-EVIDENCE-JOIN-001 #6488](https://github.com/Luminous-Dynamics/mycelix/issues/6488)  
**Status:** bounded, read-only join verifier over already-normalized evidence records.  
**Claim ceiling:** `joined_within_declared_scope` is an exact identity/lineage join only; not physical conformity, inspection acceptance, structural qualification or release.

## 1. Purpose

Compose two independent AeroCommons outputs without reimplementing their parsers or validators:

- [Component build packet #6345 / PR #6347](https://github.com/Luminous-Dynamics/mycelix/pull/6347): strict `AeroComponentBuildPacketV1` admission, evidence edges, calibration windows and byte-exact hashes for its four content artifacts.
- [Semantic AP242/QIF comparator #6348 / PR #6349](https://github.com/Luminous-Dynamics/mycelix/pull/6349): normalized semantic assertion comparison and reason-coded results.

The join question is whether the semantic assertions and the component packet refer to the same exact serialized unit, configuration, requirements and inspection/test evidence.

This is not a STEP or QIF parser. It consumes strict upstream types and comparator outputs. It does not create a provenance ledger, geometrical feature database, uncertainty model, certificate authority or structural acceptance decision.

## 2. Architecture

`source AP242/QIF parser or adapter`
→ `SemanticCaseV1 + observed assertion set`
→ `aero-semantic-interoperability evaluator`
→ `EvaluationReport`
→ `aerocommons-component-lineage packet + exact artifact byte verification`
→ **this join verifier**
→ `AeroEvidenceJoinReceiptV1`
→ `Symtropy structural interface / test-matrix / as-built acceptance consumer`

The join verifier imports the two existing Rust crates by local path and calls their public admission/evaluation functions. It must not copy their structs, parsers or semantic scoring rules.

## 3. Exact identity rules

1. Semantic-case `configuration` must equal packet `configuration` by both ID and kind.
2. Exactly one `persistent_identity` assertion must bind to the packet `physical_unit`; its value must identify that same serial.
3. Semantic assertions that name a packet artifact must bind to its exact `ContentArtifactV1.reference`, including artifact kind.
4. Every requirement reference must either be exactly present in packet `requirements`, or be represented by a typed explicit mapping. No fuzzy or name-only matching.
5. Every feature/characteristic reference must either bind to an exact packet identity or have an explicit mapping to a packet component/design artifact, with a basis reference and separate review disposition.
6. A measurement result must bind to the exact inspection-result artifact and the packet's inspection evidence. Its `result_id` is not interchangeable with the QIF file digest.
7. A calibration assertion must bind to the exact instrument and the packet's declared validity interval/used-at time.
8. Build/process assertions must resolve to the exact design-definition, process-plan, material-batch, machine and tooling identities present in the packet.
9. Every required assertion must have an `EvaluationReport` result. `PRESERVED` is required to close that semantic link; `SEMANTIC_LOSS`, `WRONG_SUBJECT_JOIN`, `UNSUPPORTED`, `AMBIGUOUS` or missing results leave the relevant edge unresolved.
10. No assertion result may promote the packet to physical acceptance or flight qualification.

### Explicit mapping semantics

Mappings are never fuzzy. A mapping record must include exact source and target references, typed relation, source mapping-evidence reference and an explicit review state. A mapping with no review evidence remains `REVIEW_REQUIRED`; a mapping marked reviewed by an arbitrary string is not authenticated by this verifier. Authentication/authority is delegated to the upstream Mycelix authority workflow. The v0.1 verifier therefore reports the mapping and its claim limit; it does not accept an external mapping as equivalent to an exact identity edge.

The initial positive control uses exact stable IDs wherever possible. Feature/requirement alias mappings are reserved for explicit review-required test cases rather than silently accepted.

## 4. Input binding and replay

The join manifest binds SHA-256 digests of:
- exact packet JSON bytes;
- exact semantic case JSON bytes;
- exact observed assertion-set bytes;
- exact comparator report bytes;
- exact manifest bytes (captured in the output receipt).

The verifier recomputes these digests. A changed packet, semantic case, observed assertion set, report or mapping manifest requires a new receipt. Digests establish byte identity, not source authenticity or truth.

The packet verifier must validate the packet and all four externally supplied artifact payloads before this join may be `JOINED_WITHIN_DECLARED_SCOPE`. A complete packet JSON without its declared bytes is not enough.

## 5. Join disposition

- `JOINED_WITHIN_DECLARED_SCOPE`: all required identity and semantic edges resolve exactly, the comparator report has no required loss/unsupported/ambiguity/wrong-subject results, and packet artifact bytes verify.
- `REVIEW_REQUIRED`: an explicit mapping or a non-fatal relationship requires qualified review; no acceptance claim is made.
- `REJECTED`: conflicting or false identity, wrong serial/configuration/artifact, stale revision, duplicate/conflicting mappings, or a comparator wrong-subject result.
- `INDETERMINATE`: missing inputs, report/case mismatch, absent evidence/result, unverified artifact bytes or unresolved required semantic loss.

A successful join is not a passing inspection, passing test, accepted component, adequate structural joint or approved design.

## 6. Receipt contract

`AeroEvidenceJoinReceiptV1` records:
- exact input digests and packet/semantic IDs;
- component, physical-unit and configuration identities;
- semantic evaluator benchmark/case IDs and disposition/counts;
- exact identity edges checked and mappings considered;
- unresolved required assertion IDs and reason codes;
- artifact-byte verification status;
- join disposition and verifier version/source commit;
- `authority_granted=false`, `physical_acceptance=false`, `qualification_granted=false`.

Output order is canonical and deterministic. Replaying identical bytes and versions yields the same receipt, except for explicitly supplied source commit identity.

## 7. Synthetic benchmark

v0.1 includes positive and adversarial cases:
- exact packet + preserved semantic set;
- wrong physical serial or configuration despite an otherwise preserved assertion report;
- a semantic feature aliased to the wrong component;
- requirement identity mismatch;
- measurement result references a different QIF artifact or inspection evidence;
- calibration subject/time differs from packet validity;
- missing/duplicated required assertion result;
- semantic loss, unsupported, wrong-subject and ambiguous comparator result;
- stale packet/case/report digest;
- artifact byte/digest mismatch or omitted payload;
- rights-not-admitted external semantic source;
- input-order metamorphism and one-byte digest mutation.

Targets for the synthetic contract:
- zero false joins on frozen wrong-serial/configuration/artifact negatives;
- zero silent loss of required assertions;
- 100% exact-head oracle agreement;
- 100% changed-byte detection;
- deterministic canonical receipt across input ordering.

These are software-contract targets, not measured physical/industry performance.

## 8. Industry and standards comparison

Consume CAx-IF AP242 Persistent IDs v1.8 and Cross-Domain Exchange for Downstream Uses v1.0 / STEP AP242 Edition 4 ↔ QIF 3.0 Test Cases 7/8 as metrology-focused fixtures. Do not describe these as the full factory manufacturing digital thread: CAx-IF states the cross-domain profile's current use case is metrology and that manufacturing coverage is a future expansion.

Use NIST STEP File Analyzer and Viewer 5.41 for independent AP242/PMI analysis. PC-DMIS 2026.2/SP1, Teamcenter SPDM and 3DEXPERIENCE are commercial integration comparators. Do not clone their parser/CMM/PLM functionality or claim superiority without the same models, versions, outputs and independent comparison.

The proposed Luminous differentiator is vendor-neutral, exact, independently replayable identity/semantic/evidence joining across owner-native tools—not a new universal product model.

## 9. Ownership and safety boundaries

- Mycelix #6345 owns component packet admission and byte verification.
- Mycelix #6348 owns normalized semantic comparison.
- Mycelix #6219 owns event-frontier/change-impact semantics; this verifier may consume its exact receipt but never constructs a second event dependency graph.
- Symtropy #6523/#4263 own structural interface compatibility and as-built acceptance; they consume the join receipt as upstream lineage.
- Symtropy #6540/#6484/#4165 own test obligations, lifecycle and inspection semantics.
- Symthaea #13111/#13112 uses joined claims to propose next evidence; it cannot actuate or authorize tests.
- Sol-Atlas contributes read-only site/facility/logistics context.

No AI outcome authorizes fabrication, test execution, acceptance, production release, airworthiness or flight. Non-flight synthetic examples only.

## 10. Deliverables

P0: this specification and owner map.  
P1: strict join manifest/receipt schemas and 24 synthetic known-answer cases.  
P2: thin Rust composition crate that calls the canonical packet and semantic libraries, verifies exact bytes and emits deterministic receipt; independent standard-library oracle.  
P3: Symtropy consumer contract for profile compatibility and as-built structural acceptance.  
P4: external CAx-IF/NIST and rights-cleared commercial comparison, using identical inputs and no duplicated parsers.
