# IF01 assurance-gate hardening v1

Status: Review-derived implementation specification  
Applies to: Candidate OAD → COS reference slice and its CI evidence  
Claim ceiling: Bounded executable witness only

This document turns the review findings into a small, ordered implementation gate. It does not ratify SPEC-IF-01, certify a production system, or establish manufacturing, safety, economic, ecological, or institutional outcomes.

## 1. Source-status policy

Source status is not a synonym for trust, certification, or authorization. Keep the source class attached to every admitted design and receipt.

| Source status | May inform the reference model | May support semantic admission | May alone create a production basis | May grant production authority |
|---|---:|---:|---:|---:|
| PublicDraftReference | Yes, as a declared proposal | No | No | No |
| CandidateInterface | Yes, as a candidate contract | Only in explicitly labelled candidate/reference mode | No | No |
| RatifiedSchema | Yes, within the ratified schema's exact scope and version | Yes, when current and validated | No | No |
| Implementation | Yes, as implementation evidence | Only with identified implementation and applicable conformance evidence | No | No |
| ConformanceEvidence | Yes, within tested scope and version | Supports, but does not itself perform, admission | No | No |

Production-basis creation must be a separate transition with an explicit COS policy decision and provenance. Production authorization and execution remain separate transitions. A boolean `certified` is not sufficient evidence by itself: where certification is used, bind issuer, subject, scope, profile, generation, validity interval, revocation/currentness evidence, and evidence reference. Missing or ambiguous fields fail closed or remain indeterminate; they must not be silently treated as negative proof of a real-world fact.

## 2. Required admission invariants

- Authentication establishes a principal only; it does not establish authorization.
- Authorization must bind to the exact subject, design generation, profile, scope, and validity window.
- A transport receipt is not semantic admission.
- Semantic admission is not production-basis creation, production authorization, or execution.
- Stale, superseded, profile-mismatched, mutated, or unrecognized evidence cannot be promoted by a successful retry.
- Foreign origin is preserved; recognition does not rewrite origin or imply local observation.
- Indeterminate outcomes remain indeterminate until evidence resolves them.

## 3. Manifest consistency gate

The manifest checker must deterministically:

1. Parse the JSON and reject duplicate obligation IDs.
2. Compare Rust and JSON obligation IDs as sets, report missing/extra IDs, and separately enforce a documented canonical ordering only if ordering is intentionally part of the contract.
3. Resolve every declared production symbol and test symbol against its declared file. Prefer explicit paths per symbol; avoid relying on substring matches across unrelated files.
4. Confirm every referenced source, test, and SMT artifact exists at the checked-out commit.
5. Verify each declared Git blob identity against the file at that commit; do not confuse a Git blob SHA with a SHA-256 digest.
6. Emit per-obligation states: implemented, partial, missing, or ambiguous, with links to exact source/test/proof identities.
7. Preserve the manifest claim ceiling and reject any automatic promotion to `FormallyClosed` based only on structural consistency, test presence, or bounded SMT output.

A failure should identify the obligation and failed check rather than return only a generic assertion.

## 4. Evidence-receipt contract

Use one canonical schema and producer vocabulary. The receipt should contain:

- obligation IDs covered by the run;
- artifact path and explicitly named artifact digest algorithm/value;
- full source commit SHA;
- workflow/run identity and attempt;
- runner/OS and solver name/version;
- exact command and expected result;
- actual solver output (or a durable artifact reference) and output digest;
- timestamp only from a supported, trustworthy run-time source;
- status, claim ceiling, and excluded claims.

CI must validate the generated receipt against the checked-in JSON Schema before upload. Include positive and negative schema fixtures: valid receipt; missing required field; malformed commit; unknown status; malformed obligation ID; unexpected property. The uploaded file must be the exact validated receipt. If a timestamp cannot be obtained reliably, omit it only if the schema makes it optional; do not fabricate one.

## 5. Suggested CI stages

1. JSON/schema validation and duplicate-ID check.
2. Rust/JSON obligation reconciliation and symbol/path resolution.
3. Targeted Rust unit and composition tests.
4. SMT solver execution with explicit expected-result count and captured output.
5. Receipt generation, schema validation, digest verification, and artifact upload.
6. Machine-readable summary with per-obligation status, uncovered obligations, assumptions, and claim ceiling.

All steps should be deterministic, offline where practical, and fail with actionable diagnostics. Pin or record toolchain/solver versions; avoid installing unpinned system dependencies where reproducibility matters.

## 6. Acceptance criteria

- Deliberately reordered but otherwise equivalent ledgers pass set reconciliation; duplicate, missing, or extra IDs fail.
- A nonexistent symbol, test, or artifact fails with its exact path and obligation ID.
- A changed artifact with a stale recorded identity fails.
- Every receipt emitted by CI validates against the checked-in schema.
- Negative source-status, stale-generation, authorization-binding, replay/mutation, and foreign-origin cases are tested.
- No test conflates admission with production authority, execution, qualification, or economic issuance.
- Reports explicitly identify what remains unimplemented or unproven.

## 7. Nonclaims

Passing this gate establishes only that the bounded reference artifacts and their declared evidence are internally traceable and that the tested semantics hold for the exercised cases. It does not prove production deployment, cryptographic security, network guarantees, safety, fairness, ecological benefit, economic performance, public ratification, or complete conformance to Integral.
