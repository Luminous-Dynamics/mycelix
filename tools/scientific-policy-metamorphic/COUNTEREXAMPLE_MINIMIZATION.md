# Authorization Counterexample Minimization v0.1

Status: contract plus a generic deterministic JSON reducer implementation in src/shrink.rs. Engine-integrated Cedar/OPA minimization and Rust-oracle parity remain unimplemented.

## 1. Why minimize?

A 62,464-row corpus and engine-to-engine divergence counts are useful for broad screening. They are not enough to diagnose why two policy implementations disagree. The next output should be a minimal, reproducible counterexample with an auditable reduction trace.

The minimizer's subject is a predicate, not merely a decision. A caller must state what it wants to preserve: Cedar/OPA decision disagreement; determining-policy disagreement; engine evaluation error; a specific Allow-to-Deny transition under a named mutation; or a selected policy mutation that changes a decision.

Freeze the predicate, engine versions, policy/schema digests and original case before shrinking.

## 2. Deterministic shrinking model

Define a finite, ordered set of reductions. Candidate reductions may remove or simplify optional context attributes, unreferenced entities, non-target entity attributes, parent/relationship edges, list elements and nested values, contract-optional request fields, and numeric/string values toward schema-valid boundary representatives.

Never remove or rewrite principal, action, resource, or required schema fields unless that exact field is the target of the preserved predicate. Invalid inputs belong in a separately named malformed-input panel.

Use stable ordering: case ID, path, reduction class, then candidate encoding bytes. Identical inputs, engine build and configuration must yield the same candidate sequence and chosen result. No wall-clock deadline may be used to claim a minimal result.

## 3. Minimality claim

The first implementation should claim deterministic 1-minimality, not a globally smallest counterexample.

A result is 1-minimal only when every reduction in the declared neighborhood was considered; every admissible candidate was evaluated by the frozen predicate; no single remaining reduction preserves the predicate; and the candidate set and outcomes are committed in the trace.

If a reduction is not attempted because of an evaluation error or resource cap, label the result incomplete, not minimal. Record rejected candidates and reasons. Exhaustive global minimum is a different and much more expensive claim.

## 4. Trace and evidence format

Each minimized record includes original corpus revision and case ID, original-input SHA-256, minimized-input SHA-256, engine names/versions/binary hashes/configuration, policy and schema SHA-256, preserved predicate identifier and expected outcome, deterministic reduction ordering/version, ordered reduction trace with before/after digests, candidate/evaluated/rejected counts, incomplete status if applicable, and the final local-minimum declaration with its neighborhood definition.

Never put secrets or production credentials into a corpus, trace, artifact or policy input.

### Implemented generic reducer slice

The follow-on branch now exports a generic deterministic Rust JSON reducer in
`src/shrink.rs`. The caller supplies exact source bytes, the parsed JSON
value, a predicate identifier, protected JSON Pointer paths, an evaluation
budget and a predicate callback. The reducer verifies that the exact bytes
decode to the supplied value, hashes raw source bytes separately from its
sorted-object normalized-value encoding, and records every candidate
evaluation. Its reduction neighborhood and digest-encoding profile are
explicitly versioned.

The unit tests cover source-byte versus normalized-value identity, byte/value
mismatch rejection, deterministic traversal/digests, protected PARC identity,
budget exhaustion, predicate errors and monotone scalar reduction. This is
not yet an engine-integrated Cedar/OPA shrink run; real engine invocation,
policy/schema digest binding and per-case artifacts remain open.

## 5. Semantics and diagnostics

Keep independent dimensions separate:

- final decision (Allow / Deny);
- determining policy IDs;
- policy evaluation errors;
- policies considered or matched;
- branch/condition coverage, only where the engine exposes a documented measurement.

Cedar authorization responses include determining policies and evaluation-error diagnostics; a Deny caused by no satisfied permit/forbid policy can have no determining policy IDs. OPA decision outputs and profiler/coverage information are different observables. Do not fabricate a common field by mapping one engine's profile into another engine's diagnostic vocabulary.

A policy ID in a result is evidence about that engine's recorded reason, not proof the policy is correct or that the source authorization implementation agrees.

## 6. Negative controls

Freeze small independently understood controls before expanding the generator: default deny with no matching permit; explicit permit; forbid wins over permit; missing principal/resource; wrong entity type; duplicate entity UID; unknown action; malformed or missing required attribute; extra irrelevant Actor does not change an unrelated result; and referenced Actor removal changes the target case as specified.

For each control state whether the expected result is a decision change, evaluation error, or input rejection. These categories must not be conflated.

## 7. Benchmark design

Publish separate metrics: reduction ratio (bytes/entities/attributes), predicate evaluations per counterexample, deterministic reproduction rate, 1-minimality completion rate, known-policy-mutation detection by rule/condition, decision divergence, determining-policy divergence, engine-error divergence, default-deny and forbid-precedence sensitivity, cold start, warm in-process evaluation, batch throughput, input conversion, memory and allocations.

For OPA, use its native bench/profiler for in-process measurements and separately record CLI/server overhead. For Cedar, measure the pinned in-process runner and retain diagnostics. Do not compare OPA CLI batch wall time to Cedar in-process time as equivalent units of work.

## 8. Oracle integrity

The actual Mycelix Rust authorization implementation and canonical v1 input/output contract remain required before parity can be claimed. Keep results explicitly pending until authentic source, policy artifacts and a reference contract are recovered. A second Cedar or OPA implementation is not a substitute for the missing Rust oracle.

## 9. Claim ceiling

A completed shrink run establishes that a declared engine/policy predicate is preserved by the recorded deterministic reduction sequence and is 1-minimal in the declared finite neighborhood. It does not establish global minimality, policy correctness, formal equivalence, or parity with Mycelix Rust authorization.
