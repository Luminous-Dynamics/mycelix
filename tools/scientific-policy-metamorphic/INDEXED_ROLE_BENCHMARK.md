# Indexed Rego Role Representation Benchmark v0.1

Status: first executable representation transform, separate Rego policy, full-corpus equivalence gate and eight-action native benchmark workflow are implemented on this branch. Exact-head workflow measurements remain pending; no performance gain or CI pass is claimed.

## 1. Hypothesis

The baseline policy tests membership by iterating an array of role strings. The indexed variant uses an object whose keys are role identifiers and whose values are exactly true.

For unique role identifiers, object-key membership may reduce repeated linear array scans. It may also increase input bytes, projection cost, and serialization overhead. This is an empirical hypothesis, not an assumed optimization.

The authoritative source input remains unchanged. The projection is a versioned derivative used only for a separate Rego performance experiment.

## 2. Profile

The mycelix-rego-role-index-v0.1 profile maps an actor object with roles ["contributor", "editor"] to an actor object with role_index {"contributor":true,"editor":true}, preserving other fields.

The transform rejects non-array role values, non-string or empty roles, duplicate roles and pre-existing role_index fields. It preserves every other input field. The output JSONL manifest binds the exact source engine-input SHA-256, projected JSONL SHA-256, row counts, profile ID and one input digest per action representative.

The source input SHA is checked before processing. Each source JSONL line uses the duplicate-aware strict parser. Case IDs must be unique. Exactly 62,464 rows and all eight known actions are required.

## 3. Functional qualification

1. Generate the baseline 62,464 paired inputs from the pinned source corpus.
2. Generate the indexed projection from the exact source engine-input JSONL and its manifest SHA.
3. Evaluate the full indexed batch through OPA 1.22.0 using opa-policy-indexed.rego.
4. Compare indexed outputs against the baseline OPA outputs by case ID.
5. Require exact case-set identity, zero decision divergence and zero determining-rule-ID divergence.

This is Rego representation equivalence over the frozen input panel. It does not establish Cedar equivalence or authentic Mycelix Rust authorization parity.

## 4. Performance qualification

For each of the eight actions, the workflow selects the first source-corpus representative. The indexed projection generator selects the same first occurrence in the same JSONL order; CI checks the case ID. It also checks the generated indexed-input SHA-256 against its manifest.

OPA's native opa bench --benchmem --metrics --count=5 runs against:
- the original role-array input and original Rego policy;
- the indexed role-object input and indexed Rego policy.

Record raw JSON for every action and both representations. Report the query distributions and allocation/byte metrics emitted by OPA. Do not compare separate action cases or aggregate inputs with different policy branches.

The workflow separately records:
- projection-generation elapsed time;
- indexed JSONL bytes vs original JSONL bytes;
- full-corpus one-shot indexed OPA evaluation time;
- native repeated-query benchmark output;
- policy, source, projected input, binary and host digests.

This separates query-only improvement from projection and transport costs. The native query result is not by itself an end-to-end throughput claim.

## 5. Target criteria

- Zero decision and determining-rule divergence across all 62,464 inputs.
- Zero duplicate/invalid-role acceptance in the transform unit panel.
- All eight action representatives bound to exact source case IDs and SHA-256 digests.
- Raw native OPA benchmark outputs published for baseline and indexed variants.
- Report p50/p95/p99 where the native benchmark output supports them; retain raw five-run samples and allocation/byte results.
- Compare indexed and baseline distributions per action. A candidate optimization is retained only when it improves a relevant latency or allocation metric beyond observed run noise without material regression on other actions, and when its projection overhead is reported.

No fixed nanoseconds/op target is asserted before the first pinned benchmark is available.

## 6. Security and semantic limits

A role index is a derived cache, not an authority source. Duplicate role identifiers must be rejected rather than silently collapsed into a set. False values are not generated. The source role array remains the only canonical input representation and its exact bytes remain bound to the output manifest.

Do not move the projection into production authorization until the policy contract defines canonical handling for duplicate roles, unknown roles, roles introduced by new schema versions, and every code path that consumes actor roles. This experiment does not change Cedar input shape or the absent Rust authorization oracle.

## 7. Claim ceiling

A completed qualification establishes that the indexed Rego representation produces the same decisions and rule identifiers as the baseline policy over the frozen 62,464-row panel and records matched OPA benchmark measurements on the selected representatives. It does not prove policy correctness, Cedar equivalence, Rust parity, full-system authorization safety or a general speedup on other workloads.
