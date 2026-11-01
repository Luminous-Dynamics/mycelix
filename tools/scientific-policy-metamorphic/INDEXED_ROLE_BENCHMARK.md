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


## Input resource boundary

The projection generator opens the source JSONL first, checks regular-file
metadata on that same handle, and reads at most 256 MiB plus one byte. The
actual byte count is checked before SHA-256 validation, parsing or output
generation. The boundary test accepts exactly 256 MiB only through the same
bounded-reader helper semantics (the unit test uses a small exact-limit
fixture) and rejects the first byte over the configured cap. This limits
resource consumption and closes the path-stat/open replacement gap; it does
not authenticate the file producer or replace the source manifest digest.

## 3. Functional qualification

1. Generate the baseline 62,464 paired inputs from the pinned source corpus.
2. Generate the indexed projection from the exact source engine-input JSONL and its manifest SHA.
3. Evaluate the full indexed batch through OPA 1.22.0 using opa-policy-indexed.rego.
4. Compare indexed outputs against the baseline OPA outputs by case ID.
5. Require exact case-set identity, zero decision divergence and zero determining-rule-ID divergence.

This is Rego representation equivalence over the frozen input panel. It does not establish Cedar equivalence or authentic Mycelix Rust authorization parity.

## 4. Performance qualification

The workflow retains the original first-occurrence input for each of the eight actions as a smoke panel. It then runs a wider deterministic, stratified panel built from the source generator's explicit semantic dimensions:

- role cardinality: 0, 1–2, 3–4, 5+;
- individual role identity: contributor, reviewer, editor, institution, service, migration_service and registry_admin, each represented as present/absent;
- claim stream: absent or present;
- actor/claim relationship: not applicable, owner, non-owner;
- acting/creator organization relationship: absent, equal, different, or present without a creator organization;
- attestation kind: ordinary or independent replication;
- target attestation: absent, authored by the actor, or authored by another actor.

Each action retains the single lowest SHA-256-ranked case ID per populated stratum. A greedy set-cover selector then chooses the next stratum that covers the most not-yet-seen marginal values, with SHA-ranked tie breaks. Each action must cover all 32 required marginal values across the six original dimensions plus seven role-identity flags. This produces 24–32 matched cases per action (192–256 total); the manifest records the exact stratum, case ID, rank, paths, byte lengths and SHA-256 digests for both representations. The sample profile and rank inputs are versioned. The selector operates only on the frozen 62,464 source-pinned corpus; it does not infer expected decisions.

For every selected case, CI:
1. verifies the baseline and indexed input bytes against the sample manifest;
2. invokes OPA's native \`opa bench --benchmem --metrics --count=5\` using the original array policy and original input;
3. invokes the same command using the indexed policy and the derived input;
4. preserves both raw JSON outputs and the complete paired sample manifest.

OPA's native benchmark reports repeated query evaluation; policy loading, parsing, compilation and process setup are excluded. The per-case raw outputs must remain paired by case ID and stratum. Do not compare different cases or action branches as though they were the same workload.

The workflow separately records:
- projection-generation elapsed time;
- indexed JSONL bytes vs original JSONL bytes;
- full-corpus one-shot baseline and indexed OPA evaluation time;
- the eight-case smoke panel;
- the 192–256-case stratified native query panel (24–32 per action);
- policy, source, projected input, binary and host digests.

This separates query-only improvement from projection and transport costs. A query-level win is not an end-to-end throughput claim; the added projection, serialization, network/storage bytes and cold-start path must still be included in any production evaluation.

## 5. Target criteria

- Zero decision and determining-rule divergence across all 62,464 inputs.
- Zero duplicate/invalid-role acceptance in the transform unit panel.
- All eight smoke representatives bound to exact source case IDs and SHA-256 digests.
- At least 24 and at most 32 stratified samples for every action, covering all 32 required marginal dimension values including present/absent membership for each of the seven named roles, with exact source and projected hashes.
- Raw native OPA benchmark outputs published for both representations for every matched sample.
- Report p50/p95/p99 where the native benchmark output supports them; retain raw five-run samples and allocation/byte results.
- Compare baseline and indexed distributions within each matched action/stratum. A candidate optimization is retained only when relevant latency/allocation metrics improve beyond observed noise without material regression across the represented strata, and the projection overhead is separately reported.

No fixed nanoseconds/op target is asserted before the first pinned benchmark is available.

## 6. Security and semantic limits

A role index is a derived cache, not an authority source. Duplicate role identifiers must be rejected rather than silently collapsed into a set. False values are not generated. The source role array remains the only canonical input representation and its exact bytes remain bound to the output manifest.

Do not move the projection into production authorization until the policy contract defines canonical handling for duplicate roles, unknown roles, roles introduced by new schema versions, and every code path that consumes actor roles. This experiment does not change Cedar input shape or the absent Rust authorization oracle.

## 7. Claim ceiling

A completed qualification establishes that the indexed Rego representation produces the same decisions and rule identifiers as the baseline policy over the frozen 62,464-row panel and records matched OPA benchmark measurements on the selected representatives. It does not prove policy correctness, Cedar equivalence, Rust parity, full-system authorization safety or a general speedup on other workloads.
