# Claim Policy Performance and Comparative Qualification V1

**Status:** benchmark harness and measurement protocol; no performance superiority is claimed.  
**Scope:** the Rust claim-ceiling reference evaluator, its embedded JSON Schema validation path, and policy lifecycle reference checks.  
**Baseline source:** lifecycle core qualification head 902d5b0896ce500e886b61db08d8160ba786bd3c.  
**Candidate source:** this follow-on branch; exact candidate SHA must be captured from the qualification run.

## 1. Engineering slice

The shared JSON-schema helper previously parsed and compiled each schema on every validation call. The helper is used by the meet, policy-fit and lifecycle evaluators, including receipt validation.

This candidate caches compiled validators by the **exact embedded schema bytes**:

- untrusted request/receipt instances are still parsed and validated on every call;
- byte-distinct schemas cannot share a cache entry, even when they are semantically equivalent JSON;
- only schema byte slices supplied by the crate's embedded schema constants should be passed to this private helper;
- the steady-state cache uses a read/write lock so concurrent cache hits do not serialize on a write mutex;
- first-use compilation is serialized and the cache is process-local; it is not a schema registry or durable trust store.

The new unit test asserts that exact schema bytes reuse the same compiled validator and that a byte-distinct schema receives a distinct validator. The Criterion harness warms the embedded schemas and measures the complete lifecycle evaluator against all frozen lifecycle cases. Hosted compile/test/Clippy and benchmark artifacts are required before this change is called qualified. No timing result is asserted in this document.

## 2. How to execute the benchmark

From the repository root, on the package's supported Rust toolchain:

    PACKAGE=mycelix-workspace/mycelix-space/tools/claim-ceiling-reference/Cargo.toml
    cargo generate-lockfile --manifest-path "$PACKAGE"
    cargo bench --manifest-path "$PACKAGE" --bench policy_lifecycle -- --noplot

The benchmark currently covers the 20 frozen lifecycle cases on a single evaluator thread after warming the schema cache. It measures complete evaluator latency, not only schema compilation. Criterion output is useful for comparing the same workload across exact source revisions but is not a substitute for a production request-trace benchmark or reliable p99 service latency.

For an attributable before/after comparison:

1. Use the same benchmark source, Rust toolchain (rustc -Vv), target triple, host class, CPU governor/power mode, and dependency lock.
2. Run baseline and candidate on the same otherwise idle host, alternating their order, for at least five independent process runs.
3. Preserve raw Criterion output, exact source and fixture SHA-256 digests, rustc -Vv, cargo -V, OS/kernel, CPU model, and run command.
4. Report each run's estimate and variation plus the median across independent runs. Do not pool a few runs into a false-precision confidence interval.
5. Benchmark cold start separately from warmed evaluation. Do not infer a cold-start cost from the warmed Criterion group.
6. Add a separate multi-threaded load harness before making concurrency or p95/p99 claims. Measure lock contention and allocation/RSS under 1, 8, 32, and 128 concurrent evaluators. A lower single-thread median can still regress under contention.

The checked-in benchmark is initially a measurement instrument, not a latency gate. Hosted-runner performance is too noisy to become a hard pass/fail threshold until a stable runner profile and repeated historical baseline exist.

## 3. Qualification ladder

| Gate | Required evidence | Current status |
|---|---|---|
| Schema-cache correctness | Exact-byte reuse test, distinct-byte non-aliasing test, unchanged frozen semantic corpus | Implemented in candidate; exact-head hosted result pending |
| Correctness regression | Format, all Rust tests, strict Clippy, independent jq lifecycle oracle, deterministic CLI replay | Must pass on the exact candidate SHA |
| Warm-path performance | All 20 cases, Criterion raw data and provenance bundle | Harness implemented; measurements pending |
| Cold-start performance | Fresh-process first evaluation, including both descriptor and receipt schema compilation | Not yet implemented |
| Contended performance | Multi-threaded harness, throughput and per-request latency distribution | Not yet implemented |
| Resource behavior | Peak RSS, allocations where measurable, bounded input, memory under concurrency | Follow-up |
| Long-running adversarial assurance | At least 100,000 reproducibly seeded stateful mutations plus retained minimized failures; 24 CPU-hours of fuzzing per release candidate as an initial target | Proposed target, not executed |
| Storage safety | Atomic state-update / crash / restart / multi-process race qualification | Not implemented by this performance slice |
| Consumer boundary | A lifecycle receipt alone cannot admit evidence or authorize an operation | Existing lifecycle receipt carries both grants as false; no production consumer qualification is claimed |

## 4. Benchmarks and acceptance targets

Security and performance must be evaluated separately. A faster fail-open evaluator is a regression.

### Semantic/security targets

- 100% agreement with frozen positive and negative cases and the independent oracle.
- Zero accepted rollback, equivocation, scope substitution, invalid publisher metadata, stale revocation epoch, or missing-trusted-time cases in the frozen and generated corpora.
- At least 100,000 seeded stateful cases per candidate, with fixed published seeds and minimized counterexamples retained.
- Initial target of 24 CPU-hours fuzzing per release candidate; record crashes, hangs, OOMs, corpus digest, toolchain, and exact source SHA.
- Every rejected release proposes no next policy head and causes no durable state change.
- A valid lifecycle result remains insufficient for evidence admission or action authority.

Zero failures in a generated corpus is statistical evidence, not a proof that false accepts are impossible.

### Performance targets (hypotheses, not results)

- Schema compilation count: once per unique embedded schema byte sequence per process, not once per validation call.
- Report warm and cold p50/p95/p99/max only from a purpose-built distribution harness with sufficient samples; report Criterion estimates separately.
- Measure throughput, CPU utilization, RSS/peak memory, allocation counts where available, input sizes, and concurrency.
- Track evaluator-only cost, authenticated distribution verification cost, persistent compare-and-update cost, and whole-chain cost separately.
- Keep the proposed pure-evaluator p95 target of under 1 ms for 10,000 prevalidated policy scopes as an explicit experiment hypothesis, not a production SLO, until a representative hardware/workload baseline exists.
- Set no hard performance gate until repeated results establish noise bounds. Adopt an optimization only when correctness is unchanged and repeated comparisons demonstrate a meaningful improvement.

## 5. Fair comparison with established systems

The current reference evaluator is not itself a general-purpose authorization engine. Comparing its latency directly with a Cedar or OPA authorization query would be misleading. Use three distinct workloads:

1. **Policy validation and evaluation:** translate identical representative allow/deny and malformed-policy cases into each engine's native model. Compare cold preparation/validation, warm decision latency, throughput and memory. Preserve semantic differences in the published protocol.
2. **Authenticated distribution:** compare update/metadata verification separately. TUF defines trust roles, thresholds, expiry, snapshot consistency, root rotation and metadata version rules to resist rollback, freeze, mix-and-match, key-compromise and wrong-target attacks. Mycelix should consume a TUF-compatible verifier rather than rebuild these mechanisms.
3. **Mycelix-specific lifecycle and evidence gates:** evaluate identity/audience, profile/schema pin, trusted time, revocation epoch, monotonic policy head, epistemic claim-ceiling fit, and the independent consumer boundary. Cedar/OPA are comparison points for general policy tooling; they are not substitutes for these domain semantics.

Useful reference practices:

- Cedar schema-based policy validation and its verification-guided approach: https://docs.cedarpolicy.com/policies/validation.html
- OPA signed bundles, out-of-band configured verification keys, and activation failure behavior: https://www.openpolicyagent.org/docs/management-bundles
- OPA's documented measurement guidance: https://www.openpolicyagent.org/docs/policy-performance
- TUF v1.0.36 specification (published August 5, 2026): https://theupdateframework.github.io/specification/v1.0.36/

Publish a workload adapter, input corpus digests, tool versions and raw results before making any “better than SOTA” claim. A meaningful claim should identify the exact workload and show comparable semantics, security properties, and operating costs.

## 6. Paradigm shift: a policy promotion transaction

The defensible architecture is not one larger boolean. It is a transaction with separately qualified inputs and an atomic promotion boundary:

    TUF-compatible authenticated metadata
      -> exact policy target + trusted freshness/revocation context
      -> Mycelix lifecycle evaluation
      -> conservative claim-ceiling policy-fit evaluation
      -> durable atomic compare-and-advance of policy head
      -> consumer-specific evidence admission
      -> consumer-specific action authorization

Each transition must carry exact subject/profile/source identities and an independently checkable receipt. A failure or indeterminate state must not promote the durable head. A local database/file commitment detects some corruption but does not prevent an attacker from restoring an older snapshot of both the data and its commitment. If that threat is in scope, anti-rollback state needs an independent anchor or a separately qualified hardware/remote monotonic state boundary.

The next storage tranche should test: two concurrent writers with the same prior head; stale expected head; exact retry; rejection without writes; crash before/after temp-file sync and atomic replacement; corruption; restart; scope separation; and multi-process execution. Tests must distinguish process-level locking from actual power-loss guarantees.

The next consumer tranche should attempt to turn lifecycle acceptance and policy fit into evidence admission or action authorization without the corresponding consumer-specific evidence/authority prerequisites. Both attempts must be rejected, including when the receipt is well-formed and its digests are valid.

## 7. Nonclaims

This candidate does not authenticate signatures, integrate a TUF client, obtain authoritative time/revocation state, persist anti-rollback heads, qualify canonical evidence admission, or grant operational authority. The cache optimization is not yet shown to improve measured latency. No comparison proves Mycelix superior to Cedar, OPA, or another industry system. Those claims require exact-head execution, repeatable measurements, comparable workloads, and independent review.
