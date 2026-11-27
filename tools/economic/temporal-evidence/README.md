# Mycelix Temporal Evidence Kernel

A dependency-free Rust kernel for asking a verifier-relative bitemporal question:

> What was valid at event/valid time **V**, according to the assertions verifier **X** had accepted by knowledge time **K**?

This is deliberately narrower than a general temporal database. It makes the evidence and epistemic boundary explicit instead of treating one timestamp as a universal truth. The follow-on design for authenticated, verifier-relative causal frontiers is tracked in [Issue #6680](https://github.com/Luminous-Dynamics/mycelix/issues/6680) and [VERIFIER_RELATIVE_FRONTIER_QUERY_V0.1.md](VERIFIER_RELATIVE_FRONTIER_QUERY_V0.1.md).

## Current contract

- **Valid time and knowledge time are independent.** Queries provide a valid-time instant V and a verifier-knowledge instant K.
- **Verifier-relative knowledge.** A fact is eligible only when a `VERIFIER_KNOWLEDGE_AT` assertion references that exact fact, names the queried verifier, and has a timestamp at or before K. Its opaque `knowledge_frontier_ref` is preserved but not resolved or authenticated by this v0.1 kernel.
- **Append-only correction lineage.** A correction supersedes a predecessor; it changes the as-known answer only after the queried verifier has a qualifying knowledge assertion for that correction.
- **Half-open intervals with episode identity.** Validity is `[VALID_FROM, VALID_TO)`. Temporal envelope v1.1 requires one `validity_interval_ref` on every VALID_FROM/VALID_TO assertion; endpoints and corrections cannot cross episodes. The kernel exposes `evaluate_validity_interval_at` on scan, index and prevalidated-scan engines. A missing VALID_TO is Indeterminate: a caller flag alone cannot assert OPEN.
- **No inferred open interval.** An absent `VALID_TO` is open-ended only when the caller opts in and there is no end-boundary assertion at all. An existing but unknown end boundary remains indeterminate.
- **Explicit uncertainty.** Contested claims and competing known correction branches do not resolve to a winner. Estimated and uncertain assertions require explicit uncertainty windows. A query inside an uncertain valid-time or knowledge-time boundary yields `Indeterminate`.
- **UTC-only ordering in v0.1.** Canonical `Z` timestamps are accepted, including 1–9 fractional digits for query bounds. Assertion precision must match the declared day, minute, second, millisecond, microsecond, or nanosecond granularity. Numeric offsets are rejected rather than silently normalized. A future version may add a pinned, evidence-preserving clock-normalization profile.
- **Digest shape.** Evidence digest metadata must be a lower-case `sha256:` plus 64 hexadecimal characters. This version validates the digest's syntax only; it does **not** fetch evidence bytes, recompute the digest, authenticate the source, or verify a signature.
- **No owner-sequence substitution.** Owner-local sequence/frontier information remains a separate causal-order structure; it is never interpreted as elapsed time.

## Run

From this directory:

```sh
rustup toolchain install 1.96.0 --profile minimal
cargo +1.96.0 fmt --all -- --check
cargo +1.96.0 test --locked
cargo +1.96.0 clippy --locked --all-targets -- -D warnings
cargo +1.96.0 run --locked --example historical_revocation
cargo +1.96.0 run --locked --example benchmark_query_index
```

The historical example models a revocation effective on November 18, 2026, which a verifier only learns on November 20. A query at the same valid time can therefore be indeterminate at an earlier knowledge time and valid-at-query after the verifier's receipt, without rewriting the historical knowledge state. The benchmark example constructs a prevalidated scan baseline and a separately prevalidated indexed engine, checks exact result parity, and prints validation/build/query microtimings. This avoids charging the scan query for full validation on every request while the indexed path validates once. It is a smoke benchmark, not an industry comparison or a published performance result.

## Qualification boundary

The validating `TemporalEvidenceSetV1` methods are retained as a semantic reference. `TemporalEvidenceScanV1` validates once and performs full scans; `TemporalEvidenceIndexV1` validates once and builds deterministic claim/dimension, assertion-ID, and fact/verifier indexes for repeated queries. Both paths operate on caller-supplied assertions. It is not yet:

- a Holochain DHT / checkpoint integration;
- a signature, issuer, authority-grant, or source authenticity verifier;
- a clock calibration or cross-clock conversion implementation;
- a transaction manager, accounting authority, or payment executor;
- an OCEL importer/exporter or a certified OCEL implementation;
- a complete bitemporal policy engine.

The caller is responsible for authenticating the assertions, evidence payloads, verifier identity, and knowledge-frontier receipts before constructing the set. In this v0.1 implementation, `knowledge_frontier_ref` is an opaque provenance reference; it is not resolved or authenticated. The `knowledge_frontier_ref` is currently an opaque provenance reference: v0.1 filters on the verifier-knowledge timestamp K and does not prove that a receipt is included in a particular distributed checkpoint or causal frontier. Until those adapters are implemented and independently tested, results are **structural query results over supplied data**, not authoritative legal, financial, or operational determinations.

## Benchmark direction

See [BENCHMARK_PROTOCOL_V0.1.md](BENCHMARK_PROTOCOL_V0.1.md) for matched SQL Server, XTDB, OCEL, reference-scan, and preindexed comparisons. The included benchmark example is only an initial parity-and-microtiming smoke test.

The full benchmark should compare this kernel against:

1. a straightforward reference interpreter (semantic oracle);
2. the prevalidated full-scan engine;
3. the preindexed in-memory engine;
4. selected SQL temporal-history / event-sourcing implementations where the semantics can be matched.

The benchmark must report correctness separately from speed. Required workload dimensions include assertion count, correction-chain depth, number of verifier identities, metadata-to-fact ratio, correction-branch fan-out, event-time disorder, and uncertainty/contestation prevalence. Required outputs include p50/p95/p99 query latency, throughput, memory, index-build cost, bytes scanned, and semantic disagreement. A fast incorrect answer is a failure.

No SOTA or production-performance claim is made by this crate at this stage.
