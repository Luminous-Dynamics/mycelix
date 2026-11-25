# Mycelix Temporal Evidence Kernel

A dependency-free Rust kernel for asking a verifier-relative bitemporal question:

> What was valid at event/valid time **V**, according to the assertions verifier **X** had accepted by knowledge time/frontier **K**?

This is deliberately narrower than a general temporal database. It makes the evidence and epistemic boundary explicit instead of treating one timestamp as a universal truth.

## Current contract

- **Valid time and knowledge time are independent.** Queries provide both (V) and (K).
- **Verifier-relative knowledge.** A fact is eligible only when a `VERIFIER_KNOWLEDGE_AT` assertion references that exact fact, names the queried verifier, and has a timestamp at or before (K).
- **Append-only correction lineage.** A correction supersedes a predecessor; it changes the as-known answer only after the queried verifier has a qualifying knowledge assertion for that correction.
- **Half-open intervals.** Validity is `[VALID_FROM, VALID_TO)`.
- **No inferred open interval.** An absent `VALID_TO` is open-ended only when the caller opts in and there is no end-boundary assertion at all. An existing but unknown end boundary remains indeterminate.
- **Explicit uncertainty.** Contested claims and competing known correction branches do not resolve to a winner. An uncertain boundary yields `Indeterminate` when the query falls within its uncertainty window.
- **UTC-only ordering in v0.1.** Canonical `Z` timestamps with 0 or 1–9 fractional digits are accepted. Numeric offsets are rejected rather than silently normalized. A future version may add a pinned, evidence-preserving clock-normalization profile.
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
```

The example models a revocation effective on November 18, 2026, which a verifier only learns on November 20. A query at the same valid time can therefore be indeterminate at an earlier knowledge time and valid-at-query after the verifier's receipt, without rewriting the historical knowledge state.

## Qualification boundary

This crate is a pure deterministic selector over caller-supplied assertions. It is not yet:

- a Holochain DHT / checkpoint integration;
- a signature, issuer, authority-grant, or source authenticity verifier;
- a clock calibration or cross-clock conversion implementation;
- a transaction manager, accounting authority, or payment executor;
- an OCEL importer/exporter or a certified OCEL implementation;
- a complete bitemporal policy engine.

The caller is responsible for authenticating the assertions, evidence payloads, verifier identity, and knowledge-frontier receipts before constructing the set. Until that adapter is implemented and independently tested, results are **structural query results over supplied data**, not authoritative legal, financial, or operational determinations.

## Benchmark direction

The next benchmark layer should compare this kernel against:

1. a straightforward reference interpreter (semantic oracle);
2. a naive full-scan implementation;
3. an indexed in-memory implementation;
4. selected SQL temporal-history / event-sourcing implementations where the semantics can be matched.

The benchmark must report correctness separately from speed. Required workload dimensions include assertion count, correction-chain depth, number of verifier identities, metadata-to-fact ratio, correction-branch fan-out, event-time disorder, and uncertainty/contestation prevalence. Required outputs include p50/p95/p99 query latency, throughput, memory, index-build cost, bytes scanned, and semantic disagreement. A fast incorrect answer is a failure.

No SOTA or production-performance claim is made by this crate at this stage.
