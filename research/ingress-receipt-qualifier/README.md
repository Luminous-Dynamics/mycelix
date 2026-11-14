# Ingress receipt time qualifier v0.1

A synthetic-only Rust reference evaluator for uncertain ingress-receipt time, owner-issued time qualification, stream sequence conflicts and witnessed-history status.

## Core paradigm shift

A timestamp is not a fact merely because it is a scalar. The input is an interval in a named clock/timebase and epoch, with a distinct owner-issued verification reference. The **decision cutoff is also an interval**, not an assumed perfect point.

For a qualified receipt interval `[r_min, r_max]` and qualified cutoff interval `[c_min, c_max]`:

- definitely available by cutoff: `r_max <= c_min`;
- definitely after cutoff: `r_min > c_max`;
- cutoff-indeterminate: intervals overlap.

A straddling interval is not rounded into eligibility. An unqualified timebase, mapping, verifier result or required history proof cannot be upgraded by this evaluator.

## What this crate implements

- Strict Rust data contracts for source artifact commitment, receiver/stream identity, source clock domain/epoch, source-native time interval and comparison-timebase interval.
- Explicit clock mapping reference when source and comparison timebases or epochs differ; mapping identity, validity range, normalized output interval and owner verification reference must match.
- Time-verification reference bound to the exact receipt-statement digest and required verifier profile.
- Optional sequence identity; conflicting artifact digests under the same stream/epoch/sequence are rejected as a group.
- Duplicate receipt IDs: byte-identical structured receipts collapse idempotently; same ID with different content becomes a conflict.
- Separate history status. A known fork always blocks eligibility; a policy requiring consistent witnessed history excludes inclusion-only, unavailable or indeterminate histories.
- A strict synthetic corpus of 36 frozen cases and a deterministic offline CLI.

The module consumes owner-issued verifier references. It checks reference completeness, profile identity and subject binding. It does **not** perform cryptographic signature verification, validate a time service's clock hardware, verify Merkle proofs, establish witness independence, or authenticate a source. Those remain with #6054, #5899, #5923, #1206 and #1252.

## Run

From the repository root:

```bash
cargo fmt --manifest-path research/ingress-receipt-qualifier/Cargo.toml --all -- --check
cargo test --manifest-path research/ingress-receipt-qualifier/Cargo.toml --all-targets
cargo clippy --manifest-path research/ingress-receipt-qualifier/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/ingress-receipt-qualifier/Cargo.toml --bin ingress-receipt-qualifier
```

No live timestamp provider, broker, transparency log, trust root, rights service or instrument is contacted. All fixtures are synthetic. The highest positive disposition is `eligible_by_cutoff` for the declared synthetic profile; it is not proof that the supplied verification reference is authentic.

## Composition

- Mycelix #6054 owns stream identity, clock domains, boot epochs and sequence replay.
- #5923 owns time-qualified dependency hypergraphs and evidence frontiers.
- #1206 and #1252 own append-only witnessed history and transparency equivocation.
- #5899 owns authenticated federated sensor provenance.
- #5215 owns operation-scoped source permissions and custody.
- #6195 owns parser-derived source bytes and VOEvent replay.

This crate deliberately does not create a new clock service, log, trust root, identity authority, source registry, rights evaluator or dependency graph.
