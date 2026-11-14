# IAE v0.2 vertical-slice Rust evaluator

## Run

```sh
cargo test --manifest-path benchmarks/institutional-assurance-exchange/v0.2/harness/Cargo.toml
cargo run --manifest-path benchmarks/institutional-assurance-exchange/v0.2/harness/Cargo.toml -- \
  benchmarks/institutional-assurance-exchange/v0.2/fixture.json \
  benchmarks/institutional-assurance-exchange/v0.2/oracle.json \
  benchmarks/institutional-assurance-exchange/v0.2/examples/reference-submission.json
```

## What is evaluated

- Exact UTF-8 source byte spans and requirement modality.
- Human-accepted baseline reference and exact requirement set.
- Owner-native claim receipt/source/version cross-links.
- Bitemporal claim selection: effective time and known-at time are evaluated independently.
- Authority status at action time and knowledge time.
- Claim competence for the exact required dimension; custody cannot satisfy title.
- Conditional land-use variance and preference non-blocking semantics.
- Seven query outcomes, including historical replay, current correction, missing title, wrong-dimension source, unrelated evidence, absent preference evidence and independent inspection.
- Metamorphic invariance for unrelated evidence and preference removal.
- Full operation history, UNKNOWN outcome, idempotency identity and blind-retry detection.
- Reviewer trace completeness and fixture-only execution boundaries.

The Rust harness derives each query's disposition, authority status, permitted-use projection and supporting claim/receipt set from the fixture, then compares the system submission to the separate known-answer oracle. It does not merely trust submitted labels.

## Reproducibility receipt

The output report hashes the exact fixture, oracle, submission and evaluator executable. It also records the system build SHA and configuration digest.

## Temporal contract

All timestamps must be canonical UTC strings ending in `Z`; this enables lexical ordering without a date-time dependency. Claim observations are eligible only when `effective_from <= effective_at` and `observed_at <= known_at`. Authority status events are eligible only when `effective_at <= action_at` and `observed_at <= known_at`.

This is the deterministic fixture's selection rule, not a universal interpretation rule for every legal record or jurisdiction.

## Limits

The fixture's baseline/source digests are explicit synthetic placeholders and are not cryptographically verified. The human approval is a synthetic record reference, not authenticated identity or legal approval. The oracle is not independently adjudicated. This evaluator is not a production trust framework, legal evaluator or real transaction connector.
