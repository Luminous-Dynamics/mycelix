# South African Payments Profile Evaluator

A standalone Rust reference evaluator for the Mycelix South African payments interoperability profile.

## Run

From the repository root:

```sh
cargo clippy --manifest-path tools/finance/sa-payments-profile-evaluator/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path tools/finance/sa-payments-profile-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payments-profile-evaluator/Cargo.toml -- --self-test
```

Optional `--corpus PATH` and `--schema PATH` arguments evaluate alternate fixtures. Without those arguments, the tool uses compile-time copies of the canonical checked-in Finance corpus and schema.

## What it checks

- JSON Schema Draft 2020-12 meta-schema and instance validation, with format assertions enabled;
- stable unique source, system, scenario, milestone and allocation IDs;
- source references and pinned observation dates;
- lifecycle guards preventing the known future FPS/RTGS and programme-enabler entries from being silently promoted;
- dated function allocations;
- announced-target versus completed-at-source milestone semantics;
- scenario-category dispositions and dispatch constraints;
- deterministic negative mutations for false finality, QR+ rail promotion, data-purpose misuse, target-date routing, source deletion, and future-capability promotion.

## Result vocabulary

- `PASS`: this exact synthetic corpus passes schema and reference-evaluator checks.
- `FAIL`: a structural or semantic invariant failed.
- `INCOMPLETE`: a source-dated target window elapsed without refreshed official completion evidence.

A PASS is not evidence of provider access, participant eligibility, legal authority, real payment execution, settlement finality, regulatory compliance, or production readiness. The evaluator has no network client, provider SDK, credentials, secrets or effectful command.
