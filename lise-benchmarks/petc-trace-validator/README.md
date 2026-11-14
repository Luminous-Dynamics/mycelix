# PETC Trace Validator

Standalone Rust validator for the Protected Effect Trace Contract v0.1.

## Run

```sh
cargo fmt --manifest-path lise-benchmarks/petc-trace-validator/Cargo.toml --check
cargo test --manifest-path lise-benchmarks/petc-trace-validator/Cargo.toml
cargo run --manifest-path lise-benchmarks/petc-trace-validator/Cargo.toml -- trace.json
cargo run --manifest-path lise-benchmarks/petc-trace-validator/Cargo.toml -- trace.json <expected-lowercase-sha256>
```

The validator emits a JSON report and exits 0 only when all structural/event-semantic checks pass. Exit 2 means invalid input, contract violations or digest mismatch.

## Checks implemented

- trace/system/task identity and SHA-256 shape;
- contiguous zero-based event sequence and unique event IDs;
- causal references resolve to earlier events;
- a provider dispatch requires a prior ALLOW preflight bound to the exact operation digest and a provider-contract reference;
- retry requires later reconciliation to confirmed no-effect plus the same operation digest and idempotency key under a qualified provider contract;
- provider-native acknowledgements/status queries cannot be labelled independent effect observation;
- independent-observation claims require the declared independent-observer source and capture boundary;
- duplicate/unexpected protected-action observations are rejected;
- all expected actions without observations are reported as missing, never treated as no effect;
- a reconciliation claiming full application requires observed applied evidence for every expected action.

## Limits

This validator does not authenticate source systems, verify legal authority, formally validate the JSON Schema, determine whether a source is genuinely independent, or prove that the trace inventory is complete. A digest binds bytes, not truth. Provider capability and source-dependency claims still require independent owner review.

The trace contract and validator are a research draft. The 34-case IPB-FINALITY panel is synthetic development material, not a hidden confirmatory holdout. No SOTA, production safety, exactly-once or legal-validity claim is made.
