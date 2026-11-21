# aero-adapter-run-receipt

A small Rust verifier for content-bound AeroCommons adapter-run receipts.

## Run

```sh
cargo test --manifest-path tools/aero-adapter-run-receipt/Cargo.toml
```

## API

- `parse_receipt_json`: strict nested JSON admission and structural validation.
- `compute_receipt_digest`: versioned digest over the canonical receipt body.
- `validate_receipt`: identity, status, rights-declaration, network-accounting and digest checks.
- `verify_artifact_bytes`: exact byte length and SHA-256 check for one artifact.
- `verify_artifact_payloads`: exact-set verification for all declared input, output, executable, argument-vector and environment-profile bytes.

This crate consumes the shared `aero-evidence-types::ArtifactRef`; it does not introduce a second evidence-reference type.

## Limits

The verifier parses the argument vector as JSON and cross-checks a strict environment-profile manifest against locale/timezone/network-policy metadata. It does not execute adapters, instrument network traffic, verify signatures, authenticate publishers, validate legal rights, parse AP242/QIF or establish semantic correctness. Argument and environment-profile artifacts must contain non-secret descriptors only. A valid receipt is not proof of execution. A successful run is only eligible for semantic comparison; the result of #6349 remains a separate evaluation.
