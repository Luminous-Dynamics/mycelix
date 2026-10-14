# Mycelix Life/Civic Reference Checker

This is an isolated Rust verifier for the research-stage life/civic schemas and reference corpus. It has no network access, no Holochain authority, and no service side effects.

From the repository root:

```sh
cargo test --manifest-path tools/life-civic-reference-checker/Cargo.toml
cargo fmt --manifest-path tools/life-civic-reference-checker/Cargo.toml -- --check
```

The tests check JSON Schema validity, a positive working-draft representation, evidence and human-review requirements for `applicable` assessments, pinned Git blob identities, cross-field service claim semantics, retrieval timestamps, and adversarial source-conflict and outcome-verification controls. The active service journey schema and fixtures are version 0.3.0.

Passing these tests would establish only the tested schema/corpus invariants. It would not verify that any external source is current, determine legal applicability, certify conformance, validate production Rust integration, or show that a civic service outcome occurred.
