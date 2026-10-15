# Mycelix Life/Civic Reference Checker

This is an isolated Rust verifier for the research-stage life/civic schemas and reference corpus. It has no network access, no Holochain authority, and no service side effects.

From the repository root with Rust 1.96.1 installed:

```sh
cargo +1.96.1 fmt --manifest-path tools/life-civic-reference-checker/Cargo.toml -- --check
cargo +1.96.1 test --manifest-path tools/life-civic-reference-checker/Cargo.toml --all-targets
```

The tests check JSON Schema validity, a positive working-draft representation, evidence and human-review requirements for `applicable` assessments, pinned Git blob identities, cross-field service claim semantics (completed verification requires captured independent evidence; synthetic/unavailable evidence fails closed), retrieval-state/timestamp consistency (including explicit failed-refresh times), evidence-linked official contact references, and adversarial source-conflict/outcome-verification controls. Synthetic fixture evidence must carry `retrieval_state=synthetic_fixture` and `retrieved_at=null`; real captured sources must cite a retrieval timestamp. The service journey schema remains version 0.3.0; the active service fixture corpus is version 0.6.0 and adds a synthetic control keeping response targets, category turnaround times, and restoration outcomes distinct.

Passing these tests would establish only the tested schema/corpus invariants. It would not verify that any external source is current, determine legal applicability, certify conformance, validate production Rust integration, or show that a civic service outcome occurred.


## Reproducibility boundary

The GitHub workflow checks out the exact pull-request SHA and pins Rust 1.96.1. This standalone crate currently has no checked-in `Cargo.lock`, so Cargo's transitive dependency resolution is not frozen by this repository revision. Do not describe these checks as a fully lockfile-reproducible build until a reviewed lockfile is committed and the workflow uses `--locked`.
