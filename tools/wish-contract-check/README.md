# Wish contract checker (research prototype)

This isolated Rust crate validates the Mycelix Wish-to-Outcome (W2O) request contract.

## Run

```sh
cargo fmt --manifest-path tools/wish-contract-check/Cargo.toml -- --check
cargo test --manifest-path tools/wish-contract-check/Cargo.toml
cargo run --manifest-path tools/wish-contract-check/Cargo.toml
```

## What it checks

- The embedded request schema validates against the JSON Schema Draft 2020-12 meta-schema.
- Date/time formats are checked rather than treated as annotations.
- Every file in the versioned fixture manifest has the declared expected disposition.
- JSON input with duplicate object keys is rejected before schema validation or digest computation.
- Required stable IDs in success criteria, constraints and resource needs are unique.
- `provenance.content_sha256` matches SHA-256 over RFC 8785 canonical JSON with only that digest field omitted.
- Fixture paths must be relative and may not traverse outside the fixture directory.
- The benchmark preregistration manifest must account for 120 synthetic episodes across 12 unique strata, 12 unique policy/control arms, matched-budget design invariants, zero-valued hard safety gates and an evaluator-only oracle.

Money budgets are encoded as exact decimal strings, not floating-point JSON numbers; a real action boundary must additionally validate precision against a pinned currency metadata version. The digest is an integrity check, **not** proof that the requester's statements or source claims are true. A hash also does not authenticate a producer without an independently verified signature/key binding.

## Fixture census v0.1

The manifest contains twelve cases: two accepted states (a minimal draft and a valid evidence-backed fulfilled request); schema-rejection controls for false fulfillment, a verified status without outcome receipts, unapproved public summary, consent without a record, an authorized state without an authorization reference, and a floating-point money value; one duplicate-ID semantic rejection; one digest-tampering rejection; and malformed/duplicate-key parser rejections.

The twelve request fixtures are a contract smoke test, not the 120-episode policy benchmark, not a statistically powered impact study, not a model-quality result, and not evidence of real-world wish fulfillment. The manifest declares scenarios and arms; this checker does not yet execute policy evaluations or generate 1,440 arm/episode result rows. Additional property-based, fuzz, lifecycle-transition, authorization-boundary and blinded holdout tests are required before production use.

## Evidence boundary

This checker and fixtures are research-only. The active environment did not have a Rust compiler or Cargo available when this slice was prepared, so the Rust crate has not been compiled or executed locally. Hosted exact-head checks must be observed separately; queued, pending, skipped or absent checks are not passes. No Cargo.lock has been generated yet.
