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

The digest is an integrity check, **not** proof that the requester's statements or source claims are true. A hash also does not authenticate a producer without an independently verified signature/key binding.

## Fixture census v0.1

The initial manifest contains nine cases: one minimal accepted request; four schema-rejection controls (false fulfillment, unapproved public summary, consent without a record, and an authorized state without an authorization reference); one duplicate-ID semantic rejection; one digest-tampering rejection; and malformed/duplicate-key parser rejections.

The corpus is a contract smoke test, not a statistically powered benchmark, not a model-quality measure, and not evidence of real-world wish fulfillment. Additional property-based, fuzz, lifecycle-transition, authorization-boundary and blinded holdout tests are required before production use.

## Evidence boundary

This checker and fixtures are research-only. The active environment did not have a Rust compiler or Cargo available when this slice was prepared, so the Rust crate has not been compiled or executed locally. Hosted exact-head checks must be observed separately; queued, pending, skipped or absent checks are not passes. No Cargo.lock has been generated yet.
