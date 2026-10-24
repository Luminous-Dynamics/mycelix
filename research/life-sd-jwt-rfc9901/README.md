# LIFE-SD-JWT-RFC9901-001 — Cross-implementation published disclosure vectors

This is the first executable bridge from the synthetic LIFE privacy studies to real SD-JWT implementation behavior. It runs published disclosure examples from [IETF RFC 9901 §5.1](https://datatracker.ietf.org/doc/rfc9901/) through two independently maintained Rust crates, checks their SHA-256 base64url digests against the RFC, and compares their parser behavior for the object-property and array-element forms.

## Frozen dependencies

- `affinidi-sd-jwt = 0.1.5`
- `reallyme-sd-jwt = 0.4.1`
- Rust toolchain: 1.96.0
- Edition: 2024

The Affinidi dependency is pinned to 0.1.5, a published crates.io release (rather than the newer unreleased `0.1.8` version visible in the upstream repository manifest). The first workflow invocation generates a dependency lockfile and retains it as an artifact so it can be reviewed and committed as a follow-up before this benchmark is treated as reproducibly pinned. A passing bootstrap run is not yet the final locked qualification.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-sd-jwt-rfc9901/Cargo.toml --all -- --check
cargo +1.96.0 test --manifest-path research/life-sd-jwt-rfc9901/Cargo.toml
cargo +1.96.0 clippy --manifest-path research/life-sd-jwt-rfc9901/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --manifest-path research/life-sd-jwt-rfc9901/Cargo.toml
```

The executable emits a deterministic CSV report with each expected digest, both implementation outputs, and a per-vector comparison. Tests also assert published salts, claim names, disclosure kinds, scalar values and full address fields in both implementations, so matching digests cannot conceal a mislabeled or semantically divergent fixture. Parser negative controls also feed malformed JSON, invalid array lengths, and non-string salt/name fields into both implementations. The CI workflow asserts its checkout is the exact subject SHA, runs strict formatting/tests/Clippy, compares repeated reports byte-for-byte, and retains the report, generated lockfile, manifest, and SHA-256 receipt.

## Scope and claim ceiling

Passing these tests establishes only that these exact crate versions parse five published disclosure examples and agree on the specified digest values and two disclosure shapes. It does **not** establish full RFC 9901 conformance or validate issuer signatures, key-binding signatures, nonce/audience/replay behavior, protocol profiles, privacy guarantees, performance, or SOTA superiority.

The next gates are: commit and pin the generated lockfile; add official negative vectors for duplicate/unissued disclosures, digest mismatch, malformed disclosure JSON, and key-binding-required policy; verify the examples through complete issuer/holder/verifier flows; then test OpenID4VP/HAIP conformance separately before comparing linkage and attribute-inference attacks on actual presentations.
