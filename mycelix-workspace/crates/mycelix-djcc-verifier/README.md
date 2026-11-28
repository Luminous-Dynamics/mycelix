# mycelix-djcc-verifier

A dependency-free Rust structural candidate verifier for DJCC v0.1.

## Run

From the repository root:

```sh
cargo +1.96.0 fmt --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml -- --check
cargo +1.96.0 test --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets
cargo +1.96.0 clippy --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets -- -D warnings
```

## Verified by this crate

- Eight required stage identifiers and the fixed stage dependency graph.
- Selected composition profile identity/digest and a common scope binding.
- Producer owner/version identity and source-commit syntax.
- Exact consumer input name/digest/schema binding to producer output.
- Canonical ordering of stage dependencies, input names and reason codes.
- Stage disposition/output digest presence consistency.
- Profile-scoped `NotApplicable`.
- No established root claim when a stage's corresponding ceiling is not established.

## Not verified by this crate

The crate does not authenticate stage producers, verify signatures or trusted time, reconstruct owner universes, authenticate transparency tree heads, prove inclusion/consistency, parse the JSON certificate, evaluate policy semantics, establish law/authority, execute an effect, verify IVC proofs, or adjudicate physical outcomes.

The byte values in unit tests are synthetic symbolic fixtures. This is a structural candidate, not a cryptographic trust implementation.
