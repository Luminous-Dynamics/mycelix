# Mycelix Botanical Provenance Preflight

A small, dependency-light Rust contract for checking exact lot, recipient, purpose and reviewed-rights references before a material custodian reviews a proposed botanical hand-off.

## Run

```sh
cargo test --manifest-path crates/mycelix-botanical-provenance/Cargo.toml
cargo fmt --manifest-path crates/mycelix-botanical-provenance/Cargo.toml -- --check
cargo clippy --manifest-path crates/mycelix-botanical-provenance/Cargo.toml --all-targets -- -D warnings
```

## Claim boundary

A result of `PreflightCompleteForCustodianReview` means that the supplied record identities and scopes are structurally consistent with the declared request. It is not an authorization to move material and does not prove species identity, quality, safety, efficacy, permit validity, consent authenticity, or legal permission. It deliberately omits exact coordinates, personal data and traditional-knowledge content.

The evaluator treats consent and rights as specific to one lot, recipient and purpose. A non-commercial research scope cannot imply commercial bioprospecting, export or use of associated traditional knowledge. A real deployment must connect these references to the responsible governance, evidence and authorization systems.
