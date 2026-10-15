# Mycelix Botanical Provenance Preflight

A small, dependency-light Rust contract for checking exact lot, recipient, purpose and reviewed-rights references before a material custodian reviews a proposed botanical hand-off.

## Run

```sh
cargo fmt --manifest-path crates/mycelix-botanical-provenance/Cargo.toml -- --check
cargo test --manifest-path crates/mycelix-botanical-provenance/Cargo.toml
cargo clippy --manifest-path crates/mycelix-botanical-provenance/Cargo.toml --all-targets -- -D warnings
```

## Rights-currentness checks

Each rights assessment carries a half-open validity interval in Unix seconds: valid from the inclusive start through, but not including, the exclusive end. The caller supplies an evaluation timestamp, and the preflight blocks malformed windows, decisions that are not yet valid, and decisions expired at the supplied evaluation time. This is an input check only: the contract does not attest to a secure clock, retrieve the canonical rights record, detect a newer revocation by itself, or authenticate evidence bytes. Production integration must resolve exact references and currentness through the responsible authority immediately before any governed hand-off.

## Claim boundary

A result of `PreflightCompleteForCustodianReview` means that the supplied record identities and scopes are structurally consistent with the declared request at the supplied evaluation time. It is not an authorization to move material and does not prove species identity, quality, safety, efficacy, permit validity, consent authenticity, or legal permission. It deliberately omits exact coordinates, personal data and traditional-knowledge content.

The evaluator treats consent and rights as specific to one lot, recipient, purpose, and validity window. A non-commercial research scope cannot imply commercial bioprospecting, export, or use of associated traditional knowledge. A real deployment must connect these references to the responsible governance, evidence and authorization systems.
