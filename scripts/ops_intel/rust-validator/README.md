# CrossDomainDisruptionV1 Rust fixture validator

This isolated Cargo workspace provides a Rust-first structural preflight for the
public synthetic fixture package associated with Mycelix #2942 / #4907.

## Run

From the repository root:

```sh
cargo test --manifest-path scripts/ops_intel/rust-validator/Cargo.toml
cargo run --manifest-path scripts/ops_intel/rust-validator/Cargo.toml
```

An alternate fixture directory may be supplied as the CLI's single argument.

## Scope

The validator rejects duplicate JSON object keys, trailing data, malformed JSON,
and inputs exceeding declared size/shape limits. It checks source/artifact/
subject references, frontier ancestry and timestamps, named supplier-facility
coverage, temporal-reconciliation restrictions, simulation-only attempt/effect
separation, mock-permit negative controls, protected-field omission, predicate
and mutation identifiers, and forbidden evaluator-only key names.

The isolated Cargo workspace keeps this tooling crate from becoming an implicit
member of Mycelix's broader product workspace.

## Nonclaims

This is **fixture-package structural preflight only**. It does not implement the
OPS-INTEL-001 canonical byte protocol, produce projection identity hashes,
validate operational truth, evaluate forecast quality, generate decisions, or
grant execution authority. It is not the independent semantic oracle specified
by #2950: that verifier must use a separate implementation path and independently
check the frozen protocol once #2946's byte-level rules are approved.
