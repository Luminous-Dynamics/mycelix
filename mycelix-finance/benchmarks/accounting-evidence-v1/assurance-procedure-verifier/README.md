# FIN-ASSURE-PROCEDURE verifier

Standalone, dependency-light Rust reference evaluator for the public synthetic risk/assertion/procedure/evidence corpus.

Run from this directory:

```sh
cargo test --locked
cargo fmt --all -- --check
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-assure-procedure-report.json
```

The CLI supports `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out`.

This is public synthetic conformance, not blind model evaluation, audit assurance, ISA compliance or a professional audit conclusion. Human judgment and authority remain external to the machine-checkable contract.
