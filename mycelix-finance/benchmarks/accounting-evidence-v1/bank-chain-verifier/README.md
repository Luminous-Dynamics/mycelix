# FIN-BANK-CHAIN standalone Rust verifier

This isolated package validates the public synthetic statement-chain v1.0.4 input and oracle schemas, reconstructs exact booked-balance arithmetic, checks statement-chain identity/period continuity, detects conflicts and compares its dispositions with the published reference oracle.

## Run

From this directory:

```sh
cargo test --locked
cargo fmt --all -- --check
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-bank-chain-report.json
```

The CLI accepts `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out` overrides. The emitted report binds the exact input/schema/oracle bytes and verifier source digest.

## Scope

This is a dependency-light, read-only reference verifier over parser-independent synthetic semantic fixtures. The fixture digest tokens are not hashes of raw XML. The verifier does not parse CAMT XML, authenticate a bank, prove account ownership or completeness, create a journal, initiate a payment, assert settlement finality, or establish regulatory compliance.

The oracle is public and this is a reproducibility/regression test, not blind model evaluation. A blind run requires a separate access-controlled evaluator.
