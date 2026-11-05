# FIN-ACC assertion-purpose reference verifier

Run from this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-acc-assertion-purpose-report.json
```

The CLI accepts `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out` overrides.

The evaluator is a public synthetic reference implementation of the declared assertion/procedure obligations. It is not an auditor, does not judge audit sufficiency, and does not claim ISA compliance. The public oracle is a regression oracle, not a blind holdout.
