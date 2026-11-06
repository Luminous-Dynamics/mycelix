# FIN-JOURNAL-SET verifier

Independent Rust verifier for period journal-set completeness and exact account roll-forward.

The default CLI runs immutable corpus **v1.1.0** with its versioned Draft 2020-12 input schema and public reference oracle. To replay v1.0.0, pass the historical candidate, schema and oracle paths explicitly.

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-journal-set-report.json
```

The CLI supports `--candidate`, `--candidate-schema`, `--oracle`, `--oracle-schema` and `--out`.

v1.1.0 adds FJS-026, in which two individually admissible partial reversals exceed the original amount in aggregate. The verifier tracks remaining reversal capacity cumulatively per original journal, rejects reversal-of-reversal targets and rejects invalid reversal directions.

The verifier consumes an upstream source-population qualification reference and canonical per-entry journal objects; it does not establish those upstream qualifications. All fixtures are synthetic, and the reference oracle is public rather than blind.
