# FIN-JOURNAL-SET verifier

Independent Rust verifier for period journal-set completeness and exact account roll-forward.

The default CLI runs the immutable **v1.1.0** candidate corpus, its versioned Draft 2020-12 input schema, and its public reference oracle. To replay v1.0.0, pass its candidate/schema/oracle/schema paths explicitly through the CLI flags.

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-journal-set-report.json
```

The CLI supports `--candidate`, `--candidate-schema`, `--oracle`, `--oracle-schema` and `--out`.

v1.1.0 adds an explicit public case where two individually admissible partial reversals exceed the original journal amount in aggregate. The verifier therefore tracks remaining reversal capacity cumulatively per original journal. It also rejects reversal-of-reversal chains and invalid reversal directions.

The verifier consumes an upstream source-population qualification reference and canonical per-entry journal objects; it does not establish those upstream qualifications. All fixtures are synthetic, and the reference oracle is public rather than blind.
