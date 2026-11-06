# FIN-JOURNAL-SET verifier

Independent Rust verifier for period journal-set completeness and exact account roll-forward.

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-journal-set-report.json
```

The CLI supports --candidate, --candidate-schema, --oracle, --oracle-schema and --out.

The verifier consumes an upstream source-population qualification reference and canonical per-entry journal objects; it does not establish those upstream qualifications. All fixtures are synthetic, and the reference oracle is public rather than blind.
