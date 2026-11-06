# FIN-JOURNAL-SET verifier

Independent Rust verifier for the public synthetic period-journal set corpus.

Run from this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-journal-set-report.json
```

The CLI supports --candidate, --candidate-schema, --oracle, --oracle-schema and --out.

This public regression verifier checks expected-event coverage, journal identity and replay, journal/entity/profile scope, entry-level balancing, reversal bounds, close cutoff, predecessor/opening equality, exact account roll-forward and trial-balance balance. It does not independently qualify real source-population completeness: it consumes the qualification state and source frontier commitment as inputs.

All account and currency profiles are synthetic. No raw bank payloads or production financial records are included. Public oracle results are not blind evaluation or audit evidence. A pass does not establish complete real books, statutory/tax/IFRS/GAAP compliance, audit sufficiency or authorized signoff.
