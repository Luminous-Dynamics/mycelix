# FIN-ASSURE-PROCEDURE verifier

Standalone, dependency-light Rust reference evaluator for the public synthetic risk/assertion/procedure/evidence corpus.

The default CLI runs **candidate corpus v1.1.0**, its versioned Draft 2020-12 schema and its public reference oracle. Each case result includes both the overall stand-back disposition and an explicit disposition for every risk/assertion pair. A case is not considered clear merely because one of several required pairs is supported.

Run from this directory:

```sh
cargo test --locked
cargo fmt --all -- --check
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-assure-procedure-report.json
```

The CLI supports `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out`. To replay v1.0.0, explicitly pass the v1.0.0 candidate, input schema, public oracle and oracle schema.

v1.1.0 checks that the performed procedure's purpose matches the required assertion-purpose pair; that each evidence item is bound to the reporting entity and period; that population/denominator, evidence origin, reliability and contradictions satisfy the declared contract; and that analytical differences above threshold remain unresolved. Human judgment and authority remain external.

This is public synthetic conformance, not blind model evaluation, audit assurance, ISA compliance or a professional audit conclusion.
