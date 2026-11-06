# Bank Migration Corpus Evaluator

A standalone Rust reference evaluator for the effects-disabled bank migration corpus.

## Run

From the repository root:

```sh
cargo test --manifest-path tools/finance/bank-migration-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/bank-migration-evaluator/Cargo.toml -- --self-test
```

Optional `--corpus PATH` and `--schema PATH` arguments evaluate alternate frozen fixtures.

## Checks

- JSON Schema Draft 2020-12 meta-schema and instance validation;
- fixed v1 case IDs/categories and category-to-disposition mapping;
- record census, stable source snapshot, pagination and duplicate source identities;
- exact retry versus changed fingerprint conflict;
- late-event append-only lineage;
- integer minor units and instrument/FX distinction;
- unknown lifecycle quarantine and accepted-versus-final separation;
- legal-entity scope binding;
- rollback only where no external effect is possible and the effect census is complete;
- indeterminate state on ambiguous external outcomes; and
- raw local-time preservation when timezone/DST meaning is unknown.

The tool has no network client, financial provider SDK, credentials or side-effectful command.

## Result vocabulary

- `PASS`: this exact synthetic corpus passes schema and semantic checks.
- `FAIL`: at least one structural or semantic invariant is violated.
- `INCOMPLETE`: reserved for a future evaluator mode that consumes a time-bounded source or execution receipt and cannot yet qualify completion. An incomplete migration scenario in the corpus is itself a valid negative case whose expected disposition is checked; it is not a PASS of a real migration.

A PASS does not prove bank data completeness, accounting correctness, production migration safety, external settlement, regulatory compliance, cutover authority or successful rollback.
