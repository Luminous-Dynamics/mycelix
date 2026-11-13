# FSRE public source-register validator

A small dependency-free Rust validator for the FSRE public-data discovery register.

It checks:
- exact header and ten required fields;
- non-empty publisher, dataset, period, geography, units/denominator and admissibility note;
- unique stable source IDs;
- absolute HTTP(S) source URL shape;
- supported ingestion-state vocabulary;
- deterministic counts by declared ingestion state.

It **does not** download, snapshot, extract, license-check, authenticate or qualify any external source. A row in `DISCOVERED_NOT_EXTRACTED` remains undiscovered as evidence and must never be counted as an extracted dataset. The tool does not automatically promote source states.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-source-register/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-source-register/Cargo.toml -- \
  research/fsre/ZA_PUBLIC_SOURCE_REGISTER_V0_1.psv
```

## Input

Strict pipe-delimited text with the exact header in the public source register. Embedded pipe characters and empty fields are rejected; quoting is not implemented. Comments beginning with `#` and blank lines are ignored.

The output counts only the declared source states. It does not assess evidentiary value, source rights, extraction correctness, denominator comparability or whether a source supports a particular claim. Those require the source-specific replay protocol and independent review.
