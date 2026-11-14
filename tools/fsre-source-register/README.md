# FSRE public source-register validator

A dependency-free Rust validator for the FSRE South African public-data source register.

It checks:
- exact header and ten required fields;
- non-empty publisher, dataset, period, geography, units/denominator and admissibility note;
- unique stable source IDs;
- absolute HTTP(S) source URL shape;
- supported ingestion-state vocabulary;
- deterministic counts by declared ingestion state.

The register separates the DWS 2025 Green Drop report collection, the bounded national-summary replay, the SAnews municipal-denominator source, the WRO catalog metadata, and the individual PDF resource whose licence is not provided. It also records Unmapped Dataworks UM-006 as an external early-access comparator lead only: the register retains its public release metadata and fingerprint, but no third-party records are imported or mirrored. The source register contains 15 entries: 11 `DISCOVERED_NOT_EXTRACTED`, three `EXTRACTED_UNREVIEWED` metadata/summary rows, and one `BLOCKED_RIGHTS_OR_GAP` PDF resource.

The validator does **not** download, snapshot, extract, license-check, authenticate or qualify any external source. It never promotes source states automatically. A `DISCOVERED_NOT_EXTRACTED` row remains unextracted; an `EXTRACTED_UNREVIEWED` row does not imply authenticated source evidence, complete system-level data or suitability for a particular claim.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-source-register/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-source-register/Cargo.toml -- \
  research/fsre/ZA_PUBLIC_SOURCE_REGISTER_V0_1.psv
```

## Input

Strict pipe-delimited text with the exact header in the public source register. Embedded pipe characters and empty fields are rejected; quoting is not implemented. Comments beginning with `#` and blank lines are ignored.

The output counts only the declared source states. It does not assess evidentiary value, source rights, extraction correctness, denominator comparability or whether a source supports a particular claim. Those require the source-specific replay protocol and independent review.
