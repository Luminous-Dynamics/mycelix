# Biochar batch quality passport auditor

Standalone Rust auditor for a **synthetic batch** and a versioned, end-use-specific quality profile.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/biochar-batch-passport/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/biochar-batch-passport/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/biochar-batch-passport/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/biochar-batch-passport/Cargo.toml -- \
  research/terra-preta/tools/biochar-batch-passport/fixtures/synthetic-biochar-batch.json \
  /tmp/biochar-batch-report.json
```

## Gates

1. Feedstock origin, prior use, contamination screening and ecological-retention evidence.
2. Production unit, process record, temperature trace, residence time, oxygen-control method, and mass accounting.
3. Homogenization, representative sampling, increment count, seal / custody, laboratory identity and retained sample duration.
4. Every required analyte bound to the expected unit, dry-matter basis, method, laboratory, accreditation scope, LOD/LOQ, replicates and expanded uncertainty.

The quality profile owns all thresholds and method identifiers. The auditor contains no globally hardcoded contaminant limits.

## Conservative limits

For a maximum (L), result (x) and expanded uncertainty (U):
- pass if (x+U le L);
- fail if (x-U > L);
- otherwise indeterminate.

For a minimum (L), pass if (x-U ge L), fail if (x+U < L), otherwise indeterminate.

Censored results below LOD/LOQ can support a maximum criterion only where the applicable censoring bound is below the limit. Missing, incompatible or out-of-scope tests cannot pass.

## Hard boundary

The report always states:
- carbon removal: not assessed by v0.1;
- agronomic suitability: not assessed by v0.1.

A synthetic profile pass is not EBC/IBI/WBC certification, a regulatory determination, real product safety, carbon removal, or a recommendation for a particular soil or crop.
