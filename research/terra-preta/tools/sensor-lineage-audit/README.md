# Soil sensor lineage audit

Standalone Rust validator for three separate evidence records: raw/primary sensor observations, derived soil-indicator estimates and laboratory/reference assays.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/sensor-lineage-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/sensor-lineage-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/sensor-lineage-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/sensor-lineage-audit/Cargo.toml -- research/terra-preta/tools/sensor-lineage-audit/fixtures/synthetic-nir-soc-lineage.json /tmp/sensor-lineage-report.json
```

## Semantic contract

- The raw NIR spectrum is a direct sensor signal.
- The SOC value inferred from the spectrum is a derived prediction.
- The dry-combustion reference assay is a separate physical measurement.
- The estimate links to the exact sensor observation, model and preprocessing artifact digests.
- Training samples / independence groups cannot overlap the evaluated sample/group.
- Out-of-domain and unassessed estimates must abstain.

## Boundary

All files and identities in the fixture are synthetic. The validator checks provenance and evidence semantics only. It does not measure spectral prediction error, validate calibration quality, establish uncertainty coverage or qualify an agronomic decision.
