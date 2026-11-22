# Source-aware soil spectral harmonization auditor

Standalone Rust auditor for source/version identity, native spectral support, explicit preprocessing chains, target-method comparability and physical-sample split integrity.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/spectral-harmonization-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/spectral-harmonization-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/spectral-harmonization-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/spectral-harmonization-audit/Cargo.toml -- \
  research/terra-preta/tools/spectral-harmonization-audit/fixtures/synthetic-harmonization-registry.json \
  /tmp/spectral-harmonization-report.json
```

The auditor rejects double reflectance-to-absorbance transforms, discontinuous transform chains, output outside native axis coverage, direct target-comparability claims across method/unit/extraction/depth mismatches, bridge-required mappings without independent bridge evidence, and physical sample copies/replicate scans crossing splits.

This is a metadata/contract auditor. It does not read or transform spectra, establish analytical equivalence, verify raw archive bytes or prove model transfer performance. Fixture identifiers and digests are manufactured.
