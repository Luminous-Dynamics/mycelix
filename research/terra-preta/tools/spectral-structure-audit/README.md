# Western Highveld soil spectral structure auditor

A standalone Rust CSV auditor for the 2026 paired handheld NIR / benchtop MIR Western Highveld release.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/spectral-structure-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/spectral-structure-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/spectral-structure-audit/Cargo.toml --all-targets -- -D warnings
cargo run --release --manifest-path research/terra-preta/tools/spectral-structure-audit/Cargo.toml -- \
  /path/to/NWU_NIR_MIR_Dataset.csv \
  research/terra-preta/tools/spectral-structure-audit/profiles/western-highveld-v2.json \
  /tmp/western-highveld-spectral-audit.json
```

## Checks

- CSV headers, required sample identifier and reference property names.
- Unique/nonblank sample identifiers and duplicate header rejection.
- Spectral column naming, numeric wavenumber suffix, strict descending axis order and duplicate-axis rejection.
- NIR and MIR band counts, bounds, gap summary and unexpected out-of-region axes.
- Finite numeric spectral values; missing spectral cells are counted, never imputed.
- Property reference values, missingness and observed range, without converting between laboratory methods.
- SHA-256 of the actual CSV and of the profile; output is create-new and never overwrites an existing report.

## Pinned release context

The profile pins Zenodo v2 (record 21640671; DOI 10.5281/zenodo.21640671), the published filename and row count (790), and the publisher's MD5 string as metadata. The auditor computes SHA-256 when the actual source file is supplied; it does not treat the publisher MD5 as a cryptographic identity or assume that the file was acquired.

## Important boundaries

This is a structural data audit, not model evaluation. A structurally sound CSV does not qualify pH/P/K/Ca/Mg/Na predictions or replace the cited reference methods.

The article describes NIR values both as reflectance in the instrument section and as converted absorbance on a common axis in the merging/validation discussion. The profile preserves this as unresolved source semantics and the audit emits a warning. Do not silently choose a transform. Resolve it by inspecting the delivered values/processing source or requesting author clarification.

The 2026 cohort extends the earlier 2024 Western Highveld MIR cohort. It must not be treated as independent external validation until sample identities are crosswalked. The dataset contains 178 released georeferenced samples; the remaining locations must not be inferred.
