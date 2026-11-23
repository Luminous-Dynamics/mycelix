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

The v2 Data in Brief article (10.1016/j.dib.2026.113160) describes 790 composite cultivated topsoil samples (0–15 cm; one 2020 winter sampling season). Its data-availability row links the older Zenodo v1 record 17510653 (803 samples), while the Zenodo record identifies v2 21640671 (790 samples) as the newer version. The profile pins v2 and records v1 as a distinct historical artifact.

The article's NIR value semantics are internally inconsistent: the specifications describe NIR reflectance, the spectral-column section says spectral values are absorbance while calling the NIR block reflectance percentage, the methods record reflectance R, and the data-merging/figure text describes converted absorbance without stating an exact conversion equation in the accessible text. The profile preserves the conflict. Do not silently transform the values or apply a second transform.

The 2026 cohort extends the earlier 2024 Western Highveld MIR cohort. It must not be treated as independent external validation until sample identities are crosswalked. The dataset contains 178 released georeferenced samples; the remaining locations must not be inferred. The article is CC BY, but confirm the versioned Zenodo data-file licence separately before redistribution. The auditor computes SHA-256 from the acquired CSV; no checksum is claimed until the actual v2 file is retrieved and hashed.

## Related-release identity overlap audit

The crate also provides a separate `sample-identity-overlap-audit` binary. It compares exact sample ID strings across two source CSVs, supports different ID column names, reports row/unique/duplicate counts and overlap fractions, hashes the sorted overlap membership, and does not include raw sample identifiers in the report.

```sh
cargo run --manifest-path research/terra-preta/tools/spectral-structure-audit/Cargo.toml --bin sample-identity-overlap-audit -- \
  /path/to/previous-mir.csv "Sample Name" "2024-mir" \
  /path/to/paired-nir-mir.csv "Sample Name" "2026-nir-mir" \
  /tmp/sample-overlap-report.json
```

The audit uses exact string matching without normalization. Different ID formats may conceal semantic overlap; no exact match therefore does not prove independent sampling. A source-reviewed crosswalk remains required when names or ID-generation schemes differ. Any overlapping sample IDs must remain in one group across training, validation and test. Duplicate IDs within one source are reported and must be grouped; they are not silently counted as independent samples.
