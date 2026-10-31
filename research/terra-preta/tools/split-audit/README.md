# Terra Preta row-level split audit

This standalone Rust CLI checks a frozen row-to-split CSV before fitting a model. It is deliberately narrower and more auditable than a model framework: it verifies that the supplied reviewed grouping identifier does not appear across train, validation, test, or excluded splits, and that each required evaluation split is populated.

## Input

Exact CSV headers:

```csv
row_id,group_id,split
obs-001,study-A,train
obs-002,study-A,train
obs-003,study-B,validation
obs-004,study-C,test
obs-005,study-D,excluded
```

`row_id` must uniquely identify a measurement row. `group_id` must be a reviewed source identifier, not a row number. Groups may have repeated rows within one split; the same group crossing splits is rejected. IDs that are blank, common placeholders, or contain leading/trailing whitespace are rejected to prevent silent aliasing. Allowed split labels are `train`, `validation`, `test`, and `excluded`; train, validation, and test must be non-empty.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/split-audit/Cargo.toml
cargo run --release --manifest-path research/terra-preta/tools/split-audit/Cargo.toml -- \
  /path/to/frozen-row-manifest.csv /path/to/split-audit-report.json source_study_id
```

Run it independently for every required grouping dimension, e.g. `source_study_id`, `site_id`, or `material_batch_id`. The reported grouping-key label records the reviewed dimension selected by the caller; it does not prove that the CSV's `group_id` values correspond correctly to that dimension.

The report contains SHA-256 of the original manifest bytes, a digest of canonical group-to-split assignments, and only the input filename rather than the full local path. A change to source rows or group assignments changes the corresponding digest.

## Interpretation limits

The 2026 review “Illusions of Accuracy: Widespread Data Leakage Distorts Machine Learning Results in Biochar Research” reports that hierarchical and point-level leakage is common in published biochar machine-learning evaluations (doi:10.1111/gcbb.70173). This tool addresses one testable failure mode—group identifiers appearing in multiple splits—but it does not detect undocumented related publications, shared sites under different names, mislabeled experiment IDs, preprocessing leakage, test-set tuning, or causal-identifiability problems. These require crosswalk review and workflow audit.

A successful split audit is necessary but not sufficient for a scientifically independent holdout, a fair SOTA comparison, calibrated prediction, or agricultural efficacy. Preserve the reviewed source crosswalk, split manifest, model freeze, prediction file, and report together.