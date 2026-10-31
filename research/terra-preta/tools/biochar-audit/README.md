# BiocharDS structural audit

A small standalone Rust command-line tool to establish a repeatable inventory of a downloaded BiocharDS CSV before model development.

## Use

Download the publisher-provided `BiocharDS_V1.0.csv` from the dataset linked in the paper:

- Paper: https://doi.org/10.1038/s41597-023-02867-9
- Data collection: https://doi.org/10.6084/m9.figshare.c.6622375.v1
- The paper describes a CSV with 2,439 rows and 209 columns, linked to 367 source studies. These counts are reference checks only; the tool reports discrepancies instead of force-fitting an input to these expectations.

Run from a Rust-enabled environment:

```sh
cargo test --manifest-path research/terra-preta/tools/biochar-audit/Cargo.toml
cargo run --release --manifest-path research/terra-preta/tools/biochar-audit/Cargo.toml -- \
  /path/to/BiocharDS_V1.0.csv /path/to/audit-report.json
```

Dependencies resolve in the standalone tool's own Cargo workspace. Save the report alongside the immutable original dataset and record the source URL, download timestamp, and license. The JSON report records only the input filename, not the full local path.

## What it reports

- SHA-256 digest of the exact input bytes;
- a report contract at [audit-report.schema.json](./audit-report.schema.json) for downstream validation;
- row and column counts, plus an advisory comparison with the published row count;
- rows that do not match the header width;
- missingness by column and in aggregate;
- exact duplicate rows after first occurrence;
- blank and colliding headers after case/punctuation normalization;
- possible study/source and treatment-group key columns, with distinct counts and missingness;
- numeric-looking values per column;
- warnings that identify when study-level validation or treatment-effect estimation should be blocked.

## Strict interpretation limits

This is a **structural audit**, not a scientific quality certificate. It does not validate digitized measurements against source papers, establish a causal estimand, solve correlated effect sizes, check publication bias, guarantee a valid train/test split, prove model accuracy, establish local South African applicability, or demonstrate material safety/efficacy.

The treatment-group heuristic must be checked against the original spreadsheet's `Explanation and unit` and `Reference` sheets. A detected header is a candidate identifier, not an approved key. If no credible source-study key is present in the CSV, do not run row-random train/test validation: retain or separately download the full workbook and construct a reviewed grouping crosswalk.

Missingness is preserved as a finding. No imputation, row deletion, unit conversion, column rename, or duplicate removal is performed automatically. An exact duplicate row can be a data-quality issue or a legitimate duplicated comparison requiring source review; it is flagged, never silently deleted.

## Research separation

The pipeline must retain distinct sources:
1. deterministic synthetic fixtures (software invariants only);
2. literature-extracted comparisons (heterogeneous published methods);
3. new laboratory characterizations;
4. controlled pot/field experiments.

Do not merge these classes without a documented measurement mapping and analysis plan. A successful audit does not make a claim about crop response, fertilizer substitution, carbon permanence, or registration status.
