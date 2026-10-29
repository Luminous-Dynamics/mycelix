# Multi-key dependency leakage audit

This Rust CLI audits a frozen row-to-split manifest for transitive leakage through multiple reviewed dependence dimensions. Unlike a single-key check, it builds connected components over all selected keys: any rows that share a publication, study, experiment, site, material batch, curve, or other selected ID are connected, including through intermediate rows.

## Input

CSV must contain `row_id`, `split`, and every dependency column selected on the command line. Example:

```csv
row_id,split,publication_id,study_id,site_id,material_batch_id
r1,train,pub-a,study-a,site-1,batch-1
r2,train,pub-a,study-b,site-2,batch-2
r3,validation,pub-b,study-c,site-3,batch-3
r4,test,pub-c,study-d,site-4,batch-4
```

In practice, any sharing of a selected dependency connects rows transitively; the connected component must be contained in exactly one of train, validation, test, or excluded.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/dependency-audit/Cargo.toml
cargo run --release --manifest-path research/terra-preta/tools/dependency-audit/Cargo.toml -- \
  /path/to/frozen-row-manifest.csv /path/to/dependency-audit-report.json \
  publication_id,study_id,experiment_id,site_id,material_batch_id,curve_id
```

The command-line dependency columns are an explicit choice and must come from a reviewed source crosswalk. Do not include unrelated identifiers or row IDs as dependency keys. The tool reports original-input SHA-256, a stable digest of row-to-component assignments, component counts by split, and per-key missingness/distinct-value statistics. The report stores only the input filename, not the full local path.

Blank and common placeholder dependency values are counted as missing. A row with no usable selected dependency identifier fails closed. Leading/trailing whitespace, duplicate row IDs, duplicate/unknown split labels, and empty required splits are rejected. Missing values in some dependency columns are allowed but produce warnings; these warnings must be resolved or explicitly accepted in the study protocol before a confirmatory claim.

## Why connected components?

Suppose row A shares a source publication with row B; row B shares a site with row C. A split could put A in train and C in test while each simple single-key comparison looks plausible unless both relationships are followed together. Connected-component construction catches that transitive path and rejects the manifest if the component spans split boundaries.

## Limits

This is a structural leakage sentinel, not a guarantee of scientific independence. It cannot detect shared sources that are absent from the crosswalk, synonymous or mislabelled identifiers, test-set reuse, leaky preprocessing, feature/target leakage, or an invalid causal estimand. The selected columns and their values need an expert-reviewed source crosswalk. Run separate manifests for different intended deployment claims; a study/source-grouped split does not automatically certify geographic, batch, time, or material-family generalization.
