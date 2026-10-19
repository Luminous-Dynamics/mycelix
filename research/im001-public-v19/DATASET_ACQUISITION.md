# IM-001 — Official Dataset Acquisition / Freeze Contract

The primary corpus remains **CSE-CIC-IDS2018**, using the University of New Brunswick Canadian Institute for Cybersecurity's official dataset page and public AWS bucket.

The official source documents seven attack scenarios and an AWS acquisition route using `s3://cse-cic-ids2018/`; the experiment must record source/citation terms, exact acquired objects, byte sizes and SHA-256 hashes before transformation.

## Freeze contract

The dataset-manifest freezer converts an acquired directory into a deterministic relative-path / byte-size / SHA-256 manifest and intentionally refuses empty/nonexistent directories.

The following remain blocked until the real corpus is acquired:

- dataset object/file hashes;
- exact object listing;
- derived-table hashes;
- client partitions;
- transformation revision;
- final held-out attack lineage;
- final metric thresholds.

## Epistemic boundary

No dataset file hashes are claimed by this branch. Selecting the official route reduces ambiguity around *what to acquire* while preserving the real preregistration gate around *which exact files, transformations and lineages are used*.
