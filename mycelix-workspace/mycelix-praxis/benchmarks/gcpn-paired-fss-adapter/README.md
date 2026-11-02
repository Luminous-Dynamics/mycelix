# GCPN paired common-source pFSS adapter v0.1

This crate joins two independently versioned GCPN packages:
- gcpn-common-source-comparison: frozen case frame, verification identity and typed availability;
- gcpn-probabilistic-fss: weighted ensemble pFSS and paired event-block bootstrap.

The adapter does not rebuild the score denominator from arrays. It derives the only scoreable IDs from the common-source ComparisonReport and requires the supplied artifacts to match that exact set.

## Exact scoreable cohort

A case is scoreable only if:
- it is eligible under the frozen comparison contract;
- candidate forecast is available by the as-of cutoff;
- reference forecast is available by the same cutoff;
- common verification is available by the same cutoff;
- candidate, reference and verification artifact digests exactly match the manifest;
- initialization, valid time and lead exactly match the manifest;
- both forecast arrays share case ID, dimensions, observation values, valid mask and area weights;
- grid, mask and spatial-support identity match the common-source manifest.

Missing candidate/reference forecasts, missing verification and excluded cases remain in the source report and outside the paired FSS score. Missing a grid artifact for a paired-verifiable case is a fatal adapter error. Unmanifested or duplicate artifacts are rejected.

## Paired uncertainty

The adapter calls paired_block_bootstrap. Every event-block draw is identical for candidate and reference. The report gives candidate pFSS, reference pFSS, and the paired candidate-minus-reference pFSS interval. It does not subtract two independently bootstrapped marginal confidence intervals.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. There are no external dependencies.

## Source snapshot boundary

This branch is stacked on the common-source comparison engine branch and includes a source snapshot of the pFSS crate from the pFSS parent PR so the two local path dependencies can be tested together without merging either draft. The pinned parent source head at snapshot time is 57a19b3ffe1f23858a93c9c5bfc7cbe0ddb6d979. The snapshot is byte-for-byte copied from that branch; it must be reconciled with the parent PR before any merge.

## Non-claims

Synthetic adapter success does not establish operational forecast skill, calibration, disaster impact or national MHEWS performance. No real weather datasets are fetched or redistributed. The common reference itself remains subject to observation-error and representativeness uncertainty.
