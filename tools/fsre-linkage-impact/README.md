# FSRE linkage impact oracle E0

A dependency-free Rust benchmark evaluator that measures pairwise linkage quality, false cluster bridges, entity fragmentation and their downstream population-denominator impact.

## Why this exists

Pairwise precision/recall can hide a catastrophic cluster bridge. One predicted cluster may merge distinct people, assets or service entities. A separate split may offset the raw count error, making a count-only metric look healthy even though both identity failures are serious.

## Input and as-of semantics

The manifest freezes benchmark ID, canonical scope digest, effective-time cutoff and recorded-time cutoff. Each record supplies namespace, gold entity ID, predicted cluster ID, gold population weight, candidate cluster weight, effective time and recorded time.

- Names are keyed by namespace; equal strings in PERSON and ASSET are distinct.
- Records effective after the target cutoff are excluded from the target population.
- Records recorded after the knowledge cutoff remain in retrospective gold truth, but are excluded from candidate clustering. Any candidate assignment on such a record is counted as late-arrival leakage.
- `-` means the candidate system did not assign the record to a cluster.
- Gold weight must be consistent within namespace plus gold entity.
- Candidate weight must be consistent within namespace plus predicted cluster.

All weights are nonnegative integer benchmark units. The evaluator does not authenticate gold labels.

## Metrics

- Pair TP / FP / FN, precision and recall.
- Predicted cluster bridges: one cluster contains multiple gold entities.
- Gold entity fragmentation: one gold entity appears in multiple predicted clusters.
- Population mass at risk from false bridges.
- Potential duplicate mass from fragmentation.
- Gold population weight, predicted cluster weight and signed denominator bias.
- Cluster-count bias.
- Late-arrival leakage and future-effective exclusions.

Bridge and fragmentation impacts are not netted against each other. A zero aggregate denominator bias does not erase either failure.

## Run

    cargo fmt --manifest-path tools/fsre-linkage-impact/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-linkage-impact/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-linkage-impact/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-linkage-impact/Cargo.toml -- tools/fsre-linkage-impact/fixtures/bridge-and-split.psv

## Claim boundary

This is an evaluator for declared benchmark manifests. It does not perform record linkage, verify gold adjudication, establish legal identity, certify source completeness, prove source independence, or authorize a population decision. It measures downstream effects on the declared gold truth.
