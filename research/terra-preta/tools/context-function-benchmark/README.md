# Context-conditioned soil-function benchmark

Standalone synthetic-only Rust auditor to keep absolute condition, peer-reference position, longitudinal within-unit change and explicit guardrails separate.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/context-function-benchmark/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/context-function-benchmark/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/context-function-benchmark/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/context-function-benchmark/Cargo.toml -- \
  research/terra-preta/tools/context-function-benchmark/fixtures/synthetic-context-function.json \
  /tmp/context-function-report.json
```

## What is checked

- Only synthetic evidence is accepted by v0.1.
- Every indicator has separate direct measurement, reference distribution, context, repeated same-unit observations and guardrail metadata.
- Peer percentile is emitted only for an in-domain reference with sufficient sample and independent-site support.
- Out-of-domain / unassessed / sparse groups abstain; no reference group is silently substituted.
- The evaluated independence group cannot occur in peer-reference training groups.
- Repeated dates must be strictly ordered and belong to one experimental unit.
- q90/q95/q99 distances are labelled peer-reference distances, not guaranteed recoverable or causal gaps.
- The report contains no composite health score and no intervention/treatment effect.

## Important caveats

The Gaussian reference CDF is a manufactured v0.1 software approximation. A real SHAPE-SSA-style deployment requires a fitted hierarchical Bayesian model and its posterior predictive distribution, calibration, spatially independent evaluation, peer-group sample support and model/data lineage.

The endpoint change is descriptive. It does not adjust for serial correlation or estimate a causal management response. A causal extension needs treatment assignment, controls, blocking, site/plot hierarchy, predeclared estimand and valid missing-data handling.

Any production guardrail must be approved by qualified local authorities and retain its explicit source. The example guardrail is synthetic only and has no agronomic authority.
