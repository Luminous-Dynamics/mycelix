# FSRE three-list capture–recapture E0

A dependency-free Rust reference for exact sensitivity bounds on the unobserved all-zero cell of a three-list capture table.

## Model

Seven observed binary capture patterns are supplied exactly once: 100, 010, 001, 110, 101, 011 and 111. The unseen 000 cell is related through an explicit three-way cross-product interaction:

\[
\theta_{ABC} =
\frac{n_{111} n_{100} n_{010} n_{001}}
     {n_{110} n_{101} n_{011} n_{000}}.
\]

Let \(R=n_{111}n_{100}n_{010}n_{001}/(n_{110}n_{101}n_{011})\). For a declared \(\theta_{ABC}\in[\theta_{\min},\theta_{\max}]\), the unseen count is bounded by \(R/\theta_{\max}\) and \(R/\theta_{\min}\). An unbounded \(\theta_{\max}\) gives a zero lower bound. The no-three-way-interaction baseline is the special case \(\theta_{ABC}=1\).

## Safeguards

- Exact checked rational arithmetic; no floating point or pseudocounts.
- Every observed pattern must occur exactly once.
- Any zero pattern withholds numeric bounds rather than hiding sparsity through arbitrary correction.
- Shared ancestry on any pair of source paths, or unknown ancestry on any source, suppresses the conditional no-three-way-interaction baseline while retaining declared theta sensitivity.
- Distinct ancestry does not establish independence.
- Incomplete pattern-table coverage withholds all numeric results.
- Incomplete population frame preserves within-frame hidden-cell bounds but withholds target-population totals.
- A declared no-three-way-interaction assumption must be consistent with a theta interval containing one.
- Deterministic rendering under input row permutation.

## Known-answer fixture

Observed pattern values: n100=40, n010=30, n001=50, n110=10, n101=20, n011=25, n111=20. The cross-product ratio is 240 and observed union is 195. With theta interval [1/2, 2], exact hidden-cell bounds are [120, 480], giving target totals [315, 675] under the declared-complete frame. The conditional theta=1 baseline is 240 hidden and 435 total.

This fixture is synthetic. Its values demonstrate the algebra and are not a field estimate.

## Run

    cargo fmt --manifest-path tools/fsre-three-list-capture/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-three-list-capture/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-three-list-capture/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-three-list-capture/Cargo.toml -- tools/fsre-three-list-capture/fixtures/known-answer.psv

## Claim boundary

The theta interval is a declared sensitivity assumption, not a sampling confidence interval. The conditional no-three-way baseline is not authenticated truth. E0 does not authenticate sources, prove population-frame completeness, establish source independence, or validate that the seven-pattern table exhausts the target population. A three-list estimator is not automatically better than two-list DSE: sparse/zero patterns, heterogeneity, referrals and source ancestry can make the model weakly identified or invalid.
