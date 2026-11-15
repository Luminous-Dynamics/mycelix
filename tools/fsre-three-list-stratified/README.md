# FSRE stratified three-list capture–recapture E0

A Rust wrapper around the exact single-stratum three-list reference crate. Each stratum has its own seven observed capture patterns and its own three-way interaction sensitivity interval.

## Model

For stratum \(s\):

\[
R_s =
\frac{n_{111,s}n_{100,s}n_{010,s}n_{001,s}}
     {n_{110,s}n_{101,s}n_{011,s}},
\quad
n_{000,s}=\frac{R_s}{\theta_{ABC,s}}.
\]

With separate declared sensitivity intervals per stratum and no cross-stratum restrictions (a Cartesian-product identified set), the aggregate hidden-cell interval is the sum of the per-stratum endpoints. This is sharp only when all listed strata are estimable and the stratum universe is declared complete. Target population totals additionally require a declared-complete population frame.

## Pooling is not stratification

The known-answer fixture has two strata:
- Rural: \(R=90\), theta interval [1,3], hidden interval [30,90].
- Urban: \(R=240\), theta interval [1/2,2], hidden interval [120,480].

The exact stratified hidden interval is [150,570]. The pooled seven-cell table has ratio 315, while the sum of conditional no-three-way stratum baselines is 330. The pooled ratio is therefore reported only as a diagnostic and is never substituted into the stratified interval. Separate stratum interaction intervals do not imply statistical independence of the source lists; shared/unknown ancestry still suppresses every conditional point baseline.

This is a synthetic counterexample to treating pooled and stratified models as interchangeable.

## Qualification gates

- Exactly seven observed patterns per stratum.
- Zero observed pattern in any stratum makes that stratum unestimable; aggregate totals are withheld but other stratum results remain visible.
- Incomplete pattern universe withholds every numeric result.
- Incomplete stratum universe permits within-declared-strata bounds but withholds target-population hidden bounds and totals.
- Incomplete population frame allows target hidden bounds for a complete stratum universe but withholds population totals.
- Shared or unknown source ancestry suppresses all conditional no-three-way point baselines.
- Each stratum must have its own positive theta lower bound; theta intervals are not silently pooled.
- The pooled cross-product ratio is a diagnostic only.

## Run

    cargo fmt --manifest-path tools/fsre-three-list-stratified/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-three-list-stratified/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-three-list-stratified/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-three-list-stratified/Cargo.toml -- tools/fsre-three-list-stratified/fixtures/two-strata.psv

## Claim boundary

The per-stratum theta intervals are declared sensitivity assumptions, not confidence intervals. Summing their endpoints is sharp only under the declared separable-strata model with no additional cross-stratum restrictions. The wrapper does not authenticate the strata universe, population frame, capture records, lineage, or independence assumptions.
