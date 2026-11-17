# FSRE capture–recapture E0 reference

A dependency-free Rust reference for two-list population coverage under an explicit source-dependence sensitivity interval.

## Mathematical contract

For a stratum, the observed cells are:

- \(n_{10}\): source A only;
- \(n_{01}\): source B only;
- \(n_{11}\): overlap;
- \(n_{00}\): missed by both, unobserved.

The cross-product odds ratio is

\[
\theta=\frac{n_{11}n_{00}}{n_{10}n_{01}},
\qquad
n_{00}=\theta\frac{n_{10}n_{01}}{n_{11}}.
\]

The implementation computes the implied hidden-cell and population-total sensitivity endpoints using exact rational arithmetic under caller-declared \(\theta\) bounds. They are model-implied expected-cell values, not sampling confidence intervals and not a claim that the unobserved cell is identified from two lists.

If the upper \(\theta\) bound is unbounded, the population upper bound is also unbounded. If overlap is zero, this two-list reference abstains from estimating the hidden cell.

## Independence and linkage

Lincoln–Petersen and Chapman estimates are separate conditional baselines. They are emitted only when the input explicitly declares independence, perfect linkage and within-stratum homogeneity. Each remains labelled as conditional and unauthenticated. These labels are supplied assumptions, not authenticated facts.

Two different source identifiers do not imply source independence. Shared declared ancestry blocks emission of the independence point-estimate baselines while leaving the dependence-sensitivity interval available. Distinct declared ancestry still does not establish independence; any emitted LP/Chapman result remains conditional on the caller-declared assumption. A perfect-linkage declaration is also only an input assertion. False-positive and false-negative linkage are not yet modelled in this E0 implementation.

## Stratification

The report includes per-stratum estimates and their sum, plus a pooled calculation for comparison. Pooled estimates can differ materially from stratified estimates when capture propensities differ. The pooled calculation is a comparator, not a substitute for a justified stratification model. If the declared stratum universe is incomplete, the crate still reports observed per-stratum quantities but withholds the target-population aggregate.

## Run

    cargo fmt --manifest-path tools/fsre-capture-recapture/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-capture-recapture/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-capture-recapture/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-capture-recapture/Cargo.toml -- tools/fsre-capture-recapture/fixtures/dependence-strata.psv

## Claim ceiling

The E0 engine does not authenticate source lineage, independence, linkage quality, frame completeness, within-stratum homogeneity or \(\theta\) sensitivity bounds. It does not estimate sampling uncertainty, model capture heterogeneity or infer false-match rates. Its outputs are conditional mathematical consequences of the supplied counts and assumptions only.

## Declared ancestry paths

Each ancestry field may be a slash-separated root-to-register path, for example `national-register/utility-account`. A shared component at any path depth is treated as declared common upstream ancestry and suppresses both per-stratum and pooled Lincoln–Petersen / Chapman point-estimate baselines. The odds-ratio sensitivity interval remains available.

The literal `UNKNOWN` state also withholds independence point estimates. Empty path components are rejected. Distinct declared paths do not prove statistical independence; they only mean no common ancestor is declared in the supplied paths. The path and its contents are caller assertions, not authenticated lineage.
