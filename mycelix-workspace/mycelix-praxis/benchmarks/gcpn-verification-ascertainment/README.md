# GCPN Verification Ascertainment v0.1

Standalone, dependency-free Rust engine for qualifying the outcome-verification sample behind forecast scores.

## What it does

- Accounts for every eligible case at a frozen as-of cutoff.
- Preserves typed missingness reasons rather than converting missing outcomes into negative outcomes or zero scores.
- Reports complete-case means and sharp finite-frame bounds for bounded scores.
- Reports stratum-level denominators, coverage and bounds, including all-missing strata.
- Computes Horvitz–Thompson and Hájek estimates only when a propensity model is named, every eligible case has a declared inclusion probability, and every probability meets the caller-declared positivity floor.
- Reports inverse-weight diagnostics and effective sample size without clipping weights.
- Evaluates only caller-supplied pattern-mixture sensitivity deltas; it never chooses a preferred delta.
- Enforces paired candidate/reference scores against the same case ID, common verification source/version and the same as-of cutoff.

## Example

```sh
cargo run --manifest-path Cargo.toml
cargo test --manifest-path Cargo.toml
```

The CLI runs a small deterministic synthetic fixture and prints observed-case summaries, no-assumption bounds, IPW diagnostics and an explicitly supplied sensitivity grid.

## Interpretation

The complete-case mean describes verified cases only. The finite-frame bounds require only the declared score range. IPW relies on conditional exchangeability given the declared propensity covariates and on a complete eligible frame; the code cannot establish those assumptions. The sensitivity grid is an assumption probe, not identification of missing-not-at-random outcomes.

The package does not estimate causal warning effectiveness, track people, generate forecasts, issue warnings, set operational thresholds or authorize action. Correlated-system uncertainty should be assessed with the separate independent-system bootstrap programme, not an IID case bootstrap.
