# GCPN probabilistic Fractions Skill Score v0.1

A dependency-free Rust implementation of deterministic FSS-equivalent scoring and the Necker et al. probabilistic FSS (pFSS) for threshold-event ensembles.

## What is implemented

- Explicit threshold and inequality convention.
- Ensemble-to-event-probability construction at each grid cell.
- Area-weighted neighbourhood forecast/observation fractions.
- Weighted FBS, WFBS and pFSS.
- Truncated or full-window boundary policies.
- Strict missing-data rejection or pairwise-complete support, with effective support counts.
- Irregular masks and positive cell weights.
- Typed degenerate no-event handling; no silent conversion to perfect score.
- Reproducible independent-Bernoulli random-reference Monte Carlo matched to observed event frequency, exact ensemble size, grid, mask, missingness and scale.
- Random-null mean, 95th percentile, and upper-tail Monte Carlo p-value.

## Formula

For centre i, the forecast neighbourhood fraction is the area-weighted average of ensemble event probabilities over the neighbourhood. The observed fraction is the area-weighted average of binary observed events. The scorer aggregates squared fraction difference and the worst-case denominator over valid centres:

- FBS = weighted mean((forecast_fraction - observed_fraction)^2)
- WFBS = weighted mean(forecast_fraction^2 + observed_fraction^2)
- pFSS = 1 - FBS/WFBS

The forecast fraction is formed by combining the ensemble and spatial neighbourhood probabilities, as defined by Necker et al. (2024). This crate does not implement member-averaged FSS, ensemble-mean FSS or ensemble-aggregated FSS under the pFSS label.

## Interpretation

Do not interpret pFSS > 0.5 or the older 0.5 + observed-base-rate/2 heuristic as universal proof of useful skill. ECMWF notes that the usual 0.5 cutoff is only a rule of thumb and that calibration affects interpretation. Antonio & Aitchison (2025) derive a scale-dependent random-forecast reference. This crate currently supplies an empirical finite-ensemble Bernoulli null matched to event frequency, member count and scoring support.

The null tests against spatially independent random events only. It is not a replacement for climatology, persistence or an empirically correlated baseline. The result must be reported with the full threshold/scale/lead/event-frequency slice; the scale curve may not be monotonic.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned and there are no external dependencies.

## Qualification boundary

Synthetic fixtures only. This crate does not fetch data, perform geospatial reprojection, wrap longitude, calculate CRPS/Brier/energy/variogram scores, assess calibration, or establish operational warning value. The rectangular grid requires a declared projection/physical scale in any real-data adapter. Confidence intervals over multiple forecast cases must resample independent event/time blocks, never individual grid cells as IID.

## Research references

- Necker et al. (2024), The fractions skill score for ensemble forecast verification, QJRMS, DOI: https://doi.org/10.1002/qj.4824
- Antonio & Aitchison (2025), How to Derive Skill from the Fractions Skill Score, Monthly Weather Review, DOI: https://doi.org/10.1175/MWR-D-24-0120.1
- ECMWF (2023), Verifying high-resolution forecasts: https://www.ecmwf.int/en/about/media-centre/science-blog/2023/verifying-high-resolution-forecasts

## Status

Standalone scorer and empirical reference null authored. Exact-head formatting, tests, strict Clippy and deterministic CLI qualification remain pending.
