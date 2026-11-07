# Rainfall intensity-regime audit v0.1

This Rust CLI produces a descriptive, site-clustered audit of event preferential-flow proxy outcomes across fixed rainfall peak-intensity bands. It is designed to inspect the nonlinear threshold/plateau relationship reported by Li et al. (2025); it does not estimate a causal threshold or reproduce the paper's random forest partial-dependence curve.

## Run

    cargo test --manifest-path research/terra-preta/tools/intensity-regime-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/intensity-regime-audit/Cargo.toml -- \
      site-event-drivers.json /tmp/intensity-regime-report.json

## Fixed bands

| Band | Range |
|---|---:|
| low | peak intensity <5 mm/h |
| rising | 5 to <12 mm/h |
| transition | 12 to <13 mm/h |
| high | >=13 mm/h |

These are descriptive bins based on the 2025 paper's reported partial dependence shape: strong increase around 5–12 mm/h and a plateau above ~13 mm/h. They are not calibrated hydraulic thresholds and should not be treated as causal discontinuities.

## Statistical contract

- Input is one row per unique site-level rainfall event; duplicate site/event pairs are rejected.
- Only events with cumulative rainfall >=2 mm and soil moisture available enter the bands.
- VT maximum/median/median+SD and NSR remain separate outcomes.
- Each band reports positive, negative and indeterminate events plus conditional prevalence and identification bounds.
- Bootstrap samples whole sites with replacement (cluster bootstrap), preserving event dependence within each selected site. The intervals condition on method-evaluable events; identification bounds separately show indeterminate uncertainty.
- Fixed seed and requested replicate count are recorded; xorshift sampling is deterministic and intended for reproducibility of this audit, not cryptographic use.

## Interpretation boundary

A site-level event prevalence curve can show whether the reported intensity shape is present in the data. It cannot identify causation because antecedent moisture, texture, canopy, vegetation, runoff, site heterogeneity and predictor interactions are not adjusted. Use a preregistered multivariable model (GAM, spline/hinge regression, then Random Forest) with nested site/ecoregion holdout to test conditional intensity dependence. No event-level row may leak across site folds.