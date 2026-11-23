# Prospective soil-twin forecast audit

Synthetic-only Rust verifier for immutable pre-outcome forecasts and later independent reference observations.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml -- \
  research/terra-preta/tools/prospective-forecast-audit/fixtures/synthetic-prospective-forecasts.json \
  /tmp/prospective-forecast-report.json
```

## Audited contracts

- Target identity includes site, experimental unit, variable, unit, method, depth, spatial support and future target window.
- Forecast training cutoff and issue time must precede target window start.
- All timestamps use canonical UTC `Z` so lexicographic order is meaningful.
- Evaluation independence group cannot appear in training groups.
- Every target has one candidate, persistence and climatology forecast.
- Model, input, forecast-receipt and observation digests must be syntactically valid SHA-256.
- Method, unit, depth and spatial support must exactly match.
- Out-of-domain and unassessed forecasts remain in the issuance denominator but are not scored; abstention reasons are separate.
- Candidate skill compares to both baselines only on the exact target set scored by that candidate.
- Output includes MAE/RMSE/bias, Normal CRPS, interval coverage/width and Winkler score.

## Important limits

- The receipt and source digests are self-declared in this synthetic tool. The input SHA-256 binds the exact suite bytes; it does not authenticate source origin or prove a real forecast existed at the claimed time.
- The Normal CRPS uses a numerical error-function approximation and only supports a declared Normal predictive distribution.
- The interval is scored as supplied. V0.1 does not prove that the submitted interval corresponds to the submitted predictive distribution.
- One target per experimental unit is used in the fixture. Real forecast uncertainty must account for repeated site/plot/season dependence.
- All targets and forecasts are manufactured. No real soil model or field prediction is validated.
