# Prospective soil forecast audit

Synthetic-only Rust verifier for immutable pre-outcome soil forecasts and later independent reference observations.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/prospective-forecast-audit/Cargo.toml -- \
  research/terra-preta/tools/prospective-forecast-audit/fixtures/synthetic-prospective-forecasts.json \
  /tmp/prospective-forecast-report.json
```

The tool checks temporal order, evaluation/training group separation, immutable receipt uniqueness, method/unit/depth/spatial-support matching and model applicability-domain status. It calculates MAE/RMSE/bias, Normal CRPS, interval coverage/width and Winkler score, plus matched candidate skill versus persistence and climatology.

The Normal CRPS is a numerical approximation via an error-function approximation. Real use needs validated predictive distributions, reference-method uncertainty, independent site/season groups and prequential observations. The synthetic receipt digest is not origin authentication.
