# Soil trial operating-characteristics benchmark

Deterministic Monte Carlo benchmark for the blocked paired soil-trial ITT estimator. All data are manufactured. It reports Type-I error / power conditional on complete-trial qualification, alongside qualification/abstention rates, bias, RMSE, non-adherence and exact randomization resolution.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/soil-trial-operating-characteristics/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/soil-trial-operating-characteristics/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/soil-trial-operating-characteristics/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/soil-trial-operating-characteristics/Cargo.toml -- \
 research/terra-preta/tools/soil-trial-operating-characteristics/fixtures/synthetic-soil-trial-oc.json \
 /tmp/soil-trial-oc-report.json
```

## Important limits

The simulator is not a field-trial power calculator. Its DGP assumptions are explicit and synthetic. The v0.1 paired sign-flip test assumes valid within-block randomization and no interference. Missing endpoints and possible/unassessed interference abstain; no complete-case subset estimator is emitted. Monte Carlo rejection rates are conditional on qualification and must be read alongside the qualification rate. No confidence-interval coverage is emitted because a validated interval procedure is not implemented here.
