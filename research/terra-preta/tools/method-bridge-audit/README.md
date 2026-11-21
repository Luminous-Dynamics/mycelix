# Method bridge audit

Synthetic-only Rust auditor distinguishing rank concordance, numerical agreement and shared-threshold decision agreement.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/method-bridge-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/method-bridge-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/method-bridge-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/method-bridge-audit/Cargo.toml -- \
  research/terra-preta/tools/method-bridge-audit/fixtures/synthetic-rank-only.json \
  /tmp/method-bridge-report.json
```

Rank-only comparison between non-equivalent field and laboratory methods can emit Spearman rho, but not bias, MAE, RMSE or limits of agreement. Numerical agreement requires asserted equivalence and exact property/unit/basis/support matches. Decision agreement requires a common declared threshold; a synthetic threshold remains synthetic. V0.1 does not compute cluster-bootstrap intervals or field performance.
