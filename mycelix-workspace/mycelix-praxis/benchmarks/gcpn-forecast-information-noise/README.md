# GCPN Forecast Information and Noise v0.1

Standalone dependency-free Rust implementation of a weighted vector projection diagnostic for forecast information, information error and orthogonal noise.

## Diagnostic convention

For each case, the engine:
1. computes raw forecast bias and raw RMSE against verifying values;
2. forms forecast and verifying anomalies from a frozen climatology;
3. removes the case-mean raw forecast bias from the forecast anomaly;
4. computes the weighted projection onto the verifying anomaly;
5. reports forecast information (FI), information error (IE), orthogonal noise error (NE), forecast/truth activity and debiased RMSE.

With the same normalized weighted inner product, the projection construction satisfies debiased_RMSE^2 = IE^2 + NE^2. FI is not clipped and may be negative or exceed one.

The case-mean bias removal is an explicit v0.1 convention; the raw bias and raw RMSE remain visible. It is not a fitted calibration method.

## Fail-closed controls

- Climatology, source/version, temporal support and spatial support are frozen by contract.
- Every case vector and positive cell-weight vector must have matching dimensions.
- Zero truth-anomaly norm yields a typed degenerate status and unavailable FI/IE/NE, not NaNs.
- Post-processing provenance is mandatory when a processing step is declared; missing digests, evaluation-outcome use and overlapping training cutoffs are rejected.
- Per-case outputs are retained; no pooled statistic silently hides heterogeneous regimes.

## Run

```sh
cargo run --manifest-path Cargo.toml
cargo test --manifest-path Cargo.toml --locked --offline
cargo clippy --manifest-path Cargo.toml --locked --offline --all-targets -- -D warnings
```

Sampling confidence intervals use the separate independent-system qualification programme. The package does not generate forecasts, fit calibrators, or claim real-world forecast superiority.
