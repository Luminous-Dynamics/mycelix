# GCPN Forecast Information and Noise v0.1

Standalone, dependency-free Rust implementation of the weighted information/noise projection diagnostics described by Bonavita & Geer.

## Diagnostic convention

For each case, the engine:
1. reports raw weighted mean bias and raw RMSE against the verifying values;
2. forms forecast and verifying anomalies against the same frozen climatology;
3. subtracts the separate weighted mean of each anomaly vector;
4. computes the weighted projection of the mean-centered forecast anomaly onto the mean-centered verifying anomaly;
5. reports forecast information (FI), information error (IE), orthogonal noise error (NE), forecast/truth activity, and the anomaly-centered error standard deviation.

With the same normalized weighted inner product:

- `anomaly_centered_error_std² = IE² + NE²`;
- `raw_RMSE² = raw_bias² + anomaly_centered_error_std²`.

The first identity attributes the centered error spread to information error and orthogonal noise. The second retains mean bias as a separate source of total error. FI is not clipped and may be negative or exceed one.

The engine does not fit or apply calibration. It records declared post-processing provenance and rejects confirmatory leakage. The raw forecast and verification are compared on identical cases, climatology, source/version and support.

## Fail-closed controls

- Climatology, source/version, temporal support and spatial support are frozen by contract.
- Every case vector and positive cell-weight vector must have matching dimensions.
- Zero truth-anomaly norm yields a typed degenerate status and unavailable FI/IE/NE, not NaNs.
- Post-processing provenance is validated when an operation is declared; missing fit digests, evaluation-outcome use and overlapping training cutoffs are rejected.
- Per-case outputs are retained; no pooled statistic silently hides heterogeneous regimes.
- Sampling uncertainty remains with the independent-system bootstrap programme.

## Run

```sh
cargo run --manifest-path Cargo.toml
cargo test --manifest-path Cargo.toml --locked --offline
cargo clippy --manifest-path Cargo.toml --locked --offline --all-targets -- -D warnings
```

This package is a deterministic synthetic verification diagnostic, not a forecast model, calibrator, operational warning service or claim of real-world forecast superiority.
