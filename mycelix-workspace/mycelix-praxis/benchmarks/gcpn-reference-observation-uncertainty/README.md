# GCPN Reference-Observation Uncertainty v0.1

A dependency-free Rust qualification engine for a narrow, explicit additive observation-error model.

## Estimator

For a declared model `observed = latent_target + known_bias + epsilon`, with zero-mean error and known total error variance:

`Var(epsilon) = measurement_variance + representativeness_variance + 2 * covariance`

the engine reports:
- Naive observation-space MSE.
- Error-corrected latent-target MSE estimator: `mean((forecast - (observed - known_bias))^2 - total_error_variance)`.
- Separate bias, measurement variance, representativeness variance and cross-covariance summaries.
- Unique declared error-lineage roots and independent-system counts.

The corrected estimator is unbiased under the declared additive model and known parameters. Corrected estimates can be negative in finite samples; they are never clipped. This does not imply universal latent-target identification.

## Fail-closed conditions

Unknown bias, measurement variance, representativeness variance or covariance disables correction. Support, temporal support, source, version, calibration epoch and as-of mismatches reject input. Different native spatial support requires a declared observation-operator identity and an evaluated support digest matching the predictand; this engine does not itself apply the operator.

The 2x2 measurement/representativeness covariance matrix must be positive semidefinite.

## Run

```sh
cargo run --manifest-path Cargo.toml
cargo test --manifest-path Cargo.toml --locked --offline
cargo clippy --manifest-path Cargo.toml --locked --offline --all-targets -- -D warnings
```

This is synthetic method qualification, not sensor calibration, forecast certification, causal inference or operational readiness.
