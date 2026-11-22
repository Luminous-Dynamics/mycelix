# Randomized soil intervention causal audit

Standalone Rust evaluator for a **synthetic two-arm randomized complete-block trial**.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/randomized-soil-trial-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/randomized-soil-trial-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/randomized-soil-trial-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/randomized-soil-trial-audit/Cargo.toml -- \
  research/terra-preta/tools/randomized-soil-trial-audit/fixtures/synthetic-randomized-trial.json \
  /tmp/randomized-trial-report.json
```

## Estimator

Within each block:
`(treatment follow-up - treatment baseline) - (control follow-up - control baseline)`.

The ITT estimate is the mean block contrast. The two-sided exact randomization p-value enumerates all (2^B) within-block sign flips under the sharp null for 2–20 blocks. ITT uses assigned treatment, not treatment received. Subsample counts are reported separately and never inflate randomized-unit n.

## Claim boundaries

- Only a plot-level randomized complete-block design is accepted in v0.1.
- Missing outcomes, incomplete blocks, non-0.5 assignment probability, future preregistration, support mismatch and invalid allocation digests reject.
- Possible or unassessed interference causes causal estimate abstention.
- No per-protocol estimate, observational effect, mechanistic claim or transportability claim is emitted.
- A schema-valid synthetic allocation record does not prove that a real trial was randomized as declared.
