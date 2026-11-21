# Synthetic NIR screening and confirmatory referral bake-off

This Rust crate compares **measurement-routing policies**, not spectral prediction models. The fixture is manufactured and the evaluator's hidden reference values are never passed to deployable selector functions.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/spectral-screening-referral/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/spectral-screening-referral/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/spectral-screening-referral/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/spectral-screening-referral/Cargo.toml -- \
  research/terra-preta/tools/spectral-screening-referral/fixtures/synthetic-referral.json \
  /tmp/referral-report.json
```

## Policies

- NIR-only (no referral).
- All-reference testing (upper-cost/reference baseline).
- Seeded random fixed-budget referral.
- Fixed uncertainty threshold.
- Highest uncertainty first.
- Applicability-domain-first referral.
- Highest expected threshold-decision risk first.
- Perfect-information hindsight oracle (evaluator-only, not deployable).

The decision-risk policy is a Gaussian threshold-risk heuristic. It is **not** a full Bayesian expected value of sample information (EVSI) implementation. The hindsight oracle uses synthetic truth only after selection and is only a bounded comparison.

## Selector/evaluator separation

`ScreenCandidate` contains only NIR-visible prediction and approved synthetic decision-profile metadata. The separate `EvaluatorOnly` structure contains hidden synthetic reference truth. The `select` API accepts only `ScreenCandidate` and `Config`; it has no reference to `EvaluatorOnly`.

A test mutates evaluator truth and verifies that deployable policy selection is unchanged. The report identifies its sample IDs with a SHA-256 digest; it does not disclose an external data identity.

## Interpretation

The output compares referral fraction, cost, false positives, false negatives, weighted decision loss and regret against full-reference and hindsight-only bounds. It is not an instrument calibration, soil-property prediction, treatment effect, agronomic recommendation, or proof that NIR can replace wet chemistry.

Real-data use remains blocked pending:
- exact Zenodo version and source SHA-256;
- resolution of NIR reflectance vs converted absorbance semantics;
- sample/cohort identity crosswalk and valid grouping keys;
- property-specific model calibration / applicability-domain evaluation;
- independently approved agronomic thresholds and false-positive / false-negative costs;
- real reference-method uncertainty, repeatability and confirmatory-test performance.
