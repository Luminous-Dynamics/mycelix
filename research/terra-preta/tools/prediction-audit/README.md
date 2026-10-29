# Grouped prediction audit

This standalone Rust CLI computes descriptive metrics and a deterministic group-cluster bootstrap interval for an already-frozen held-out prediction file. It does **not** train a model, decide which groups are independent, validate source labels, or prove agronomic efficacy.

## Input contract

CSV must contain these exact column names:

```csv
group_id,actual,candidate_prediction,baseline_prediction,interval_lower,interval_upper
study-a,10.0,11.0,8.0,8.0,13.0
study-b,20.0,19.0,18.0,16.0,22.0
```

Each row is one scored observation. `group_id` must identify the true independent cluster used for resampling (for example, a source study, site, or batch consistent with the preregistered split). Do not use row IDs or treat repeated endpoints from one experiment as independent groups. The tool requires at least two groups and rejects blank groups, group IDs with whitespace aliases or placeholder values, non-finite numeric values, missing required fields, and reversed prediction intervals. Results with fewer than ten groups carry an explicit warning that the bootstrap interval is exploratory; ten is a warning threshold, not proof of adequate power.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/prediction-audit/Cargo.toml
cargo run --release --manifest-path research/terra-preta/tools/prediction-audit/Cargo.toml -- \
  /path/to/held-out-predictions.csv /path/to/prediction-report.json
```

An optional third argument fixes the bootstrap RNG seed. Otherwise, a stable default is used. The JSON output contains a SHA-256 digest of the exact CSV bytes, observation and group counts, candidate/baseline MAE and RMSE, candidate signed bias, empirical coverage and width of the candidate's nominal 90% prediction intervals, the mean 90% interval score, relative MAE improvement, and a 95% percentile interval from 2,000 group-cluster bootstrap replicates. The interval score is width plus a penalty of 20 times the distance for a miss outside the interval; lower scores are better, balancing sharpness and coverage. The report stores only the input filename, not the full local path. The CLI refuses to overwrite the input file if the report path resolves to the same file.

## Interpretation

- A positive relative MAE improvement means the candidate's MAE is lower than the supplied baseline's MAE on this file.
- The bootstrap resamples whole groups, preserving all rows in each sampled group; it is not a row-wise bootstrap.
- Prediction-interval coverage is descriptive for the supplied held-out observations. The evaluator assumes the supplied intervals are nominal 90% intervals and records that assumption. The interval score is a proper score for that nominal level, but neither metric demonstrates calibration on future or shifted populations.
- If baseline MAE is zero, relative improvement is undefined and no relative-improvement interval is reported.
- A confidence interval excluding zero does not prove causal identification, field efficacy, safety, profitability, or regulatory compliance.
- The input's held-out status, split integrity, source crosswalk, and model freeze must be verified separately. Run the split-manifest verifier and preserve the frozen split manifest alongside the report.

The tool reports metrics only; target thresholds must be predeclared in the benchmark protocol and cannot be chosen after inspecting results. Do not score on the training data or reuse this output as evidence of local South African applicability unless the evaluation level and provenance support that claim.
