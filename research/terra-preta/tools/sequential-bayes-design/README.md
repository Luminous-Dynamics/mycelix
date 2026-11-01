# Sequential Bayesian experiment-design benchmark v0.1

Synthetic-only Rust prototype for sequential categorical observations and Bayesian updating.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-sequential-scenario.json \
  sequential-bayes-report.json
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-sequential-infamily-control.json \
  sequential-bayes-infamily-report.json
```

## Policies

- `static_eig`: selects using the initial prior's expected information gain (static ranking baseline).
- `conditional_eig`: recomputes expected information gain after every observed outcome.
- `decision_focused_evsi`: selects by expected value of sample information for a declared service decision.
- `discrepancy_guarded_evsi`: uses decision-focused EVSI plus an EIG tie-break and, after a low-predictive-probability event, prioritizes a predeclared diagnostic modality if one remains.
- `seeded_random`: deterministic feasible baseline.

For every selected action the evaluator samples a categorical observation from the hidden synthetic world. The selector updates a finite posterior over candidate models using the observed outcome and candidate likelihoods. The discrepancy flag is triggered when the observation's posterior-predictive probability falls below the preregistered threshold.

## Separation and metrics

Selector-facing types contain model priors, candidate-model likelihoods, declared decision utilities, eligible actions, costs, strata and modality metadata. They do not contain hidden true observation probabilities or true decision utilities. A negative-control test mutates all evaluator truth fields and verifies that the first selector choice is unchanged.

The report aggregates repeated deterministic seeds and records true synthetic decision utility/regret, predictive surprise events, diagnostic measurements, final posterior entropy, prequential negative log predictive score, prequential multiclass Brier score, and action-selection frequencies. It also scores every final posterior against the same evaluator-only held-out probes, using expected negative log score and expected multiclass Brier score overall and by target stratum. These common probes are not selected by the policies and do not consume acquisition budget; this removes the direct comparison confound caused by policies selecting different measurement modalities. A nuisance-signature control demonstrates that information gain can be substantial while decision EVSI is zero: calculated EIG is 0.3774776716 nats versus zero EVSI for the declared service decision.

## Calibration interpretation

Prequential scores describe prediction on each policy's selected measurement stream; the measurement operators can differ between policies, so those values are operational diagnostics rather than a fully matched calibration contest. Use `mean_heldout_validation_log_score`, `mean_heldout_validation_brier_score`, their standard errors / 95% normal-approximation intervals, and `validation_by_stratum` for the matched comparison: they score every policy's final posterior against the same fixed evaluator-only probe set and synthetic truth distribution.

Intervals are normal-approximation intervals over independent replicate means with the standard error estimated from the sample variance. They are not Bayesian credible intervals, do not account for scenario-family uncertainty, and should be supplemented with paired bootstrap intervals when comparing policies across large preregistered scenario suites. Common random numbers reduce action-level Monte Carlo noise but do not make different adaptive policies identical or remove the need for replicated worlds.

## Limits

This is an exact finite categorical model, not a continuous soil simulator. It assumes conditionally independent action outcomes and does not model correlated sensor noise, heteroscedastic likelihoods, continuous measurements, censored data, a learned discrepancy posterior, or physical water/solute equations. A surprise is evidence that the candidate model mixture assigned low predictive probability; it does not by itself identify the missing physical mechanism.

No field data or physical experiment is authorized. The synthetic true world is deliberately outside the candidate model family to test misspecification handling, not to represent a measured soil regime.
