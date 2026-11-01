# Sequential Bayesian experiment-design benchmark v0.1

Synthetic-only Rust prototype for sequential categorical observations and Bayesian updating.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-sequential-scenario.json \
  sequential-bayes-report.json
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

The report aggregates repeated deterministic seeds and records true synthetic decision utility/regret, predictive surprise events, diagnostic measurements, final posterior entropy, mean negative log predictive score, multiclass Brier score and action-selection frequencies.

## Limits

This is an exact finite categorical model, not a continuous soil simulator. It assumes conditionally independent action outcomes and does not model correlated sensor noise, heteroscedastic likelihoods, continuous measurements, censored data, a learned discrepancy posterior, or physical water/solute equations. A surprise is evidence that the candidate model mixture assigned low predictive probability; it does not by itself identify the missing physical mechanism.

No field data or physical experiment is authorized. The synthetic true world is deliberately outside the candidate model family to test misspecification handling, not to represent a measured soil regime.
