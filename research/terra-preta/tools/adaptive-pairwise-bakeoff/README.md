# Adaptive pairwise design bake-off v0.1

Synthetic-known-truth Rust benchmark for a failure mode hidden by additive experiment scoring: redundant measurements can be individually attractive, while two complementary modalities can be more informative jointly.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/adaptive-pairwise-bakeoff/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/adaptive-pairwise-bakeoff/Cargo.toml -- \
  research/terra-preta/tools/adaptive-pairwise-bakeoff/fixtures/synthetic-pairwise-scenario.json \
  adaptive-pairwise-report.json
```

## Policies

- Seeded random feasible acquisition.
- Static additive greedy acquisition, which scores each measurement independently.
- Pair-aware joint look-ahead, which evaluates single actions and candidate pairs using selector-visible pair interaction estimates.

All policies first satisfy hard target-stratum minimums and then spend the remaining budget. The exact feasible-subset oracle evaluates evaluator-only synthetic truth for at most 20 eligible actions.

## Selector / evaluator isolation

The selector receives only `PolicyContext`, `PolicyAction`, and `SelectorPair`. Those types do not contain `synthetic_truth_standalone_value` or `synthetic_truth_interaction`. The evaluator reads those fields only after selection. A negative-control test mutates every hidden truth value and requires all policy selections to remain identical.

A pair interaction is a signed change in joint decision utility beyond the sum of its two standalone contributions. A negative value is redundancy; a positive value is complementarity. Unlisted interactions are zero in this deliberately small v0.1 fixture, not evidence that omitted real-world interactions are absent.

## Designed fixture

The fixture gives two nearby tracer measurements high standalone model scores, but a negative joint interaction. CT pore geometry and an event tracer each have lower standalone scores but a positive joint interaction. The exact oracle and pair-aware policy are expected to select the complementary pair under the same budget and coverage requirements; static additive greedy is expected to select the redundant pair.

These numbers are deliberately synthetic. They are not measured soil values, instrument performance estimates, or evidence that CT plus tracer is universally optimal.

## Important limits

This is **not** a full sequential Bayesian design algorithm. It does not simulate observation likelihoods, condition a posterior on measurements, estimate correlated sensor-noise covariance, represent higher-order interactions, or update model-discrepancy states. It uses one-step pair look-ahead with a static pair-interaction estimate. The next scientific extension must model likelihoods and sequential posterior predictive adequacy, not merely add more score weights.

No empirical-data analysis, sample collection, material handling, land access, field trial or experiment authorization is provided by this tool.
