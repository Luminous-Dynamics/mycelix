# Adaptive experiment policy bake-off v0.1

Synthetic-only Rust benchmark for comparing seven experiment-selection policies under a shared cost budget.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/adaptive-design-bakeoff/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/adaptive-design-bakeoff/Cargo.toml -- \
  research/terra-preta/tools/adaptive-design-bakeoff/fixtures/synthetic-adaptive-scenario.json \
  adaptive-design-report.json
```

## Policies

- seeded random feasible sampling;
- expert-fixed priority;
- one-factor-at-a-time order;
- Bayesian expected-information-gain proxy;
- representative-only;
- transparent R-IDeA-inspired comparator (not a reproduction of the published method);
- robust decision-focused selection with minimum target-stratum coverage and model-disagreement/error-amplification penalties.

## Leakage boundary

The selector receives only PolicyAction records and a separate PolicyContext containing the budget, seed and target-coverage constraints. Both types exclude evaluator-only synthetic truth, and the selector receives no Scenario reference. The exact feasible oracle and utility evaluator read synthetic truth only after policy selection. Policies that violate mandatory target-stratum coverage retain their raw synthetic utility but report null oracle regret because they are outside the oracle's feasible comparison set. The runner rejects every data class other than synthetic-known-truth and caps exact oracle enumeration at 20 candidates.

The fixture contains an attractive but ineligible action, high-information/misspecified actions, three target strata, different model disagreement and a fixed budget. The 14 authored tests check oracle utility, exclusion, cost limits, deterministic replay, coverage and regret ordering, empirical data-class rejection, model-metadata consistency, infeasible quotas, budget reservation for unmet target strata, EIG attraction to misspecified actions, expert-priority coverage failure, selector isolation from evaluator-only truth, a 32-seed invariant sweep, target-distribution sensitivity, and undefined regret for coverage-infeasible policies even when their raw utility exceeds the constrained oracle.

## Interpretation boundary

The R-IDeA-inspired score is a transparent local comparator; it is not Tang et al.'s published algorithm. One fixture cannot establish general superiority. Repeat on versioned, preregistered known-truth scenario families and seeds before comparing policies. No real sample, material, field trial or experiment is authorized by this tool.


## Acquisition scores

The implementation's exact v0.1 score definitions and oracle objective are documented in `research/terra-preta/ADAPTIVE_EXPERIMENT_POLICY_BAKEOFF_V0.1.md`. All greedy scores are divided by action cost. The robust policy reserves budget for the remaining target-stratum minimums before accepting a candidate. The R-IDeA-inspired score is not the published R-IDeA algorithm.

