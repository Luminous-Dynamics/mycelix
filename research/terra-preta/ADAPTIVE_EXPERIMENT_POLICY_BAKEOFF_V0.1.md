# Known-truth adaptive experiment policy bake-off v0.1

**Status:** synthetic software benchmark only. The scenario fixture includes evaluator-only truth labels. It does not represent soil observations and cannot authorize or recommend any physical experiment.

## 1. Objective

Implement the work in #5599 as a deterministic, machine-readable bake-off of candidate experiment-acquisition policies under matched cost budgets. Evaluate whether selection improves the target service decision without concentrating all measurements where the current model is already confident.

This is deliberately a benchmark harness, not a claim to reproduce Tang et al.'s R-IDeA algorithm. Its representative/informative/de-amplifying (R-IDeA-inspired) score is a transparent comparator. The published method remains the reference for future exact reproduction.

## 2. Policies

1. **Random feasible:** seeded random order over eligible actions.
2. **Expert fixed:** descending, preregistered expert priority.
3. **One-factor-at-a-time:** preregistered design order, representing a conventional sequential factor-screening schedule.
4. **Bayesian EIG proxy:** mean expected information gain across candidate models, divided by action cost.
5. **Representative-only:** target-stratum weight and unmet-coverage deficit, divided by action cost.
6. **R-IDeA-inspired:** representativeness + model-averaged informativeness + de-amplification score, divided by cost. This is not an implementation of the published R-IDeA acquisition function.
7. **Robust decision-focused:** first respects declared minimum action coverage for each target stratum, then ranks by conservative decision value, informativeness, representativeness and error-amplification control with a model-disagreement penalty.

Every policy uses the same candidates, cost, model score inputs and budget. Ineligible actions are excluded before ranking, never merely penalized.

## 3. Separation of selector inputs and synthetic truth

The scenario carries:
- target stratum weights and minimum coverage;
- total discrete cost budget;
- feasible/ineligible action state and reason;
- action cost and target stratum;
- model-specific predicted decision value, expected information gain and error-amplification risk;
- expert and one-factor-at-a-time ranks;
- evaluator-only synthetic truth decision value.

The truth field is included only because the fixture is explicitly synthetic-known-truth. Acquisition scoring code must not read it. Evaluation code may read it only after selection and computes weighted utility / regret against a feasible oracle. No empirical datasets may be passed to this v0.1 harness.

## 4. Metrics

For every policy, report:
- selected action IDs, per-action reason code and deterministic selection rank;
- total cost and budget remaining;
- per-stratum target weight, selected count, minimum count and coverage status;
- weighted synthetic decision utility;
- regret and normalized regret versus the exact feasible oracle under the same budget and coverage constraints;
- mean selected action model disagreement and error-amplification risk;
- whether all minimum-coverage requirements were met.

The exact oracle enumerates all feasible subsets for small fixtures and is evaluation-only. Cap candidate count to 20 until a stronger exact/approximate optimization method is separately qualified.

## 5. Adversarial controls

The fixture must include:
- information-rich actions whose synthetic truth utility is low because candidate models omit a mechanism;
- a lower-weight target stratum that an information-only policy tends to neglect;
- action pairs whose model-specific decision values disagree;
- explicitly ineligible actions with attractive model scores;
- differing action costs;
- actions with poor representation and high error-amplification risk.

Tests must prove that selector functions do not consume the truth field, infeasible high-score actions are never selected, deterministic seeds reproduce random selections, cost limits are obeyed, coverage metrics are correct, and the oracle is never exposed to candidate scoring.

## 6. Proposed qualification targets (not results)

- 100% exclusion of ineligible actions.
- 100% budget compliance.
- Deterministic replay for fixed inputs and seed.
- Robust policy meets every feasible declared minimum-coverage constraint.
- On the adversarial fixture, robust decision-focused acquisition has lower normalized regret than information-only EIG and expert-fixed policies.
- Report all baseline failures; do not tune fixture truth and score fields after seeing policy output without versioning the scenario.
- No policy may use evaluator-only truth in ranking.

The benchmark's small synthetic scenario demonstrates logic, not general performance. Repeat across many preregistered scenario families and seeds before drawing policy-level conclusions.

## 7. Reference

Tang, R., Sloman, S. J., and Kaski, S. (2026). *Representative, Informative, and De-Amplifying: Requirements for Robust Bayesian Active Learning under Model Misspecification*. AISTATS 2026, PMLR 300, 3016–3024. https://proceedings.mlr.press/v300/tang26d.html

The published paper motivates the risk and the reference comparator; the current benchmark's transparent R-IDeA-inspired score is not a reproduction of their method.
