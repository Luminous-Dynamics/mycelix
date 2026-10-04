# AC-062: Endogenous Forecast Evaluation

## Purpose

AC-062 establishes a causal-evaluation boundary for economic cognition. It records whether an observed outcome was uninfluenced, intervention-affected, or causally unresolved, rather than treating every deviation from a prior analysis as ordinary forecast error.

## Why ordinary forecast scoring is insufficient

Once a policy analysis can inform a governed decision, the resulting intervention may change the path that the analysis described.

Examples:

- a predicted downturn is avoided after an authorized intervention;
- a warning changes private behaviour and contributes to the resulting event;
- several models coordinate through policy and market responses, changing the evidence observed later.

A passive forecast score cannot distinguish those cases.

## Evaluation boundary

`EconomicOutcomeEvaluation` binds:

- exact analysis identity and fingerprint;
- optional exact scenario identity;
- exact target identities with native units;
- explicit forecast/scenario horizon start and end;
- exact observed-outcome snapshot fingerprint;
- information cutoff and outcome-capture timestamps;
- evaluation timestamp;
- intervention and governance references;
- uncertainty and missing-data references.

## Separate causal dimensions

`EconomicEvaluationKind` distinguishes observational backtests from scenario outcomes.

`EconomicOutcomeInfluence` distinguishes:

- `NoKnownIntervention`;
- `InterventionAffected`;
- `InfluenceUnknown`.

`EconomicMeasurementStatus` distinguishes complete, partially observed, and invalidated measurement states.

These dimensions are intentionally separate. A partially observed result can still be intervention-affected; an intervention-affected result can still be measured completely.

## Anti-leakage invariants

Validation rejects:

- information cutoffs after the analysis was generated or after the horizon began;
- analyses generated after the evaluated outcome was captured or after the horizon began;
- horizons whose end precedes their start;
- outcome capture before the evaluation horizon begins;
- observation-only backtests carrying intervention/governance references;
- `InterventionAffected` outcomes without intervention or decision evidence;
- `ScenarioOutcome` evaluations without exact scenario identity;
- scenario bindings that differ from the analysis;
- outcome snapshot fingerprints that do not match the supplied snapshot;
- mismatched snapshot capture times;
- complete measurements carrying missing-data references;
- partially observed measurements without missing-data evidence;
- invalidated measurements without limitation evidence;
- complete measurements captured before the horizon ended.

## What the evaluator does not do

AC-062 does not assign a universal accuracy score, reward policy success, or infer causal effect from correlation alone. It creates an auditable information boundary from which later causal or statistical evaluation can operate.

## Strategic significance

The resulting feedback loop becomes:

observations → scenarios → analyses → governance → intervention → observed outcome → evaluation → institutional memory.

This is particularly important for AI-mediated economies because BIS identifies correlated model behaviour as a possible source of procyclicality and contagion, while IMF scenario work treats AI's macroeconomic effects as dependent on diffusion, assumptions, and policy response rather than a single deterministic trajectory.

## Qualification

Repository CI is the qualification source. No local cargo test pass is claimed from an environment without the repository checkout.

## Measurement and horizon semantics

Target native units are descriptive measurement metadata only; AC-062 does not perform unit conversion or valuation. The horizon is the period being evaluated, while `information_cutoff_at` is the latest information permitted to define the claim. Validation requires the information cutoff to be at or before horizon start and requires the referenced analysis to exist before horizon start, preventing an ex-post observation from masquerading as an ex-ante forecast.

Measurement status is also semantic: `Complete` cannot carry missing-data references and requires capture at or after horizon end; `PartiallyObserved` requires explicit missing-data evidence; `Invalidated` requires an explicit uncertainty or missing-data limitation.
