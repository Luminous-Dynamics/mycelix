# mycelix-business-derived-forecast-actual

Deterministic transaction-level projection into forecast evaluation actuals.

A forecast outcome such as “total item demand from 08:00–09:00” is usually not one raw POS transaction. This crate therefore keeps two types separate:

- `MetricObservation`: one normalized witness-derived external observation;
- `DerivedMetricActual`: a deterministic evaluation value derived from a preregistered set of those observations.

The aggregate is never promoted into a raw observation.

## Projection v0.1

`ActualProjectionSpec` is preregistered no later than the exact forecast target plan and binds:

- target-plan digest;
- source input identity;
- aggregation semantics (`Sum` in v0.1);
- registration time; and
- deterministic projection digest.

For every preregistered target, projection selects exact replayed observations matching input, metric, scope, unit/scale and the half-open interval `[window_start, window_end)`. Values are accumulated with checked `i128` arithmetic.

Each derived actual binds:

- exact target ID and semantics;
- projection and target-plan digests;
- canonical campaign-replay digest;
- derived fixed-point value;
- source-observation count;
- source-observation-set digest; and
- deterministic derived-actual/evidence identities.

A target with no matching accepted transactions produces an evidence-backed zero with a source count of zero; it does not disappear from evaluation.

## Derived forecast cases

`DerivedForecastCase` keeps candidate/baseline forecasts paired with a `DerivedMetricActual`. `score_derived_cases` preserves the same conservative scoring rule as the existing shadow scorer: candidate abstention inherits baseline absolute error rather than escaping the denominator.

## Boundary

This crate does not itself prove the upstream export is complete and does not authorize business actions. Strong qualification must construct its replay through `mycelix-business-campaign-replay`, require the preregistered target plan, and retain upstream completeness limitations until independently discharged.
