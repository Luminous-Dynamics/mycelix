# Event-observation policy observability benchmark v0.1

**Status:** synthetic evaluator, not an active sensor policy. The evaluator receives a high-resolution known-truth trace and a candidate schedule with the exact readings sampled from that trace. A candidate policy must never receive the truth event catalogue, hidden event times, or unsampled observations.

## Why this benchmark is needed

Event-resolved soil observation has a fundamental identifiability limit: a model cannot reconstruct a short-lived pulse that the sampling schedule never observed without additional physical assumptions. The 2026 drought/rewetting literature reports that short post-rewetting pulses can make a material contribution to seasonal emissions in studied systems. The 2026 soil-sensing review argues for sensing at appropriate spatial and temporal supports, while existing commercial and public soil-health testing largely focuses on a selected laboratory indicator panel.

This benchmark isolates the sampling-policy question before adding soil-model complexity: does a policy spend its sensor budget where it can observe short events, or does it produce a reassuring but under-sampled trace?

## Candidate/evaluator separation

The evaluator accepts two separate files:
1. **Known truth**: dense time series, pulse windows and exact latent response. Evaluator-only.
2. **Candidate schedule**: timestamps, observed values, resource costs and budget. No event labels or truth windows are allowed in this file.

Every candidate observation must match an exact timestamp in the known-truth stream, and its reported response must match the evaluator's source value at that timestamp. V0.1 does not interpolate or synthesize a measurement at an unobserved time.

This is an offline controlled benchmark with a manufactured source stream. A real offline replay can only be called a valid schedule counterfactual when a validated high-rate source stream or physical sensor model supports the requested observations. Coarse recorded traces cannot reveal unseen pulse values.

## Metrics

For each known-truth pulse:
- truth peak and truth positive-anomaly integral;
- candidate observations within the pulse window;
- whether at least one in-window candidate sample exceeded the declared detection threshold;
- candidate observed peak and peak error when detected;
- pulse integral estimate only if candidate samples include both exact event boundaries and every within-window gap is below the predeclared cap;
- explicit missing-boundary or gap-exceeded status otherwise.

Across the schedule:
- sample count and fraction of available high-rate observations;
- maximum schedule gap;
- pulse recall and missed-pulse truth exposure;
- false alarms on sampled observations outside truth pulse windows;
- budget compliance;
- resource vector: sample energy, wake energy, compute cycles and transmitted bytes.

The resource vector is not collapsed into a scalar. Application-specific utility weights can be added only in a separate, declared decision layer.

## Proposed policy comparisons

At identical sample and resource budgets, compare:
- fixed-rate 1/2/4/6/12 observations per hour;
- threshold-triggered sampling;
- variance / change-triggered sampling;
- uncertainty-threshold sampling;
- entropy-based EIG;
- decision-focused EVSI;
- risk-aware goal-oriented acquisition;
- hysteresis + cooldown adaptive schedules;
- Symthaea HDC-only, CfC-only and HDC-LTC sensing policies where they can be run through the same causal candidate interface.

The same hidden event truth and sensor cost model must be used across policies. Publish a Pareto frontier over pulse recall, missed-pulse exposure, false alarms, peak error, integrated-mass error, sample count, wakeups, energy and transmission.

## Adversarial tests and targets (proposed, not achieved)

- Narrow high-consequence pulse immediately after the policy chooses to sleep.
- Two pulses with different width and amplitude under identical average sampling budget.
- High sensor variance but no decision-relevant event.
- Slow baseline drift that creates false triggers.
- Long idle periods with false wake-up storms.
- Sensor outage exactly during the pulse peak.
- Timestamp substitution, duplicate samples, fabricated response values and over-budget schedules.
- Zero evaluator acceptance of a sample timestamp/value not supported by the known-truth source.
- Event recall and missed-pulse exposure measured separately; a high average prediction score cannot hide missed rare events.
- Nominal 90% prediction coverage 85–95% on independent site/event groups in a future calibrated model; v0.1 does not estimate intervals.

## Links to the broader programme

- Mycelix #6447 / PR #6448: event-resolved soil resilience metrics.
- Symthaea #9817: decision-focused active sensing and resource-vector Pareto policies.
- Symthaea #11446: experiment design under model discrepancy and covariance.
- Mycelix #6405 / #6420: field GHG and seasonal event coverage.
- Mycelix #5528: hydraulic hysteresis and event forcing.
- Mycelix #5529: nutrient boundary mass balance.

## Boundaries

The fixture is manufactured. This evaluator does not establish real emissions, physical sensor performance, energy savings, field resilience or a causal management effect. No physical trial, access or sensing installation is authorized.
