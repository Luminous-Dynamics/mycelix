# Event-resolved soil resilience benchmark v0.1

**Status:** synthetic-only event-analysis contract. One scenario represents one event in one experimental unit. The auditor is descriptive; it does not estimate causal treatment effects, predict field response, infer microbial mechanisms, or certify resilience.

## Scientific motivation

Recent field studies show that drought and warming can suppress average soil respiration while amplifying short-lived rewetting pulses. A 2026 hourly field experiment in temperate pasture found distinct drivers for overall soil respiration and rewetting-induced pulses. A 2026 dryland study reports that 3–5 day post-rewetting N-oxide pulses contributed up to 18% of seasonal emissions in its studied systems. These are domain-specific findings, not transferable emission factors for South Africa.

A soil may return to a similar final value while still generating a substantial transient nutrient or GHG pulse. Static pre/post snapshots and coarse annual means can therefore miss consequential event behaviour.

## Four separate response dimensions

1. **Resistance / disturbance magnitude:** maximum absolute deviation from a pre-event baseline during the stress window, with signed departure and timing retained.
2. **Recovery:** post-stress mean, signed departure from baseline and descriptive recovery fraction relative to the observed stress peak.
3. **Recovery time:** first time an independently declared tolerance band is met for the required consecutive observations. No tolerance, no recovery-time claim.
4. **Pulse burden:** time-integrated absolute deviation from baseline over stress and recovery. It is calculated only when the event onset and recovery endpoint are observed and every consecutive time gap is within the predeclared maximum. It never bridges a missing event or long gap.

Do not collapse these into a single resilience score. A high recovery fraction does not cancel a harmful peak or cumulative pulse burden.

## Event and observation contract

Each scenario declares one event, event type and forcing; one experimental unit and independence group; baseline, stress and recovery windows; response variable, unit, method and support; an ordered sequence of repeated observations with response and forcing uncertainty; a maximum permitted gap and minimum phase counts; and optional recovery tolerance with approval provenance.

Repeated sensor readings are time-series observations from the same experimental unit. They are not independent treatment replicates. V0.1 analyses one event and does not aggregate across plots or infer between-treatment effects.

## Event-window conventions

Time is elapsed hours relative to declared event onset:
- baseline: `baseline_start_hours <= t <= event_start_hours`;
- stress: `event_start_hours < t <= stress_end_hours`;
- recovery: `stress_end_hours < t <= recovery_end_hours`.

The event-start observation is the last baseline-side observation and anchors integration. The recovery-end observation is the last recovery-side observation and anchors the integral. Both boundaries must be measured at their exact declared times for the pulse integral to qualify.

## Metric definitions

Let `B` be the arithmetic mean of baseline observations.

- Stress peak: the stress observation maximizing `abs(response - B)`; report its value, signed departure, absolute departure and time.
- Recovery mean: arithmetic mean over recovery observations.
- Recovery fraction: `1 - abs(recovery_mean - B) / abs(stress_peak - B)`. It can be negative after an overshoot; it is not clamped. It is null when the stress peak did not differ from baseline.
- Recovery time: first recovery observation beginning the required run of consecutive observations inside `[B - tolerance, B + tolerance]`, minus stress-end time. Consecutive samples must themselves be no farther apart than the declared maximum gap.
- Pulse burden: piecewise-linear integral from event onset to recovery end, split exactly at any baseline crossing. Report positive excursion, negative excursion, signed net area and absolute anomaly area separately; each has unit `response_unit × hours`. The absolute area is the sum of positive and negative magnitudes. It is null if either boundary is absent or any interval exceeds the maximum gap.

Per-observation standard errors are retained in the input, but v0.1 does **not** propagate event-metric uncertainty because serial covariance, drift, shared calibration and event-sampling dependence are not modelled. Outputs are descriptive only.

## Missingness and abstention

- If minimum observations in any phase are not met, event response metrics remain null and status is `insufficient_phase_coverage`.
- If an event boundary is not observed or a time gap exceeds the configured maximum, pulse burden is null with a reason; no interpolation across the gap is allowed.
- If no recovery tolerance is declared, recovery time is null.
- If a pulse is too short or cadence too sparse to resolve it, abstain; do not call the event pulse-free.
- Events outside the observed horizon cannot be assigned a recovery time.

## Future model comparisons

Compare sparse pre/post measurements, daily/weekly event averages, event-resolved high-frequency observations, static-state soil models, path-dependent hydraulic-memory models (#5528), event-aware GHG and nutrient-fate budgets (#6405, #6420, #5529), and hierarchical longitudinal/state-space models. A meaningful improvement must show better held-out peak/timing/pulse-burden prediction at equivalent sampling cost, with calibration, mass-balance uncertainty and site/event transfer reported.

## Proposed qualification targets (not achieved)

- Zero integrals that bridge gaps beyond the maximum interval.
- Synthetic pulse-capture sensitivity evaluated across pulse width, amplitude, timing and missingness.
- Separate error / coverage reporting for peak magnitude, peak time, recovery fraction, recovery time and integrated burden.
- Nominal 90% event-prediction intervals target 85–95% empirical coverage on independent site/event groups in a future calibrated model.
- Explicit abstention where cadence or observation support cannot resolve the event.
- No pseudo-replication across readings, cores, sensors or treatment units.
- No causal treatment claim from a single time series.

## Literature

- Tiwari et al. (2026), *Drought and warming-induced drying suppress soil respiration but amplify rewetting-induced pulses in a temperate pasture*, Global Change Biology, DOI [10.1111/gcb.71005](https://doi.org/10.1111/gcb.71005).
- Dryland rewetting-pulse study (2026), Agriculture, Ecosystems & Environment, DOI [10.1016/j.agee.2026.110680](https://doi.org/10.1016/j.agee.2026.110680).

## Boundaries

Synthetic controls are not field observations. This benchmark authorizes no drought manipulation, warming, amendment application, fertilizer change, sampling access, or physical intervention. Actual trials require qualified local scientific review, appropriate permissions, ecological safeguards and preregistration.
