# Known-truth event sampling policy evaluator v0.1

**Status:** synthetic evaluator-only benchmark. It evaluates schedules proposed by an external policy engine; it does not choose sampling times or control sensors.

## Separation of policy and evaluator

The latent high-frequency truth is available only to the offline evaluator after the policy schedule is frozen. Policy actions contain decision times and observation IDs that were available at those decisions. They do not contain future truth values. The evaluator resolves requested sample times against the hidden synthetic truth after validation.

This prevents the common but invalid replay shortcut: reconstructing an unsampled event peak by interpolating a coarse time series and pretending the policy would have observed it.

## Scenario

The manufactured 24-hour soil-respiration response has a stable baseline, a forcing change before event onset, a narrow transient pulse during the declared event window, post-pulse recovery, and a high-frequency source truth on a fixed time grid.

Three policies are included:
1. Fixed 6-hour cadence: baseline/endpoint samples can miss the narrow pulse.
2. Adaptive from past: denser samples are requested only after earlier measured observations.
3. High-rate oracle: every truth time point is sampled; evaluation ceiling only, not a deployable policy.

## Policy action contract

Every requested sample stores a unique sample ID, decision time, requested sample time, action mode (precommitted or adaptive_from_past), observation IDs actually available to the policy at that decision, and a reason code.

A policy may refer only to prior observations sampled at or before its decision time. A sample cannot be justified by an observation that has not yet been acquired. Precommitted schedules have no observation basis.

The evaluator rejects future-observation leakage, duplicate IDs, sample times absent from the known truth grid, schedule time reversal and budgets over the scenario envelope.

## Metrics

- Event capture / miss against the known pulse interval and minimum departure.
- True peak magnitude/time versus sampled peak magnitude/time.
- Absolute and relative peak error.
- True event integrated absolute anomaly.
- Sampled trapezoid estimate only when both event boundaries are sampled and every interval stays within the maximum integration gap.
- Absolute and relative integrated-burden error where integration qualifies.
- Sample count, sample energy and maximum event-window gap.
- Explicit integration abstention for missing boundaries or excessive gaps.

The evaluator never bridges an unobserved pulse. If a schedule misses the pulse, that is a miss, not a zero-flux observation. If the event integral is under-resolved, it remains null.

## Baselines

Compare fixed cadence, forcing-threshold trigger, uncertainty trigger, decision-risk trigger, adaptive-from-past and high-rate oracle. All deployable schedules must be compared under matched sample-count and energy budgets. The synthetic oracle is only a ceiling.

## Qualification targets (proposed, not achieved)

- Zero future-observation references.
- Zero qualified integrals across gaps larger than the predeclared event maximum.
- Pulse-capture recall, peak error and integrated-burden error by pulse width / timing / amplitude.
- Report energy and total sample cost alongside capture.
- At matched sensor energy, adaptive sampling should reduce missed high-consequence pulses versus fixed cadence without unacceptable false wakeups.
- Report Pareto frontiers; do not hide cost in a single accuracy score.
- Future calibrated 90% event-response intervals target 85–95% empirical coverage on independent site/event groups.

## Extension plan

The next version should add multiple pulse families, sensor measurement noise/drift, waking latency, variable sample cost, forcing thresholds, multiple sensors/modalities, competing event priorities and regret-weighted decisions. Symthaea #9817 owns policy selection; Mycelix owns the reproducible evaluator and evidence receipt.

## Sources

- 2026 dryland rewetting pulse paper, Agriculture, Ecosystems & Environment, DOI https://doi.org/10.1016/j.agee.2026.110680: 3–5 day rewetting pulses contributed up to 18% of seasonal N-oxide emissions in that study.
- Tiwari et al. (2026), Global Change Biology, DOI https://doi.org/10.1111/gcb.71005: hourly observations showed drought/warming suppression of overall soil respiration while rewetting pulses could be amplified in the studied temperate pasture.

These are not South African emission factors and are not used as numeric response assumptions in this fixture.

## Boundary

All data are manufactured. This is not a sensor driver, real-time monitor, field emission estimate, field trial authorization or physical sampling instruction. Real replay requires an actual high-rate stream and explicit measurement model.