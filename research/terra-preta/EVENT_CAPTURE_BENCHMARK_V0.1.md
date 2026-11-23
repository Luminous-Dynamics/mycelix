# Known-truth soil event-capture benchmark v0.1

**Status:** synthetic schedule evaluator. Policies submit only requested timestamps; the evaluator receives a separate dense latent trace and scores the frozen schedule. The tool does not execute or optimise a sensing policy.

## Why a separate benchmark is needed

Singh et al. (2026), *How Important Are the Critical Points in Selecting the Optimal Samples for Accurate Estimation of Subsurface Soil Moisture?*, Geophysical Research Letters, DOI [10.1029/2026GL124661](https://doi.org/10.1029/2026GL124661), selected informative training samples from hourly ISMN data at eight sites. Uncertainty-based critical-point selection performed well in some Dwc sites with 10% training data, while distribution-based selection performed well in BWk sites over training fractions of 10–62%. This is evidence for climate-dependent training-point selection.

That is not live sensor scheduling. Selecting rows from an already-observed hourly record does not prove that a sensor policy would capture a narrow pulse while asleep. A sparse series cannot reveal its own counterfactual unobserved peak. The event-capture benchmark therefore uses manufactured high-rate truth in a distinct evaluator input.

## Separation of concerns

1. **Policy submission:** policy identity/version, code and config digests, family, and requested timestamps. It contains no truth values or sampled measurements.
2. **Evaluator-only truth:** dense high-rate latent response/forcing, baseline window, event boundaries and detection thresholds. It is a separate file and is not passed to the policy process.
3. **Scorer:** resolves only exact requested timestamps against the truth grid, constructs sampled observations, and calculates event capture and resource metrics.

This CLI accepts separate policy and evaluator files. That separation is necessary but not sufficient to prove a future policy process was truth-blind; an actual sandbox/runner must enforce filesystem and data access isolation. The static synthetic policy schedules in this PR are benchmark fixtures, not live Symthaea policy executions.

## Event-level metrics

For each event and submitted schedule:
- event detected / missed using an explicit minimum departure threshold;
- first detection time and detection delay;
- true and sampled peak magnitude/time and absolute peak error;
- true event-integrated absolute anomaly;
- sampled trapezoid estimate only if exact event boundaries are sampled and every inter-sample gap is within the declared event-integration maximum;
- absolute and relative integral error when the integral qualifies;
- number of samples inside the event window;
- explicit reason when an event integral is unavailable.

The baseline is computed from the evaluator's designated pre-event truth window. It is never passed into the policy submission. A policy detects an event only if an acquired sample crosses the declared detection threshold. Event onset, end and peak are known only to the evaluator.

## Resource and comparison metrics

- requested / acquired sample count;
- per-sample cost and total cost units;
- configured sample and cost budget;
- samples outside all event windows;
- false triggers outside events;
- policy family and frozen schedule digest.

A future bake-off should compare fixed 1h/3h/6h/12h/daily cadence, stratified/distribution selection, random schedules with declared seeds, uncertainty-triggered, physical threshold/event-triggered, entropy/EIG, decision-EVSI and an evaluator-only hindsight oracle. Compare at matched cost or show full Pareto frontiers; do not emit a single composite winner score.

## Fail-closed rules

- Policy and evaluator inputs are separate files with separate SHA-256 digests.
- The policy schema has no measurement-value or latent-truth fields; unknown fields are rejected.
- Schedule times must be strictly ordered and exactly match an evaluator truth timestamp. No interpolation or nearest-neighbour sampling.
- Sample and cost budgets are enforced.
- Pulse integration requires measured event boundaries and no excessive gap. Otherwise it is null with a reason.
- A missed event is not labelled event-free; report missed-event status explicitly.
- No truth-derived baseline, event labels, peak, forcing or response values are supplied to the policy process by this evaluator.
- Output files are create-new and never overwritten.

## Synthetic fixture

The fixture contains one broad and one narrow pulse with a dense hourly truth series, plus three equal-budget static schedules: fixed cadence, event-focused and pseudo-random. Their purpose is to verify the scorer's behaviour under known truth, not claim that an adaptive policy has been trained or tested.

## Qualification targets (proposed, not achieved)

- At matched sample/energy cost, improve narrow-pulse detection and event-integral accuracy versus fixed cadence without unacceptable false-trigger burden.
- Zero false claims of event absence when no acquired sample resolves the event.
- Zero use of evaluator truth by the policy process in a future isolated runner.
- Performance reported by pulse width, amplitude, timing, antecedent state and outage regime.
- Explicit abstention when sample spacing exceeds the event integration limit.

## Literature and related work

- Singh et al. (2026), Geophysical Research Letters, DOI [10.1029/2026GL124661](https://doi.org/10.1029/2026GL124661): physics-aware critical-point selection for subsurface soil moisture.
- Zhao (2026), *Data Assimilation and Modeling Frontiers in Soil–Water Systems*, Water 18(4), 440, DOI [10.3390/w18040440](https://doi.org/10.3390/w18040440): multi-source observation operators, data assimilation, predict-then-verify twins, tracer constraints and decision-aware benchmarking.
- Mycelix #6447 / PR #6448: descriptive event response and pulse accounting.
- Symthaea #9817: generic active-sensing policy engine; this evaluator is its soil event consumer, not a duplicate optimizer.

## Boundaries

All fixture values are manufactured. The benchmark is not an emissions model, field observation, operational sensing policy, climate emission factor, causal experiment or intervention recommendation. Real deployment requires high-rate sensor characterization, local scientific review, permission, preregistered sampling, and a sandbox that enforces policy/evaluator separation.
