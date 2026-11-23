# Active event-sampling schedule evaluation v0.1

**Status:** synthetic-only evaluator of frozen schedules. This repository component does not generate a sampling policy. Symthaea EDGE-ACTIVE-SENSING-001 #9817 owns policy generation; this evaluator scores the schedule after it is frozen and before evaluator-only truth is revealed to the candidate.

## Paradigm

Optimize decision-relevant event capture per complete observation cost, not prediction quality conditional on a supplied input stream. But a schedule cannot be credited with observing data it did not request. Counterfactual schedules require a known-truth synthetic process or a sufficiently high-rate source stream with an explicit sensor measurement model. Sparse historical observations cannot reveal what an alternative schedule would have measured.

## Hidden-truth isolation

The input contains a fixed policy trace set and multiple evaluator-only truth cases. All policies are evaluated against every truth case using the same frozen trace. In this fixture, the public context stream is identical for a pulse case and a no-pulse case; only the evaluator's response truth differs. Thus a candidate cannot select a different schedule after seeing whether the hidden pulse exists.

In confirmatory use, the candidate process must receive only:
- context observations already acquired and made available;
- sensor modality / latency / rate / health contract;
- remaining observation budget;
- its own state and permitted prior observations.

It must not receive event start/end/peak, future target-response values, evaluator truth, or oracle metrics. The synthetic fixture file itself is evaluator-owned and must not be mounted into the candidate process.

## v0.1 controlled process

The response signal is a triangular transient above a stable baseline. Its hidden start, end, peak and amplitude are deterministic. A context signal has a measured step at the same time in both hidden cases. Candidate schedules contain only requested modality and request timestamp; context actions must bind to a public observation ID. The evaluator computes target sensor values at acquisition time after latency.

## Policy arms

- `fixed_sparse`: fixed low-rate target sampling.
- `context_trigger_burst`: context samples followed by a dense target-sample burst.
- `hindsight_oracle`: evaluator-only upper bound using hidden event timing; never a deployable policy.

The evaluator does not claim that the burst policy is optimal. It shows a particular sensitivity/cost trade-off that Symthaea can use when developing and comparing its own policies.

## Metrics

For each policy × hidden truth case:
- target and context sample counts; wake count;
- pulse detected at a predeclared response-departure threshold;
- detection delay and peak-time error;
- sampled peak anomaly, peak magnitude error and capture ratio;
- event-window integral only when exact hidden event boundaries are both sampled and every within-event gap is within the maximum;
- false-burst count when target samples are concentrated in a no-pulse risk window;
- maximum target sample gap;
- energy, assay cost, communication bytes and total action count.

Resource dimensions stay separate. Do not add mJ, assay-cost units, bytes and time into one scalar without a preregistered application utility conversion. Report a Pareto frontier rather than a universal single score.

The pulse integral is the anomaly area in the response's declared unit × hours. It is not automatically a seasonal gas mass. A physical flux integral needs the correct area / mass basis, chamber calibration, time coverage and uncertainty model.

## Budget and timing contract

- Request times are strictly increasing; an action cannot be requested before the preceding action completes.
- Context actions must reference an existing public context observation at exactly request time + context latency.
- Target measurements occur at request time + target latency.
- Target sample spacing, sample counts, total action counts, energy, assay cost and communication bytes are bounded.
- A schedule that violates a contract is rejected, not clipped or silently downsampled.
- No target integral is emitted when an event boundary is missing or the maximum within-event gap is exceeded.

## Adversarial cases

- same public moisture step with and without a latent response pulse;
- pulse narrower than the fixed sampling interval;
- event starts immediately after the fixed policy sampled;
- context-triggered burst in a no-response case;
- target sensor latency shifts acquisition off the pulse;
- sample-budget / rate / action-order / clock violations;
- oracle-labelled schedule accidentally marked deployable.

## Proposed qualification targets (not achieved)

- Exact resource and timing accounting for every action.
- Zero evaluator-truth fields in the candidate action schema.
- Zero accepted schedules that exceed sample, latency, rate or resource budgets.
- Zero qualified integrals without observed event boundaries and adequate within-event cadence.
- Compare fixed, adaptive and oracle schedules across pulse width, amplitude, onset and no-pulse families.
- Demonstrate at least one predeclared Pareto improvement over fixed sparse sampling without increasing missed high-consequence event exposure in the declared regime.
- Report worst-case detection delay, false bursts and pulse-area error, not just average event recall.

## Related work

- Symthaea EDGE-ACTIVE-SENSING-001 #9817 (policy generation).
- Symthaea OLI-EXPERIMENT-001 #11446 (adequacy-gated experiment selection).
- Mycelix #6447 / PR #6448 (event-resilience outcome audit).
- Mycelix #6405 / #6420 (field GHG and seasonal event coverage).
- Mycelix #5527 (multi-year observation and persistence).

## Boundaries

The fixture is synthetic and the policy traces are authored test schedules, not Symthaea output or hardware measurements. No battery-life saving, South African emission factor, field event forecast, causal management effect or agronomic recommendation is established. No field sampling or intervention is authorized.
