# Soil active-sensing benchmark v0.1

**Status:** deterministic synthetic task/evaluator adapter. It provides a known-truth soil-event environment to test sampling schedules and Symthaea's generic decision-focused planner. It is not a new VoI optimizer and does not make physical energy or field-emission claims.

## Why this benchmark matters

An hourly or daily observation design may miss a narrow post-rewetting pulse even when the final response returns close to baseline. A high-frequency reference stream lets us test what the sensor would have measured at a policy-requested timestamp without inventing counterfactual observations by interpolation.

The 2026 Africa spectroscopy review identifies a persistent transferability gap across laboratory, field and imaging platforms; PLSR remains dominant, and properties with indirect spectral responses (available P/K) have more uncertainty than directly responsive constituents such as organic carbon. The benchmark therefore keeps sensor acquisition / event detection distinct from chemical reference measurement and downstream agronomic decision.

## Ownership

- **Mycelix** owns the versioned soil task profile, synthetic scenario generator, evaluator metrics, receipts and evidence separation.
- **Symthaea #9817 / PR #9821** owns generic active-sensing policy selection and decision-focused value-of-information.
- **Symthaea #11446** owns model adequacy and experiment-selection reasoning.
- **Symtropy #1240 / #1467** can later supply coherent environmental snapshots and deterministic physical rehearsal.
- **Sol-Atlas #206** owns source and spatial evidence; it may render coverage and observation support but cannot convert a simulated sample into a field observation.

## v0.1 task

A scenario has:
- public task contract: horizon, initial state, maximum sample count, detection threshold and resource accounting;
- hidden evaluator truth: high-rate minute-resolution moisture and response series, optional event onset/width/amplitude and exact event integral;
- sensor model: deterministic timestamp-bound noise plus declared per-acquisition and wake costs;
- candidate policy profile: fixed cadence or rewetting-triggered cadence.

The policy callback receives only the current time, public observation history and public task contract. It does not receive event onset, width, amplitude, future values or oracle diagnostics. The current implementation executes candidate/evaluator simulation in one process; this is **not a hostile-process security boundary**. Confirmatory evaluation must move oracle ownership into a separately permissioned evaluator process.

## Baselines

1. Fixed cadence at an explicitly declared interval.
2. Rewetting-triggered cadence with an observed moisture-change threshold, short fast-sampling window and cooldown.

Later comparisons should include adaptive variance, response threshold, entropy-only sampling, Symthaea decision-focused VoI and adequacy-gated robust VoI from #9817. This PR does not reimplement those algorithms.

## Metrics

- Whether a pulse was detected from actually sampled observations.
- Detection delay relative to the hidden event start.
- Number of observations within the hidden event window.
- Peak magnitude estimate/error from sampled event-window observations.
- Sparse-sample trapezoidal pulse integral estimate/error, or explicit abstention when fewer than two event-window observations exist.
- Maximum observed sampling gap.
- Sample count, sensor energy and wake energy under the synthetic cost profile.
- False positive detection on no-pulse controls.
- Budget exhaustion / compliance.

All resource values in v0.1 are synthetic accounting inputs, not hardware measurements. Compute, latency, communication and memory-write costs are not physically measured here and remain outside this evaluator's qualified metrics.

## Known-truth and replay boundary

The hidden reference series is generated at one-minute resolution and is directly sampled at the policy-selected timestamps under the declared sensor model. There is no interpolation of a coarse recorded dataset to claim the counterfactual sensor reading. For real data, counterfactual schedule evaluation is permissible only when a high-rate source stream or validated sensor model supports the requested timestamps; otherwise mark the replay restricted.

## Hostile controls

- A short pulse immediately after the policy's last sleep decision.
- A large moisture/rewetting change with no response pulse.
- Pulse width below a fixed sampling interval.
- Sample-budget exhaustion.
- Duplicate/non-monotonic timestamps and invalid policies.
- Repeated schedule replay must be deterministic for the same seed/configuration.
- Candidate decisions must be a pure function of public contract/history, never hidden oracle values.

## Qualification targets (proposed, not achieved)

- At matched sample/energy budgets, reduce worst-case missed-pulse exposure versus fixed cadence without unacceptable false triggers.
- Report detection delay and pulse burden error separately from detection accuracy.
- Demonstrate decision-focused VoI improves a preregistered task-loss objective over entropy-only selection where irrelevant high-entropy observations exist.
- Report Pareto frontiers over recall, false alarms, delay, pulse error, samples, energy and assay cost; no unapproved scalarization.
- Zero candidate access to event truth, future observations or evaluator labels in a separately permissioned run.
- No physical battery-life, sampling-cost or emission claims before hardware/instrument validation.

## Scientific grounding

- Tiwari et al. (2026), drought/warming and high-frequency respiration responses, DOI [10.1111/gcb.71005](https://doi.org/10.1111/gcb.71005).
- Dryland rewetting N-oxide pulse study (2026), DOI [10.1016/j.agee.2026.110680](https://doi.org/10.1016/j.agee.2026.110680).
- Hmimou et al. (2026), *Soil Property Monitoring in Africa via Spectroscopy: A Review*, Environments 13(4), 228, DOI [10.3390/environments13040228](https://doi.org/10.3390/environments13040228).
- Zhao (2026), *Data Assimilation and Modeling Frontiers in Soil–Water Systems*, Water 18(4), 440, DOI [10.3390/w18040440](https://doi.org/10.3390/w18040440).

## Boundaries

All data in v0.1 are synthetic. No N2O/NO emission factor, soil-function response, field energy cost, treatment effect, or agronomic threshold is inferred. The task generator does not authorize field sensing or intervention.
