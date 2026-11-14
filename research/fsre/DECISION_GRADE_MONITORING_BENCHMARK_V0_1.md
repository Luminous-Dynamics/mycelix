# FSRE decision-grade monitoring benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research protocol. No sensor-placement algorithm, field deployment, comparative performance result or SOTA advantage is claimed.

## 1. Paradigm shift: optimise decisions enabled, not sensors installed

The conventional sensor-placement objective is often framed as observability, coverage, detection probability, localisation accuracy or cost per sensor. Those are important technical measures, but they do not prove that a utility receives trustworthy data early enough to make a safe and useful decision.

A sensor can be:
- optimally placed but unmaintained or miscalibrated;
- physically measuring at time (t), but the observation reaches the control room after the action window;
- accurate in isolation but redundant with another sensor sharing the same power or communications failure;
- sensitive to a leak but unable to distinguish a consequential event from harmless operational variation;
- deployed in well-instrumented areas while vulnerable cohorts and remote communities remain unobserved.

The benchmark therefore evaluates **decision-grade monitoring**: the quality, timeliness, freshness, independence and action relevance of information as it reaches the responsible decision process, and whether that information produces independently verified service improvement.

This is an integration/assurance hypothesis, not a claim that FSRE invented sensor observability, sensor placement, Bayesian value of information, or water-network monitoring.

## 2. Existing SOTA and task-specific baselines

The benchmark must include, where the task and data permit:

1. Structural-observability sensor placement, including van Gemert et al. (9 November 2026), demonstrated on real Brabant Water networks and designed to reduce dependence on exact hydraulic parameters where the topology is known.
2. Continuous score-based pressure placement, including Hu & Khu (2 November 2026), which uses continuous residual scoring rather than only binary thresholds and reports robustness under uncertain/spatially varying noise.
3. Probabilistic sensor placement with Dynamic Residual Masking, including Damonti & Bernasconi (November 2026), whose study reports 91.4% detection accuracy with five sensors and 4.2% false positives in its own evaluated configuration.
4. Cascading-aware graph-signal sensor placement, including Huang et al. (2026), which combines structural, hydraulic and leakage-related risk propagation.
5. Observability-Gramian/binary optimisation, sensitivity/Jacobian-based placement, topology-only placement, entropy/information-gain placement, greedy placement and utility-native sensor layouts.
6. Current operator practice: control-room alarm rules, inspection schedules, sampling plans, maintenance, calibration, crew dispatch and laboratory turnaround.

The 2026 review of sensor/sampling placement in water distribution and wastewater networks reports substantial method trade-offs and identifies lifecycle maintenance, sensor degradation, delayed response and operational constraints as incompletely represented in many optimisation studies.

These are not directly comparable headline scores. A 91.4% detection score from one network/event split is not a universal sensor-placement SOTA value. Exact network, scenario, noise, threshold, label, sample, hardware and holdout details must be frozen before making comparisons.

## 3. Evidence-to-decision timeline

Every candidate measurement is evaluated across the full timeline:

1. **Sample/phenomenon time:** when the sensor or sampler observes the physical process.
2. **Transmission time:** when the measurement reaches the relevant data system or control room.
3. **Quality-control time:** when calibration, validation, laboratory review or plausibility checks finish.
4. **Decision availability time:** when the responsible decision role can lawfully and practically use the evidence.
5. **Decision cutoff:** the point at which an operational choice must be made.
6. **Action/response time:** when the intervention can actually begin and when its service effect is observed.

The as-of evidence gate uses the declared decision-availability time, not merely sensor sampling time. The benchmark preserves the original evidence and later corrections. It must not allow a later laboratory result or delayed telemetry packet to leak into an earlier decision replay.

## 4. Sequential value-of-information accounting

For a candidate measurement (m) acquired at time (t), let:
- (EVSI_m(t+Delta_m)) be the expected reduction in decision loss when the measurement is actually available after turnaround/communications delay (Delta_m);
- (C_{acq,m}) be acquisition and analysis cost;
- (C_{life,m}) be allocated lifecycle cost, including maintenance, calibration, replacement and data transmission;
- (C_{delay,m}) be the service loss or opportunity cost accumulated while waiting;
- (C_{risk,m}) be the operational/safety cost of obtaining or using the observation.

Then the benchmark compares:

[
NetValue(m)=EVSI_m(t+Delta_m)-C_{acq,m}-C_{life,m}-C_{delay,m}-C_{risk,m}.
]

The exact cost and loss model must be declared by the study owner. A measurement that is highly informative but arrives after the decision deadline may have zero or negative decision value for that episode.

The candidate-selection policy is sequential: select at most one strictly positive net-value measurement from a frozen evidence snapshot; after an authorised observation becomes available, recondition the hypothesis weights and select the next measurement. Do not sum overlapping EVSI values from several candidates evaluated against the same prior.

The comparison must include static placement, greedy information gain, sequential one-step EVSI, finite-horizon or adaptive acquisition where available, and current operator practice. FSRE does not claim the greedy policy is globally optimal.

## 5. Matched benchmark protocol

For identical network topology, hydraulic/physical state, event set, sensor budget, lifecycle spend, noise assumptions, population/service boundaries and action authority, compare:

- B0: existing utility sensor layout and operating practice;
- B1: topology-based/structural observability placement;
- B2: hydraulic sensitivity/Jacobian or observability-Gramian placement;
- B3: entropy/information-gain/greedy placement;
- B4: continuous-score and probabilistic placement methods;
- B5: cascading-aware graph/risk-propagation placement;
- B6: sequential EVSI measurement acquisition;
- B7: qualified human operator + monitoring team;
- B8: strong AI/optimisation agent with the same data, budget and authority;
- B9: FSRE decision-grade composition: owner-native monitoring outputs, claim-specific freshness, as-of admissibility, sequential EVSI, portfolio/resource constraints and independent outcome reproduction.

Run ablations that remove, one at a time: data-latency treatment, freshness policy, sensor maintenance, correlation/common-cause evidence, service-cohort floors, authority checks, sequential reconditioning, or independent outcome verification.

Do not compare algorithms on different network/event distributions or grant one method privileged information or a larger sensor/maintenance budget.

## 6. Required metrics

Report a vector, not one detection score:

| Dimension | Metrics |
|---|---|
| Detection | Event precision/recall, false alarms per sensor-day, missed-event severity, detection delay |
| Localisation | Top-1/top-5 source localisation, spatial error, uncertainty calibration |
| Observability | Structural/functional observability, sensor count, coverage by hydraulic zone and population served |
| Data quality | Missingness, drift, calibration error, noise sensitivity, uptime and stale evidence rate |
| Timeliness | Observation-to-availability latency, QC/approval latency, fraction arriving before decision deadline |
| Information value | Net EVSI, decision regret, expected loss reduction per total acquisition/lifecycle cost |
| Service outcome | Safe service-hours preserved, time below floor, cumulative deficit, recovery time and relapse |
| Equity | Monitoring coverage and service outcomes by protected cohort and geography; unresolved denominators remain explicit |
| Operations | Crew workload, sample turnaround, maintenance burden, schedule instability, analyst effort and integration cost |
| Reproducibility | Independent reference agreement, input digest, owner/source lineage and as-known-at-the-time replay |

All confidence intervals and denominators must be explicit. Random-row splitting alone is insufficient; hold out catchments, utilities, event families and time periods where data allow.

## 7. Hard gates and proposed comparative targets

For deterministic conformance:
- 0 future observations or delayed evidence used before the declared availability time;
- 0 stale or invalid sensor evidence accepted outside its claim-specific policy;
- 0 known-effective revocations accepted;
- 0 false service-floor satisfaction claims;
- 0 unauthorised consequential actions;
- 100% deterministic agreement with the independent reference on frozen cases;
- 100% reporting of sensor uptime, freshness, data missingness, lifecycle assumptions and denominators.

After operator data access and powered held-out evaluation, proposed comparative targets are:
- at least 10% lower decision regret than the strongest matched baseline under the same total lifecycle budget and action constraints;
- or at least 10% lower lifecycle cost per independently verified safe service-hour preserved, without worsening any protected cohort's service floor;
- no reduction in detection sensitivity for critical events beyond a pre-registered non-inferiority margin;
- calibrated uncertainty and interval coverage reported separately by event family, geography and service cohort.

These are hypotheses to test, not current results or guarantees of SOTA.

## 8. Integration boundary

- Existing domain engines and sensor-placement algorithms remain authoritative for their modelling tasks.
- Mycelix supplies dependency/common-cause, provenance, authority and temporal receipts.
- Sol-Atlas supplies spatial reachability, access and population boundaries.
- Symtropy supplies physical model feasibility and state receipts.
- Symthaea proposes hypotheses and sequential information-gathering candidates; it does not authenticate observations or authorize actions.
- The as-of evidence gate evaluates whether a record was available and valid at a frozen decision cutoff.
- FSRE composes claims and measures decision/service outcomes.
- Operators and lawful authorities retain sensor procurement, sampling, dispatch and actuation authority.

## 9. Falsification

The FSRE contribution is not useful if:
- existing sensor-placement and utility systems already produce the same decision outcomes with equivalent temporal/provenance guarantees;
- evidence turnaround is not the bottleneck and the added qualification layer only adds latency;
- sequential acquisition provides no measurable decision value over current operator practice;
- the method improves aggregate outcomes while reducing a protected cohort's service below its authorised floor;
- extra data governance or audit cost exceeds the value of the independently verified improvement.

The objective is not a larger or more intelligent sensor network. It is a smaller, better-maintained and more decision-relevant monitoring system whose evidence arrives when it can change a safe and equitable service decision.
