# FSRE Decision Value and Active Investigation Benchmark v0.1

**Status:** Research protocol proposal; no operational value-of-information result is claimed.  
**Version:** 0.1.0  
**Date:** 2026-11-13  
**Primary owner:** Symthaea causal diagnosis / experiment selection (#10366), composed through the FSRE service and evidence contracts.  
**Claim ceiling:** This protocol specifies how to compare information-acquisition policies. It does not authorise data collection, intervention, sensor installation or operational actuation.

## 1. Research thesis

A resilient system should not merely reason over the data it happens to have. It should help determine **which observation, inspection, sample, test or human inquiry is worth acquiring next**, because that evidence could change a consequential decision.

The paradigm shift is:

- from collecting more data to acquiring decision-relevant information;
- from maximizing entropy reduction to maximizing the value of a safe, authorised decision;
- from one-shot diagnosis to sequential investigation with explicit stopping and abstention;
- from prediction accuracy alone to verified decision quality under information, time, cost and distribution constraints.

This is especially important in water systems. A sensor can improve anomaly detection while being useless for distinguishing two interventions. A laboratory sample may have low general information gain but high decision value if it separates a reversible maintenance action from an expensive plant shutdown. Conversely, an observation with high information gain can have zero practical value when every safe option remains the same.

Recent water-resources research identifies decision-relevant information selection as an underexplored part of policy learning and argues that information representation shapes decision quality. A separate 2026 systematic review of water-distribution leak detection reports inconsistent benchmark datasets, scenario definitions and evaluation practices that limit fair comparison. These support a benchmark based on matched decisions and acquisition budgets, not a single headline accuracy number.

References:
- Zaniolo, Giuliani & Herman (2026), “Toward Joint Information Selection and Policy Learning in Water Resources Management”, *Water Resources Research*, DOI: https://doi.org/10.1029/2025WR042745
- Farah & Shahrour (2026), “Standardization and Benchmarking of Datasets and Evaluation Protocols for ML Water Leak Detection”, *Water*, 18(18), 2267, DOI: https://doi.org/10.3390/w18182267

## 2. Formal decision contract

Let:
- (H) be the set of competing, explicitly scoped hypotheses about a service condition or failure mechanism;
- (E) be currently admissible evidence, including unknowns, provenance, validity and dependence;
- (A(E)) be the actions that are physically feasible, within the declared service boundary, legally and operationally authorised, and satisfy hard safety, rights, privacy and equity constraints;
- (L(a,H)) be the pre-registered loss or outcome function for action (a) under hypothesis (H);
- (m) be a candidate measurement or investigation, with possible outcomes (y), acquisition cost (C(m)), delay cost (D(m)), and any measurement risk.

The decision-value of sample information is:

```text
EVSI(m | E) = min[a in A(E)] E[L(a,H) | E]
             - E_{y ~ p(y | E,m)}[ min[a in A(E,y)] E[L(a,H) | E,y] ]
             - C(m) - D(m)
```

This is an evaluation construct, not a licence to trade away fundamental constraints. Hard constraints are applied before expected-loss comparison; they are not soft penalties that can be outweighed by a sufficiently large predicted benefit. Measurement itself must be safe, authorised, purpose-limited and rights-compliant. If measurement is not authorised or has unacceptable risk, it is not a candidate regardless of calculated EVSI.

(A(E)) may contain a documented no-op, safe hold, or abstention choice. When no consequential action is authorised, the system may still propose an authorised read-only measurement or recommend human review; it must not infer actuation authority from information value.

### 2.1 Distinguish four quantities

1. **Information gain:** how much an observation is expected to reduce uncertainty about hypotheses.
2. **Decision value:** how much the observation is expected to improve a decision under the declared loss and constraints.
3. **Operational option value:** whether observation preserves future reversible options, reduces lock-in or avoids an irreversible commitment.
4. **Realised service effect:** what changed in observed service outcomes after an authorised action, under an appropriate evaluation design.

They are not interchangeable. Lower entropy does not prove a better decision; a better decision recommendation does not prove a service effect.

## 3. Candidate investigation actions

An investigation action is a typed, scoped proposal, not an arbitrary prompt to “get more data”. Examples include:

- inspect a named sewer segment or pump station;
- obtain a meter calibration record or perform an authorised meter check;
- acquire a bounded interval of pressure, flow, power or pump telemetry;
- collect a wastewater sample using a documented chain of custody and laboratory method;
- reconcile an incident log with a maintenance completion record;
- ask an identified operator a bounded operational question;
- resolve an asset, catchment, service-cohort or municipality identity ambiguity;
- test whether a suspected dependency shares a common-cause source;
- wait for a scheduled sample or use an existing authorised observation instead of acquiring a new one.

Each proposal must state: question; hypothesis contrast; expected outcomes; owner and authority; exact subject/geography/time scope; method; cost; delay; safety and privacy implications; data retention; quality and calibration requirements; expected decision change; stopping rule; and evidence needed to verify completion.

A query that cannot plausibly change the safe action set or expected service outcome should normally not be selected, except where required for safety, legal compliance, audit or rights protection.

## 4. Sequential policy and stopping rules

The policy operates as a controlled investigation loop:

`evidence snapshot → competing hypotheses → admissible measurements → expected decision value → authorised acquisition → observation receipt → independent update → stop / continue / abstain`

At each step it must:

1. Freeze the current evidence frontier and exact time/scope basis.
2. Preserve competing hypotheses and counterevidence rather than collapse uncertainty to one narrative.
3. Exclude measurements lacking owner authority, safe method, lawful purpose or sufficient data-quality prerequisites.
4. Estimate outcome-conditioned decision value and include cost, delay, availability and distributional effects.
5. Compare with the best no-new-measurement decision and the best alternative measurement.
6. Emit a bounded recommendation with explicit uncertainty and expected decision change.
7. Acquire nothing until the relevant owner/operator grants permission.
8. Bind the actual observation to its source, method, instrument, time, geography, chain of custody and validity.
9. Recompute the hypothesis and action set without silently overwriting earlier evidence.
10. Stop when expected marginal decision value is non-positive, a pre-registered evidence threshold is met, the budget/time limit is reached, or the remaining hypotheses are not distinguishable at the available resolution.
11. Abstain and identify the unresolved uncertainty if no safe, authorised decision is supported.

The policy must be able to recommend **no measurement** and **no intervention**. Those are valid outputs, not failures to be hidden.

## 5. Benchmark task families

Begin with at least 50 independently specified challenge cases across the following families. Use new scenario seeds and a held-out, sealed set for final evaluation. A larger confirmatory set should be sized from the desired confidence bounds and expected failure rates rather than chosen to manufacture a reassuring pass rate.

| Family | Hidden condition / uncertainty | High-value investigation candidate | Adversarial control |
|---|---|---|---|
| Missing sewer flow | Unmetered branch or incomplete inlet coverage | Bounded flow survey / inspection | No measurement must not become zero flow |
| Wet-weather confounding | Rainfall-driven infiltration/inflow | Rainfall-aligned flow and level observation | Rainfall not contemporaneous with the measured interval |
| Meter drift | Biased flow or pressure instrument | Calibration record / independent reference meter | Drift is common to both nominally separate sources |
| Clock mismatch | Delayed, skewed or reordered telemetry | Clock-health and timestamp reconciliation | Future or overlapping observation |
| Common-cause outage | Shared electricity, communications or access failure | Independent backup-power / communications check | Two recovery options share the same latent dependency |
| Reticulation blockage | Sewer segment restriction | Targeted inspection or level/flow check | Plant inlet alone is non-identifying |
| Pump degradation | Intermittent pumping or control issue | Pump current, vibration or run-hour review | Sensor does not observe actual hydraulic delivery |
| Laboratory uncertainty | Sparse or invalid effluent sample | Repeat accredited sample under defined protocol | Missing sample falsely treated as compliant |
| Boundary change | Catchment, municipality or system merge/split | Identity and boundary crosswalk | Naïve longitudinal join across changed identifiers |
| Supplier / workforce scarcity | Spares or qualified staff unavailable | Confirmed stock and shift-capacity observation | Inventory listing is stale or shared across sites |
| Conflicting records | Different sources report incompatible completion/state | Independent field receipt or operator reconciliation | Two records duplicate one originating source |
| Irreversible option | A decision would close a future recovery path | Small reversible inspection or staged test | Apparent short-term benefit hides option destruction |

The synthetic challenge set should include both adverse and benign cases, low-risk cases where no action is needed, multiple plausible causes, and cases where the correct answer is “not identifiable”.

## 6. Matched-cost baselines and ablations

All policies receive the same initial evidence, hypothesis set, allowed measurement catalogue, authority constraints, cost model, wall-clock budget and compute budget.

Baselines:
- **B0 — Existing operator practice:** actual runbook, routine sampling and experienced operator choices.
- **B1 — Fixed cadence:** scheduled measurements independent of current hypotheses.
- **B2 — Random acquisition:** randomly select from the same admissible catalogue under the same total cost.
- **B3 — Maximum entropy reduction:** select the measurement expected to reduce hypothesis entropy most.
- **B4 — Maximum uncertainty reduction:** target the largest predictive interval or variance reduction.
- **B5 — Highest predicted incident probability:** investigate the highest-risk score without explicit decision-value calculation.
- **B6 — Operator/expert prioritisation:** qualified human investigation team with the same evidence and acquisition budget.
- **B7 — FSRE decision-value policy:** owner-qualified measurements ranked by constrained expected decision value.
- **B8 — Simulation oracle:** best action under known synthetic ground truth; an upper bound only, never a deployable comparator.

Ablations:
- without causal hypotheses;
- without the time-qualified dependency graph;
- without spatial or service-cohort context;
- without acquisition cost and delay;
- without distributional service measures;
- without owner/authority checks (test environment only);
- without independent observation and outcome verification.

Do not compare a full-information oracle to a real-world policy as if they had equal information. Do not give FSRE a richer measurement catalogue or extra model access than the baselines.

## 7. Metrics

### Primary decision metrics

- **Decision regret:** realised or ground-truth loss difference against the best admissible decision with the same information budget.
- **Decision correction rate:** fraction of cases where an acquired observation changes an initially wrong decision to a correct admissible one.
- **Verified decision improvement per total acquisition cost:** include sensor, lab, inspection, personnel, delay, integration and independent verification costs.
- **Unnecessary acquisition rate:** measurements that do not change the action set, decision or a required safety/rights conclusion.
- **Time to decision-quality evidence:** not merely time to model output.
- **Safe abstention precision/coverage:** when the policy abstains, how often is the case genuinely non-identifiable or insufficiently evidenced; how often can it still provide a supported answer?

### Epistemic and operational metrics

- calibration of hypothesis probabilities and predictive intervals;
- false-cause and false-localisation rates, reported by attack family and cohort;
- sensitivity to missingness, source dependence, clock error and boundary changes;
- evidence frontier completeness and lineage integrity;
- observation failure / invalid sample / sensor outage rate;
- decision stability under irrelevant evidence reorderings and harmless perturbations;
- option preservation and reversibility;
- measurement burden and operator-hours;
- false reassurance and false escalation rates;
- independent reproduction time and cost.

### Distributional and rights metrics

Report results by service cohort, geography and relevant service tier where lawful and sufficiently aggregated. Track who bears measurement burden, who receives the service improvement, who experiences outage/delay, and whether the policy systematically deprioritises low-data communities. Do not publish sensitive personal or critical-infrastructure details. A high national mean cannot conceal service deprivation in a small cohort.

## 8. Provisional targets and release gates

These are prospective research targets, not achieved results:

| Metric | Target / gate |
|---|---:|
| Unauthorised measurements or actuation | 0 |
| Silent conversion of unknown to zero | 0 |
| False causal attribution on sealed negative controls | 0 |
| Scope/time/owner binding for every acquired observation | 100% |
| Deterministic replay agreement on frozen synthetic cases | 100% |
| Nominal 90% predictive interval coverage | 85–95% on held-out data after calibration protocol is pre-registered |
| Improvement over strongest matched-cost baseline | Positive with uncertainty interval excluding zero before making a superiority claim |
| False localisation | Report per family; no numeric target until ground truth and resolution are defined |
| Evidence acquisition cost | Fully accounted; do not omit labour, travel, lab or delay costs |
| Distributional service guardrails | No subgroup degradation masked by an aggregate gain |

Zero failures in a small suite do not provide a useful production probability bound. Report numerator, denominator, attack coverage, confidence interval, evaluator version and held-out split. A benchmark pass only supports its declared scope.

## 9. Relationship to current SOTA and industry

The leak-detection literature is useful but not a direct FSRE leaderboard. A 2026 systematic review of 90 water-distribution studies describes inconsistent benchmark inputs and evaluation definitions; a separate 2026 study reports high detection performance on more than 47,000 simulated Net3 scenarios, but detection and localisation are different tasks, and drinking-water network results do not transfer automatically to wastewater reticulation or municipal service outcomes.

Compare to:
- specialist hydraulic and water-network simulators on common calibrated physical tasks;
- IN-CORE, InfraRisk, REFIT, PRIIA and other interdependency tools where task fit and licensing permit;
- existing utility runbooks and human expert investigation;
- fixed-cadence and risk-priority monitoring;
- maximum-entropy and prediction-uncertainty sampling;
- the operator's current decision and measurement process.

The testable FSRE advantage is not that it predicts every failure better. It is that it may choose a lower-cost, decision-relevant observation; recognize when evidence is insufficient; preserve owner authority; and link a decision to an independently observed service outcome. If a specialist or incumbent workflow does this more accurately, safely and cheaply, FSRE should adopt or interoperate with it.

## 10. Integration with Luminous owners

- **Symthaea:** competing hypotheses, causal diagnosis, active experiment/measurement selection, uncertainty and stopping/abstention.
- **Symtropy:** physical model adequacy, measurement model, scenario dynamics, uncertainty propagation and sensor/inspection effect on physical-state identifiability.
- **Sol-Atlas:** spatial coverage, catchment topology, sensor/inspection placement, cohort boundaries and geographic provenance.
- **Mycelix:** evidence and source ownership, measurement permission, operator commitments, data-use constraints, authority and completed-observation receipts.

The benchmark is a consumer of those owner-native results. It must not create a duplicate dependency ledger, generic evidence ledger or independent authority system.

## 11. Falsification

The decision-value hypothesis is weakened or falsified if:
- simple fixed-cadence, risk-based or expert workflows deliver equal or better decisions at lower total cost;
- EVSI estimates are poorly calibrated or systematically overvalue measurements that do not change the action set;
- measurements improve model metrics but do not improve verified decisions;
- the policy repeatedly acquires data from well-instrumented communities while neglecting low-data cohorts;
- more information increases false attribution, privacy burden or unjustified intervention;
- the system cannot account for dependence between ostensibly independent evidence sources;
- real operators cannot use the outputs within existing safe work practices.

## 12. Immediate execution sequence

1. Add this protocol to the FSRE benchmark index and link Symthaea causal diagnosis / experiment-selection owner #10366.
2. Define a 50-case synthetic public challenge set with frozen hypotheses, admissible measurements, cost/delay, expected observation distributions and ground truth held outside the policy interface.
3. Implement a deterministic decision-value reference evaluator independent of Symthaea's model implementation.
4. Compare matched-cost baselines B0–B8 and ablations, recording complete per-case receipts.
5. Have an independent reviewer inspect loss functions, probability calibration, rights/safety constraints and sealed-case generation.
6. Only after synthetic evaluation, seek operator-approved historical replay and read-only shadow evaluation. Any prospective measurement, installation or intervention requires explicit operator and data-owner permission.

**Current status:** protocol only. No active measurement policy, EVSI result, field decision, sensor deployment or intervention is claimed in this commit.
