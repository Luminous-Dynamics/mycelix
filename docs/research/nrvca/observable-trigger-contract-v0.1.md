# NRVCA Observable Trigger Contract v0.1

**Status:** synthetic trigger-engine specification and known-answer traces  
**Machine-readable fixture:** [observable trigger traces](../../../fixtures/nrvca-observable-trigger-traces-synthetic-v0.1.json)  
**Implementation:** \`mycelix-workspace/tools/nrvca-validator/tests/observable_trigger_contract.rs\`  
**Tracking issue:** [NRVCA-009 / #5425](https://github.com/Luminous-Dynamics/mycelix/issues/5425)

## Why trigger semantics matter

NRVCA-005 currently maps stressor tags directly to actions, such as releasing a storage reserve when a scenario carries a low-source-availability tag. That is a useful evaluator shortcut, but not an operationally complete adaptive policy. A usable decision rule needs a measurable signpost, unit, threshold, sampling/persistence definition, lead time, reset behavior, and an explicit response to missing or delayed information.

This work follows and composes established Dynamic Adaptive Policy Pathways (DAPP) and monitored decision-rule methods; it does not claim to invent trigger-based planning.

- Haasnoot et al. (2013), *Dynamic Adaptive Policy Pathways: A Method for Crafting Robust Decisions for a Deeply Uncertain World*: https://doi.org/10.1016/j.gloenvcha.2012.12.006
- Murgatroyd & Hall (2021), *Selecting Indicators and Optimizing Decision Rules for Long-Term Water Resources Planning*: https://doi.org/10.1029/2020WR028117
- Haasnoot et al. (2018), *Designing a monitoring system to detect signals to adapt to uncertain climate change*: https://doi.org/10.1016/j.gloenvcha.2018.08.003
- Hermans et al. (2017), *Designing monitoring arrangements for collaborative learning about adaptation pathways*: https://doi.org/10.1016/j.envsci.2016.12.005

The literature emphasizes that triggers depend on observable indicators, appropriate thresholds and timely/reliable monitoring; trigger design is not just a Boolean condition.

## Paradigm shift: monitor → validate → signal → authorize → act → verify → re-arm

Each declared rule specifies:
- canonical indicator ID and unit;
- activation comparator and threshold;
- number of consecutive valid readings required;
- separate recovery threshold and recovery persistence (hysteresis);
- maximum observation age;
- cooldown and re-arm requirements;
- action cost, estimated action lead time and preconditions;
- reasoned fallback on missing, stale or invalid data;
- version, provenance/claim ceiling and decision-right boundary.

This v0.1 is **recommendation-only**. It emits deterministic audit events from synthetic observation traces; it cannot command a pump, grid, crew or public service. Thresholds, sample rates, costs and lead times are entirely illustrative and uncalibrated.

## Synthetic example rules

| Action | Signpost | Activation | Recovery/re-arm | Illustrative lead time / cost |
|---|---|---|---|---|
| Release storage reserve | Source availability ratio | ≤ 0.65 for 2 consecutive valid samples | ≥ 0.75 for 2 samples; 3-step cooldown | 1 step / 2 synthetic cost units |
| Deploy backup power | Grid availability ratio | ≤ 0.20 for 1 valid sample | ≥ 0.80 for 2 samples; 4-step cooldown | 0 steps / 3 synthetic cost units |
| Preposition repair crews | Predicted repair delay | ≥ 24 hours for 2 consecutive valid samples | ≤ 18 hours for 2 samples; 2-step cooldown | 1 step / 1 synthetic cost unit |

These are test values, not proposed thresholds for Johannesburg or any actual utility. They do not represent local service observations, physical calibration, dispatch authorization, or real action costs.

## Missing and stale data

A missing observation breaks the persistence sequence and produces a manual-review event. A stale observation is likewise ineligible to activate a rule. Neither is converted to zero, reused silently as the current reading, or treated as “normal operation.” The synthetic trace tests an eventual trigger after two new fresh readings: the action may be recommended only once the evidence contract is satisfied.

A future live system needs sensor/device identity, calibration and quality flags, synchronized clock, observation and ingest times, expected sampling cadence, communications status, and a policy for contradictory indicators. Missing information may require human escalation, but the appropriate real-world emergency fallback must be specified by authorized operators; this test harness does not choose or execute a physical intervention.

## Hysteresis, cooldown and action prerequisites

Activation threshold and recovery threshold are separate to avoid repeatedly switching a recommendation on small fluctuations around one boundary. A trigger remains latched until enough recovery readings have arrived and cooldown has elapsed. A single spike does not satisfy a two-reading rule.

Preconditions are checked before issuing the recommendation. If the action plan is not authorized or the required reserve/backup/crew is unavailable, the recommendation is suppressed and the trace records the reason. This makes “signal detected” distinct from “action available” and “action authorized.”

The lead-time check compares the declared time until a protected service floor is expected to be breached with the declared action lead time. If the action lead time exceeds the remaining lead time, the audit event includes a late-warning flag. The test does not predict physical breach times; they are synthetic inputs.

## Rust tests / acceptance targets

The integration test replays six frozen traces: reserve activation with recovery and re-arming; grid activation/cooldown/re-arming; two-sample repair delay; a one-sample spike; missing and stale data; and an unavailable action precondition.

It checks:
- exact fire events and full per-sample audit decision logs;
- monotonic evaluation time, unique observation IDs, valid unit/value domains and valid quality status;
- persistence, hysteresis, cooldown and lead-time warning;
- missing/stale data causing manual review rather than a trigger;
- no recommendation when mandatory action preconditions are absent;
- duplicate timestamps, incompatible units, out-of-range values and undeclared indicators failing closed;
- display-label changes not changing decisions;
- every trigger carrying a rule version, observation reference, action ID, reason, lead-time fields and synthetic evidence status;
- action costs staying within the synthetic single-action cap and automatic dispatch remaining disabled.

Static inspection of the checked-in data cannot replace executing the Rust integration test on the exact branch head. No CI pass is claimed until the focused workflow completes successfully.

## Next steps to qualify a real trigger policy

1. Replace synthetic indicators only with permissioned, documented operational sources and measured units.
2. Validate the signpost’s timeliness, reliability, data-quality behavior, and institutional ability to act on it.
3. Test thresholds on held-out historical events and compare missed-trigger, false-trigger, lateness and cost trade-offs.
4. Assess the entire action chain: preconditions, resource availability, action lead time, trigger-to-action verification, cooldown/reset and interactions between simultaneous actions.
5. Review decision rights, safety boundaries, privacy and fail-safe behavior with the responsible operator before any shadow use.
6. Compare policies against incumbent rules and DAPP/RDM baselines under identical observations, constraints and evaluation budgets.

The contribution here is making trigger semantics testable and auditable, not claiming the illustrative rules are robust in a real water system.
