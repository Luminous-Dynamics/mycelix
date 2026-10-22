# NRVCA Scenario Evidence Contract v0.1

**Status:** synthetic-only contract / draft research artifact  
**Companion fixture:** [nrvca-scenario-evidence-manifest-v0.1.json](../../../fixtures/nrvca-scenario-evidence-manifest-v0.1.json)  
**Stress evaluator:** [NRVCA-005 / PR #5410](https://github.com/Luminous-Dynamics/mycelix/pull/5410)  
**Tracking issue:** [NRVCA-006 / #5417](https://github.com/Luminous-Dynamics/mycelix/issues/5417)

## Why this is the next step

NRVCA-005 verifies that a 5-scenario × 3-strategy outcome matrix is complete and that its deterministic scores reproduce their frozen answers. It does not yet prove that the scenario descriptions have all relevant dependencies, that a repeated-event case includes the history needed to make sense of reserve depletion, or that future users cannot relabel an assumption as an observation.

This work adds executable provenance. A scenario is no longer only a human-readable story: its stressor IDs resolve to canonical definitions; dependency and history requirements are structured; evidence tiers have required fields and claim ceilings; and source links have roles.

The conceptual direction composes existing Decision Making under Deep Uncertainty (DMDU), Robust Decision Making, decision scaling, and adaptive-pathways methods. It is not a claim that NRVCA invented those methods. The US Climate Resilience Toolkit describes stress-testing strategies across plausible futures and analyzing where they are vulnerable. The 2026 Water Utility Climate Alliance guide is specifically about applying DMDU to water-utility planning. The World Bank guidance notes that multiple approaches can be combined for large, complex decisions.

- US Climate Resilience Toolkit: <https://toolkit.climate.gov/course-lesson/decision-making-under-deep-uncertainty>
- Water Utility Climate Alliance / The Water Institute (2026): <https://www.cakex.org/documents/beginners-guide-decision-making-under-deep-uncertainty-water-utilities>
- World Bank, Investment Decision Making Under Deep Uncertainty: <https://ppp.worldbank.org/library/investment-decision-making-under-deep-uncertainty>

## Evidence-tier rules

| Tier | What it permits | Minimum gate |
|---|---|---|
| **unknown** | State that the quantity is unknown; do not substitute zero | Explicit limitations |
| **synthetic_assumption** | Test software logic or policy behavior on declared illustrative assumptions | Explicit synthetic status, not-calibrated state, scope, limitations and no data-source references claiming support |
| **expert_judgment** | Report an elicited view as expert judgment | Identified record, method, date, limitation and source role |
| **official_observation** | Report an observation within the source's scope | Publisher, artifact, retrieval record, valid time, unit, geographic coverage, license/access status, source role and SHA-256 digest |
| **operator_measurement** | Report recorded operational data within coverage and instrument limits | Same provenance essentials as an observation, with operator-data source role |
| **published_estimate** | Report a publisher's estimate under its stated method | Source and method reference, valid time, unit, coverage, rights, digest and source role |
| **model_output** | Report a versioned model output | Model/version, configuration digest, input-manifest digest, unit, valid time, coverage and declared validation status |

These fields are **necessary provenance gates, not proof of truth**. A well-formed digest can still be a digest of the wrong bytes; an observation does not prove causality; a model output does not become calibrated merely because a version and hash exist. The future acquisition pipeline must recompute digests over acquired bytes, verify the artifact identity, retain transformation steps, and review model validity separately. The current Rust contract only checks the metadata shape, status transitions, and digest syntax.

Methodological references are stored separately from source evidence. A DMDU guide can support the methodological choice to stress-test decisions; it cannot substantiate a Johannesburg outage duration, drought magnitude, pressure-system failure rate, or delivered-water quantity. In v0.1, all methodological links have the role **methodological_context_only** and none is attached as evidence for a stressor.

## Current synthetic contract

The companion manifest defines the six stressors used in NRVCA-005:

- **source_availability_low**
- **heat_demand_high**
- **grid_disruption**
- **powered_pressure_system_down**
- **repair_crew_delay**
- **reserve_depleted**

All six remain **synthetic_assumption**, **not_calibrated**, with a fictional geographic scope and empty source-evidence references. Their qualitative flags have no calibrated numeric magnitudes.

Two dependencies are enforced within the fixture's declared scope: **powered_pressure_system_down** requires **grid_disruption**; **reserve_depleted** requires **source_availability_low**. These are **scenario-encoding constraints**, not universal physical laws. A future model must represent additional causes and alternatives rather than silently turning this small fixture into a general causal model.

The repeated-disruption scenario has three structured history tokens: a prior disruption, prior reserve drawdown, and repeat disruption. A label such as “prolonged” is not sufficient evidence that reserves have actually been drawn down. The history remains synthetic; this manifest only makes the assumption visible and testable.

The current synthetic numeric outcomes and the 0/5, 1/5 and 4/5 robust pass counts are unchanged. Those ratios describe membership in this unweighted stress set; they are not probabilities of real-world success. The adaptive strategy still fails the prolonged/repeated-disruption case.

## Rust checks and negative controls

The companion integration test checks:

1. Every stressor used by the five scenarios resolves to exactly one evidence definition; no missing or orphan definition is accepted.
2. Every scenario has one structured history condition and satisfies the dependency and temporal rules.
3. All stressor and strategy-scenario records retain their synthetic evidence ceilings.
4. Source references are resolvable and role-compatible; method references cannot back empirical claims.
5. Attempted promotion to an official observation without provenance fails closed.
6. An empirical claim with an invalid digest shape fails; dangling source references, unsupported stressor combinations and missing repeated-event history fail.
7. Scenario probabilities are not inferred from sample counts, and adding a label or metadata does not create empirical status.

The digest syntax test does not recompute artifact bytes. Exact-byte verification is an acquisition-pipeline responsibility and must precede any claim that source provenance has been qualified.

## Benchmarks to target

### Synthetic contract qualification

- 100% of declared stressor IDs and scenario-history conditions resolve exactly once.
- 100% deterministic pass/fail reproduction across frozen and mutated fixtures.
- Zero missing source references or incompatible reference roles accepted.
- Zero unsupported evidence-tier promotion; zero unknown-to-zero conversion.
- Zero unmet dependency or temporal preconditions accepted.
- Zero changes to NRVCA-005 outcome values when this metadata layer is added.
- Exact commit SHA and focused workflow status attached to each claimed test result.

### Empirical and operational qualification (not yet achieved)

A serious comparison must first acquire permitted, version-pinned local data and a reviewed engineering model. Then compare the same interventions and constraints against the incumbent decision process and credible DMDU baselines, using:

- worst-case service and underserved-service shortfall;
- floor-breach frequency over an explicitly defined stress set, clearly separated from probabilities;
- recovery time, cumulative delivered-service loss and reserve exhaustion;
- fully loaded lifecycle cost, including contingencies and intervention failure;
- robustness and regret across declared scenarios;
- performance under held-out historical events and deliberately withheld future conditions;
- calibration and uncertainty diagnostics where a probabilistic model is justified;
- distributional outcomes by relevant groups, only where lawfully supported data can resolve them;
- sensitivity to scenario construction, model family, source vintage, price/unit conventions, and policy thresholds.

Do not advertise a percentage improvement over “SOTA” until there is a real comparator, common inputs, predeclared metrics, held-out evaluation, uncertainty analysis, and independent reproduction. The immediate benchmark is **correct provenance and fail-closed behavior**, not invented operational superiority.

## South Africa acquisition gate

NRVCA-002 continues to track direct source-vintage acquisition and reconciliation. The Stats SA GDP release for the quarter ended June 2026 notes routine revisions and the 2026/27 rebasing/benchmarking process; an official page saying that a release is expected later is not proof that a particular current workbook is available or absent. Before using a national series, acquire the exact artifact, verify publisher and publication vintage, record licensing/access conditions, hash the bytes, document revisions and bridge differences, and preserve the unmodified source.

This scenario manifest is **not** evidence of Johannesburg or South African water security. A future water/energy pilot needs authorized utility/catchment time series, spatial and temporal coverage, service/pressure/inflow/outage definitions, a calibrated model with validation data, explicit system boundaries, and separate review of operational safety and privacy. Until those inputs exist, unknowns stay unknown and synthetic tests stay synthetic.
