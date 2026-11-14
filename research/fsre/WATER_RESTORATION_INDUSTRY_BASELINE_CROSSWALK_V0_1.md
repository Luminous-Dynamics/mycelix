# FSRE water restoration industry baseline crosswalk v0.1

**Date:** 2026-11-14  
**Status:** Research comparison contract. No deployment, procurement evaluation, matched benchmark, or field superiority claim is made.

## 1. Strategic conclusion

The water-sector baseline is already capable. FSRE must not position itself as a replacement for hydraulic modelling, water-quality simulation, restoration planning, SCADA integration, or established water digital twins.

The stronger opportunity is to compose those systems with an independently reviewable evidence envelope: source observations, decision-time admissibility, owner-qualified resource capacity, shared dependencies, cohort-specific service floors, authority validity and observed service recovery.

The integration hypothesis is:

**existing domain engine → owner-native model/evidence outputs → FSRE qualification → operator decision and authority → independently observed outcome**

FSRE should add value without changing the hydraulic result produced by the qualified domain engine. A mismatch in domain simulation remains a model/adapter issue, not something the assurance layer should conceal.

## 2. Public and commercial baselines

| Baseline | Documented capability | Appropriate FSRE comparison | What FSRE must not claim |
|---|---|---|---|
| US EPA EPANET 2.2 | Extended-period hydraulic and water-quality simulation; pipes, nodes, pumps, valves, tanks and reservoirs; pressure-dependent demand, tank/pump controls, energy, water age, source tracing and emergency/security analysis. Public-domain software. | Model fidelity, input completeness, scenario reproducibility, extraction of pressure/flow/quality trajectories and stable model receipts. | Replacing the hydraulic or water-quality engine, or surpassing its numerical fidelity without matched tests. |
| US EPA WNTR 1.5 | EPANET-compatible Python package for water-distribution resilience; can generate/modify networks, inject disruptive events, add response/repair strategies, simulate demand/quality and compute topographic, hydraulic, water-quality, security and economic metrics. | Disruption/repair scenario baselines, resilience trajectories, restoration strategy comparisons, resource/cost trade-offs and reproducible result envelopes. | Claiming WNTR lacks repair strategies or resilience metrics; it has them. |
| Bentley OpenFlows Water / WaterGEMS / WaterCAD / HAMMER | Commercial water-distribution modelling and operations tooling; hydraulic and extended-period analysis, transient/water-hammer analysis, calibration, scenario studies, pump scheduling, GIS/CAD/database/SCADA integration and emergency-condition analysis. | Operator-native hydraulic results, model calibration, asset data integration, scenario replay and model governance. | Assuming FSRE provides superior hydraulic modelling, calibration, or live SCADA operations. |
| Bentley OpenFlows WaterSight | Water-network digital twin and analytics; unifies SCADA, GIS, models and IoT data for leak/anomaly detection, network operations, pump optimization and emergency response simulation. | A real configured deployment is a serious operator baseline for live data and response workflows. Compare actual contracts, data, alert quality, human effort and service outcomes. | Treating vendor marketing claims as independently measured outcomes, or assuming all operators use the same configuration. |
| Water-supply restoration research (Naeimi & Davidson, 2025) | Evaluates 15 alternative post-disruption restoration plans, including crew management, task prioritisation, damage discovery and rerouting, using Anytown and Los Angeles water networks. | Matched restoration plans, customer-days without service, crew/task priorities, repair-resource assumptions and restoration time. | Claiming that FSRE invented crew-aware restoration planning or the 15-plan comparison. |

Official documentation and primary references:
- EPA EPANET: https://www.epa.gov/water-research/epanet
- EPA WNTR documentation and current version: https://usepa.github.io/WNTR/
- WNTR resilience metrics: https://github.com/USEPA/WNTR/blob/main/documentation/resilience.rst
- Bentley OpenFlows Water: https://www.bentley.com/products/openflows-water/
- Bentley OpenFlows WaterSight: https://www.bentley.com/en/products/openflows-watersight/
- Naeimi & Davidson (2025), DOI: https://doi.org/10.1061/JWRMD5.WRENG-6452

## 3. Where FSRE can add value

The proposed differentiation is not "more accurate hydraulics". It is a set of cross-system checks that many domain tools do not claim to provide as one neutral, portable contract:

1. **Evidence admissibility:** distinguish source-observed, system-recorded, valid, expired and revoked evidence as of the decision cut-off.
2. **Owner-preserving capacity qualification:** do not turn an unknown resource, staffing receipt, model validity range or authority state into a positive capacity.
3. **Portfolio composition:** detect shared resource overdraw and common-cause exposure across multiple options, even if each option is individually feasible.
4. **Cohort-specific service floors:** expose who remains below authorised minimum service, rather than treating system-wide average flow or pressure as a sufficient service outcome.
5. **Time-expanded accounting:** reconcile resource stocks, replenishment received, consumption, loss and closing balances across intervals.
6. **Independent outcome verification:** compare the planned service trajectory with independently observed quantity, quality, duration and population reach.
7. **Cross-vendor portability:** preserve native identifiers, schema versions, units, timestamps, owners and opaque evidence references when combining EPANET/WNTR, commercial tools, SCADA, GIS and operator records.

These are hypotheses until the same evidence and scenario are run through incumbent workflows and FSRE.

## 4. Matched benchmark design

Use a common set of public and operator-authorised instances. At minimum:
- EPANET example networks for hydraulic and water-quality sanity checks;
- WNTR disruption/repair scenarios, with explicit version and solver settings;
- Anytown or another published restoration-planning instance where licensing and source data permit;
- a synthetic multi-interval water recovery scenario with known stock, replenishment, crew limits, option expiry, shared power/road domains and cohort floors;
- a participating operator's own configured model, SCADA/telemetry path, runbook and dispatch/inventory workflow, where access is authorised.

Freeze the network model, demands, hazard, damage, initial stock, replenishment schedule, crew/shift limits, service quality thresholds, population boundary, decision information frontier, and allowed optimisation budget. Use the exact same inputs for all methods where their interfaces permit. Document any excluded feature or adapter transformation.

### Required baseline lanes

- H0: operator's actual runbook, configured tools and human planning workflow.
- H1: EPANET alone for hydraulic/quality outcome.
- H2: WNTR disruption and resilience analysis.
- H3: WNTR or operator restoration/repair strategy baseline.
- H4: configured Bentley/other commercial platform actually used by the operator.
- H5: published restoration-plan method/benchmark, where reproducible.
- H6: FSRE qualification wrapper consuming the domain-native outputs.
- H7: FSRE plus an independently verified service-outcome replay.

The comparison must not pretend that a hydraulic simulator, a repair scheduler, a digital twin, and an evidence verifier solve the same task. Report which layer each result qualifies.

## 5. Metrics

Publish a vector, not one universal score.

| Metric family | Measures | Owner/method |
|---|---|---|
| Hydraulic/quality adequacy | pressure, flow, demand satisfaction, tank level, disinfectant residual/quality, continuity | EPANET/WNTR/operator model; calibrated against observations |
| Restoration | time to first safe service, time to stable recovery, customer-days without service, repair completion, reroute effect | Domain restoration model plus observed service |
| Resource feasibility | stockouts, overdraw, qualified crew-hour overload, delivery delay, utilisation and reserve | Inventory/workforce owner plus independent stock-flow verifier |
| Evidence admissibility | stale/expired/revoked/future-known evidence false-accepts, unresolved evidence, missing lineage | Independent FSRE replay |
| Common-cause resilience | shared feeder/road/control domain concentration, affected-option count, service loss under common failure | Mycelix dependency evidence plus domain model |
| Distributional service | per-cohort service-hours, deficit, longest below-floor duration, safe-quality violations and missingness | Authorised cohort floors plus independently observed outcomes |
| Operational economics | mobilisation, repair/replenishment cost, lifecycle liabilities, energy use, operator hours | Operator finance and runbook |
| Computational performance | runtime, memory, solver gap, concurrency, adapter cost, p50/p95/p99 latency | Exact configured build, frozen hardware and workload |

## 6. Qualification targets

Deterministic synthetic conformance:
- no hydraulic result is rewritten by the FSRE wrapper;
- 100% agreement with an independent reference on declared stock-flow and temporal-admissibility cases;
- zero acceptance of future-known, expired, revoked or unit-incompatible evidence;
- zero shared-resource overcommitment accepted as feasible;
- zero protected-cohort floor violations hidden by aggregate surplus;
- every reported result retains its source owner, schema/version, timestamp, units and evidence reference;
- unknown required inputs produce an explicit abstention or claim ceiling.

Operational superiority targets must be set only after a measured incumbent baseline and stakeholder-agreed minimums. For example, reductions in customer-days without service, time below a safe floor, or operator effort require the same hazard, population boundary, crew budget, data access and cost horizon. Publish non-inferiority of the domain model alongside any gain from better evidence qualification.

## 7. Deployment wedge

The initial product should be an assurance and integration service over tools already owned by the utility:
1. read-only import of model, GIS, SCADA-summary, inventory, crew and service-floor evidence;
2. reproducible as-known-at-the-time qualification of a proposed restoration plan;
3. resource/common-cause/cohort-floor checks;
4. independent outcome replay after the event or exercise;
5. operator-facing report with unresolved facts and explicit claims the evidence does not support.

Do not begin with live actuation, automatic rationing, crew dispatch or switching authority. Those require separate operational control, safety cases and legally accountable operator decisions.

## 8. Falsification

Narrow the FSRE proposition if a configured incumbent already provides equivalent cross-system time validity, owner provenance, shared-resource/common-cause composition, protected-cohort service floors and independent outcome replay at lower cost and burden. If FSRE adds only metadata, a dashboard, or another schema without changing false-acceptance, reproducibility or service-outcome measurement, it has not established a valuable new capability.
