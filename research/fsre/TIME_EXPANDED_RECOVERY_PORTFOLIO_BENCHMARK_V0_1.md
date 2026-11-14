# FSRE time-expanded recovery portfolio benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research and interoperability protocol. No multi-period scheduler, temporal portfolio evaluator, operational result or field validation is claimed.

## 1. Purpose and claim boundary

The current `fsre-portfolio-reference` evaluates one frozen scenario, interval and evidence snapshot. It does not establish that a proposed portfolio remains feasible over a multi-hour or multi-day disruption. A plan can be feasible at hour 1 and fail at hour 6 because stock is depleted, replenishment is delayed, crews are unavailable, routes close, capacity changes, or authority expires.

The next research unit is a **time-expanded, cohort-specific service trajectory supported by owner-qualified resource and authority evidence**. This is not a claim to invent multi-period resilience optimisation, restoration scheduling or resource-constrained project scheduling. The proposed contribution is an evidence and claim-qualification layer over existing planning methods, with independent outcome reproduction.

## 2. Prior art and comparators

- Naeimi and Davidson (2025), *Improving Restoration of Water Supply Service Following Disruptions*, compares 15 water restoration plans covering crew management, task prioritisation, immediate damage discovery and rerouting, using REWET on Anytown and Los Angeles networks. The authors find that effective prioritisation depends on network size/layout and damage pattern. DOI: https://doi.org/10.1061/JWRMD5.WRENG-6452.
- A 2024 *Natural Hazards and Earth System Sciences* paper frames infrastructure recovery as a resource-constrained project scheduling problem. https://nhess.copernicus.org/articles/24/2285/2024/.
- Liu et al., *Spatiotemporal resource-dependent resilience optimization of infrastructure networks under epistemic disruption uncertainty*, Reliability Engineering & System Safety 278, article 113623 (scheduled issue February 2027; DOI assigned 2026), develops a two-stage, multi-period distributionally robust model for pre-disaster repair-resource pre-positioning and post-disaster restoration scheduling. It uses a total-variation ambiguity set and an MILP reformulation; its numerical case study is an electricity grid, not water. DOI: https://doi.org/10.1016/j.ress.2026.113623.
- A 2025 intermittent-water-supply study applies multi-horizon adaptation pathways and multi-objective optimisation across cost, equity and supply hours using the D_Town benchmark network. https://iwaponline.com/ws/article/25/5/900/107639/Adaptation-pathways-for-intermittent-water-supply.

These are method comparators, not evidence that FSRE improves restoration. A fair comparison must freeze scenarios, information, resource budget, service floors, model/tool version, hardware, tuning budget, solver gap, analyst effort and operator workflow. Product brochures and abstracts do not constitute benchmark runs.

## 3. Time-expanded evidence contract

Discretise the evaluation horizon before running a replay. Each interval has a stable ID, start/end instant, time basis and duration. All records must bind to the same scenario and compatible time basis.

| Record family | Required facts | Owner |
|---|---|---|
| Service requirement | Quantity and quality floor by service, cohort and interval | Service owner / authorised process |
| Service observation | Delivered quantity, quality, denominator, missingness and uncertainty | Operator / telemetry / laboratory |
| Option state | Seven-predicate disposition and validity interval | Mycelix, Sol-Atlas, Symtropy, operator |
| Option capacity | Feasible quantity by interval and operating state | Symtropy / operator |
| Resource stock | Opening inventory, reservation, expiry, unit and scope | Operator / inventory owner |
| Replenishment | Received quantity, delivery time, source, status and uncertainty | Supplier / operator |
| Resource use | Consumption per option/interval and provenance | Operator / domain model |
| Workforce | Skill, qualified hours, shift/rest and fatigue limits | Operator |
| Access | Route state and mobilisation time by interval/hazard | Sol-Atlas / operator |
| Authority | Issuer, purpose, scope, revocation and valid-from/to | Canonical authority owner |
| Common cause | Failure-domain state and affected options by interval | Mycelix dependency owners |
| Physical state | Operating envelope, ramps, constraints and model-domain validity | Symtropy |
| Cost/liability | Mobilisation, replenishment, overtime, lifecycle and environmental cost | Finance / economic owner |

Every record retains source owner, schema/version, evidence reference, revision/digest where available, geography, denominator, time basis, observation time, validity and admissibility. Missing observations are not carried forward as healthy. An order is not stock until owner-qualified delivery evidence says it was received. An authority grant does not remain valid beyond its scope or expiry.

## 4. Temporal semantics

### Resource conservation

For resource (r), interval (t):

[
I_{r,t+1}=I_{r,t}+R_{r,t}-U_{r,t}-L_{r,t}
]

where (I) is opening stock, (R) is replenishment actually received, (U) is consumption and (L) is loss/expiry/removal. Preserve units, time windows, source and uncertainty. Reject negative stock, double consumption, unsupported replenishment, undeclared unit conversion and out-of-scope use. Unknown stock is not unlimited stock. Per-period crew-hour capacity is different from cumulative fuel or chemical inventory.

### Time-varying qualification

An option is usable only in intervals where all owner-qualified predicates remain current and in scope. Include mobilisation, setup, ramp-up, active period, access window, shutdown/transition and evidence expiry. A physical receipt valid at interval 1 cannot silently qualify interval 5.

### Service trajectories

For each cohort (k), scenario (w) and interval (t), report delivered safe service (D_{k,w,t}) against the authorised floor (F_{k,w,t}):

[
D_{k,w,t}geq F_{k,w,t}.
]

Surplus in one cohort cannot compensate for a protected cohort's shortfall. Measures include safe service-hours above floor, cumulative deficit, longest below-floor episode, unsafe-quality duration, time to first floor restoration, time to stable recovery, relapse and uncertainty by cohort/interval.

### As-known-at-the-time

At time (t), a replay may use only evidence observable/admissible by (t), plus forecasts labelled as forecasts. Future incident reports, deliveries or repair outcomes must not leak into earlier decisions. Preserve what was known then, forecast then, learned later and known in hindsight. Adaptive plans must be non-anticipative.

### Authority

This profile evaluates a proposed or historical plan. It does not dispatch crews, switch infrastructure, ration service, order supplies or execute actions. Live operational authority and safety cases remain with operators and lawful authorities.

## 5. Challenge families

Build a public synthetic suite and a separately prepared sealed set, each with positive, negative and unknown-evidence controls:

1. Snapshot illusion: feasible at interval 1 but stock depletes before the horizon.
2. Replenishment uncertainty: delivery delayed, reduced, rejected or not received.
3. Resource overdraw: overlapping options consume the same inventory.
4. Crew collision: tasks fit separately but exceed concurrent qualified hours/rest limits.
5. Option expiry: physical, spatial, staffing or authority receipt expires mid-horizon.
6. Access-window closure: route closes before a required delivery.
7. Common-cause transition: a shared feeder, road or control domain fails after interval 1.
8. Quality relapse: quantity recovers but quality later breaches the safe floor.
9. False stable recovery: one above-floor interval is followed by another failure.
10. Cohort scarcity: aggregate service remains high while a cohort spends multiple intervals below floor.
11. Hindsight leakage: later observations must not alter earlier knowledge states.
12. Future option preservation: an immediate plan consumes stock, route or repair capacity needed later.

## 6. Matched baselines

Under identical scenarios, source snapshots, horizon, budgets, floors and information frontiers, compare: actual operator runbook; static one-interval feasibility; deterministic multi-period RCPSP; domain restoration optimisers (including REWET for water-distribution tasks); robust/distributionally robust multi-period optimisation; existing asset/workforce/inventory tools; qualified human operators; strong AI planning agent with identical information and no extra authority; full FSRE; and ablations removing cohort floors, inventory carryover, time-valid authority, common-cause transitions, uncertainty or as-known-at-the-time controls.

Do not force all methods into one schema or universal score. Record unavailable comparisons and method limitations.

## 7. Metrics and hard gates

Deterministic conformance gates:
- 100% agreement with an independently authored stock-flow arithmetic reference;
- zero negative inventory or unsupported resource capacity accepted;
- zero reuse of the same resource/crew capacity across overlapping intervals;
- zero future-information leakage;
- zero unqualified cohort-floor satisfaction claims;
- zero reliance on authority outside its valid interval;
- 100% temporal and source lineage for qualified records;
- unknown required resource, service floor, option state or authority produces NOT_ESTIMABLE.

Operational studies report service-hours by cohort, cumulative deficit, longest below-floor duration, time to first/stable restoration, stockouts, crew overcommitment, resource utilisation/buffer, lifecycle cost, runtime/optimality gap, schedule instability, analyst effort, distributional outcomes and forecast calibration. Targets such as a 10% reduction in deficit/cost require matched baselines, variance and preregistration; solver optimality gap is not a service outcome.

## 8. Implementation roadmap and boundary

1. Independent exact stock-flow verifier with overflow, negative-stock, unknown-input and scope controls.
2. Three-interval synthetic fixtures with inventory carryover, replenishment, crew shifts, option expiry and cohort floors.
3. Separately authored oracle and exact differential checks.
4. Time-frozen replay controls preventing hindsight leakage.
5. Only after operator data access, benchmark against actual restoration planning, inventory/crew systems and domain optimisers.

The as-known-at-the-time primitive is implemented separately in [the decision-time replay reference](../../tools/fsre-decision-time-replay/README.md) and an independent oracle. It does not yet compose automatically with the multi-interval stock-flow and portfolio tools.

The current `fsre-portfolio-reference` is a one-interval feasibility checker. The stock-flow reference verifies resource arithmetic. Neither is a multi-period scheduler. This profile remains a research protocol until the complete temporal composition is implemented and independently evaluated.
