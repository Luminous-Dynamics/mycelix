# FSRE time-expanded recovery portfolio benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research and interoperability protocol. No multi-period scheduler, temporal portfolio evaluator, operational service result, or field validation is claimed.

## 1. Why a one-interval feasible portfolio is insufficient

The current \`fsre-portfolio-reference\` checks a proposed allocation in one frozen scenario, interval and evidence snapshot. It does not establish that the same service can be maintained through a multi-hour or multi-day disruption.

A portfolio can be feasible at hour 1 and fail at hour 6 because diesel, chemicals, battery state, tanker capacity, qualified crew hours, access windows, temporary storage or authority expires. A static plan can count a resource repeatedly across periods, assume replenishment that never arrives, or declare recovery from one above-floor interval even though the service collapses again.

The next research unit is a **time-expanded, cohort-specific service trajectory supported by owner-qualified resource and authority evidence**. It evaluates the complete declared horizon, not a single snapshot.

This is not a claim to invent multi-period resilience optimisation, restoration scheduling, or resource-constrained project scheduling. Those are established approaches. FSRE's hypothesis is that proposed schedules can be qualified against a portable evidence contract preserving owner semantics, service floors, temporal validity and independently reproduced outcomes.

## 2. Prior art and competitive boundary

Existing work already treats recovery as a time-dependent and resource-constrained problem:

- Naeimi and Davidson (2025), *Improving Restoration of Water Supply Service Following Disruptions*, Journal of Water Resources Planning and Management 151(7), compare 15 water-supply restoration plans, including crew management, task prioritisation, immediate damage discovery and rerouting. DOI: https://doi.org/10.1061/JWRMD5.WRENG-6452
- A 2024 Natural Hazards and Earth System Sciences paper formulates infrastructure recovery as a resource-constrained project scheduling problem, emphasising performance through event and recovery phases rather than damage cost alone. https://nhess.copernicus.org/articles/24/2285/2024/
- A 2026 study on spatiotemporal resource-dependent infrastructure resilience develops a two-stage, multi-period distributionally robust optimisation model jointly addressing pre-disaster repair-resource pre-positioning and post-disaster restoration scheduling under epistemic disruption uncertainty. It demonstrates the approach on an electricity-grid benchmark and compares with stochastic optimisation. The full DOI and version should be confirmed before a formal publication citation is frozen.
- A 2025 intermittent-water-supply study uses multi-horizon adaptation pathways and multi-objective optimisation across cost, equity and supply hours, applied to the D_Town benchmark network. https://iwaponline.com/ws/article/25/5/900/107639/Adaptation-pathways-for-intermittent-water-supply

These are method comparators, not evidence that FSRE already improves restoration plans. Tool version, scenario support, data, hardware, tuning budget and solver gap must be recorded. Where source metadata is incomplete, retain it as a research lead until the full citation is verified.

## 3. Time-expanded evidence contract

The horizon must be discretised before evaluation. Each interval has a stable ID, start/end instant, time basis and duration. Time-varying records bind to the same scenario and compatible time basis.

| Record family | Required facts | Native owner |
|---|---|---|
| Service requirement | Quantity and quality floor by service, cohort and interval | Service owner / authorised public process |
| Service observation | Delivered quantity, quality, denominator, missingness and temporal uncertainty | Operator / telemetry / laboratory owner |
| Option state | Seven-predicate recovery disposition and validity interval | Mycelix, Sol-Atlas, Symtropy, operator |
| Option capacity | Feasible service quantity by interval and operating state | Symtropy / operator |
| Resource stock | Opening inventory, reserved quantity, expiry, unit and scope | Operator / inventory owner |
| Replenishment | Confirmed quantity, delivery interval, source, status and uncertainty | Supplier / operator |
| Resource use | Consumption per option and interval, unit, method and provenance | Operator / domain model |
| Workforce | Skill, qualified crew hours, duty/rest window and fatigue constraints | Responsible operator |
| Access | Route state and mobilisation time by interval and hazard | Sol-Atlas / operator |
| Authority | Issuer, purpose, scope, revocation state and valid-from/to | Canonical authority owner |
| Common-cause state | Failure-domain state and affected options by interval | Mycelix dependency owners |
| Physical state | Operating envelope, ramp-up/ramp-down, process constraints and model-domain validity | Symtropy |
| Cost and liability | Mobilisation, replenishment, overtime, lifecycle, environmental and future maintenance cost | Finance / economic owner |

Every record retains source owner, schema/version, evidence reference, source revision/digest where available, geography, denominator, time basis, observation time, validity and admissibility. Missing observations are not carried forward as healthy. A stale authority grant is not extended by a schedule. A replenishment order is not stock until owner-qualified delivery evidence says it was received.

## 4. Required temporal semantics

### 4.1 Resource conservation

For each resource \(r\) and interval \(t\), use an explicit stock-flow identity:

\[
I_{r,t+1}=I_{r,t}+R_{r,t}-U_{r,t}-L_{r,t}
\]

where \(I\) is opening available stock, \(R\) is owner-qualified replenishment received, \(U\) is allocated consumption, and \(L\) is declared loss, expiry or removal.

Balances retain unit, time window, source and uncertainty. Reject negative inventory, double consumption, unsupported replenishment, undeclared unit conversion, and use outside scope. Missing stock or replenishment is unknown, not unlimited supply.

Per-period capacity (such as qualified crew-hours available during a shift) is distinct from cumulative inventory (such as diesel remaining in a tank).

### 4.2 Time-varying option feasibility

A recovery option is valid only during intervals in which all required owner-qualified predicates remain current and in scope. Feasibility accounts for mobilisation start, setup, ramp, active duration, access window, shutdown/transition and evidence expiry. A model result valid at interval 1 is not silently valid at interval 5.

### 4.3 Service trajectories and floor exposure

For every declared cohort \(k\), scenario \(w\) and interval \(t\), report delivered safe service \(D_{k,w,t}\) against the authorised floor \(F_{k,w,t}\):

\[
D_{k,w,t}\geq F_{k,w,t}
\]

A portfolio may be marked infeasible where a floor is violated; surplus from one cohort cannot compensate for a protected cohort's shortfall.

Required temporal measures include safe service-hours above the floor, cumulative service deficit, longest continuous below-floor episode, unsafe-quality duration, time to first floor restoration, time to stable recovery across a declared stability window, repeated relapse after apparent restoration, and uncertainty/missingness by cohort and interval.

### 4.4 As-known-at-the-time and no future leakage

A plan evaluated as of time \(t\) may use only evidence observable and admissible at or before \(t\), plus forecasts explicitly labelled as forecasts. Later incident reports, future deliveries or eventual repair outcomes must not leak into earlier decisions. Each replay distinguishes what was known then, what was forecast, what became known later and what an independent evaluator knows in hindsight.

For adaptive plans, decisions must be non-anticipative: two scenario paths indistinguishable at time \(t\) cannot receive different actions at \(t\) solely because of future information unavailable then.

### 4.5 Authority and safe control boundaries

This profile evaluates a proposed or historical plan. It does not dispatch crews, switch infrastructure, ration service, order supplies or execute recovery actions. Live operational systems retain separate authority, safety cases and human/operator control. A grant must be valid for the option, service, purpose, location and full active interval, with revocation handled by the owner-native authority system.

## 5. Benchmark challenge families

Build a public synthetic conformance suite and a separately prepared sealed set. Each challenge needs positive, negative, unknown/missing-data controls, and independently adjudicated expected results.

1. **Snapshot illusion:** feasible in the first interval but depletes a shared resource before the horizon ends.
2. **Replenishment uncertainty:** a scheduled delivery is delayed, reduced, rejected or never received; it is not counted as stock prematurely.
3. **Resource overdraw:** two options consume the same physical stock in overlapping intervals.
4. **Crew collision and shift boundary:** tasks each fit a crew schedule alone but exceed concurrent qualified crew-hours or rest constraints.
5. **Option expiry:** a physical, spatial, staffing or authority receipt expires mid-horizon.
6. **Access-window closure:** a route is available during mobilisation but inaccessible before the next required delivery.
7. **Common-cause transition:** a shared feeder, road or control domain fails after the first interval and blocks multiple nominal alternatives.
8. **Quality relapse:** quantity is restored but quality falls below the safe floor in a later interval.
9. **False stable recovery:** one above-floor interval is followed by another failure; the stability window is not satisfied.
10. **Cohort scarcity:** aggregate service remains high while one cohort falls below its floor for several intervals.
11. **Evidence delay and hindsight leakage:** later observations do not change what the replay says was knowable at earlier decision points.
12. **Future option preservation:** a cheap immediate action consumes a resource, route or repair capability needed later; report lost future options, not only first-interval performance.

## 6. Matched baselines and ablations

For the same frozen scenario, source snapshot, horizon, resource budget, service floors and information frontier, compare:
- B0: actual operator runbook and response process;
- B1: static snapshot feasibility (the current one-interval reference);
- B2: deterministic multi-period resource-constrained project scheduling;
- B3: domain-specific restoration optimisation / simulator;
- B4: robust or distributionally robust multi-period optimisation where probabilities are uncertain;
- B5: existing asset/workforce/inventory planning tools;
- B6: qualified operator/human planning team;
- B7: strong AI planning agent with identical information and no additional authority;
- B8: FSRE time-expanded evidence qualification and independent outcome verification;
- ablations removing cohort floors, inventory carryover, time-valid authority, common-cause transitions, uncertainty, or as-known-at-the-time controls.

Relevant comparator families include restoration RCPSP, crew routing and repair scheduling, two-stage/multi-period robust optimisation, and adaptive-pathway optimisation for water systems. They should be compared on their intended tasks, not forced into one universal score.

## 7. Metrics and proposed gates

Hard conformance gates on deterministic cases:
- 100% exact agreement with an independently authored stock-flow arithmetic reference;
- zero negative inventory or unsupported resource capacity accepted;
- zero reuse of the same resource/crew capacity across overlapping intervals;
- zero future-information leakage in time-frozen replays;
- zero unqualified cohort-floor satisfaction claims;
- zero reliance on authority outside its valid interval;
- 100% lineage and temporal binding for every qualified capacity, replenishment, service and authority record;
- any unknown required resource, service floor, option state or authority interval produces NOT_ESTIMABLE.

For operational studies, report safe service-hours per cohort, cumulative deficit and longest below-floor duration, time to first and stable restoration, stockout and workforce overcommit intervals, resource utilisation and unused buffer, lifecycle/emergency procurement costs, solution runtime/optimality gap/schedule instability/analyst effort, distributional/restorative outcomes separately, and forecast calibration where applicable.

Targets such as a 10% reduction in cumulative deficit or lifecycle cost should be set only after a matched baseline, operator constraints, variance estimate and preregistration. A solver's optimality gap is not a proxy for service improvement.

## 8. Implementation boundary and roadmap

The next implementation should be a deterministic, read-only multi-interval verifier consuming owner-produced option, stock, replenishment, workforce, authority and service-floor receipts. It should not schedule tasks or solve a physical model.

Suggested increments:
1. Add an independent exact stock-flow verifier with overflow, negative-stock, unknown-input and scope controls.
2. Add three-interval synthetic fixtures with resource carryover, crew shifts, option expiry and cohort floors.
3. Independently implement the reference evaluator and compare exact results before adding forecasts or robust optimisation.
4. Add time-frozen replay tests that prevent future leakage and verify evidence as it was known then.
5. Only after operator data access, benchmark against actual restoration planning, inventory/crew systems and relevant domain optimisers.

The existing one-interval \`fsre-portfolio-reference\` remains a bounded static feasibility checker. It must not be represented as a multi-period restoration scheduler. This longer-horizon profile is a research direction and is not yet implemented.
