# FSRE recovery portfolio benchmark extension v0.1

**Date:** 2026-11-14  
**Status:** Research protocol; no portfolio-level field result or comparative advantage is claimed.

## 1. Why qualifying options individually is not enough

A set of individually qualified recovery options can still fail as a portfolio. Several options may depend on the same upstream feeder, road corridor, fuel source, supplier, control plane, workforce pool, specialist crew, permit, authority grant or communications service. They may also compete for the same finite pump, tanker, generator, treatment chemicals, operators or mobilisation window.

Therefore, `qualified_option_count` is not a resilience metric by itself.

The relevant question is whether a portfolio can preserve a declared service floor across a pre-registered family of disturbances and simultaneous recovery constraints. This is a service-outcome property of the portfolio under scenarios, not an intrinsic score of the assets or organisations.

## 2. Key design principle: consume owner-native failure evidence

This extension does not introduce a new dependency graph, failure-domain registry, resource ledger, scheduler or physical simulator.

- Mycelix supplies canonical time-qualified dependencies, common-cause/cut-set results, authority and provenance.
- Sol-Atlas supplies owner-qualified spatial reachability, route/access constraints and population boundaries.
- Symtropy supplies physical model feasibility, capacity and model-domain receipts.
- Operators supply workforce, inventory, supplier, safety, mobilisation and dispatch constraints.
- Symthaea may propose competing causal explanations and discriminating observations; it cannot promote its own hypothesis to an owner-qualified fact.
- FSRE composes the frozen receipts into service-lane/portfolio benchmark outputs.
- An independent evaluator measures achieved service outcomes.

The portfolio evaluator may compare stable owner-native identifiers and explicit scenario result receipts. It must not infer independence merely from different asset, organisation, vendor or option IDs.

## 3. Benchmark input contract

A pre-registered portfolio task should freeze the following inputs:

| Input | Required fields | Owner / boundary |
|---|---|---|
| Portfolio | option IDs, service/cohort IDs, declared portfolio membership | study manifest |
| Option qualification | seven predicate dispositions, scenario digest, validity interval, owner references | existing FSRE qualification envelope |
| Common-cause frontier | failure-domain/cut-set IDs, affected option IDs, source revision, scope and temporal validity | canonical Mycelix dependency owner |
| Spatial frontier | access/route status under each hazard and time slice; population served | Sol-Atlas |
| Physical capacity | feasible service capacity and domain limits under scenario | Symtropy |
| Resource demand | resource ID, amount, unit, time window, substitutability and owner reference | operator / inventory owner |
| Resource supply | available amount, unit, time window, geographic/service scope | operator / supplier owner |
| Workforce | skill class, crew count, duty window, fatigue/availability constraints | responsible operator |
| Authority | grant ID, issuer, purpose, scope, expiry/revocation state | canonical authority owner |
| Service floor | minimum acceptable service quantity and quality per lane and time interval | service owner / regulator / affected community, as authorised |
| Cost | acquisition, mobilisation, recurring, lifecycle and externality boundary | responsible finance/economic owner |
| Observation | source references, freshness, quality, missingness, uncertainty and correlation lineage | native source owner |

Each value must retain its declared unit, denominator, population, time basis, geography, source identity, version and evidence admissibility. Missing resource capacity is unknown, not zero; a structural zero requires an explicit boundary rationale.

## 4. Portfolio evaluation semantics

For each frozen scenario (s), time interval (t), and service lane (k), evaluate only options whose owner-native receipts are current and valid for that exact scope.

Let (q_{i,s}) be the qualified capacity supplied by option (i) under scenario (s), after applying owner-provided physical, spatial, dependency and authority dispositions; let (x_{i,s,t}) be the portion scheduled in interval (t); let (r_{i,j,s,t}) be resource (j) used by option (i); let (c_{j,s,t}) be the owner-reported available capacity of resource (j); and let (F_{k,s,t}) be the pre-registered minimum service floor for lane (k).

The selected portfolio must satisfy resource capacity:

[
sum_i r_{i,j,s,t}x_{i,s,t}leq c_{j,s,t}
]

and service-floor constraints:

[
mathrm{delivered}(k,s,t)geq F_{k,s,t}.
]

The evaluator does not calculate (q) from raw physical or dependency data. It consumes already-qualified, owner-native scenario receipts. If a required receipt, capacity, unit, time window or population boundary is absent or conflicting, it returns `NOT_ESTIMABLE` / `NOT_QUALIFIED` for the affected claim. It does not assume independence, invent a substitute resource, or grant operational dispatch authority.

For the initial deterministic reference, use fixed integer/rational quantities and exact unit conversions. Probabilistic survival is a later lane and requires calibrated event frequencies and independently validated joint distributions; do not convert scenario counts into probabilities.

## 5. Required portfolio metrics

Report a vector, not a universal resilience score.

| Metric | Definition | Failure mode detected |
|---|---|---|
| Scenario service-floor coverage | Fraction of pre-registered scenario-time-lane cells meeting the declared floor | Options exist but cannot preserve the service |
| Worst-case service deficit | Maximum floor shortfall by lane, interval and scenario | Averages conceal catastrophic local failures |
| Safe service-hours preserved | Time integral above the minimum quality/service floor | Volume delivered unsafely or too late |
| Recovery latency | Time from disruption to first safe floor restoration and to stable recovery | Nominal capacity without timely mobilisation |
| Qualified option survival | Count and identity of options with valid receipts after each owner-supplied shock | False redundancy |
| Common-cause exposure | Portfolio share and pairwise structure exposed to the same owner-defined cut sets | Shared upstream dependency |
| Resource feasibility | Number and severity of simultaneous resource capacity violations | Options compete for the same scarce resources |
| Spatial reachability | Population/cohort share still reachable under the scenario | Access failures masked by aggregate capacity |
| Authority continuity | Options remaining within live purpose/scope grants after revocation/partition | Governance failure or unauthorised action |
| Distributional service floor | Coverage and deficit reported by pre-registered cohort/geography | Aggregate gains conceal displaced harm |
| Total cost per verified service unit | Lifecycle cost divided by independently verified safe service improvement | Cheap plans with hidden recurring liabilities |
| Abstention / evidence debt | Share of claims not estimable and exact missing/contradictory evidence | Overconfident reporting |

For every ratio, publish its denominator and missingness. Report scenario cells and distributions, not only an aggregate mean.

## 6. Sealed challenge families

A first portfolio challenge set should contain deterministic, independently adjudicated scenarios for:

1. Nominal independent fallback: options have no declared shared critical cut set and sufficient resource capacity.
2. Hidden common upstream dependency: distinct assets/options share an owner-identified feeder, source, corridor or control plane.
3. Common workforce bottleneck: options are individually staffed but simultaneous mobilisation exceeds available qualified crews.
4. Supplier / consumable concentration: nominally distinct suppliers share the same upstream source or constrained inventory.
5. Transport and access compound event: physical capacity exists but routes and receiving locations are inaccessible under the hazard.
6. Water-energy-telecom cascade: power and communications degradation jointly limit pumping, telemetry and control.
7. Authority revocation / federation partition: previously qualified options lose permission or cannot establish current authority.
8. Temporal mismatch: evidence appears current individually but not at a common evaluation time or compatible time basis.
9. Demand spike and equitable rationing: total capacity is insufficient; measure which service cohorts are protected and which bear the deficit.
10. Repair crew and asset contention: multiple plans rely on the same specialist crews, spare parts or equipment at overlapping times.

Every family must contain positive controls, negative controls, missing-evidence controls and compound interactions. Challenge authors should hold back one independently reviewed set from model/evaluator developers.

## 7. Matched comparison against industry and operator practice

For identical frozen scenarios, source snapshots, budgets, permissions and outcome measurements, compare:

- B0: current operator runbook and real workflow;
- B1: N-1 / simple redundancy-count baseline;
- B2: canonical dependency/cut-set analysis without service-floor portfolio composition;
- B3: strongest available domain-specific simulator / restoration optimiser, e.g. IN-CORE, InfraRisk, REFIT or PRIIA where the task fits;
- B4: incumbent EAM/APM / infrastructure operations workflow;
- B5: qualified human operator/expert panel;
- B6: strong AI agent with equivalent permitted information and action restrictions;
- B7: FSRE full composition;
- ablations: remove common-cause receipts, shared resource constraints, spatial access, authority checks, independent verifier, or outcome measurement one at a time.

Do not force all comparators into one schema or score them outside their intended method family. Record tool/version, calibration, data access, tuning effort, analyst time, integration time, compute, model limitations and unavailable comparisons. Product documentation is a capability lead, not benchmark evidence.

## 8. Provisional hard gates and targets

Hard gates for the declared test pack:

- 0 false portfolio-safe / false service-floor-satisfied results in sealed negative controls.
- 0 false-independent options under owner-supplied common-cause controls.
- 0 resource capacity or unit/time-window violations accepted.
- 0 unauthorised consequential actions.
- 100% deterministic agreement with an independently authored integer/rational reference on frozen cases.
- 100% claim lineage for qualified outputs, including each source owner and scope boundary.
- Any unknown required capacity, service floor, authority, or common-cause receipt produces an explicit abstention rather than optimistic completion.

After operator data access and a powered holdout exist, proposed comparative targets are:

- at least 10% lower decision regret or lifecycle cost per independently verified safe service unit than the strongest matched baseline, without reducing any protected cohort's service floor;
- at least 10% improvement in worst-case service deficit or safe service-hours preserved under the same resource budget;
- calibrated nominal 90% service intervals with 85–95% coverage on held-out operational periods, reported separately by service cohort and hazard family.

These are hypotheses, not SOTA claims. They should be revised after variance, sample size, practical effect thresholds and operator constraints are preregistered.

## 9. Why this could be a meaningful field contribution

Resilience curves and service-performance measures are established research concepts; FSRE must not claim to invent them. The distinctive, testable contribution is to connect owner-qualified recovery options, explicit common-cause and resource competition evidence, service- and cohort-specific floors, authority-preserving portfolio comparison, and independent outcome reproduction.

This turns “we have three backup routes” into the falsifiable question “under which declared shocks, for which populations, with which shared resources and permissions, can this portfolio actually preserve the service floor?”

The 2026 comparative review of infrastructure interdependency tools identifies seven tools explicitly modelling cross-sector dependencies and cascading effects among 24 tools in scope; it finds differentiated method strengths and no one tool capturing the full breadth of interdependencies. That supports a shared evaluation layer, but does not prove FSRE is superior or that any industry product lacks equivalent functionality. Reference: Shamsi, Helmrich & Deegan (published 7 November 2026), https://doi.org/10.1007/s10669-026-10162-2.

## 10. Implementation boundary and next experiment

Do not implement another dependency graph or general-purpose simulator. First run a paper-and-fixture benchmark using owner-produced synthetic receipts from Mycelix, Sol-Atlas, Symtropy and the operator. Independently review the input/output contract. Only then build a small deterministic portfolio reference evaluator over the frozen receipts and a disjoint test set.

The first real-water study must remain blocked until lawful, system-level Green Drop identifiers, operational observations, workforce/resource evidence, owner authority and a defensible historical cohort are available. National aggregate percentages are not a substitute for system-level evidence.
