# Beyond Trajectory Clearance: An Airspace Coordination Research Programme V1

**Status:** Research thesis and benchmark contract; not an operational design approval  
**Authority ceiling:** Research simulation, candidate planning, interoperability, and assurance evidence only  
**Date:** 2026-10-15  
**Parent:** Mycelix AeroCommons [PR #5024](https://github.com/Luminous-Dynamics/mycelix/pull/5024)

## Executive thesis

The opportunity is not to train one smarter air-traffic controller. It is to build an **open, verifiable coordination fabric for heterogeneous airspace users**: one in which independently built models, operators, infrastructure providers, evidence services, and competent authorities can exchange intentions and constraints without pretending that every vehicle shares one dynamics model or that coordination software is the source of safety authority.

The potential differentiator for Luminous Dynamics is the composition of five capabilities under one testable contract:

1. explicit model applicability and uncertainty boundaries;
2. source-pinned, time-qualified spatial evidence;
3. independently recomputed hard constraints;
4. federated, time-dependent resource planning;
5. reproducible scenario, counterfactual, and adversarial evidence.

None of these ideas alone is new to aviation. FAA UTM concepts already describe distributed flight-intent sharing and strategic deconfliction; ICAO calls for interoperability with ATM; EUROCONTROL ECHO 2 addresses integration with higher-altitude and space-linked operations; NASA research highlights the difficulty of conflict resolution when corridor constraints reduce maneuvering options. The research opportunity is to make the handoffs between those concerns explicit, composable, and independently testable—not claim to replace them.

## Paradigm shifts

### 1. From aircraft points and predicted lines to qualified reachable volumes

A point estimate and nominal trajectory are not a complete description of a future aircraft state. Position error, stale reports, wind, control limitations, turn/acceleration bounds and intent changes can cause the actual state to diverge from the nominal path.

**New abstraction:** an intent is evaluated against a model-qualified space-time envelope over an explicit horizon. It carries uncertainty semantics, assumptions, validity, and a precise model version. When a model cannot bound the relevant behavior, the answer is unresolved—not a smaller guessed buffer.

**Research question:** for a declared vehicle/regime and input domain, can a model construct a conservative envelope with explicit error bounds, and can an independent oracle detect any envelope under-approximation in the benchmark domain?

**First step:** preserve the current toy kernel as a synthetic baseline and define the envelope interface separately. Do not label a caller-supplied margin, velocity-scaled guard band, or empirical confidence score as a validated reachable set.

### 2. From static airspace lanes to a dynamic space-time resource fabric

A corridor can reduce one kind of coordination complexity while removing options needed to resolve conflicts. NASA's 2025 strategic path-planning flight evaluation reported less successful maneuvers in tightly constrained flow-corridor conditions, where available degrees of freedom were reduced. A fixed lane cannot be the only conflict-management answer.

**New abstraction:** represent qualified route segments, temporary restrictions, arrival/departure windows, landing/charging resources, weather-dependent availability and contingency options as separate typed resources with explicit time intervals and provenance.

A plan requests a set of compatible space-time resources; a planning service proposes assignments; an independent verifier checks resource conservation, constraint eligibility and evidence freshness. The system must be able to compare corridor-based and less-constrained alternatives.

**Research question:** does adaptive topology increase the number of feasible conflict-free plans without violating any hard constraint, and under what traffic densities or disruption regimes does it stop helping?

### 3. From a single controller to federated coordination with bounded authority

Distributed information exchange is already a foundational part of UTM concepts. The paradigm shift is not “put air traffic on a ledger.” It is to make each handoff's authority, freshness, identity, sequence and failure semantics machine-checkable.

**New abstraction:** each provider publishes versioned intent/constraint claims with bounded validity; participants reconcile conflicts explicitly; stale or partitioned views can surface known impacts but cannot prove that no impact exists. Local planners may continue generating proposals during a partition, but a proposal must not become an authorization merely because peers agree.

**Research question:** when providers disagree, restart, replay messages, lose connectivity or present forked histories, can all conforming implementations converge on the same safe-to-report classification—or return unresolved/degraded rather than manufacture agreement?

**Boundary:** Mycelix can preserve coordination records, verifiable receipts and governance around the protocol. DIDs, consensus, signatures and reputation do not establish physical truth, airworthiness or legal clearance.

### 4. From optimizing a scalar score to safety-first feasible-set planning

A weighted sum can hide unacceptable tradeoffs: a cheap or fast plan may numerically compensate for a violated hard constraint. Hard aviation restrictions and model applicability cannot be treated as another cost term.

**New abstraction:** apply constraints in lexicographic stages:

1. input/evidence admission;
2. model applicability and uncertainty qualification;
3. authoritative restrictions and hard feasibility gates;
4. independent verification;
5. optimization only among eligible candidates;
6. performance, access, environmental and recovery reporting.

Cost, throughput, energy, emissions, noise, delay and equitable access remain useful objectives **inside** the feasible set. They cannot turn an infeasible candidate into an eligible one.

**Research question:** can adversarial tests demonstrate that no choice of performance weights ever admits a candidate already rejected by a hard gate?

### 5. From one universal dynamics model to a model federation

Fixed-wing aircraft, multirotors, rotorcraft, powered-lift aircraft, gliders, high-altitude balloons and high-speed vehicles have meaningfully different dynamics and operating regimes. A taxonomy does not implement those dynamics; neither does a general AI model.

**New abstraction:** each trajectory model advertises a versioned applicability domain: valid profile/regime, state variables, coordinate/time basis, uncertainty behavior, known omissions, limits, evidence basis and failure modes. The shared coordinator chooses from qualified models or returns unresolved. Independent monitors check the outputs that matter to the hard constraints.

Symthaea can rank hypotheses, find inconsistent evidence, propose candidate alternatives and explain why a monitor disagrees. It must not widen a model's domain, fill absent source fields, or overrule the independent checker.

**Research question:** when two models apply to the same declared domain, do their output envelopes and disagreement bounds behave as expected under a differential corpus? When only one model applies, is the scope explicit? When neither applies, is geometric clearance impossible to obtain?

### 6. From a simulation result to a reproducible assurance artifact

A green scenario result only means the implementation matched an expected result under the supplied assumptions. It does not show that the input was authentic, that the assumptions cover reality, or that the oracle is independent.

**New abstraction:** every experiment produces a receipt binding repository commit, model/build identity, scenario manifest, source snapshots, ruleset, toolchain, checker/oracle identity, outputs, and exit status. Expected results are checked by an independently implemented oracle or by properties that do not simply repeat the implementation.

Tests should include negative controls, permutation invariance, exact time-boundary tests, replay/idempotency tests, model-domain boundary tests, mutation tests, and adversarial missing/stale/contradictory data. The corpus itself has a version and digest.

**Research question:** can a third party reproduce the same classification and independently verify the result from pinned artifacts without trusting the plan producer?

### 7. From airspace-only optimization to joint air/ground/community systems

Airspace capacity is coupled to runways, vertiports, charging, energy availability, emergency landing options, weather, ground exposure and the communities below. FAA AAM materials likewise emphasize infrastructure—not just the aircraft—and ICAO's UTM framework treats ground risk, interoperability and environmental/economic effects as part of the broader problem.

**New abstraction:** Sol-Atlas provides a read-only spatial projection of source-qualified infrastructure and context; Symtropy explores capacity/resource allocations; Mycelix preserves intent and evidence lineage; Symthaea can generate advisory forecasts and explanations. Keep observed state, inferred state, scenario forecasts and candidate decisions as separate layers.

**Research question:** can feasible plans reduce delay, energy use, noise/ground exposure and emergency recovery time while keeping all hard gates intact and reporting who bears the costs and benefits?

### 8. From nominal throughput to recoverable capacity and option value

A schedule can look excellent in an undisturbed day and still be brittle: a closed vertiport, a newly active restriction, a provider partition, stale capacity data, or an emergency mission can remove the only remaining feasible continuation. Optimizing the current flight set is not the same as keeping future safe choices available.

**New abstraction: a recovery-option set.** Alongside each research candidate, record which alternate time windows, route/resource combinations, landing options, or other declared contingencies remain eligible under specified disruption assumptions. Each option must name its dependencies, validity interval, evidence frontier, and the independent feasibility profile that qualified it. A recorded option is not a guarantee: it can expire or become unavailable, and it must never be represented as live without current evidence.

**Paradigm shift:** treat *recoverability* as a system-level property of an allocation, rather than measuring only per-flight conflicts, nominal throughput, or delay. The research question becomes: *how many independently checked continuations remain when the environment changes, and who loses service when they do not?*

A useful comparison should replay identical pinned demand under at least three policy families: (a) nominal-throughput-first scheduling, (b) policy-defined contingency capacity, and (c) an adaptive option-preserving strategy. Do not prescribe a universal reserve percentage: the correct policy depends on jurisdiction, infrastructure, mission mix, weather and declared operating assumptions, and has to be tested rather than guessed. NASA's [Provider of Services for Urban Air Mobility prototype simulation (X5)](https://ntrs.nasa.gov/citations/20250001367) is a relevant baseline because it investigated demand-capacity balancing and sequencing/scheduling; it does not validate any Mycelix allocation rule.

Measure separately: eligible recovery options before/after each event, time to recover, delay distributions, unserved missions by declared class and region, emergency-service availability, capacity left idle, energy/noise/ground exposure where qualified inputs exist, and computation cost. Preserve worst-case and subgroup outcomes. No weighted average may turn a hard-gate violation into an acceptable trade, and an aggregate improvement must not conceal a material access penalty.

**Falsifiers:** the option-preserving policy wins only because it sees easier inputs; stale or closed resources are counted as available; an emergency label silently preempts another mission; performance improves by relaxing a hard constraint; or total throughput rises while predeclared emergency/community outcomes silently worsen.

The next machine-readable case, [AIR-PB-013](../../tools/airspace-sim/fixtures/paradigm_shift_benchmarks.json), makes this an explicit research backlog item. It is a test contract, not an implemented planner or measured result.

### 9. From conflict-free traffic to auditable, policy-bound access

Preventing incompatible occupancy is necessary but not sufficient for a legitimate airspace commons. Under scarcity, a scheduler also decides who waits, who gets a slot, which missions absorb disruption, and which communities bear noise or ground-risk burdens. A high-throughput result can hide a concentrated service penalty.

There is a concrete standards gap to examine rather than assume away: [ASTM F3548-21](https://store.astm.org/f3548-21.html) covers strategic conflict detection, aggregate intent conformance, constraint awareness and interoperability, but explicitly says that this edition does **not** establish fairness or equitable-access requirements; it does include logging that can inform future requirements. Mycelix can investigate transparent allocation evidence without pretending it can set aviation policy.

**New abstraction: an authority-bound allocation policy plus an access ledger of outcomes.** A policy is a versioned input that names its competent authority, effective interval, eligible decision fields, priority/tie-break semantics, exceptions and supersession. Each request gets an explainable allocation outcome—accepted, deferred, rejected or unresolved—with a reason linked to that exact policy. The research report measures service rate, delay and denial/defer reasons by predeclared mission/service group and region, including denominators and missing-data counts.

**Non-negotiable boundary:** do not infer priority from Mycelix reputation, provider membership, commercial size, model confidence, or hidden defaults. Such a field may influence allocation only if an applicable, authorized policy explicitly permits it. The system must not select its own public priorities, declare a policy universally lawful, or use an equity metric to waive a safety/model/evidence gate. Mycelix preserves evidence and policy provenance; it does not create flight authority.

The first falsifiable experiment is a metamorphic test: hold requests, capacity, safety constraints and authorized policy constant while permuting input order and changing metadata that the policy does not authorize as decision-relevant. Outcomes should remain invariant. Then measure the policy's intended trade-offs transparently across declared groups under scarcity, keeping safety eligibility, efficiency, resilience and access-distribution results as separate dimensions.

The machine-readable case [AIR-PB-014](../../tools/airspace-sim/fixtures/paradigm_shift_benchmarks.json) is research backlog, not an implemented allocator or a claim that any particular allocation policy is fair. The goal is to make normative decisions explicit and reviewable rather than bury them inside an optimizer.


### 10. From disconnected confidence scores to compositional uncertainty

A pipeline can be wrong even when every component looks locally plausible. A sensor reports a position bound; an adapter transforms the coordinate frame; a clock adapter aligns timestamps; a tracking service propagates the state; a dynamics model predicts forward. If one boundary drops units, latency, a common datum bias, a clock offset, a correlation, or a model residual, the final “uncertainty” may be narrower than the states the evidence actually permits.

**New abstraction: an uncertainty dependency graph.** Each component declares the quantity it bounds, units and coordinate/time basis, domain and validity horizon, bound/distribution semantics, source provenance, and dependencies needed by its composition rule. Transform and propagation nodes retain their contribution and assumptions. The system combines *sets/bounds or qualified probabilistic quantities according to an explicit rule*; it does not add arbitrary margins, treat every error as independent, or turn a confidence number into a guaranteed physical bound.

This is not a claim that uncertainty propagation is new. Set-propagation and reachability methods have a mature research base ([Althoff, Frehse and Girard, 2021](https://doi.org/10.1146/annurev-control-071420-081941)); NASA has published trajectory conflict-detection methods using variable four-dimensional uncertainty bounds ([NASA report](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20200000495.pdf)); and a May 2026 study examined uncertainty quantification for state-based conflict detection and resolution in U-space ([Rahman, Ellerbroek and Hoekstra, 2026](https://doi.org/10.1016/j.ress.2025.112111)). The distinct Mycelix research question is whether uncertainty metadata, dependence, validity, source lineage and failure semantics survive *every interoperability boundary* and can be independently challenged with reproducible cross-stage counterexamples.

**First experiment:** build a synthetic pipeline with analytically controlled observation error, shared/common-mode datum bias, clock skew, message latency, transform error and model residual. Compare the emitted envelope against an independent set-based oracle. Deliberately remove dependence IDs, duplicate a source, reorder stages, expire evidence, and substitute an unsupported composition rule. Each mutation should either produce the same permitted envelope or reduce the result to unresolved/degraded—never silently shrink the claimed bound.

**Falsifiers:** an oracle state falls outside an envelope advertised as conservative; shared errors are treated as independent without justification; a missing component defaults to zero; or a result cannot be reconstructed from exact source, transform, model, rule and checker identities. An “uncertainty budget” is not a safety guarantee unless its semantics and domain are qualified.

The machine-readable case [AIR-PB-015](../../tools/airspace-sim/fixtures/paradigm_shift_benchmarks.json) remains the full-pipeline research contract. The narrow test-only scalar affine implementation in [uncertainty.rs](../../tools/airspace-sim/src/uncertainty.rs) is tracked by AIR-PB-017 through AIR-PB-019. AIR-PB-018 separates **source origin** (`source_id`) from **latent dependency identity** (`dependency_id`) so two origins can share one declared error while one origin can contribute distinct error modes. AIR-PB-019 adds a **typed, time-qualified dependency contract** naming quantity kind, unit, reference frame, time basis, half-open validity and provenance status. The prototype accepts only synthetic position error in synthetic millimetres, one synthetic local frame and one synthetic epoch in integer seconds. A dependency cannot combine contributions with inconsistent contract metadata; `valid_until_s` is excluded; future and expired evidence return distinct errors. The contract accepts only `declared_unverified`, never an unsupported claim of authentication because this module has no verifier.

The test-only module now declares 23 Rust test functions and uses 22 hand-derived vectors in `airspace-affine-uncertainty-v3`, covering unsupported units/frames/time bases, provenance-label rejection and temporal boundaries in addition to origin/dependency arithmetic. Independent host-side checks are not Rust compilation or execution evidence. The primitive is not connected to trajectory geometry; it does not transform coordinate frames, propagate clock latency, model multi-axis/nonlinear dynamics, authenticate sources or prove declared bounds cover physical reality. The next full-pipeline gate is to propagate typed contract records across adapters and challenge them with a separately implemented verifier before any real-world model admission.


### 10A. Separate bounded-set containment from probabilistic risk calibration

AIR-PB-020 is a new research backlog item prompted by the May 2026 paper [Uncertainty quantification on state-based conflict detection and resolution algorithms](https://doi.org/10.1016/j.ress.2025.112111). That study models navigation-position and velocity errors with Gaussian assumptions and compares Monte Carlo with analytical approximations. Its findings emphasize that detection probability can vary with uncertainty and encounter geometry over the look-ahead horizon. A scalar affine interval and a calibrated probability are therefore not interchangeable:

- **Set containment:** prove that all states in the declared finite-domain or qualified reachable set fall inside the reported interval. A result outside the envelope is a critical falsifier.
- **Probability calibration:** test a stated distribution against a pinned holdout corpus using reliability diagrams, Brier score, calibration error and uncertainty intervals, stratified by encounter geometry, uncertainty magnitude, horizon and source-quality class.
- **Policy separation:** calibration scores and risk estimates remain advisory metrics. They cannot waive a hard constraint, qualify an unvalidated model or turn stale/unknown evidence into a normal result.

The proposed first synthetic target is expected calibration error <= 0.05 with predeclared binning and minimum sample-size rules. This target is for the synthetic benchmark only; it does not imply an acceptable aviation risk level. AIR-PB-020 requires a misspecified-distribution control and explicit cases at the look-ahead boundary before any claim of probabilistic utility.

### 11. From multiple green checks to evidence of assurance diversity

Two checkers that share a parser, transformation function, model assumptions, or generated expected outputs may reproduce the same defect. Agreement is evidence only to the extent that the verification paths are capable of failing differently and have actually been challenged. NASA's [runtime-assurance recommendations](https://ntrs.nasa.gov/citations/20220015734) explicitly warn that the monitor must itself be fit for purpose; redundant labels alone do not establish this.

**New abstraction: an assurance dependency graph plus a mutation ledger.** Record who generated each expected result, which sources and libraries each checker uses, which predicates are independently derived, and where components remain shared. Then deliberately mutate the plan and its evidence: boundaries, units, clock/datum, source freshness, uncertainty dependencies, capacity totals, authority scope and serialization. Include legitimate near-boundary counter-controls so a checker cannot pass by rejecting everything.

This is not a claim that N-version programming, independent verification or mutation testing are new. The research shift is to treat *the strength of the assurance argument itself* as an object that can be attacked and measured. For each hard invariant, report which checker detected each fault, which faults survived, whether detectors share the suspected faulty dependency, and what remains outside the tested domain. Do not convert these dimensions to a single “assurance score.”

**Falsifiers:** a shared-library mutation fools all checkers; any injected hard-gate violation escapes without an explicit coverage gap; a crash, timeout, mismatch or disagreement is interpreted as approval; or a reported pass cannot be traced to pinned source/build/dependency identities and predeclared fault outcomes.

The machine-readable case [AIR-PB-016](../../tools/airspace-sim/fixtures/paradigm_shift_benchmarks.json) is a research contract only. The current repo does not yet implement multiple independent airspace checkers or this mutation campaign. The intended next step is not to claim the monitor is independent, but to create evidence of what it catches—and where correlated failure remains possible.


## Cross-project reference architecture

| Layer | Primary responsibility | Project fit | Forbidden shortcut |
|---|---|---|---|
| Evidence ingress | Pin source bytes/version, rights, freshness, time/datum, quality and authenticity state | Sol-Atlas contracts plus future reusable receipt verifier | “The URL parsed, therefore the data are true/current/licensed” |
| Spatial/temporal adapter | Explicit frame, axis, units, vertical datum, time scale, transform direction and bounded error | Sol-Atlas geometry boundary; Symtropy qualified transforms | “Same numeric coordinates means same place” |
| Model admission | Select an explicit versioned model only within its qualified domain | Mycelix AeroCommons registry | “Vehicle label or plausible velocity means the toy model applies” |
| Prediction and advisory | Produce hypotheses, candidate paths, uncertainty summaries and explanations | Symthaea, advisory only | “AI confidence can replace evidence or hard checks” |
| Hard-constraint monitor | Recompute eligibility/impact independently and retain unresolved reasons | Mycelix research kernel, eventually independently implemented oracle | “Optimizer or producer self-certifies its own plan” |
| Resource planner | Assign time-bounded route/infrastructure capacity among eligible candidates | Symtropy optimization behind gates | “Static edge capacity equals time-dependent airspace capacity” |
| Coordination log | Versioned intent/constraint changes, expiry, conflicts, replay and audit | Mycelix, with explicit authority ceiling | “Consensus/ledger receipt grants flight clearance” |
| Assurance/evaluation | Adversarial corpora, counterfactuals, differential checks and exact-head receipts | All four, with independently reproducible results | “Queued checks or matching expected values prove real-world safety” |

## Falsifiable benchmark programme

The proposed tiered scorecard in [AIRSPACE_BENCHMARK_TARGETS_V1.md](AIRSPACE_BENCHMARK_TARGETS_V1.md) sets target gates for contract correctness, uncertainty composition, mutation resistance, cross-implementation reproducibility, bounded-workload performance and planning/community outcomes. These are proposed targets only, not measured results or aviation acceptance criteria.

A machine-readable benchmark profile lives at `tools/airspace-sim/fixtures/paradigm_shift_benchmarks.json`. Most cases remain **research backlog**, not completed simulations; AIR-PB-017 has a narrow test-only implementation pending exact-head verification, while AIR-PB-015 and AIR-PB-016 remain full research backlogs. Each case specifies a hypothesis, baseline, required evidence, hard invariants and a falsifier.

The first qualification ladder is:

1. **Admission tests:** unsupported, missing and mismatched models perform zero geometry work. This is the first implemented slice, pending exact-head test results.
2. **Metamorphic tests:** input order, exact duplicate deliveries and irrelevant record additions must not alter classification except where the contract explicitly says otherwise.
3. **Model tests:** compare nominal paths with conservative envelopes, test applicability boundaries, and verify that the independent oracle detects deliberate under-approximations.
4. **Resource tests:** test contention, shared capacity, half-open time intervals, expiry, recovery options and the invariant that optimization cannot rescue a hard-ineligible plan.
5. **Federation tests:** partitions, stale snapshots, reordering, forks, contradictory authorities and revocation.
6. **Counterfactual system tests:** compare corridor-first and topology-flexible planning under identical demand and constraints; report feasibility, delay, compute cost and externalities without hiding infeasible runs.
7. **Independent reproducibility:** generate pinned evidence receipts and re-run with a separately implemented checker.

No case may describe a synthetic win as a safety validation or operational clearance.

## Research and interoperability grounding

- [FAA UTM Concept of Operations v2.0](https://www.faa.gov/researchdevelopment/trafficmanagement/utm-concept-operations-version-20-utm-conops-v20): distributed flight-intent sharing, operation volumes, coordination and conflict management.
- [FAA UTM overview](https://www.faa.gov/uas/advanced_operations/traffic_management): UTM is complementary to air traffic services and depends on interoperable operational capabilities.
- [ICAO UTM Framework](https://www.icao.int/utm-guidance): interoperability, safety, security, environmental and economic considerations, and integration with ATM.
- [EUROCONTROL ECHO 2](https://www.eurocontrol.int/project/european-concept-higher-altitude-operations): higher-altitude platforms and space launch/re-entry integration research.
- [NASA/TP-20250003243, published July 2025](https://ntrs.nasa.gov/citations/20250003243): flight evaluation of strategic path planning; reports difficulties when tight flow corridors reduce conflict-resolution degrees of freedom.
- [NASA Runtime Assurance of Aeronautical Products, published January 2023](https://ntrs.nasa.gov/citations/20220015734): explains runtime-assurance challenges and that the monitor itself must be fit for purpose.
- [NASA Software Assurance and IV&V, published 8 April 2026](https://sma.nasa.gov/news/articles/newsitem/2026/04/08/reducing-risk-in-software-driven-hazards-across-nasa-programs-how-software-assurance-and-iv-v-strengthens-mission-safety): discusses common-cause failure, dissimilar redundancy, targeted fault injection, and time/data-validity assurance.
- [Baker and Habli, “An Empirical Evaluation of Mutation Testing For Improving the Test Quality of Safety-Critical Software,” IEEE Transactions on Software Engineering (2013)](https://doi.org/10.1109/TSE.2012.56): evaluates mutation testing as a complement to structural coverage in safety-critical software.

## What would constitute a real breakthrough?

Not a more impressive demo or a single high utilization score. A meaningful breakthrough would be a **publicly reproducible, independently checked benchmark** showing that heterogeneous candidate models, stale/conflicting data, provider partitions and time-dependent capacity can be composed while:

- every unsupported model remains unresolved;
- hard constraints cannot be traded away by optimization;
- independent checking finds intentionally injected plan/evidence faults;
- eligible alternatives and their tradeoffs are explainable;
- recovery and ground/community impacts are measured, not hidden;
- another implementation can replay the same corpus and reproduce the same scoped result.

That would be an extensible research infrastructure others can challenge and build on—not a claim to replace an air navigation service provider or civil aviation authority.

## Explicit non-claims

This profile does not implement certified reachable sets, aircraft dynamics, geodetic transformations, authenticated operational feeds, legal authorization, approved separation minima, or real-time air traffic control. The next engineering milestones must earn each capability through dedicated models, independent oracles and evidence from the exact revision being assessed.
