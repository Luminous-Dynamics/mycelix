# FSRE research track: service trajectories and option-preserving resilience v0.1

**Status:** Research proposal and benchmark specification.  
**Date:** 2026-11-13  
**Claim ceiling:** No field result, SOTA claim, resilience guarantee or validated physical model is asserted.

## 1. The proposed paradigm shift

A static asset state ("up/down"), a dependency graph, a recovery-time objective and a simulation of one failure are each useful but incomplete descriptions of resilience.

The FSRE research hypothesis is that essential-service resilience should be evaluated through two linked objects:

1. **The service-capability trajectory:** how much safe, acceptable service is delivered over time, including safe degradation, unsafe conditions, interruptions, recovery and unknown intervals.
2. **The qualified recovery-option set:** which alternative ways of restoring or sustaining the service remain physically feasible, reachable, resourced, staffed, authorised and safe under the same disruption.

This changes the central question from "did the asset survive?" to:

> Under this declared disruption and evidence boundary, what service capability remained available, for whom, for how long, and which independent recovery options were actually executable?

This is not a new country-scale simulator. It is a service-lane benchmark that consumes owner-native outputs from the existing systems.

## 2. Option preservation as a resilience property

An apparent alternative is not automatically a real recovery option. A candidate option counts as *qualified for the declared scenario and time window* only when all required predicates are evidenced:

| Predicate | Existing owner / evidence source | Rejection condition |
|---|---|---|
| Physical feasibility and capacity | Symtropy domain model, model adequacy and scenario receipts | Capacity, quality, operating envelope or uncertainty is insufficient/out of domain |
| Spatial reachability and access | Sol-Atlas topology, geography, provenance and resilience alternatives | Route is inaccessible, stale, outside the modelled area or not independently evidenced |
| Dependency independence for the fault under test | Mycelix time-qualified dependency hypergraph and common-cause assessment | Candidate shares the failed failure domain, or dependency closure is unknown/incomplete |
| People, skills, materials and energy | Operator-approved resource/workforce records and qualified domain constraints | Required staff, credential, spare part, energy or logistics are unavailable |
| Current operational authority | Mycelix scoped, purpose-bound, revocable authority and operator approval | Authority is absent, expired, revoked, conflicted or outside jurisdiction |
| Safety, rights and distributional conditions | Declared service contract, operator constraints and independent evaluator | Quality, safety, affordability, accessibility, environmental or rights guardrail fails |
| Reversibility and abortability | Intervention proposal, operating procedure and execution boundary | Action cannot be bounded, monitored, stopped or compensated as required |

Do not count routes, vendors, assets or credentials as independent simply because their identifiers differ. Independence must be assessed against typed common failure domains and the evidence frontier. A missing dependency edge is **unknown**, not independence.

Do not sum candidate alternatives into a universal resilience score. Report the qualified option set by service, scenario, time window, failure-domain class, capacity range, resource dependencies, authority state and evidence quality. A count may be shown as a convenience only when the underlying option records and their independence classes remain inspectable.

This benchmark consumes the existing Mycelix hypergraph, Sol-Atlas alternatives, Symtropy model results and Mycelix authority records. It must not create a second dependency engine, spatial graph, physical simulator, identity system or authority ledger.

## 3. Service capability contract

Each benchmark service/cohort lane declares, before evaluation:

- service identifier and responsible operator;
- service unit and measurement method (for example, compliant household-water service-hours, not an unspecified "resilience score");
- service area and protected cohort scope;
- observation horizon and sampling/validity rules;
- target service fraction and minimum safe service floor;
- quality constraints that cannot be traded for availability;
- affordability, accessibility, safety, environmental and rights constraints;
- minimum service continuity requirement and recovery-stability window;
- allowed observations, source provenance and maximum observation-hold duration;
- baseline, comparator and uncertainty method;
- explicit unknown and abstention rules.

Each service area and cohort is reported independently before any summary. Avoid averaging away a community that experienced a severe outage because another area improved.

## 4. Service trajectory measures

Let (a_s(t)) be observed service availability as a fraction of the declared target, and (u_s(t)) denote whether the observation is valid for the required quality/safety conditions. Let (T_s) be the declared service target and (F_s) the minimum safe service floor.

For known intervals only:

[
D_s = int max(0, T_s-a_s(t)),dt
]

(D_s) is the cumulative service deficit (fraction-seconds or the declared service-unit equivalent). It does not by itself express health, water-quality, affordability or environmental harm; those remain separate measures.

Report, at minimum:

- cumulative service deficit, with the unit and integration rule;
- time below the minimum service floor;
- time with quality/safety non-compliance;
- safe degraded-service duration;
- full-service duration;
- longest contiguous unsafe-service episode;
- time to first known breach;
- time to stable safe recovery;
- time to stable full restoration;
- known-observation coverage and unknown duration;
- distribution of all measures by service area and declared cohort.

**Unknown is a first-class outcome.** Do not forward-fill a stale observation indefinitely, interpolate across a gap without an approved model, or count an unknown interval as healthy. Bound how long an observation may be held; report the remainder as unknown. Quality violations and capacity shortfalls must remain separately visible.

The reference evaluator in `tools/fsre-service-trajectory` implements a deliberately small deterministic subset: right-continuous observation holding up to a declared maximum age, explicit unknown intervals, service deficit integration, minimum-floor exposure, quality non-compliance and stable-recovery timing. It does not verify source authenticity, model physical processes, infer causes or aggregate different services.

## 5. Recovery-option measures

For each scenario/time window, publish:

- options proposed, rejected, unknown and qualified;
- rejection/abstention reasons;
- qualified capacity and the uncertainty range;
- distinct failure-domain classes represented;
- common-cause overlap and cut-set information from the canonical dependency owner;
- time to mobilise, travel/logistics constraints and skill requirements;
- current authorisation state and required approvals;
- options lost or gained after each observed disruption or intervention;
- cost and resource consumption for each executable option.

A recovery option is not executable merely because a model found a graph path. Require physical feasibility, current dependencies, available people/resources, an accountable operator and current authority.

Potential research indicators include the number of independently qualified recovery portfolios, recovery-option diversity by failure domain, the minimum number of distinct failure domains separating alternatives, and the fraction of candidate options that survive compound-failure and authority-revocation challenges. These are separate descriptive indicators; no scalar resilience score is proposed.

## 6. Benchmark families

### R1 — Graceful degradation

Inject a fault that removes primary capacity while a declared fallback remains. Check that the service trajectory degrades only within its declared safe envelope and that fallback capacity is not assumed to be unlimited.

### R2 — False redundancy and hidden common causes

Give a service multiple apparent providers that share a power, communications, upstream supplier, control plane or common maintenance dependency. The system must not overstate independent recovery options. Use Mycelix's canonical hypergraph and the hidden common-cause track (#5981).

### R3 — Human fallback with missing capability

Declare a manual fallback but remove required trained staff, credentials, procedures, communications or shift coverage. The option must become unavailable or unknown; the model must not treat the existence of a manual-control node as proof of safe manual operation.

### R4 — Compound disruption and resource contention

Combine disruptions that compete for the same repair crews, spares, transport, fuel, energy or finance. Compare the option set and service trajectory with an unconstrained model. Report the bottleneck and uncertainty instead of silently assuming unlimited resources.

### R5 — Observation gaps and contradictory evidence

Inject stale, delayed, duplicated, missing and conflicting observations. The evaluator must expose unknown periods; the reasoning system must abstain where evidence coverage is insufficient for the claim.

### R6 — Authority loss during recovery

Revoke a grant, expire a delegation or introduce a jurisdiction/purpose conflict after a recovery proposal is produced but before execution. The option must no longer be executable. Revalidation must occur at execution time.

### R7 — Alternative-path inflation

Duplicate a provider, split one asset into multiple identifiers, add a parallel route that shares a common failure domain, or insert an unqualified source. The qualified independent-option result must not increase merely because the graph contains more labels or paths.

### R8 — Federation partition and local continuity

Partition three or more independent operator nodes. Verify that each retains safe local operation, cannot acquire another node's authority, records unresolved conflicts, and reconciles evidence without silent last-write-wins.

### R9 — Recovery maintenance and adaptation

After a fault, let operators inspect, repair and adapt the service model. Measure whether the next similar event becomes less damaging without creating hidden dependency concentration or degrading a different cohort.

## 7. Matched baselines and ablations

Use the same frozen scenario, source frontier, resource limits, operator policy and evaluation horizon for:

- current operator runbook and actual operational process;
- simple N-1/redundancy analysis or domain-specific resilience method;
- existing dependency and asset-management tooling;
- best qualified domain simulator;
- strong AI agent with equivalent permitted data/tools;
- qualified human operators/engineers;
- full FSRE composition;
- FSRE ablations without spatial alternatives, physical adequacy, causal investigation, authority binding or independent outcome verification.

A comparator must receive the same information and tuning budget. Publish any unavailable comparator. Component benchmark wins do not transfer automatically to FSRE.

## 8. Hard gates and provisional targets

Hard gates:

- zero accepted unauthorised or out-of-scope consequential actions in the declared adversarial suite;
- zero duplicate effective mutations in retry, replay, timeout and partition tests;
- zero false-safe classifications in frozen quality/unsafe and unknown-observation controls;
- zero false-independent recovery options in frozen common-cause and alternative-path inflation controls;
- 100% abstention when required authority is revoked, the dependency frontier is incomplete for the requested independence claim, or required quality evidence is unknown;
- 100% exact metric agreement with the independent reference calculation on deterministic fixtures.

These are conformance targets for the specified suite, not statistical guarantees about production deployments. Always publish sample sizes, confidence bounds and the full error list. Zero observed failures in a small synthetic corpus is not proof of zero risk.

Do not set a universal target for recovery-time improvement or option count before obtaining a service baseline. After baseline data and an operator-approved study design exist, pre-register practical-effect thresholds and confidence intervals for each service lane. Compare costs over the full lifecycle.

## 9. Causal and economic evaluation

Symthaea should propose competing explanations and choose information-gathering actions by expected value of information, subject to safety, permission, cost and privacy limits. The experiment is not successful because a hypothesis becomes more confident; it is successful when the new observation discriminates among hypotheses and improves the next decision.

For each intervention, evaluate:

- direct service effects;
- spillovers to connected services and neighbouring areas;
- displacement of failure or cost to another community;
- reporting/measurement changes;
- recurring operating costs and lifecycle liabilities;
- local skills, wages and supplier participation where the study has authority and reliable data;
- distribution of benefit and harm, with uncertainty.

Use a pre-registered before/after design and, where possible, matched controls or stepped-wedge introduction. Account for seasonality, planned maintenance, demand, weather, tariffs, capital works and concurrent interventions. Independent evaluators should reproduce the outcome from retained evidence.

## 10. Research hypothesis and falsification

The research hypothesis is that a composed system that measures service trajectories and tests the real executability of recovery options will make better resilience decisions than approaches that optimise static asset state, declared redundancy or isolated model performance.

Falsify or narrow this hypothesis if:

- it does not outperform the operator workflow on matched, service-relevant tasks;
- recovery-option qualification adds delay/cost without reducing unsafe choices or service deficit;
- current tools already provide equivalent cross-organisation functionality at lower total cost;
- unknown/abstention rates make it unusable in realistic data conditions;
- the composition centralises authority or data in ways that reduce local autonomy or increase systemic risk;
- measured gains in one service lane create material unmeasured harm elsewhere.

## 11. Implementation boundaries

- Mycelix owns authority, commitments, evidence lineage and the canonical dependency semantics.
- Sol-Atlas owns spatial evidence, asset geography, topology and alternative paths.
- Symtropy owns physical/industrial model execution and model adequacy.
- Symthaea owns causal hypotheses, diagnostic experiment selection and uncertainty-aware reasoning.
- The operator retains dispatch and physical control.
- An independent evaluator verifies service outcomes and the claim ceiling.

The new trajectory evaluator is only a deterministic metric implementation for time-series observations. It must not become a new dependency graph, physical simulator, evidence ledger, benchmark-manifest system, authority service or national control plane.
