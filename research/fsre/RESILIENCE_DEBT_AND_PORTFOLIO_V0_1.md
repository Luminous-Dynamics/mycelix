# FSRE research track: resilience debt and option-preserving investment v0.1

**Date:** 2026-11-13  
**Status:** Research proposal and benchmark specification.  
**Claim ceiling:** No deployed resilience-debt calculation, capital-allocation result, field outcome or SOTA claim is asserted.

## 1. The strategic shift

Most asset and continuity systems help record condition, failure modes, criticality, maintenance backlog, recovery plans and business-continuity measures. FSRE should not recreate those records or replace maintenance planning.

The research opportunity is to connect those records to the **future recovery options of an essential service**.

A service may appear stable today while accumulating deficiencies that reduce its ability to withstand the next disruption:
- deferred maintenance and unresolved repeat faults;
- backup assets that share hidden upstream dependencies;
- insufficient trained staff or expired credentials;
- single-source suppliers, long lead times or unavailable spares;
- stale, missing or disputed observations;
- untested manual fallback and recovery procedures;
- expired or ambiguous authority to mobilise cross-organisation resources;
- recovery paths whose physical capacity or geographic access is no longer qualified.

Call the derived, service-specific view of these deficiencies **resilience debt**.

This is an engineering construct, not a new financial-accounting liability, a universal risk score or a separate authoritative register. It is a way to describe how accumulated, evidenced deficiencies erode the service's safe capability and the set of recovery options available under declared scenarios.

## 2. Resilience debt is a derived view, not a new ledger

Each debt item should be projected from existing owner-native records and retain their exact evidence references.

| Debt dimension | Existing owner / source | Example deficiency |
|---|---|---|
| Physical condition / maintenance | Operator EAM/work orders; IBM Maximo or equivalent where deployed; Symtropy model constraints | Deferred pump repair, unresolved repeat fault, inadequate reserve capacity |
| Dependency concentration | Mycelix time-qualified dependency hypergraph #5923 and hidden common-cause work #5981 | Nominally distinct supply paths share the same failure domain |
| Spatial / access fragility | Sol-Atlas #21 and provenance/geometry owners | Only fallback route is inaccessible during the declared hazard |
| Model adequacy | Symtropy #4392/#3787 | Recovery capacity or process model is unvalidated or outside its domain |
| Workforce / competence | Operator-approved roster, qualification and maintenance records | A declared manual fallback has no qualified on-shift operator |
| Supply / logistics | Approved supplier, spares and contract records | Critical spare part has a single source and lead time longer than the recovery window |
| Evidence / observability | Mycelix evidence lineage and source records | Critical service state is stale, unavailable or disputed |
| Authority / coordination | Mycelix identity, grants, commitments and revocation records | Required mutual-aid or cross-jurisdiction authority is expired or not agreed |
| Recovery practice | Operator-approved drills, procedures and post-incident reviews | Recovery procedure is untested or failed its last qualified exercise |

FSRE does not own a second maintenance system, supplier registry, workforce database, identity service, evidence ledger or dependency graph. It produces a reproducible read-only projection over the records owned by the operator and specialist subsystems.

Each debt item should include:
- stable source-record identifier and source owner;
- affected service/cohort and geography;
- deficiency mechanism and supporting/contradicting evidence;
- observation time, valid time, retrieval time and age;
- affected disruption/failure-domain scenarios;
- operational consequence and safe-service constraint;
- responsible owner and remediation candidate;
- resource/cost range and lead-time estimate, where evidenced;
- unknowns, uncertainty, expiry and qualification status;
- exact source, model, dependency and evaluator digests.

Unknown source records remain unknown. A missing maintenance work order is not proof that maintenance is complete; the absence of a supplier record is not proof of supply independence.

## 3. Do not collapse debt into one scalar

Resilience debt must remain a multidimensional set of deficiencies. Publish, by service, geography, cohort and declared scenario:

- count and age distribution of known deficiencies by type;
- affected services and required capability floors;
- number of scenarios in which no recovery option is qualified;
- recovery-option set before and after the deficiency is resolved;
- capacity, staffing, access, resource and authority constraints;
- cost and time-to-remediate ranges;
- unknown and disputed evidence coverage;
- historical trend of newly introduced, remediated and recurring debt items.

Useful units are specific to the deficiency: days since a qualified exercise, overdue maintenance hours, unfilled critical-role shifts, spare-part lead time, hours of service below the safe floor, or a named scenario in which the service has no qualified recovery option. They should never be summed across unrelated units.

A dashboard can display a count or trend as a convenience, but the underlying items, evidence, affected cohorts and reason codes must remain inspectable. There is no universal debt score.

## 4. From risk reduction to option-preserving investment

Traditional investment comparisons can focus on asset condition or expected failure reduction. FSRE should test a complementary question:

> Which investment portfolio preserves the greatest set of safe, independent and executable recovery options across a declared scenario set, subject to budget, workforce, environmental, rights and service constraints?

Candidate interventions may include:
- replacing a deteriorated asset;
- adding a backup route or provider that is genuinely independent of the tested failure domains;
- investing in spare parts and local repair capacity;
- training or cross-qualifying maintenance personnel;
- improving telemetry, measurement coverage or laboratory verification;
- diversifying critical suppliers;
- establishing mutual-aid and cross-authority arrangements;
- validating or improving operating procedures;
- investing in energy efficiency, storage or alternative supply where the qualified domain model supports it.

An alternative should only increase the qualified option set when its physical capacity, access, dependencies, resources, staffing, safety and authority are evidenced.

## 5. Portfolio evaluation under deep uncertainty

Evaluate an investment portfolio against a frozen, operator-approved scenario family. Reuse existing systems:
- Symtropy for physical and resource-feasibility trajectories;
- Mycelix #5923/#5981 for declared dependencies and hidden common-cause evidence;
- Sol-Atlas #21 for spatial context and alternatives;
- Symthaea #10366 for hypotheses, value-of-information and diagnostic experiment selection;
- Sol-Atlas #338 for the canonical preregistered run manifest;
- Mycelix #2006 and #4816 for authority, study boundaries and multidimensional outcomes.

For each portfolio, report separately:

1. Total lifecycle cost: capital, operations, staff, training, spares, integration, audit, transition and exit costs.
2. Service outcome: service deficit, safe degraded-service duration, quality exposure, time to stable recovery and unknown duration.
3. Recovery options: options gained/lost, capacity, mobilization time and independence by failure domain.
4. Distribution: service effects by declared area/cohort; worst-affected lane and lower-tail continuity, not only the mean.
5. Robustness: expected, worst-case and tail outcomes under the declared scenario distribution or ambiguity set.
6. Regret: difference from the best portfolio after the scenario is known; report the evaluation oracle separately from deployable decision information.
7. Information value: whether collecting another observation would change portfolio selection enough to justify its cost, delay and risk.
8. Constraints: budget, staffing, water quality, safety, accessibility, affordability, environmental and rights floors.

Produce a Pareto set rather than one weighted "best national plan". The operator and legitimate authority choose among feasible portfolios. The model does not approve capital expenditure, procurement, tariffs, eligibility or physical operations.

## 6. Benchmark families

Extend the current option-preserving R1–R9 families with:

### R10 — Deferred-maintenance debt accumulation

Increase a known maintenance backlog or defer one critical repair. Test whether the projected debt view identifies the affected service and whether the qualified recovery-option set deteriorates under the declared fault.

### R11 — Competence and shift-coverage loss

Remove a qualified staff member, invalidate a credential or introduce a shift gap. A manual fallback requiring that capability must become unavailable or unknown.

### R12 — Supplier concentration and long lead times

Make apparent alternatives share an upstream supplier or impose a realistic lead-time/resource constraint. Verify that the option set and recovery trajectory do not assume unlimited procurement.

### R13 — Recovery investment lock-in

Add a backup that improves one nominal reliability indicator but creates a new common dependency, maintenance burden or supplier concentration. Check whether the portfolio evaluator exposes the transferred liability.

### R14 — Distributional resilience debt

Allow one service area/cohort to accumulate maintenance backlog or receive a less capable fallback while an aggregate service metric remains stable. Ensure the evaluation exposes the harmed cohort and does not average it away.

### R15 — Information acquisition before capital commitment

Compare acting immediately with acquiring a discriminating observation (for example, a flow-meter validation, laboratory test, asset inspection or supplier dependency confirmation). Measure whether the observation changes the feasible portfolio or reduces decision regret sufficiently to justify the cost.

## 7. Matched baselines

Compare the identical service/scenario/data boundary against:
- operator's existing maintenance and capital plan;
- the deployed EAM/APM and asset-criticality workflow;
- ordinary N-1 / redundancy and business-continuity analysis;
- best domain simulator and existing reliability-centred maintenance method;
- strong AI planning agent with identical data and action permissions;
- qualified multidisciplinary operator/expert panel;
- FSRE option-preserving portfolio evaluation;
- ablations without dependency qualification, workforce/resource feasibility, authority checks, spatial evidence, causal investigation or independent outcome verification.

Use equal scenario coverage, information, tuning time, compute budgets and human effort. If proprietary software cannot be independently run, state that limitation and do not infer superiority.

## 8. Proposed metrics and qualification gates

| Dimension | Measure | Initial gate / target |
|---|---|---|
| Debt classification | Precision/recall by debt family on planted synthetic cases | 100% exact reason-code agreement on deterministic reference fixtures |
| Unknown evidence | False-clear rate for missing/stale/disputed records | Zero false-clear results on frozen negative controls |
| Option preservation | False-independent recovery-option rate after common-cause, identity-duplication and supplier-concentration mutations | Zero false-independent options on frozen controls |
| Feasibility | Model out-of-domain / capacity / resource rejection | No unqualified physical result accepted as a viable option |
| Distribution | Service-deficit and recovery metrics by cohort/service area | No aggregate-only success claim; worst-affected lane published |
| Investment effect | Change in qualified option set and service trajectory per portfolio | Independently recomputed from frozen model/source/evaluator revisions |
| Economics | Total lifecycle cost per independently verified service-improvement unit | Compare against strongest incumbent plan with full cost and uncertainty interval |
| Decision value | Expected/worst/CVaR regret and value-of-information improvement | Preregistered comparison against matched human and heuristic portfolios |

These are software and study-design qualification targets, not field guarantees. Zero errors in a small fixture does not estimate production error probability. Operational thresholds and practical-effect margins require an operator-approved baseline and statistical power analysis.

## 9. Falsification

The research thesis should be narrowed or rejected if:
- the derived debt view fails to expose known recovery bottlenecks more accurately than the incumbent maintenance/continuity view;
- option qualification adds more cost and delay than the value of prevented false redundancy or unsafe recovery decisions;
- the best available incumbent platform already performs the complete cross-operator, independently verified workflow at lower lifecycle cost;
- data quality and unknown coverage make the portfolio comparison unusable;
- the result encourages underinvestment in one community to improve aggregate service statistics;
- the investment policy creates new concentration, lock-in or loss of institutional autonomy;
- model-generated improvements cannot be independently observed in the service delivered.

## 10. First implementation slice

Do not build a new resilience-debt database. Begin with a deterministic read-only report over synthetic fixture records:
1. define 12–20 synthetic debt items spanning the dimensions above;
2. bind each to existing owner identifiers and evidence references where available;
3. add positive, negative, stale, duplicate and unknown controls;
4. compute the before/after qualified recovery-option set for two or three small investment portfolios;
5. publish separate service, distribution, resource, authority and cost dimensions;
6. add a second independent reference calculation after the semantics stabilise;
7. only then adapt the report to public-data and operator-authorised source records.

The initial machine-readable example is [`ZA_RESILIENCE_DEBT_SYNTHETIC_V0_1.json`](scenarios/ZA_RESILIENCE_DEBT_SYNTHETIC_V0_1.json). It contains 3 synthetic service lanes, 14 synthetic debt items, 6 recovery-option candidates and 3 remediation portfolios. All baseline recovery candidates remain not qualified; the proposed remediation portfolio is explicitly marked `NOT_EVALUATED` until a separate post-remediation evidence bundle exists. An independent tool-side structural check confirmed JSON parseability, unique debt/option identifiers, allowed predicate states and the absence of any option promoted to qualified by identifier count alone. This is a fixture-structure check, not a Rust evaluator run or physical validation.

This static tranche does not authorise any operational actuation or capital/procurement decision.
