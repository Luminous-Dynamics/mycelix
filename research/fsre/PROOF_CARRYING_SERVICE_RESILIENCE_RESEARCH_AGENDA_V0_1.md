# FSRE proof-carrying service resilience: research agenda v0.1

**Date:** 2026-11-14  
**Status:** Research agenda and proposed evaluation protocol. Not a claim of field efficacy or state-of-the-art superiority.

## Executive thesis

FSRE should not attempt to become another universal infrastructure simulator, digital-twin platform, enterprise asset-management suite, GIS or restoration scheduler.

Its defensible research position is a **proof-carrying service-recovery and decision-assurance layer** that composes results from those systems and checks whether a proposed recovery decision is:

1. admissible from the evidence actually available at the declared decision time;
2. supported by the correct owner, scope, identity, time basis and provenance;
3. feasible under shared failure domains, resources, workforce, access and authority;
4. sufficient for every protected service cohort, not merely the aggregate;
5. reproducible from frozen inputs by a separate evaluator; and
6. bounded to the service, geography, scenario, horizon and assumptions actually evaluated.

The product is not a universal resilience score. It is a reproducible decision record containing qualified, unqualified and unknown claims, with scope and limitations intact.

## 1. Paradigm shifts

### 1.1 Infrastructure resilience → safe service resilience

An asset matters because of the service it makes possible. The outcome boundary should therefore be the service delivered to affected people and environments, not the count or nominal capacity of assets.

Every study must declare the service definition, geography, affected cohorts, quantity and quality floors, time interval, denominator, disturbance scenario, actual observations and missingness. For water, a volume delivered without qualifying water quality is not safe water service. A missing laboratory result is not a pass.

### 1.2 Nominal redundancy → portfolio survivability

An option is not independent because it has a different asset ID, vendor or address. It is independent only with respect to a declared set of common failure domains and constraints.

Evaluate shared electricity feeders, upstream suppliers, telecommunications, transport corridors, depots, specialist crews, spare parts, permits, control planes, authorities and spatial access. Then evaluate whether surviving options can operate simultaneously without exhausting the same resources.

The unit of qualification is the **portfolio under a declared disruption**, not an isolated backup asset.

### 1.3 Confidence scores → claim admissibility

A scalar confidence score cannot express whether a fact belongs to the right owner, service, cohort, denominator, period, clock, scenario or decision cutoff.

A claim envelope should carry at least:
- claim and service identifiers; protected cohort and geography;
- scenario, horizon, interval and denominator;
- source owner, opaque source reference and provenance;
- observation, availability, valid-time and revocation-time semantics;
- evidence snapshot, decision cutoff and clock basis;
- evaluator identity/version, input digests and replay command;
- disposition such as SUPPORTED, NOT_SUPPORTED, NOT_ESTIMABLE, NOT_RECONCILED or OUT_OF_SCOPE;
- uncertainty, known omissions, unresolved unknowns and non-claims.

No universal confidence number should override a missing owner, incompatible clock, absent denominator or non-admissible observation.

### 1.4 One-interval feasibility → time-expanded feasibility

A portfolio can be feasible in each isolated interval yet impossible over the horizon because inventory is depleted, replenishment is late, authority expires, crews become unavailable, roads close or a common dependency fails.

The time-expanded composer should join the existing per-interval portfolio evaluator, as-of evidence replay and stock-flow validator over a common time axis. It must preserve exact interval boundaries, units, resource identity, scenario, evidence snapshot and decision cutoff.

The initial composer is a read-only checker. It does not optimise allocations or schedule crews. Replenishment must not be credited before its arrival time is represented; unknown timing should remain unknown or use an explicitly conservative lower bound.

### 1.5 Mean recovery curves → service-outcome distributions

Keep each cohort's complete trajectory and show when its minimum safe service was first restored, sustainably restored and lost again. Report the worst cohort, cumulative deficit, longest continuous deficit, quantiles and tail outcomes alongside averages.

Do not allow a high aggregate recovery curve to hide a neighbourhood, hospital, school or vulnerable population below its floor. Averages are secondary to non-negotiable minimum-service constraints.

### 1.6 Static data collection → decision-aware investigation

The value of a measurement is the expected improvement in a decision, net of its acquisition cost, delay, operational risk and effect on the decision horizon.

Compare greedy one-measurement-at-a-time EVSI, finite-horizon adaptive measurement policies, an operator's normal information-gathering procedure, no-new-measurement and random-measurement baselines.

Measure decision regret, avoided unsafe decisions, time to decision, information cost and the rate at which new evidence changes the preferred feasible portfolio. More data is not automatically better.

### 1.7 Self-reported success → independent outcome reconstruction

The planning or qualification engine must not score its own success. A separate evaluator should reconstruct delivered quantity and quality from owner-authorised telemetry, laboratory results, service registers, operator logs, repairs, stock movements and authority records.

Planning inputs and outcome labels must be separated. Preserve the evidence lineage, missingness and corrections used by the outcome evaluator.

## 2. Architecture and ownership boundaries

FSRE is the composition and assurance layer, not the owner of all domain truth.

| System / actor | Owns | Must not be made responsible for |
|---|---|---|
| Mycelix | dependency and common-cause receipts, source ownership, identity/provenance and authority evidence | declaring physical asset capacity or granting operational authority because a claim exists |
| Sol-Atlas | geography, access, population boundaries, cohort identity and spatial comparability | physical plant behaviour or operational resource balances |
| Symtropy | physical feasibility, capacity, service process and environmental/quality models | authenticating source ownership or granting operational authority |
| Symthaea | investigation hypotheses, root-cause candidates, uncertainty decomposition and measurement selection | self-certifying hypotheses as source evidence |
| Operator / utility | workforce, resources, permits, safety procedures, control and real-world execution | retroactively rewriting evidence available at an earlier decision cutoff |
| FSRE | claim composition, portfolio qualification, temporal joins, service-floor protection and reproducible decision records | replacing owners' ledgers, simulators, scheduling systems or control rooms |

Interfaces should be narrow, versioned and receipt-based. Use adapters to translate incumbent outputs into declared claim envelopes. Never silently rewrite source data or grant one subsystem authority over another subsystem's domain.

## 3. Competitive landscape and where FSRE should fit

The comparative review by Shamsi, Helmrich and Deegan (version of record 7 November 2026) identifies 24 tools in the energy, water and transport scope, with seven explicitly supporting cross-sector infrastructure interdependency assessment. It finds an uneven capability landscape and no single tool that captures the breadth of interdependencies. This supports an interoperability and assurance thesis; it does not prove that FSRE outperforms those tools.

| Comparator | Established strength | FSRE's testable complementary contribution |
|---|---|---|
| IN-CORE | integrated hazard, infrastructure and socioeconomic/community recovery analysis | independently qualify specific service, cohort, authority and evidence claims made from its outputs |
| InfraRisk | modular hazard impact, cascading effects and recovery simulation | reconcile simulation outputs with owner-qualified operational feasibility and service floors |
| REFIT | probabilistic cascading-failure analysis and domain-model integration | qualify resulting recovery decisions, data admissibility and resource feasibility |
| CIrcle | participatory dependency and causal mapping | bind qualitative organisational knowledge to explicit claim scope and observed service outcomes |
| PRIIA | hypergraph-based restoration prioritisation | test whether the prioritised portfolio is simultaneously feasible and protects every cohort |
| Kaleidoscope | spatial vulnerability and resilience planning | preserve identity, denominator and service-level evidence across geography and time |
| CISIA-Pro | deterministic predefined fault propagation and scenario analysis | independently check owner, time, resource and service claims before decisions are treated as qualified |
| REWET | water-network restoration planning and simulation | compare FSRE's service/evidence qualification directly against physical network restoration plans on matched water-network cases |
| Palantir Foundry Ontology / AIP | shared operational ontology, connected data/models, actions and human/AI workflows | publish machine-verifiable qualification receipts consumable by operational workflows without taking control of them |
| Bentley iTwin | infrastructure digital-twin data integration, visualisation and lifecycle engineering | qualify claims made from twin outputs; do not recreate the twin platform |
| IBM Maximo | enterprise asset management, maintenance, inventory, work orders and asset-performance workflows | qualify cross-asset, cross-organisation service recovery plans and evidence lineage across incumbent systems |

The 2026 review reports that comparable runtime and hardware benchmarks were unavailable across the seven interdependency tools. FSRE must therefore publish hardware, data volume, runtime, memory, tuning effort, solver budget and outcome methodology before making any performance claim. Absence of a comparable benchmark is not evidence that FSRE is faster or more accurate.

NIST SP 800-160 Vol. 2 Rev. 1 already recognises cyber-resiliency engineering as a systems-engineering discipline used alongside systems security and resilience engineering. FSRE should build on that work and established resilience engineering; it must not claim to have invented the discipline.

## 4. Benchmark programme

### 4.1 Comparator ladder

Use matched scenarios and identical permissions, source data, evidence cutoff, budgets and outcome measurements:

- **B0 — operational baseline:** current utility runbook, operator and coordination procedure.
- **B1 — nominal redundancy:** N-1 counts and simple capacity summation.
- **B2 — dependency baseline:** common-cause/cut-set analysis without cohort service composition.
- **B3 — scheduling baseline:** resource-constrained project scheduling (RCPSP) and deterministic MILP.
- **B4 — sector-specific baseline:** strongest applicable physical restoration model, including REWET for water-distribution restoration.
- **B5 — uncertainty-aware baseline:** robust or distributionally robust multi-period optimisation where data and assumptions support it.
- **B6 — enterprise baseline:** incumbent EAM/APM, GIS, inventory, work-management and operational workflows.
- **B7 — expert baseline:** qualified human operator/engineer panel.
- **B8 — AI baseline:** strong planning agent under identical read-only permissions, context, tools and time budget.
- **B9 — FSRE:** full composition, plus controlled ablations of each assurance layer.

Do not treat unavailable comparator features as losses. Document each system's intended task, supported inputs, outputs, assumptions and unsupported functions.

### 4.2 Primary operational endpoints

Select one primary endpoint before holdout access:

1. decision regret against the best feasible action under the realised scenario and the same authorised action set;
2. worst-cohort cumulative deficit against pre-registered safe-service floors; or
3. lifecycle cost per independently verified unit of safe service improvement.

Secondary measures: safe service-hours, time to first and stable floor restoration, longest continuous below-floor interval, unsafe-quality exposure, displaced/unserved population, surviving options per shock, common-cause concentration, stockouts, crew contention, authority continuity, access failure, operating cost, analyst time, abstention rate and evidence-acquisition cost.

Report full service trajectories, worst case and pre-registered quantiles/tail metrics. Averages cannot compensate for a protected cohort's floor violation.

### 4.3 Safety and statistical targets

Proposed engineering targets—not achieved results:

| Measure | Research target / acceptance rule |
|---|---|
| Frozen deterministic evaluator cases | 100% agreement with the declared answer set |
| Known-infeasible negative controls | zero false-qualified results observed; publish numerator, denominator and uncertainty |
| False-safe operational classification | one-sided 95% upper confidence bound below 1% before limited decision-support deployment; under independent Bernoulli assumptions this requires at least 299 zero-failure decision episodes |
| False-independent common-cause claims | zero observed in blinded common-cause negatives, with exact and cluster-aware bounds |
| Cohort-floor false positives | zero observed in independently adjudicated floor-miss cases |
| Unauthorized consequential actions | zero; test harness must have no operational write credentials or dispatch path |
| Evidence lineage | 100% of qualified claims have owner, source reference, scope, snapshot, cutoff, clock and evaluator lineage |
| Identity-link precision | at least 99% on a blinded, adjudicated holdout; publish coverage, recall and subgroup performance |
| Nominal 90% uncertainty intervals | 85–95% empirical coverage on sufficiently powered, distribution-matched holdouts |
| Forecast calibration | target at least 10% relative Brier improvement over the strongest matched baseline, only where probabilistic forecasts are in scope |
| Root-cause localisation | precision@5 and recall@10 targets of at least 0.90, only on independently adjudicated fault sets with explicit candidate universes |
| Read-only partition tolerance | 24-hour operation without any new consequential mutation; unknown outcomes require reconciliation before writes resume |

The public 50-case recovery challenge remains a regression suite. Zero failures across 50 independent negative controls still gives a one-sided 95% exact binomial upper bound of about 5.82%. It does not establish a sub-1% failure rate. The 299-episode illustration assumes independence; correlated variants from one hazard, utility or scenario generator require cluster-aware analysis and may need substantially more cases.

Report false-safe rate and evidence coverage together. A system cannot improve apparent safety by abstaining on all difficult cases, nor improve apparent coverage by promoting unknown evidence.

### 4.4 Required challenge families

Create public regression, blinded independent qualification and prospective read-only shadow sets. Reserve at least one complete hazard family, geography, utility or operator period as a holdout.

Include:
- shared feeder, fuel supplier, telecom, access road and control-plane failure;
- crew skill, shift overlap, simultaneous mobilisation, fatigue and rest constraints;
- inventory depletion, delayed/non-arrived replenishment, loss, expiry and unit mismatch;
- permit and authority expiry/revocation during the horizon;
- spatial closures, travel windows and isolated cohorts;
- water-energy-telecom cascades and common upstream suppliers;
- quantity restored but quality still unsafe or subsequently relapses;
- aggregate service recovered while one protected cohort remains below floor;
- cohort and asset identity merges, splits and boundary changes;
- delayed, future, stale, revoked and contradictory observations;
- incomplete ownership, unknown capacity and missing denominators;
- repair crew, spare parts and equipment contention across options.

The independent reviewer must author a disjoint corpus and adjudicate expected outcomes without tuning evaluators against it. Keep the answer key sealed until the implementation freeze.

## 5. Research tracks and engineering sequence

| Track | Deliverable | Exit condition |
|---|---|---|
| R0 — exact-head qualification | repair formatting, Clippy, semantic golden and focused FSRE failures without weakening gates | every required focused job reports a conclusion on the same exact SHA; skipped generic CI remains explicitly unqualified |
| R1 — time-expanded evidence composition | bind each interval's portfolio to the exact as-of evidence snapshot and stock-flow receipts | deterministic reference, byte-for-byte CLI output, negative controls and passing exact-head workflow |
| R2 — independent qualification | disjoint reviewer-authored temporal and common-cause challenge corpus | no tuning on sealed cases; signed adjudication record and uncertainty bounds |
| R3 — outcome reconstruction | separate cohort-level quantity and quality trajectory evaluator | independent source lineage, denominator, missingness and complete trajectories |
| R4 — adaptive investigation | finite-horizon measurement-acquisition benchmark versus greedy EVSI and operator baseline | paired decision-regret, delay, cost and safety analysis |
| R5 — matched operational evaluation | compare with runbooks, RCPSP/MILP, REWET, relevant interdependency tools and incumbent enterprise workflows | pre-registered primary endpoint, matched inputs, hardware/runtime disclosure and independent analysis |
| R6 — read-only shadow deployment | utility-approved historical/prospective replay with read-only credentials | local safety case, operator review, audit and rollback; no autonomous dispatch |

The R0 and R1 tracks can proceed in parallel. Do not make R1 qualification dependent on unrelated repository-wide runners, but do not call it passed until its own exact-head CI concludes.

## 6. South African water research boundary

Green Drop is a strong candidate proving ground because the decision problem is about safe sanitation service, municipal capability, shared constraints and outcomes—not simply plant counts.

The first historical study must establish whether source records permit lawful, comparable, system-level longitudinal analysis. The 2022 and 2025 audit populations must not be treated as a fixed cohort until identity, ownership and boundary changes are reconciled. National percentages are context, not a system-level operational baseline. Rights-blocked or unlicensed sources must not be silently ingested into a production pipeline. Obtain owner-authorised machine-readable records and independent review before prospective use.

For wastewater, define the service outcome at an appropriate receiving-environment and population boundary; treatment quantity alone is not proof of safe treatment. For potable water, delivery volume alone is not proof of potable quality. Keep lab-confirmed safety measures separate from modelled capacity.

## 7. Explicit non-claims

This agenda does not claim:
- FSRE is faster, more accurate or safer than any named tool;
- a passing public synthetic fixture proves real-world validity;
- a portfolio feasibility report proves a service was delivered;
- an as-of replay authenticates a source or establishes physical truth;
- a common-cause graph proves independence outside its declared scope;
- a digital twin is an operational authority;
- any benchmark target has already been achieved.

Every external comparison must be run on a matched task with declared data, permissions, budgets and independently reconstructed outcomes.

## References and comparator documentation

- Shamsi, N., Helmrich, A. & Deegan, M. (2026-11-07), *Toward system-of-systems resilience: a comparative framework for infrastructure interdependency assessment tools*, Environment Systems and Decisions. https://doi.org/10.1007/s10669-026-10162-2
- Naeimi, S. & Davidson, R. A. (2025), *Improving Restoration of Water Supply Service Following Disruptions*, Journal of Water Resources Planning and Management 151(7). https://doi.org/10.1061/JWRMD5.WRENG-6452
- Poulin, C. & Kane, M. B. (2021), *Infrastructure resilience curves: Performance measures and summary metrics*, Reliability Engineering & System Safety 216, 107926. https://doi.org/10.1016/j.ress.2021.107926
- NIST SP 800-160 Vol. 2 Rev. 1 (2021), *Developing Cyber-Resilient Systems: A Systems Security Engineering Approach*. https://csrc.nist.gov/pubs/sp/800/160/v2/r1/final
- Johnson, K., Leandro, J. & Ahadzie, P. (2024), *Quantifying hazard resilience by modeling infrastructure recovery as a resource-constrained project scheduling problem*, Natural Hazards and Earth System Sciences 24, 2285–2302. https://nhess.copernicus.org/articles/24/2285/2024/
- Palantir Foundry Ontology documentation: https://www.palantir.com/docs/foundry/ontology/overview/
- Bentley iTwin Platform documentation: https://developer.bentley.com/
- IBM Maximo Application Suite documentation: https://www.ibm.com/products/maximo
