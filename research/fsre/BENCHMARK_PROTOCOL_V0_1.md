# FSRE Benchmark Protocol v0.1

**Status:** Proposed protocol; targets below are research targets, not achieved results.  
**Version:** 0.1.0  
**Date:** 2026-11-13

## Parent programme alignment

This protocol is a qualification profile for existing Mycelix programmes, not a replacement benchmark for them:

- [CIV-RES-000 #2006](https://github.com/Luminous-Dynamics/mycelix/issues/2006) owns civic resilience composition, the Johannesburg profile, privacy planes, civic study manifests, effect measurement and human-authority boundaries.
- [CIV-ECON-006 #4816](https://github.com/Luminous-Dynamics/mycelix/issues/4816) owns the Civilization Twin's multi-scale, multi-time-horizon physical/economic/institutional/ecological research and multidimensional outcome vector. It explicitly rejects a universal civilization scalar.
- [Civic automation portfolio #5015](https://github.com/Luminous-Dynamics/mycelix/issues/5015) owns the opportunity/ownership matrix and ranked Johannesburg service-reliability portfolio, including water/sanitation and paid-work pathways.

FSRE benchmark outputs must therefore remain conditional, service-specific and multidimensional. For civic interventions, separately report direct effects, spillover effects, displacement effects and measurement/reporting effects as required by CIV-RES-000. A fall in reported incidents, a rise in ticket closures, or a better aggregate service number is not automatically evidence of improved lived outcomes.

## 1. Benchmark objective

Measure whether the FSRE architecture improves essential-service resilience and accountable intervention compared with existing workflows and strong individual components.

Do not benchmark "national intelligence" as one opaque score. Decompose the system into claims that can be falsified and independently reproduced.

The primary outcome metric is:

**Total cost per independently verified unit of service improvement within one declared service lane**, subject to hard safety, rights, privacy and equity constraints. This is not a universal societal score and must not be aggregated across unrelated domains.

Examples of service units include reliable household water-hours, wastewater treatment compliance-hours, electricity served during constrained periods, or emergency-response time saved. The selected unit and quality boundary must be pre-registered for each study.

## 2. Comparator landscape

Use existing standards and platforms as interoperability foundations and comparators, not straw men.

| Comparator | What it measures or provides | FSRE comparison question |
|---|---|---|
| World Bank GovTech Maturity Index (GTMI 2025) | 48 indicators across core government systems, online services, citizen engagement and GovTech enablers; covers 197 economies | Does FSRE improve operational capability beyond maturity and adoption indicators? |
| OECD Digital Government Outlook 2026 | Digital public infrastructure, data governance, interoperability and AI-in-government adoption/impact assessment | Can FSRE turn interoperability and evaluation into an executable service loop? |
| GovStack | Modular, interoperable digital-government building blocks | Can FSRE interoperate with existing building blocks without reimplementing them? |
| X-Road | Secure data exchange between organisations | Does FSRE add evidence qualification, intervention authority and outcome reconciliation above exchange? |
| Open Contracting Data Standard | Structured procurement data from planning through implementation; reported adoption in 50+ governments | Can FSRE connect public-contract records to independently observed delivery and service outcomes? |
| Industrial digital twins and domain simulators | Equipment/process modelling and optimisation | Does cross-domain dependency and authority context improve intervention decisions versus domain-only simulation? |
| Enterprise AI agents | Model/agent performance on task and workflow benchmarks | Do evidence gates reduce harmful actions and increase verified task completion under equal constraints? |
| Human expert teams and existing runbooks | Existing operational baseline | Does FSRE produce incremental value over the actual workflow, not only over a weak demo? |

References:
- [World Bank GTMI 2025](https://www.worldbank.org/en/programs/govtech/gtmi-2025-update)
- [OECD Digital Government Outlook 2026](https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/strengthening-digital-public-infrastructure-and-data-governance_2c7323c7.html)
- [GovStack](https://govstack.global/)
- [X-Road](https://x-road.global/)
- [Open Contracting Data Standard](https://www.open-contracting.org/data-standard/)

The GTMI is explicitly a maturity snapshot, not a country-performance ranking. FSRE results must not be presented as a replacement GTMI score.

## 2.1 Dedicated infrastructure interdependency tool comparators

A newly published open-access comparative review (Shamsi, Helmrich & Deegan, 7 November 2026) identified 24 infrastructure tools across energy, water and transport; seven explicitly model cross-sector interdependencies and cascading effects. Include these as method-specific comparators where licensing/access and task fit permit:

| Tool | Method family | FSRE comparison lane |
|---|---|---|
| IN-CORE | Integrated hazard, infrastructure-network, fragility and socioeconomic recovery environment | Community-scale service/recovery and socioeconomic outcome coupling |
| InfraRisk | Modular coupled-infrastructure simulation for power, water, transport, hazard, cascade and restoration | Compound disruption and recovery trajectory |
| REFIT | Probabilistic cascading-failure simulation with domain-solver coupling | Probabilistic cascade, uncertainty and investment portfolio scenarios |
| CIrcle | Participatory workshops, expert input and causal mapping | Institutional/organisational dependency elicitation and qualitative pathway coverage |
| PRIIA | Hypergraph dependency and optimisation-based restoration prioritisation | Post-disruption prioritisation and restoration plan comparison |
| Kaleidoscope | Spatial predictive analytics and asset co-location/vulnerability analysis | Spatial exposure, co-location and investment prioritisation |
| CISIA-Pro | MATLAB-based deterministic fault-propagation simulator | Explicit deterministic cascade semantics and critical-component identification |

This list complements enterprise-product comparators (Palantir, Maximo, iTwin, EcoStruxure) rather than replacing them.

The current public reference implementation now includes a 30-test recovery-option evaluator and a separately authored 4-test differential reference evaluator. The production evaluator's negative-control matrix exercises all 7 required predicates against 8 non-qualified dispositions (56 combinations), then tests missing, stale and wrong-owner evidence for every predicate. The independent reference consumes the same frozen scenario and evidence inputs but does not import the production crate. On the qualified synthetic revision, both implementations agreed on all 7 option dispositions. This is deterministic conformance on one public fixture, not proof of independent evidence authenticity, specification correctness, live owner integration or field reliability. The methods are not interchangeable: some are participatory, some spatial/data-driven, some deterministic or probabilistic simulation, and some integrated system-of-systems environments. Do not force every tool through one synthetic input schema or score a method outside its declared purpose.

The 2026 review reports no single tool dominates all assessed dimensions and that comparable runtime/hardware benchmark data were unavailable across the seven. This is a reason to build a shared service-level task/evidence/metric contract with tool-specific adapters, not to claim FSRE already outperforms these systems. Record tool/version, access constraints, calibration, source-data needs, method family, validation level, compute, tuning and analyst effort.

Primary reference: https://doi.org/10.1007/s10669-026-10162-2

## 2.2 Wastewater service-chain completeness and flow-balance lane

The South African wastewater benchmark must not treat treatment-works performance as a proxy for the full sanitation service. Its declared boundary runs from wastewater generation and sewer collection through conveyance, treatment, discharge/reuse and the receiving environment. The first executable source replay is limited to a published DWS national-summary release; it does not extract system-level rows or infer operational conditions.

The separate `tools/fsre-flow-balance` reference evaluates the six required boundary terms (`generation`, `other_inflows`, `treated`, `bypass`, `reuse`, `storage_change`) and derives an interval for the unaccounted residual. It refuses to calculate that residual when a required term is unknown; `NOT_APPLICABLE` must be an explicit boundary-scoped `[0,0]` claim. Positive residuals are not attributed to a leak, overflow, illegal discharge, pollution, theft or responsible party. Negative residuals are preserved as accounting mismatch rather than clipped.

| Measure | Proposed target | Hard interpretation rule |
|---|---:|---|
| Required-term lineage and shared scope/time/unit binding | 100% | No silently omitted term or mismatched period/geography/time basis |
| Unknown/zero/not-applicable distinction | 100% on sealed controls | Unknown must never be converted to zero |
| Interval arithmetic against independently computed reference cases | 100% deterministic agreement | Include sign-crossing, negative and wide intervals |
| Nominal 90% uncertainty-interval coverage | 85–95% on held-out observed periods | Coverage only after the uncertainty model is calibrated and pre-registered |
| False causal attribution in sealed negative controls | 0 | Any false attribution blocks use for consequential claims |
| Residual localisation precision/recall | No target until independent system/catchment ground truth exists | Report abstention and spatial resolution; do not invent a baseline |
| Unauthorised data/control mutation | 0 | Read-only shadow study first; no actuation path in this reference |

### Matched sewer sensor-placement comparators

Do not treat sensor placement or graph localisation as an unexplored contribution. A 2026 *Water Research* study proposes topology-derived flow inference (TDFI) for pipe-specific minimum detectable inflow/infiltration and deterministic Sequential Backward Selection (SBS) for sensor layout; it evaluates one synthetic and two real sewer networks. A 2025 *Water Research* study reports 7–15% higher I&I detection reliability using entropy-based sensor placement on its evaluated networks. A 2026 *Water Research* LSTM–GCN sewer-localisation study reports top-two source ranking above 90% on single-source synthetic events and top-one localisation on its 39-junction real-world sewer subnetwork. These published numbers belong to different task configurations and are not directly comparable to each other or to FSRE outcomes.

Where reproducible, include TDFI+SBS, entropy placement, LSTM–GCN, current operator layouts, and uniform/random placement on identical topology, sensor budget, time window, event catalogue, noise, missingness and ground truth. Report false alarms per sensor-day and network length/time, minimum detectable I&I, detection delay, top-1/top-2/top-k branch ranking, network-distance localisation error, calibration/abstention, sensor lifecycle cost and SCADA/GIS integration burden. Compare FSRE decision value and verified service/environment outcomes as an additional layer; do not claim that the service qualification profile replaces specialist sensor-placement or sewer-localisation methods.

A 2026 PRISMA systematic review of 90 ML/DL water-distribution leak-detection studies reports major variation in datasets, scenario definitions, train/test splits, metrics and validation; only 7 studies used cross-validation, 9 temporal/independent holdouts, and 42 did not report a reconstructible validation procedure. This is adjacent evidence from drinking-water distribution networks, not wastewater sewer I&I, but supports strict spatial/temporal holdouts, sealed event sets and a minimum reporting contract. Ref: Farah & Shahrour, *Water* 18(18), 2267 (2026), https://doi.org/10.3390/w18182267.

For sewer benchmarks, random-row splits are insufficient. Require held-out catchment/WSA topologies and held-out operating periods, distinguish unseen-event from unseen-network transfer, and report leave-one-catchment-out / leave-one-WSA-out performance. Stratify single-source vs multi-source events, wet vs dry weather, normal vs degraded instrumentation, known vs novel sensor placement, and in-domain vs out-of-domain hydraulic regimes. Freeze the target network and challenge cases before tuning. Report calibration and abstention as well as top-k localisation; publish the per-network results and the worst supported network rather than only a pooled mean. No universal accuracy target is meaningful until the event definition, label quality, sensor budget and minimum detectable flow are fixed.

The six-term balance is a transparent accounting surface, not a hydraulic solver or full wastewater model. The current implementation uses conservative interval arithmetic and preserves correlation-group labels but does not model joint dependence. A broad interval may be the correct answer. Symtropy remains the owner for physical models, calibration and uncertainty propagation; this consumer reference does not replace that work.

The national-summary replay checks only arithmetic and source-reported boundaries. It verifies 2022 rates against the separately sourced 850-system denominator and 2025 rates against the 848-system denominator, identifies the respective 2020/21 and 2023/24 report periods, and flags the reported decommissioning/merging boundary change. It does not turn national system percentages into household-level outcomes or claim system identity linkage. System-level historical replay requires source rights, extracted source rows and dual transcription/reconciliation; prospective work requires operator-authorised read-only access.

## 2.3 Decision-value and active-investigation benchmark

FSRE's investigation question is not only whether a model can identify a likely fault, but whether it can choose the next authorised observation that most improves a safe decision under a matched acquisition budget. The dedicated protocol is [Decision Value and Active Investigation v0.1](DECISION_VALUE_AND_ACTIVE_INVESTIGATION_V0_1.md), aligned with Symthaea causal diagnosis and experiment selection [#10366](https://github.com/Luminous-Dynamics/symthaea/issues/10366).

The benchmark distinguishes information gain from decision value. EVSI measures the expected change in the best **admissible** decision after a candidate observation, minus acquisition and delay cost. Physical feasibility, rights, privacy, operator authority and safety are hard filters, not soft penalties. The policy must be able to choose no measurement, no intervention, or abstention.

Matched-cost baselines include existing operator practice, fixed-cadence monitoring, random admissible sampling, maximum-entropy sampling, maximum-prediction-uncertainty sampling, incident-risk ranking, expert prioritisation, and a synthetic-only information oracle. All policies receive the same initial evidence, candidate measurement catalogue, cost/delay model, authority limits and compute budget.

The Rust reference now ranks candidate measurements from one frozen snapshot and marks only the **single highest strictly positive net-EVSI** candidate as the next acquisition. Candidate EVSI values are not additive: measurements can be redundant and their expected benefits can overlap. After an authorised, safe measurement is actually observed, the reference conditions hypothesis weights from the owner-qualified outcome and re-ranks the next candidate set. This is a greedy one-step policy, not a claim of globally optimal multi-step sensing; compare it with finite-horizon dynamic programming or search before making a sequential-optimality claim.

Primary measures include decision regret, correction of initially wrong decisions, verified decision improvement per total acquisition cost, unnecessary acquisition rate, time-to-decision-quality evidence, safe-abstention precision/coverage, calibration, false attribution/localisation, option preservation and distribution of measurement burden/service benefits. The first target is at least 50 independently specified synthetic challenge cases with held-out sealed cases; confirmatory set size must be powered for the claimed failure bound.

The scientific motivation is supported by 2026 water-resources work on decision-relevant information selection (Zaniolo et al., DOI 10.1029/2025WR042745) and a 2026 systematic review documenting inconsistent water-distribution leak-detection benchmarks (Farah & Shahrour, DOI 10.3390/w18182267). Those findings motivate better task definitions; they do not establish FSRE superiority.

## 2.4 Temporal identity continuity and boundary-aware comparison

The unit of a longitudinal resilience comparison is a time-qualified **entity population**, not a label or national total. Green Drop demonstrates why: WRO describes a 2022 all-sector audit of 995 works; SAnews reports an 850-system municipal comparison denominator for 2022; the 2025 full audit covers 848 municipal systems across 144 WSAs. These are different population scopes, not interchangeable denominators. The reported 850-to-848 change is attributed to decommissioning or merging, but no individual system continuity is inferable from the aggregate counts.

The new [Temporal Identity Continuity and Crosswalk Benchmark v0.1](IDENTITY_CROSSWALK_BENCHMARK_V0_1.md) defines explicit continuity, rename, transfer, merge, split, decommission, creation and unresolved events. The dependency-free Rust reference at `tools/fsre-identity-crosswalk` binds both endpoints to population scope and owner class, validates transition cardinality, event scope/date/evidence consistency, conflicting same-date assignments and evidence/review-state promotion, and reports scope/owner-class changes separately. Its fixture is synthetic and deliberately contains no real system identifiers. The key invariant is that identity continuity does not imply population-cohort comparability.

An external comparator now exists in Unmapped Dataworks UM-006: its 12 November 2026 metadata release reports 3,103 records across Green Drop 2024, Blue Drop 2023/2024 and No Drop 2025, and identifies Demarcation Board codes on municipality records. Its documentation says cross-assessment system links remain future work. The dataset is early-access, API/key access is constrained, and its terms state that underlying DWS facts remain subject to source-owner rights. FSRE records this as a metadata-only comparator lead; no records are copied, downloaded or republished.

Future matched baselines should include source-native stable identifiers, deterministic owner-supplied crosswalks, exact-code joins, manual MDM workflows, string-only matching, fuzzy name/geography matching, temporal graph/event resolution, and expert adjudication. Each method receives the same rights-cleared records, auxiliary geography, report-vintage boundary, adjudication budget and hardware. Split evaluation by WSA and report cycle to prevent near-duplicate leakage.

Metrics include pairwise precision/recall by event type, false-continuity rate, exact merge/split event accuracy, unresolved/abstention precision and coverage, boundary-scope error (municipal versus all-sector), effective-date accuracy, provenance completeness, reviewer agreement, subgroup performance by ownership class/WSA/province, manual adjudication effort and cost per accepted transition. A false merge can be more harmful than an unresolved row; do not collapse all relation types into one F1 score.

Prospective research gates: 100% cardinality conformance on sealed synthetic cases; zero synthetic/unresolved/disputed promotion; 100% of source rows carry an explicit population scope and assessment cycle; and at least 99% precision among accepted real identity links before making consequential longitudinal claims, with coverage published beside precision. The 99% figure is a proposed release target, not a measured result or confidence bound. Real-system matching remains blocked until lawful system-level records and independently reviewed labels exist.

## 3. Baselines and ablations

Every evaluation should compare the same frozen inputs and scenario set against at least:

- **B0 — Existing workflow:** actual operator runbook, tools and staff process.
- **B1 — Data integration only:** existing APIs/data-exchange stack without FSRE reasoning or authority lifecycle.
- **B2 — Domain model only:** best available domain-specific simulator or digital twin.
- **B3 — General AI assistant/agent:** strong available model with equivalent tools and access but without the complete FSRE control/evidence architecture.
- **B4 — Human expert:** appropriately qualified operators or analysts working with the same information.
- **B5 — FSRE complete:** all qualified layers connected.
- **Ablations:** FSRE without Symthaea reasoning, without dependency graph, without spatial context, without Mycelix authority binding, and without independent verification.

If a comparator cannot be run under the same conditions, report the missing comparison. Do not infer superiority.

## Existing Luminous benchmark owners — mandatory reuse

This programme must not fork already-defined benchmark infrastructure:

- **Pre-registered run manifest and execution receipt:** use Sol-Atlas [IBAP-RUN-001 #338](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/338) as the canonical owner for frozen task/data/split/model/resource/metric identities, execution receipts and independent evaluations. FSRE adds the societal-service estimand and operator-defined outcome contract as a domain profile, not a competing manifest format.
- **Dependency graph shape and temporal semantics:** use Mycelix [#5923](https://github.com/Luminous-Dynamics/mycelix/issues/5923), [#6005](https://github.com/Luminous-Dynamics/mycelix/issues/6005) and [#6013](https://github.com/Luminous-Dynamics/mycelix/issues/6013). The synthetic water fixture in this PR is a service-level consumer case; it is not a replacement for the canonical hypergraph evaluator or its Rust/Go reference verifiers.
- **Causal discovery beyond declared inventory:** use Mycelix [#5981](https://github.com/Luminous-Dynamics/mycelix/issues/5981) for hidden common-cause discovery via controlled interventions. Graph closure and causal discovery are distinct benchmark lanes.
- **Physical/social model adequacy and runtime comparison:** use Symtropy [CIVDYN-BENCH-QUAL-001 #4392](https://github.com/Luminous-Dynamics/symtropy/issues/4392) and [CIVDYN-ENGINE-COMPARE-001 #3787](https://github.com/Luminous-Dynamics/symtropy/issues/3787). A runtime win is valid only after matched behavioural semantics and model-adequacy gates.
- **Causal diagnosis and experiment selection:** use Symthaea [CEB-CAUSE-001 #10366](https://github.com/Luminous-Dynamics/symthaea/issues/10366) and [REA-RUST-008 #9911](https://github.com/Luminous-Dynamics/symthaea/issues/9911). FSRE evaluates downstream service value and operator safety, not a second causal engine.
- **Spatial resilience and alternatives:** use Sol-Atlas [SOL-ATLAS-004 #21](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/21) with its existing provenance/validation owners. A graph alternative is not an authorised or physically capable substitute until a domain authority and model establish that.

This mapping makes FSRE an integration benchmark over existing owners. Existing benchmark results are not automatically FSRE results, and FSRE does not inherit any upstream component's performance or maturity claim.

## 4. Test tiers

### T0 — Contract correctness

Test schemas, version negotiation, signature scopes, freshness, source revocation, invalid units, malformed geometry, stale observations, missing data and contradictory evidence.

Hard acceptance conditions:

- 100% rejection of expired, revoked, wrong-scope or unauthorised grants in the declared conformance suite.
- 100% rejection of malformed, stale or digest-mismatched evidence where policy requires rejection.
- 0 unauthorised mutations accepted in the declared red-team suite.
- 0 duplicate effective mutations in retry, timeout and replay tests.
- 100% of consequential mutations link to the exact proposal, authority grant, preconditions and execution receipt.
- Replays of deterministic reference scenarios produce identical semantic results for identical versions, inputs and seeds.

A single hard-gate violation fails the release candidate. Do not average safety failures into a weighted score.

### T1 — Structural dependency and cascade tests

Use synthetic graphs with known ground truth, then expand to public inventories.

Measure:

- Dependency completeness and provenance coverage.
- Single-point-of-failure recall.
- Common-cause exposure recall.
- Minimal cut-set recovery on tractable graph sizes.
- Critical-service impact classification.
- Fault localisation precision@k and recall@k.
- False-positive and false-negative rates.
- Runtime, memory and scaling by nodes, relations and hyperedges.

Provisional targets:
- 100% detection of declared single-point and common-cause failures in exact synthetic fixtures.
- At least 0.90 precision@5 and 0.90 recall@10 for root-cause localisation on blind, labelled fault-injection cases.
- Report exact graph size, density, hyperedge cardinality, hardware, toolchain and all seeds.
- No claim of physical accuracy from a structural graph alone.

### T2 — Forecasting and counterfactual tests

Evaluate every forecast against held-out data and strong domain baselines.

Measures:
- Brier score and log score for probabilistic events.
- Calibration error and reliability diagrams.
- 50%, 80% and 90% interval coverage and sharpness.
- Decision regret relative to an oracle using realised outcomes (reported as an evaluation bound, not an achievable runtime baseline).
- Sensitivity to missing, stale and adversarial observations.
- Performance under distribution shift and previously unseen combinations of events.

Provisional targets:
- At least 10% relative improvement in Brier score over the strongest calibrated baseline on a pre-registered holdout, without worse calibration.
- Nominal 90% prediction intervals should achieve 88–92% empirical coverage on sufficiently large holdouts; report subgroup and event-class coverage separately.
- Out-of-domain cases must produce an explicit abstention or qualified fallback, not an unqualified point estimate.

### T3 — Operational replay and shadow mode

Replay historical incidents with time-ordered data. Then run live read-only shadow mode with the operator.

Measures:
- Time to detect, diagnose, prioritise and issue a reviewable proposal.
- Operator acceptance, modification and rejection rates.
- False alarms per service-month.
- Missed critical events.
- Dispatch and restoration time.
- Evidence completeness and time to independent audit.
- Total analyst and operator hours.
- Cost per verified service improvement.

Targets should be agreed with the operator after a baseline period. Initial pilot hypotheses may include a 15% reduction in median detection-to-reviewed-proposal time, but only if the comparison is paired, the workload is comparable and alert burden does not increase materially.

### T4 — Supervised intervention

Only reversible, low-risk, operator-authorised work should enter the first live pilot. No automated water shutoff, eligibility decision, procurement award, emergency declaration or other high-impact act.

Measures:
- Actual service availability and quality.
- Repair and maintenance time.
- Energy and material use.
- Direct and lifecycle cost.
- Repeat incidents.
- Distributional impact by service area and population group, using privacy-preserving aggregation.
- Complaints, appeals and adverse events.

A pilot succeeds only if it improves the pre-registered service outcome and does not materially worsen safety, water quality, affordability, accessibility or distributional equity.

### T5 — Federated and cross-jurisdiction tests

Test three or more independently administered nodes.

Scenarios:
- Network partitions and late-arriving evidence.
- Revoked credentials and expired delegation.
- Divergent local policies.
- Conflicting asset records.
- Model/provider outage.
- Loss of one region's compute or storage.
- Schema version skew.
- Export and re-hosting by a second operator.

Measure local continuity, recovery time, data exposure, synchronisation correctness, unresolved conflict handling and the proportion of services that remain operable without a central coordinator.

## 5. South African water pilot benchmark

### Evidence context

South Africa's 2025 Green Drop assessment reports 396 of 848 municipal wastewater treatment systems (47%) in a critical state, compared with 334 (39%) in 2022; the assessed systems relate to the 2023/24 financial year. Separately, the 2025 No Drop Progress Report was released on 31 March 2026. Parliament's 9 June 2026 briefing reported national non-revenue water at 47.3%, compared with 47.4% in the 2023 Full No Drop Report. Green Drop wastewater-treatment performance and No Drop drinking-water distribution efficiency are different indicators and must remain separate. Treat the 47.3% figure as a dated national progress-report value, not a current Johannesburg baseline or a measure of physical leakage alone.

References:
- https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- https://www.gov.za/news/media-advisories/government-activities/minister-pemmy-majodina-releases-green-drop-report-0
- https://www.parliament.gov.za/press-releases/media-statement-water-committee-warns-of-deepening-crisis-as-municipalities-fail-implement-corrective-water-plans
- https://www.gov.za/news/media-statements/water-and-sanitation-releases-2023-full-blue-drop-report-05-dec-2023

A September 2025 Department of Water and Sanitation eThekwini Water Security Dashboard reports a 12-month average non-revenue-water figure of 58.8% and cites 25% as a target. This is a dated, municipality-specific dashboard observation, not a national current value or a verified present-day baseline.

Reference: https://www.dws.gov.za/documents/2025-09%20PWSeT%20dashboard.pdf

### Pilot boundary

Start with one willing water-services authority and one defined service area. Public reports can support initial research, but operational deployment requires utility-approved asset data, water-balance data, work-order history, service interruptions, quality measurements and data-sharing arrangements.

The synthetic fixture in this directory is for structural cascade tests only. It is not a model of eThekwini or any other municipality.

### Service metrics

- Water balance: system input volume, authorised consumption, apparent losses and real losses.
- Reliability: service hours, outage duration, pressure and restoration time.
- Water quality: compliance by sampling point and time, with laboratory provenance.
- Wastewater: treatment performance and effluent compliance.
- Energy: kWh per kilolitre delivered/treated, with measurement boundaries declared.
- Maintenance: leak detection-to-repair, backlog age, repeat failure rate.
- Finance: verified cost per kilolitre saved or restored, operating cost and collection effects.
- Equity: reliability and affordability across service areas, avoiding public disclosure of household-level records.

Do not optimise non-revenue water at the expense of access, affordability or water quality. Separate physical losses, apparent losses and authorised unbilled consumption where evidence permits.

## 6. Provisional engineering targets

These are proposed targets for experiment design, not claims about current performance.

| Dimension | Initial target | Qualification |
|---|---:|---|
| Authority enforcement | 0 accepted unauthorised actions | Hard gate; no tolerance |
| Replay | 100% semantic determinism for fixed reference inputs | Deterministic reference engine only |
| Evidence integrity | 100% rejection of declared corruption and policy-invalid evidence | Exact conformance suite |
| Duplicate mutation | 0 duplicate effective actions | Retry/partition test suite |
| Synthetic cascade truth | 100% on declared deterministic fixtures | Structural model only |
| Fault localisation | ≥0.90 precision@5 and recall@10 | Blind labelled suite |
| Probability forecasts | ≥10% relative Brier-score improvement over strongest baseline | Held-out dataset and calibration required |
| 90% forecast intervals | 88–92% empirical coverage | Adequate sample size; report subgroup coverage |
| Local degraded mode | Essential read-only functions survive a 24-hour federation partition | No unsafe write authority during partition |
| Auditability | 100% of consequential actions have complete authority/evidence/execution lineage | Hard gate |
| Pilot service impact | ≥15% reduction in a pre-registered response-time measure | Hypothesis, subject to paired baseline and safety non-regression |
| Cost-effectiveness | Lower total cost per verified service improvement than B0 | Include integration, compute, maintenance, training and audit costs |

Any numerical target should be revisited after a baseline and feasibility measurement. A failure to meet a target must be reported, not concealed by redefining the metric.

## 7. Experimental controls

Every benchmark run must freeze:

- Repository and source revision.
- Dataset version, snapshot time and content digests.
- Model and prompt/configuration version.
- Tool permissions and external data access.
- Hardware, operating system and compiler/toolchain.
- Random seeds and scenario order.
- Baseline implementations and tuning budget.
- Evaluation metric code and acceptance thresholds.

Use a blind holdout that is not used for feature selection or tuning. Run ablations. Include negative controls, corrupted evidence, stale evidence, missingness, adversarial data and out-of-distribution scenarios. Report all attempted runs, including failed and inconclusive runs.

For operational pilots, use pre-registered before/after and, where possible, matched-control or stepped-wedge designs. Account for seasonality, planned maintenance, capital works, rainfall, tariffs, demand changes and other confounders. Independent evaluators should be able to reproduce the calculation from retained evidence.

## 7.1 Verifier and model-disagreement qualification

The verifier is itself a fallible system. Qualification must test the evaluator and upstream model outputs, not assume that a separately named verifier is independent or correct.

Required challenge families:

- **Differential evaluator:** implement the declared metric/qualification semantics independently and compare outputs on the same frozen cases. If two evaluators disagree, the result is unresolved; no favourable result is selected silently.
- **Metamorphic invariants:** reorder input rows where order is semantically irrelevant; duplicate display labels without changing stable IDs; inject unknown fields; vary non-semantic metadata; test that qualified outcomes do not change unless the claim scope changes.
- **Negative-control mutation:** remove a predicate, change owner/schema, expire a grant, revoke authority, introduce a clock mismatch, insert a common failure ancestor, mark spatial evidence stale, place a physical model out of domain, or move mobilisation beyond the horizon. The expected qualification must downgrade with an exact reason.
- **Verifier fault injection:** mutate an evidence reference, swap scenario digests, omit source rows, change validity end, duplicate records, reorder source snapshots or corrupt one golden output. The verifier must reject or expose the discrepancy.
- **Model disagreement:** run more than one qualified domain model or parameterisation against the same scenario and record disagreement in service trajectory, capacity, mobilisation and portfolio ranking. Do not average model families by default. Report model identity, adequacy domain and calibrated error.
- **Leave-one-model-out sensitivity:** remove one model or source family and measure whether option qualification or portfolio ranking changes. Large changes indicate model dependence and should trigger review, not false confidence.
- **Independent reproduction cost:** record analyst hours, time-to-reproduce, required permissions/data, compute, specialist expertise and whether a second organisation can rebuild the claim from retained source-owner evidence.
- **Selective abstention:** measure risk-versus-coverage. Correctly abstaining on incomplete/ambiguous evidence is a positive outcome; over-abstention that blocks all useful cases is also measured.

Core metrics:
- exact reference agreement and disagreement rate;
- false-qualified recovery-option rate;
- false-independent option rate under hidden-common-cause controls;
- false-safe classification rate;
- reason-code precision/recall;
- stale/unknown detection sensitivity and false-stale rate;
- abstention precision, coverage and selective risk;
- report lineage completeness;
- independent reproduction time/cost;
- service/portfolio rank stability under plausible model and source perturbations.

Do not publish a verifier-accuracy percentage without stating the complete challenge set, attack families, number of cases, evaluator versions and confidence interval. Zero failures in a small synthetic test suite is a conformance observation, not a production probability bound.

## 8. Evidence classes and claim ceilings

| Class | Evidence | Maximum claim |
|---|---|---|
| E0 | Design, type checks or static inspection | The design or structural property is specified |
| E1 | Unit tests and deterministic synthetic fixtures | The tested reference behaviour works under the declared cases |
| E2 | Reproducible public-data replay | Performance on the named public dataset and version |
| E3 | Operator-provided historical data | Performance on the supplied operational sample, subject to provenance and selection limits |
| E4 | Shadow-mode observations | Operational usefulness without intervention authority |
| E5 | Independently evaluated supervised intervention | Measured effect within the particular pilot boundary |
| E6 | Replicated multi-site/multi-jurisdiction evidence | Evidence of transportability across the named settings |

A higher evidence class is not implied by repository size, test count, benchmark speed, model novelty, a signed receipt or a successful demonstration.

## 9. Reporting format

Every public result should publish:

1. Question and pre-registered hypothesis.
2. Baseline and comparator versions.
3. Dataset, time window and inclusion/exclusion rules.
4. Scenario and configuration digests.
5. Metrics, confidence intervals and subgroup outcomes.
6. Failed runs, missing data and deviations.
7. Threats to validity and claim ceiling.
8. Independent reproduction instructions and artefacts.

## 10. Research questions

1. Does a dependency hypergraph identify common-cause service failures better than pairwise asset graphs under the same evidence?
2. Does combining spatial context and physical simulation improve fault localisation over either alone?
3. Does the evidence/authority/execution loop reduce unsafe or duplicated actions compared with a strong AI agent using ordinary tools?
4. Does federation improve continuity under network partition without creating conflicting authority?
5. Which data-quality defects contribute most to wrong infrastructure decisions?
6. Does the system reduce the cost of independently verified service improvements after accounting for integration and ongoing stewardship?
7. Can the architecture remain useful when the underlying AI model, cloud provider or political administration changes?

The objective is to answer these questions with matched experiments, not to assume positive results.
