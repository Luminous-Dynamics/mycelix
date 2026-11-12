# FSRE Benchmark Protocol v0.1

**Status:** Proposed protocol; targets below are research targets, not achieved results.  
**Version:** 0.1.0  
**Date:** 2026-11-12

## 1. Benchmark objective

Measure whether the FSRE architecture improves essential-service resilience and accountable intervention compared with existing workflows and strong individual components.

Do not benchmark "national intelligence" as one opaque score. Decompose the system into claims that can be falsified and independently reproduced.

The primary outcome metric is:

**Total cost per independently verified unit of service improvement**, subject to hard safety, rights, privacy and equity constraints.

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

South Africa's 2025 Green Drop assessment reports 396 of 848 municipal wastewater treatment systems (47%) in a critical state, compared with 334 (39%) in 2022. Its 2023 No Drop report separately reports 47% national average non-revenue water. These indicators must remain distinct.

References:
- https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
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
