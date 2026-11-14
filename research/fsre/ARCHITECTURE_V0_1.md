# Federated Societal Resilience Engineering — Integration and Qualification Profile v0.1

**Status:** Research proposal  
**Version:** 0.1.0  
**Date:** 2026-11-12  
**Parent programmes:** Mycelix [CIV-RES-000 #2006](https://github.com/Luminous-Dynamics/mycelix/issues/2006), [CIV-ECON-006 #4816](https://github.com/Luminous-Dynamics/mycelix/issues/4816), and [Civic automation portfolio #5015](https://github.com/Luminous-Dynamics/mycelix/issues/5015).  
**Claim ceiling:** This document defines a cross-repository integration profile and test programme. It does not establish deployed national capability, field efficacy, regulatory compliance, or state-of-the-art performance.

## 1. Purpose

Federated Societal Resilience Engineering (FSRE) is an integration and qualification profile for improving the reliability, adaptability and accountable coordination of essential societal capabilities. It composes the existing Mycelix Civic Resilience programme (#2006), Civilization Twin research architecture (#4816), and Johannesburg service-reliability portfolio (#5015); it does not replace or duplicate them.

The unit of design is not a ministry, dashboard, AI model or database. It is an **essential capability**: a service people and organisations depend on, together with the physical assets, skills, energy, data, contracts, finance, operators and institutions required to provide it.

Examples include potable water, wastewater treatment, electricity, emergency response, food logistics, transport, healthcare supply, communications and local manufacturing.

FSRE connects the full intervention lifecycle:

`Need → Evidence → Dependency model → Hypotheses → Counterfactuals → Deliberation → Authority → Execution → Observation → Independent verification → Learning`

The intended result is not central control. It is better decision quality and coordination among actors who retain their own jurisdiction, accountability and ability to disagree.

## 2. Why an integration and qualification profile is needed

Existing digital public infrastructure already provides important building blocks. GovStack promotes modular, interoperable building blocks; X-Road provides a mature approach to secure data exchange; the Digital Public Goods Standard establishes open-licensing, privacy, documentation, platform-independence and standards requirements; and the Open Contracting Data Standard covers public contracting from planning through implementation.

FSRE should reuse and interoperate with these systems rather than compete with or replace them. The same rule applies inside Luminous: CIV-RES-000 owns the civic programme, its authority/privacy/study boundaries and Johannesburg profile; CIV-ECON-006 owns the coupled multi-scale physical/economic/institutional/ecological research environment; CIV-AUTOMATION #5015 owns the Johannesburg service-reliability and paid-work opportunity portfolio. FSRE supplies a qualification profile and cross-repository outcome tests across those existing owners.

The remaining problem is the gap between exchanging information and improving a service. A data exchange does not by itself establish that an observation is true, that a model is valid, that a proposed intervention is authorised, that an action actually occurred, or that the outcome improved.

The OECD's Digital Government Outlook 2026 reports that, on average, 63% of public institutions across OECD countries are connected to their national data-interoperability system; such systems are in place in 30 of 36 OECD countries. Separately, only 28% of OECD countries report conducting any financial or non-financial assessment of the impact of government AI use cases. The 28% figure is not a measure of systematic assessment and does not describe all public-sector interventions. These are OECD-country findings, not global estimates. They indicate a gap between formal infrastructure, operational adoption and impact evaluation—not an absence of comparable systems elsewhere.

References:
- GovStack specifications: https://govstack.global/
- X-Road architecture: https://x-road.global/
- Digital Public Goods Standard: https://www.digitalpublicgoods.net/standard
- Open Contracting Data Standard: https://www.open-contracting.org/data-standard/
- OECD Digital Government Outlook 2026, DPI and data governance: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/strengthening-digital-public-infrastructure-and-data-governance_2c7323c7.html
- OECD Digital Government Outlook 2026, AI in government: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/digital-governments-at-a-turning-point_0491aad4.html

## 3. Paradigm shifts

| From | To |
|---|---|
| Agency-centred dashboards | Service- and capability-centred models |
| Static asset inventories | Versioned, time-aware dependency hypergraphs |
| Data availability treated as truth | Evidence records with provenance, validity, quality and uncertainty |
| One forecast or one recommended action | Competing hypotheses and calibrated outcome distributions |
| Digital twin as a visual replica | Socio-technical service twin spanning assets, operators, skills, energy, finance, geography and rules |
| AI output treated as a decision | Model output treated as a proposal requiring independent authority |
| Audit after delivery | Continuous evidence capture and outcome reconciliation |
| Central integration hub | Federated, independently operable institutional nodes |
| Optimisation of one department | Multi-objective optimisation across service reliability, cost, rights, equity and environmental limits |
| Model accuracy as success | Verified improvement in the service people actually receive |
| Large-scale rollout before proof | Bounded, reversible pilots with pre-registered evaluation |

The key shift is from **information integration** to **verified intervention capability**.

## 4. Integration architecture

FSRE describes six logical planes across existing owners; it is not a new monolithic runtime or a proposal to move their state into one database. The owner-native boundaries remain authoritative. The profile specifies the evidence and hand-off contracts needed to qualify a composed service workflow. The Civilization Twin remains a research environment with explicit multi-scale and multi-time-horizon semantics, not an operational command system.

### 4.1 Spatial and asset context — Sol-Atlas

Responsibilities:

- Stable identifiers for places, assets, networks and service areas.
- Geospatial relationships, topology, exposure and proximity.
- Time-aware asset state and revision history.
- Spatial provenance, coordinate reference systems, resolution and accuracy.
- Links between asset location, service catchment, ownership or stewardship, and relevant constraints.

Boundary: a spatial record is not proof that an asset exists, is functional, is legally controlled by a particular party, or is safe. Field observations and authoritative records retain their own provenance and qualification.

### 4.2 Physical and industrial models — Symtropy

Responsibilities:

- Physical and operational models with explicit assumptions and parameter provenance.
- Scenario replay, sensitivity analysis and uncertainty propagation.
- Asset and network constraints.
- Resource, energy and service-demand accounting.
- Comparison of predicted trajectories with observed outcomes.
- Explicit model-domain limits and out-of-domain rejection.

Boundary: simulation output is not field evidence. Each model must disclose its spatial and temporal resolution, calibration domain, omitted physics, uncertainty, and validation status.

The current Symtropy energy-systems code already documents important limits in its chronological-adequacy model: it is a single-zone copperplate prototype and does not model transmission, unit commitment, reserves, voltage, frequency or storage dispatch. That is useful engineering honesty; FSRE must preserve such claim ceilings rather than generalise a bounded model into a national-grid qualification.

Reference: https://github.com/Luminous-Dynamics/symtropy/tree/main/crates/domains/symtropy-energy-systems

### 4.3 Reasoning and investigation — Symthaea

Responsibilities:

- Evidence retrieval and contradiction discovery.
- Alternative causal hypotheses.
- Root-cause investigation and dependency-aware reasoning.
- Scenario generation and counterfactual comparison.
- Prediction intervals, calibration and explicit unknowns.
- Recommendations with supporting evidence, counterevidence and disconfirming tests.

Boundary: Symthaea may propose and explain. It does not create legal authority, approve public expenditure, decide eligibility for essential services, or independently execute high-impact actions. A confidence score is not a substitute for evidence.

The Symthaea public README describes the project as an experimental cognitive architecture and explicitly distinguishes reproducible cryptographic primitives from regulatory certifications and claims about general intelligence or consciousness. FSRE adopts that evidence discipline.

Reference: https://github.com/Luminous-Dynamics/symthaea

### 4.4 Trust, authority and coordination — Mycelix

Responsibilities:

- Organisation and actor identity.
- Scoped capabilities and delegated authority.
- Consent, purpose and jurisdiction constraints.
- Commitments, responsibilities and approvals.
- Evidence lineage, attestations and dispute records.
- Cross-organisation coordination and revocation.

Boundary: a signature proves that a key signed a statement; it does not prove the statement is true. A replicated ledger proves that a record was committed under protocol rules; it does not prove that the physical work happened. Mycelix must not become a universal data lake or automatically authoritative source of facts.

The current Mycelix repository describes itself as a pre-alpha research project and its maturity matrix warns that many clusters have limited multi-agent testing or no deployed UI. FSRE is therefore an integration target, not a claim of present national deployment readiness.

Reference: https://github.com/Luminous-Dynamics/mycelix

### 4.5 Execution and operations

Execution remains with the responsible operator or existing operational system. FSRE sends bounded, signed, idempotent proposals or instructions only after the responsible authority approves them.

Required properties:

- Stable logical action identity distinct from retry identity.
- Preconditions and freshness requirements.
- Explicit limits on time, scope, spend and affected assets.
- Cancellation and revocation.
- Read-only or safe degraded mode.
- Duplicate-delivery protection.
- Reconciliation after timeouts or ambiguous responses.
- Compensating actions where a safe rollback is possible.

**Unknown outcome rule:** if an external mutation times out or its result cannot be established, do not issue a second mutation until the first action is reconciled. Unknown is not success.

### 4.6 Independent assurance and outcome measurement

An evaluator separate from the proposing model and executing operator should verify:

- Evidence digests and source references.
- Authority and policy scope at the time of action.
- Model, data and scenario versions.
- Preconditions and execution receipts.
- Expected versus observed outcomes.
- Missing observations, uncertainty and contradictory evidence.
- Baseline and comparison validity.

For higher-stakes outcomes, the evaluator must be organisationally and operationally independent of the party making the claim.

## 5. Shared contracts

The first interoperable artefact should be a small set of versioned, domain-neutral contracts. It should not force every domain into one ontology.

### 5.1 EvidenceRecord

Minimum fields:

- `evidence_id`, schema version and content digest.
- Source organisation and source-system identifier.
- Subject identifiers and geographic scope.
- Observation time, valid time and ingestion time.
- Method, instrument or collection process.
- Units, coordinate reference system where applicable, and resolution.
- Quality status, uncertainty and known limitations.
- Data classification, permitted purpose, retention and access policy.
- Parent evidence, transformations and model lineage.
- Revocation, supersession or dispute status.

An evidence reference must distinguish the time an event occurred from the time it became known. Late-arriving evidence must not silently rewrite a previously issued decision.

### 5.2 CapabilityGraph

A versioned, typed hypergraph:

- Nodes represent services, assets, operators, skills, resources, organisations, locations or constraints.
- Relations express typed dependencies, alternatives, common-cause exposure, ownership or stewardship, maintenance, observation and authority.
- Every relation carries validity intervals, evidence references and a qualification state.
- Unknown or stale relations remain explicit; they are not silently treated as healthy.

For this v0.1 integration fixture, a dependency rule with requirement groups marks a consumer structurally viable only when every required group retains at least one viable alternative. This simple rule is not a second canonical dependency engine. Production integration must reuse Mycelix's time-qualified dependency hypergraph and Sol-Atlas's explicit resilience/capability graph semantics, preserving their distinctions between declared dependency, alternative, availability, unknown closure and operational causality. Capacity and dynamic behaviour remain the responsibility of qualified domain models.

### 5.3 HypothesisSet

Each hypothesis contains:

- Claim and causal mechanism.
- Supporting and contradicting evidence.
- Assumptions and scope.
- Predicted observations that would distinguish it from alternatives.
- Confidence or probability semantics, including calibration basis.
- Known confounders and missing evidence.
- Expiry or re-evaluation condition.

Do not collapse disagreement into a single unsupported narrative.

### 5.4 InterventionProposal

Each proposal contains:

- Problem statement and intended service outcome.
- Affected assets, people, organisations and jurisdictions.
- Alternatives, including no action.
- Predicted outcome distributions and uncertainty.
- Resource, energy, financial and environmental constraints.
- Rights, accessibility, affordability and distributional impact analysis.
- Preconditions, monitoring plan and abort conditions.
- Reversibility and compensating action.
- Evidence, graph, model, code and scenario digests.
- Explicit statement of what the model cannot establish.

### 5.5 AuthorityGrant

A grant must bind:

- Issuer and authorised actor.
- Specific capability and jurisdiction.
- Purpose and affected subjects.
- Permitted action class and limits.
- Monetary, temporal and operational bounds.
- Approval threshold and required co-signers.
- Start time, expiry and revocation reference.

Authority must be checked at execution time, not only when a proposal is created.

### 5.6 ExecutionReceipt and OutcomeRecord

The execution receipt records the authorised logical action, attempt identity, actor, target, precondition result, start/end times, external system reference and final state.

The outcome record separately records actual observations, measurement method, units, comparator, missingness, uncertainty and verifier. A completed task is not itself proof of an improved service.

## 6. Federation and sovereignty

Each participating institution or jurisdiction operates a local node with its own:

- Identity provider and authority policy.
- Data custody and retention.
- Model registry and qualification.
- Operational systems and emergency procedures.
- Audit and appeal processes.
- Local fail-safe procedures.

Cross-node exchange uses explicit contracts and purpose-bound data. A country can reject a foreign model, operator or policy while still exchanging permitted service facts or technical evidence.

For civic/person-related data, FSRE inherits—not replaces—the three-plane privacy architecture in CIV-RES-000 #2006: (1) public projections using delayed/coarsened/thresholded aggregates; (2) protected operational records under purpose-bound capabilities, retention and access accountability; and (3) exceptional research-enclave use under an explicit study contract, immutable snapshot, audited access, no raw-data egress and bounded disclosure. The profile must not create a new Civic-only sensitivity taxonomy or make protected raw records general model context by default.

Federation must support:

- Partial connectivity and store-and-forward exchange.
- Local operation during network partitions.
- Conflict detection rather than silent last-write-wins for high-impact records.
- Version negotiation and schema evolution.
- Cryptographic provenance without requiring public disclosure of sensitive payloads.
- Data minimisation and selective disclosure.
- Exit, export, migration and independent reimplementation.

No single node, company, model or jurisdiction should be able to disable the whole federation.

## 7. Safety and legitimacy invariants

These are hard gates, not weighted objectives:

1. No AI system receives legal, democratic or operational authority merely by producing a recommendation.
2. No automated denial of essential services, benefits, liberty or legal status.
3. No population-wide social-credit or citizen-scoring mechanism.
4. No centralised collection of raw personal data where a federated query or selective disclosure is sufficient.
5. No action outside a current, scoped and revocable authority grant.
6. No high-impact action on stale, contradictory or unqualified evidence.
7. No model prediction presented as an observed fact.
8. No cryptographic receipt presented as proof of physical truth.
9. No unresolved external mutation retried as though it had failed.
10. Every consequential decision has an explanation, accountable owner and accessible challenge or appeal route.
11. Performance improvements cannot override rights, safety, affordability, accessibility or environmental constraints.
12. A deployment must remain usable in a safe degraded mode and have a tested withdrawal path.

These invariants should be translated into adversarial tests, not left as prose.

Relevant governance references include the [NIST AI RMF Generative AI Profile](https://www.nist.gov/publications/artificial-intelligence-risk-management-framework-generative-artificial-intelligence) and [ISO/IEC 42001](https://www.iso.org/standard/42001).

## 8. Candidate first qualification slice: municipal water and wastewater

Water is a useful candidate slice because it has physical assets, geographic dependencies, measurable service outcomes and public reporting. It should be selected through CIV-RES-000 #2006 and the existing Johannesburg civic automation portfolio #5015, which already prioritises water and sanitation alongside roads, stormwater, lighting, waste, civic service orchestration, procurement integrity and paid-work pathways. FSRE should qualify a bounded integration across those owners, not declare a new municipal pilot or imply a city/utility partnership.

South Africa's 2025 Green Drop assessment covered 848 municipal wastewater treatment systems for the 2023/24 financial year. The government reported 396 systems (47%) in a critical state, compared with 334 (39%) in 2022. The percentage rated good or excellent fell from 14% to 8%.

Separately, the 2025 No Drop Progress Report was released on 31 March 2026. In its 9 June 2026 briefing on the 2025 Drop progress reports, Parliament reported national non-revenue water at 47.3%, compared with 47.4% in the 2023 Full No Drop Report. This is a distinct drinking-water distribution-efficiency indicator, not a wastewater-treatment rating. Green Drop and No Drop must not be combined. Before any pilot baseline is frozen, the underlying report's observation window, denominator and municipal coverage must be reconciled with utility-approved operational data.

References:
- 2025 Green Drop results: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
- 2023 No Drop findings: https://www.gov.za/news/media-statements/water-and-sanitation-releases-2023-full-blue-drop-report-05-dec-2023

Initial workflow:

1. Build a provenance-labelled inventory from public sources and a willing utility partner.
2. Map assets and dependencies with Sol-Atlas.
3. Model service and energy constraints with qualified Symtropy models.
4. Use Symthaea to investigate competing diagnoses and recommend inspection priorities.
5. Use Mycelix to bind approved work to named authority, scope, budget and evidence.
6. Keep dispatch and physical control with the operator's existing systems.
7. Collect independent observations of repair, water quality, service continuity and cost.
8. Compare the intervention against a frozen baseline and, where feasible, a matched control.

The included synthetic scenario is only a graph-semantics fixture. It has no real asset locations, probabilities, hydraulic equations or operating instructions.

## 9. Deployment sequence

| Stage | Deliverable | Exit condition |
|---|---|---|
| 0 — Contract | Versioned evidence, graph, proposal, authority, execution and outcome contracts | Schema and adversarial contract tests pass |
| 1 — Synthetic | Deterministic dependency and cascade scenario suite | Known faults and common-cause cases reproduce exactly |
| 2 — Historical replay | Public or partner historical incidents and service records | Blind holdout results beat declared baselines without leakage |
| 3 — Shadow mode | Live read-only observation; no automatic dispatch or control | Operator and independent evaluator accept calibration and usefulness |
| 4 — Supervised pilot | Human-authorised, reversible maintenance interventions | Measured service benefit with no rights or safety regression |
| 5 — Federation | Multiple independent operators or jurisdictions | Partition, revocation, export, interoperability and local autonomy tests pass |

## 10. Existing owners and implementation boundaries

FSRE must compose with existing work rather than introduce parallel graph, evidence, benchmark or causal engines:

- **Mycelix dependency semantics:** [EMSR-PROV-003 #5923](https://github.com/Luminous-Dynamics/mycelix/issues/5923) owns the time-qualified dependency hypergraph and transitive-closure contract. [#6005](https://github.com/Luminous-Dynamics/mycelix/issues/6005) and [#6013](https://github.com/Luminous-Dynamics/mycelix/issues/6013) own topology-family scale/correctness extensions; [#5981](https://github.com/Luminous-Dynamics/mycelix/issues/5981) owns controlled intervention research for hidden common causes. FSRE consumes these results; it does not create a competing graph engine.
- **Sol-Atlas spatial and resilience context:** reuse [resilience profiles and explicit alternatives #21](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/21), existing provenance/validation/geometry owners [#54](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/54), [#59](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/59), [#65](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/65), and [pre-registered benchmark run manifest #338](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/338). Do not create a second spatial provenance or benchmark-manifest system.
- **Symtropy physical and social-system models:** align with [CIVDYN-BENCH-QUAL-001 #4392](https://github.com/Luminous-Dynamics/symtropy/issues/4392) and [ABM runtime/interoperability comparison #3787](https://github.com/Luminous-Dynamics/symtropy/issues/3787). Each domain model retains its own physical and statistical claim ceiling.
- **Symthaea causal reasoning:** reuse [CEB-CAUSE-001 #10366](https://github.com/Luminous-Dynamics/symthaea/issues/10366) and [REA-RUST-008 #9911](https://github.com/Luminous-Dynamics/symthaea/issues/9911) for competing hypotheses, causal diagnosis, value-of-information and safe next-experiment selection. Do not implement a second general-purpose experiment scheduler.

The inspected Mycelix main tree contains tools/emsr-dependency-hypergraph/examples/scale_benchmark.rs without a co-located Cargo manifest or library source in that directory. Related Mycelix issues and draft work describe the intended evaluator and reference-verifier programme, so the correct next action is to resolve the canonical crate location and exact candidate revision against [#5923](https://github.com/Luminous-Dynamics/mycelix/issues/5923), [#6005](https://github.com/Luminous-Dynamics/mycelix/issues/6005), [#6013](https://github.com/Luminous-Dynamics/mycelix/issues/6013) and [PR #6015](https://github.com/Luminous-Dynamics/mycelix/pull/6015)—not to declare the capability absent or create a duplicate implementation.

## 11. FSRE integration work packages

- **FSRE-01:** Freeze minimal cross-domain evidence, authority, intervention and outcome contracts; map them onto existing contracts before adding new types.
- **FSRE-02:** Adapt the synthetic water-service fixture to the canonical Mycelix dependency and Sol-Atlas resilience semantics; retain unknown and unqualified states.
- **FSRE-03:** Connect qualified Symtropy service/energy scenarios and model receipts without promoting the current single-zone prototype to national-grid claims.
- **FSRE-04:** Connect Symthaea's hypothesis/experiment receipts to operator review and value-of-information decisions.
- **FSRE-05:** Bind Mycelix authority grants, commitments, execution identities, revocation and outcome attestations to the intervention lifecycle.
- **FSRE-06:** Reuse Sol-Atlas issue #338 for frozen run manifests and independent benchmark reports.
- **FSRE-07:** Build the South African water-service public-data inventory and data-quality report with exact source version, rights, time coverage and non-operational boundary.

## 12. What would constitute a paradigm-level result?

A paradigm-level result would not be a large integrated dashboard or a demonstration that an AI can describe a national crisis.

It would be a reproducible result showing that independent institutions can:

- Identify critical dependencies more completely than their current siloed workflows.
- Detect and localise disruptions earlier without unacceptable false alarms.
- Compare interventions with calibrated uncertainty.
- Execute only actions authorised by the relevant operators.
- Preserve continuity during network or provider failures.
- Independently verify what changed.
- Demonstrate lower total cost per verified service improvement.
- Share only the data required for cooperation.
- Retain local control, portability and the right to exit.

Each claim must be measured against the benchmark protocol. No subsystem receives credit for an outcome it did not cause or help establish.
