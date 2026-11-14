# Federated Societal Resilience Engineering (FSRE) — Integration and Qualification Profile

- [Equity-aware service-floor benchmark](EQUITY_AND_DISTRIBUTIONAL_RESILIENCE_BENCHMARK_V0_1.md) — cohort-specific minimum service, distributional/restorative equity, procedural evidence and transgenerational liabilities.
- [Portfolio feasibility reference and independent oracle](../../tools/fsre-portfolio-reference/README.md) — read-only Rust constraint evaluator plus a separately authored differential implementation.
- [Recovery portfolio benchmark extension v0.1](RECOVERY_PORTFOLIO_BENCHMARK_V0_1.md) — common-cause and shared-resource qualification of portfolios of recovery options.

**Status:** Research architecture proposal, version 0.1.0  
**Date:** 2026-11-13  
**Maturity:** Design, synthetic scenarios and one bounded official-summary replay; not an operational or production system.

FSRE is a cross-repository integration and qualification profile for the existing Mycelix Civic Resilience programme and Civilization Twin research architecture. It specifies how independently owned spatial, physical, causal, evidence, authority and service systems can compose for resilience studies and bounded service-improvement pilots.

It is not a new civic-governance programme, a second civilization simulator, an AI government, a central national digital twin, a population-scoring system, or a replacement for existing public digital infrastructure.

## Documents

- [Architecture v0.1](ARCHITECTURE_V0_1.md) — system boundaries, contracts, federation, safety invariants and deployment model.
- [Benchmark protocol v0.1](BENCHMARK_PROTOCOL_V0_1.md) — comparison baselines, test tiers, metrics, provisional targets and evidence rules.
- [Verifier independence and model-disagreement benchmark](VERIFIER_AND_MODEL_DISAGREEMENT_BENCHMARK_V0_1.md) — differential evaluator checks, metamorphic tests, negative-control mutations, model disagreement and independent re-hosting.
- [Standards and industry crosswalk](FRAMEWORK_CROSSWALK_V0_1.md) — positioning against existing resilience, critical-entity and digital-government frameworks.
- [Competitive landscape](COMPETITIVE_LANDSCAPE_V0_1.md) — direct comparison with enterprise ontology, asset-performance, infrastructure-twin and water-operations products, with testable differentiation hypotheses.
- [South African public-data inventory](ZA_PUBLIC_DATA_INVENTORY_V0_1.md) — dated water/sanitation data sources, admissibility, missingness and the minimum operator evidence needed for a qualified service study.
- [South African public source register](ZA_PUBLIC_SOURCE_REGISTER_V0_1.psv) — machine-readable discovery register with source release, observation period, grain, denominator, currentness and extraction state.
- [South African public-data replay protocol](ZA_PUBLIC_DATA_REPLAY_PROTOCOL_V0_1.md) — exact Treasury cube/item taxonomy and extraction, null, pagination, identity and DWS/Stats SA admissibility rules.
- [Wastewater service-chain completeness benchmark](WASTEWATER_SERVICE_CHAIN_COMPLETENESS_V0_1.md) — end-to-end sanitation boundary, missing-flow accounting, source/denominator integrity, sealed tests and operator-authorised research stages.
- [Temporal identity continuity and crosswalk benchmark](IDENTITY_CROSSWALK_BENCHMARK_V0_1.md) — source-backed continuity, rename, transfer, merge, split, decommissioning, creation, unresolved identity, population-scope changes and fair longitudinal comparison.
- [Temporal identity crosswalk reference](../../tools/fsre-identity-crosswalk/README.md) — a Rust structural validator for transition cardinality, evidence class, review state and same-date conflict controls.
- [Decision-value and active-investigation benchmark](DECISION_VALUE_AND_ACTIVE_INVESTIGATION_V0_1.md) — constrained value-of-information, sequential inspection/sampling, matched-cost baselines, stopping and abstention benchmarks for Symthaea.
- [Exact decision-value reference evaluator](../../tools/fsre-decision-value/README.md) — a Rust finite-hypothesis EVSI evaluator with integer weights, explicit costs, admissibility and measurement-authority gates.
- [Interval wastewater flow-balance reference](../../tools/fsre-flow-balance/README.md) — a Rust reference evaluator for conservative interval accounting, explicit unknowns, boundary-scoped structural zeros and non-attributed residuals.
- [Recovery-option interoperability contract](RECOVERY_OPTION_INTEROP_CONTRACT_V0_1.md) — seven owner-qualified predicates, conservative federated time semantics, revalidation and authority boundaries.
- [Claim qualification profile](CLAIM_QUALIFICATION_PROFILE_V0_1.md) — distinguishes synthetic conformance, public-source reporting, model outputs, operator observations, causal effects and independent service-outcome verification.
- [Claim qualification envelope schema](schemas/claim-qualification-envelope-v0.1.schema.json) — machine-readable, claim-scoped evidence, temporal, provenance, independence and non-authorisation contract.
- [Synthetic recovery claim example](examples/claim-envelope.synthetic-recovery-v0.1.json) — a bounded software-conformance statement for the seven-option synthetic fixture, explicitly not a field claim.
- [Investment portfolio evaluation protocol](INVESTMENT_PORTFOLIO_EVALUATION_PROTOCOL_V0_1.md) — pre-registered matched portfolios, service/economic/distribution measures, causal design and falsification rules.
- [Option-preserving resilience benchmark](OPTION_PRESERVING_RESILIENCE_V0_1.md) — service trajectories, qualified recovery options, graceful degradation and falsification tests.
- [Resilience debt and option-preserving investment](RESILIENCE_DEBT_AND_PORTFOLIO_V0_1.md) — derived resilience liabilities, workforce/supplier/maintenance constraints and multi-objective investment portfolios.
- [Service-trajectory reference evaluator](../../tools/fsre-service-trajectory/README.md) — a dependency-free Rust metric evaluator for one service/cohort time series.
- [Recovery-option qualification reference](../../tools/fsre-recovery-qualification/README.md) — a dependency-free Rust evaluator that composes owner-qualified feasibility, spatial, dependency, staffing, resource, authority and safety dispositions without duplicating subsystem engines.
- [Independent recovery reference evaluator](../../tools/fsre-recovery-reference/README.md) — a separately authored Rust implementation that differentially checks scenario-scoped recovery-option dispositions without importing the production evaluator.
- [Public source-register validator](../../tools/fsre-source-register/README.md) — a dependency-free Rust validator for provenance, grain, denominator, URL and extraction-state completeness in the South African public-data register.
- [Green Drop public-summary replay](../../tools/fsre-green-drop-replay/README.md) — a dependency-free Rust replay of the official 2025 national summary, with denominator and rounding checks and an explicit non-field claim ceiling.
- [South African municipal water cascade fixture](scenarios/ZA_MUNICIPAL_WATER_CASCADE_SYNTHETIC_V0_1.json) — a fully synthetic graph for deterministic failure-propagation tests.
- [South African resilience-debt fixture](scenarios/ZA_RESILIENCE_DEBT_SYNTHETIC_V0_1.json) — synthetic maintenance, dependency, workforce, supplier, evidence and authority deficiencies with recovery-option dispositions.

## Intended system roles

| System | Responsibility within FSRE |
|---|---|
| Sol-Atlas | Spatial context: geography, assets, networks, locations and spatial provenance |
| Symtropy | Physical and industrial models: constraints, trajectories, simulation and model qualification |
| Symthaea | Evidence-grounded reasoning: hypotheses, causal investigation, scenario comparison and uncertainty |
| Mycelix | Identity, authority, commitments, cross-organisation coordination and evidence lineage |

These are bounded roles, not claims that every integration or capability is already implemented. Each subsystem retains its own maturity and qualification boundary.

## Parent programmes this profile composes

- Mycelix [CIV-RES-000: Civic Resilience composition programme and Johannesburg pilot profile #2006](https://github.com/Luminous-Dynamics/mycelix/issues/2006) owns the civic-resilience programme, civic authority/privacy boundaries, study and outcome semantics, and the Johannesburg deployment profile.
- Mycelix [CIV-ECON-006: Civilization Twin #4816](https://github.com/Luminous-Dynamics/mycelix/issues/4816) owns the multi-scale, multi-time-horizon research environment for coupled physical, economic, institutional and ecological dynamics. It explicitly rejects a universal civilization score.
- Mycelix [Civic automation portfolio #5015](https://github.com/Luminous-Dynamics/mycelix/issues/5015) owns the Johannesburg service-reliability, water/sanitation, asset-protection and paid-work opportunity portfolio.

FSRE is deliberately narrower: it defines cross-repository service-resilience integration, qualification gates and matched outcome evaluation. It does not replace the parent programmes or define another universal civic ontology.

## Reuse existing implementation owners

FSRE is an integration and outcome-evaluation programme, not a second implementation of existing subsystems. The first mapping is:

- Mycelix time-qualified dependency hypergraph: [#5923](https://github.com/Luminous-Dynamics/mycelix/issues/5923), topology benchmark [#6005](https://github.com/Luminous-Dynamics/mycelix/issues/6005) and [#6013](https://github.com/Luminous-Dynamics/mycelix/issues/6013), hidden common-cause intervention research [#5981](https://github.com/Luminous-Dynamics/mycelix/issues/5981).
- Sol-Atlas resilience profiles and alternative-path analysis: [#21](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/21); source/provenance and validation owners [#54](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/54), [#59](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/59), [#65](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/65); pre-registered benchmark-run contract [#338](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/338).
- Symtropy matched model-adequacy, causal-policy and runtime benchmark [#4392](https://github.com/Luminous-Dynamics/symtropy/issues/4392), plus ABM runtime comparison [#3787](https://github.com/Luminous-Dynamics/symtropy/issues/3787).
- Symthaea causal debugging and experiment-selection benchmark [#10366](https://github.com/Luminous-Dynamics/symthaea/issues/10366) and evidence-grounded causal diagnosis [#9911](https://github.com/Luminous-Dynamics/symthaea/issues/9911).

The FSRE water fixture is a consumer-level synthetic integration case. It does not replace the time-qualified dependency model, Sol-Atlas capability graph, causal experiment engine, or domain physics models owned by those workstreams.

## First test domain

The initial public-data research domain is municipal water and wastewater resilience in South Africa. The existing cascade and recovery-debt fixtures remain synthetic. A separate Green Drop national-summary fixture now replays a published official comparison, but it is not system-level extraction or field evidence. The first wastewater expansion is a full-chain boundary from generation and sewer collection through treatment, discharge/reuse and receiving environment; plant-level metrics alone are not a complete sanitation-service outcome.

## Governing rule

**No claim of societal improvement is accepted from model output alone.** A claimed improvement requires a declared baseline, a comparison method, observed outcome data, uncertainty, and independent verification appropriate to the stakes.

See [benchmark protocol](BENCHMARK_PROTOCOL_V0_1.md) before interpreting any result.
