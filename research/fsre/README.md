# Federated Societal Resilience Engineering (FSRE) — Integration and Qualification Profile

**Status:** Research architecture proposal, version 0.1.0  
**Date:** 2026-11-12  
**Maturity:** Design and synthetic-scenario stage; not an operational or production system.

FSRE is a cross-repository integration and qualification profile for the existing Mycelix Civic Resilience programme and Civilization Twin research architecture. It specifies how independently owned spatial, physical, causal, evidence, authority and service systems can compose for resilience studies and bounded service-improvement pilots.

It is not a new civic-governance programme, a second civilization simulator, an AI government, a central national digital twin, a population-scoring system, or a replacement for existing public digital infrastructure.

## Documents

- [Architecture v0.1](ARCHITECTURE_V0_1.md) — system boundaries, contracts, federation, safety invariants and deployment model.
- [Benchmark protocol v0.1](BENCHMARK_PROTOCOL_V0_1.md) — comparison baselines, test tiers, metrics, provisional targets and evidence rules.
- [South African municipal water cascade fixture](scenarios/ZA_MUNICIPAL_WATER_CASCADE_SYNTHETIC_V0_1.json) — a fully synthetic graph for deterministic failure-propagation tests.

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

The initial public-data research domain is municipal water and wastewater resilience in South Africa. The included fixture is synthetic and contains no municipal asset inventory, sensor observations, personal information or operational instructions. It is intended to test graph semantics only.

## Governing rule

**No claim of societal improvement is accepted from model output alone.** A claimed improvement requires a declared baseline, a comparison method, observed outcome data, uncertainty, and independent verification appropriate to the stakes.

See [benchmark protocol](BENCHMARK_PROTOCOL_V0_1.md) before interpreting any result.
