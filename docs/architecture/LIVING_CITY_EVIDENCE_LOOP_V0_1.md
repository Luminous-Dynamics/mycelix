# Living City Evidence Loop — Reference Architecture v0.1

Status: proposed contract; documentation-only milestone.
Scope: a globally reusable urban-systems evidence loop, with Johannesburg as the first validation context.
Non-goal: this document does not claim a deployed city digital twin, validated urban model, municipal integration, or production readiness.

## Decision

Start with one narrow, auditable loop:

1. ingest a sourced observation or planning assumption;
2. normalize its spatial and temporal scope;
3. represent an intervention as a versioned scenario;
4. compare a baseline with one or more alternatives;
5. publish results with uncertainty, costs, constraints, and provenance;
6. let affected people and authorized institutions review, contest, or accept the proposal;
7. record the decision and later measured outcome separately from the prediction.

The first demonstration should address a bounded Johannesburg neighborhood-scale problem such as shade/tree-canopy and pedestrian heat exposure, or water-service reliability. Choose the pilot only after confirming that suitable, legally reusable data and a defensible baseline exist. Do not invent measurements to fill gaps.

## System boundaries and ownership

| Project | Owns | Must not own |
|---|---|---|
| Sol-Atlas (sol-atlas-core, sol-atlas-leptos, sol-atlas-bevy) | Geospatial layers, coordinate/reference-system metadata, map selection, spatial and temporal visualization | Civic authorization or the truth of a source claim merely because it is rendered |
| Symthaea | Scenario hypotheses, constrained analysis, model selection, uncertainty and sensitivity analysis, explanation of recommendations | Final civic decisions; unverified claims presented as facts |
| Symtropy | Deterministic physical/agent simulation where its physics and dynamics are appropriate; reproducible scenario playback | Authoritative real-world measurements or policy authority |
| Mycelix | Provenance records, consent and access policy, review/dispute workflow, governance decisions, attribution, signed outcome receipts | A central omniscient database; public disclosure of private or sensitive personal data |

The integration contract is a versioned, implementation-neutral data model. Each repository may use idiomatic Rust types internally. Adapters must validate at the boundary; avoid making all projects depend on one large shared crate.

## Core records

Use stable opaque IDs (UUID/ULID or an explicitly documented equivalent), UTC timestamps, explicit units, and schema versions. Preserve original source values alongside normalized values.

### 1. Observation

Required fields:

- schema_version
- observation_id
- indicator — namespaced stable identifier, e.g. urban.heat.surface_temperature
- value and unit — numeric value or a typed categorical value; never unitless by accident
- geometry_ref — Sol-Atlas feature/cell ID or a geometry with declared CRS
- observed_at or valid_time_start + valid_time_end
- source — publisher, source URI or durable citation, retrieval/publication time when known, license/usage rights
- method — measurement, administrative record, remote sensing, model estimate, survey, or assumption
- quality — validation status, known limitations, and missingness
- uncertainty — typed interval/distribution/confidence statement where supported; otherwise explicitly unknown
- provenance_ref — Mycelix evidence/receipt identifier when recorded there

Observations must distinguish measured, reported, inferred, and assumed values. An estimate must never silently become an observation.

### 2. Scenario

Required fields:

- schema_version, scenario_id, scenario_version
- area_of_interest and time horizon
- baseline_refs[] — immutable observation/dataset versions
- interventions[] — typed actions, location, timing, capacity, dependencies, and operating/maintenance assumptions
- constraints[] — safety, ecological, legal, budget, accessibility, and equity constraints
- model_id, model_version, parameter_set_hash, seed where stochastic
- created_by, created_at, and review state

A scenario must be reproducible from its referenced inputs and declared model/configuration versions, or explicitly labelled non-reproducible.

### 3. Result

Required fields:

- schema_version, result_id, scenario_ref
- metric and unit
- baseline and alternative result values
- uncertainty/sensitivity information and known model limitations
- resource and lifecycle cost estimates, with currency and price-year where applicable
- trade-offs and distributional impacts where evidence supports them
- run_manifest — code revision, configuration hash, dataset hashes, execution environment, seed, and command or workflow identifier
- status — exploratory, reproduced, externally_validated, or retracted

Status is not a quality badge: each transition requires recorded evidence and a reviewer. Reproduced means another run of the specified inputs and model reproduces the declared outputs within stated tolerances. Externally validated requires a defined independent validation method and reviewer. No result becomes validated just because CI passes.

### 4. Decision and outcome

Keep the governance decision separate from model results. A decision records the responsible body/person or privacy-preserving credential, authority/mandate, decision time, rationale, dissent/appeal path, and links to the reviewed result version. An outcome record stores later measured effects, measurement method, time window, and deviations from the prediction. Never overwrite a prediction with its eventual outcome.

### 5. Work package, funding, and labor protection

A civic plan becomes an economic program only after a separate, auditable work-package decision. Do not infer a funded job from a need, a model recommendation, a budget headline, a programme target, or a procurement index page.

The work-package contract should be owned by Mycelix and link—not duplicate—the upstream case, analysis, and simulation records. At minimum, it should bind:

- the stable work-package ID/version; linked Sol-Atlas case IDs and coarse public-area references; scenario/result versions; deliverables and completion criteria;
- the proposing party, legally/organizationally authorized commissioning party, decision receipt, accountable asset/service owner, and a recorded human authorization;
- the funding source and payer, currency/price year, full-period committed amount, commitment evidence, start/end period, and whole-life labor/support costs including supervision, training, tools, protective equipment, accessibility needs, transport, insurance where applicable, and independent verification;
- the procurement/contract issuer and namespaced external reference, source and retrieval time, lifecycle stage, closing date where applicable, award/contract evidence, and applicable open-data mapping (use the [Open Contracting Data Standard](https://standard.open-contracting.org/latest/en/) where relevant rather than inventing a parallel procurement ontology);
- role/employment classification, applicable wage basis, authoritative source, qualified human reviewer, and review time. The system must never infer a legal employment regime from a programme label or default an unknown classification to a cheaper wage;
- written terms, pay cadence, worker protections, accessibility accommodations, dispute/grievance route, and separately specified outcome-acceptance and verification procedures.

Suggested lifecycle states are `discovery`, `proposed`, `classification_review`, `funding_committed`, `authorized`, `announced`, `active`, `completion_claimed`, `independently_verified`, and `closed_or_cancelled`. Treat this as a proposed state vocabulary, not an existing runtime API. State transitions require evidence: a discovery lead is not an open contract; a funding estimate is not a commitment; a commitment is not authorization; and an announced role is not proof of hiring or durable employment.

**Non-negotiable labor boundary:** service completion, public-outcome verification, and causal impact are different records. Independent review may qualify a service-resolution claim, but it must not silently erase or delay wages already earned under applicable law and written terms. Model outputs cannot classify workers, determine entitlement, blacklist a contractor, or make adverse decisions about a resident or worker.

The first useful cross-stack path is deliberately narrow:

`Sol-Atlas CivicServiceCase` → `Symthaea AnalysisReport` and, only where useful, `Symtropy SimulationResult` → human-authorized Mycelix work package and funding receipt → real-world activity recorded as an append-only case event → independently evidenced service outcome → later outcome assessment.

For Johannesburg, the existing [Sol-Atlas civic case ledger PR #109](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/109) and [Mycelix workforce pilot PR #5016](https://github.com/Luminous-Dynamics/mycelix/pull/5016) are candidate source contracts for this flow; both remain proposals until reviewed and qualified. The [Humanity Atlas scenario contract PR #1571](https://github.com/Luminous-Dynamics/symtropy/pull/1571) can provide simulation/run provenance, while [Symthaea life/civic domain mastery PR #7443](https://github.com/Luminous-Dynamics/symthaea/pull/7443) is a reasoning/evaluation proposal—not an authorization service.

Add synthetic negative controls before enabling any such flow: missing payer, uncommitted or insufficient full-period funding, unknown human-reviewed wage classification, stale/cancelled/awarded procurement notice mislabelled as open, unapproved scope, provider self-verification, and a disputed service result incorrectly used to deny wages. None of those conditions may silently advance a work package.

## Boundary interfaces

Prefer small versioned Rust types and adapter traits. These are conceptual signatures, not existing APIs:

- Sol-Atlas: SpatialFeatureRef, GeometryRef, LayerMetadata, TemporalExtent
- Symthaea: ScenarioRequest -> AnalysisReport
- Symtropy: SimulationManifest + Scenario -> SimulationResult
- Mycelix: EvidenceRecord, ReviewDecision, OutcomeReceipt

Every adapter must return structured errors for unsupported schema versions, missing units/CRS, invalid time ranges, unavailable evidence, or failed authorization. A renderer may consume results but cannot change their provenance or validation status.

Transport can initially be checked-in JSON fixtures and local Rust calls. Select HTTP, Holochain zome calls, or a message bus only when the pilot's deployment and trust boundaries are known. Do not prematurely make a network protocol the source of truth.

## First vertical slice

Build a small, synthetic fixture first, then replace it only with documented, reusable real data:

1. one bounded neighborhood or grid;
2. one indicator and one baseline period;
3. two alternatives plus an unchanged baseline;
4. one simple intervention model with explicit assumptions;
5. one Sol-Atlas view showing source coverage and missing data;
6. one Symthaea report comparing scenarios and sensitivity;
7. one Symtropy simulation only if its physical/agent dynamics add value to this scenario;
8. one Mycelix evidence/review record linking input hashes, output hashes, and the decision record.

Recommended initial indicator: pedestrian shade/heat exposure, if defensible spatial and temporal inputs are available. Otherwise select water-service reliability using official service records with clear definitions. Do not mix service-standard targets with actual observed performance.

## Verification plan

### Contract tests (all adapters)

- valid fixture round-trips without dropping units, CRS, timestamps, source rights, or uncertainty;
- reject missing units for quantitative indicators and missing CRS for spatial geometry;
- reject unsupported schema_version and invalid/empty time intervals;
- reject an observation that has no source/method classification;
- preserve unknown uncertainty rather than defaulting it to zero;
- stable canonical serialization produces the same content hash for equivalent records;
- changing a source, model, parameter, or dataset version changes the relevant manifest/hash;
- prediction, decision, and observed outcome remain distinct records.

### Reproducibility and negative controls

- same pinned inputs/model/config/seed produce identical deterministic outputs, or documented numeric tolerances for floating-point/GPU execution;
- shuffled input order does not alter order-independent aggregates;
- a deliberately corrupted input hash is rejected;
- a deliberately stale dataset/model version is detected;
- removing a required source makes the result incomplete, not silently successful;
- a synthetic scenario with no intervention does not claim an intervention benefit;
- missing data and unequal coverage are surfaced in the report and map;
- negative-control scenarios must remain in the test corpus; never weaken them to make a result pass.

### Privacy, governance, and security

- public examples contain no personal records or precise household-level sensitive locations;
- access checks are enforced at the adapter boundary, not only in the UI;
- private evidence may be referenced by opaque ID without publishing its contents;
- a decision can be challenged/retracted without erasing the audit history;
- model output alone cannot authorize spending, service denial, surveillance, or physical actuation.

### Evidence bundle

Every milestone run should produce a machine-readable manifest containing repository revisions, fixture/dataset hashes and licenses, commands, toolchain, test names/results, and output hashes. A queued CI job is not a pass. Report exact tested revisions and distinguish local results from CI results.

## Acceptance criteria for v0.1

- [ ] This contract is reviewed and linked from each participating repository's architecture documentation.
- [ ] One versioned JSON fixture exists for each core record and passes a schema validator.
- [ ] At least one contract test covers each adapter boundary.
- [ ] Negative controls for corrupted hashes, missing units/CRS, missing data, and no-op interventions fail closed.
- [ ] A tiny end-to-end fixture produces a source-to-result provenance chain that can be independently verified.
- [ ] The map visibly distinguishes observed data, estimates, assumptions, and missing coverage.
- [ ] Scenario comparison reports costs, trade-offs, uncertainty, and data/model limitations rather than a single opaque score.
- [ ] No claim of Johannesburg validation is made until real data provenance, local ground-truth comparison, and an independent review protocol are documented.
- [ ] The pilot selection includes a data-rights check and a local stakeholder review plan.

## Proposed implementation order

1. Mycelix: review this boundary contract and define the minimal evidence/review record mapping.
2. Sol-Atlas: implement the fixture and spatial metadata rendering, including missing-data visibility.
3. Symthaea: consume the versioned scenario fixture and emit a report plus run manifest.
4. Symtropy: add a simulation adapter only for variables it can meaningfully model; preserve deterministic replay and document any nondeterminism.
5. Integration: add a thin end-to-end verifier that checks hashes, schema compatibility, provenance links, and acceptance criteria.

The deliverable is a verified evidence loop, not a claim that a complete living city has already been built.
