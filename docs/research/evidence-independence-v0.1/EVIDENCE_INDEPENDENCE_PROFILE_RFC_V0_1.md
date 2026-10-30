# Evidence Independence Profile v0.1 — RFC

**Status:** Research specification and adversarial fixture contract; no production implementation or scientific-outcome claim.  
**Issue:** [#4950 — Qualify evidence independence before using replication counts in allocation](https://github.com/Luminous-Dynamics/mycelix/issues/4950)  
**Source baseline:** mycelix-desci/src/core/src/scientific_events.rs, blob 133bd274d998508b16c8add8187d34007d5cf53f.

## 1. Executive proposal

Replace the overloaded scalar concept of an “independent source” with a **versioned, multi-dimensional independence assessment over a provenance graph**.

A source can be independent in one dimension and correlated in another:

- two people can be distinct while belonging to the same organization;
- two organizations can independently analyze a mirrored copy of the same dataset;
- two labs can use a common collection site, instrument, protocol, or funding source;
- two software runs can be independently executed while sharing the same code, environment, data, and operator;
- two actor identifiers can be different strings while resolving to the same credential subject after key rotation.

The system must not collapse these states into one count. It should report the dimensions, the profile being evaluated, the evidence supporting each classification, and an explicit unknown or conflicted result where the provenance is incomplete.

**Paradigm shift:** independence becomes a reproducible query over a signed provenance graph, not a string-comparison heuristic or a self-asserted badge.

## 2. Current implementation baseline

The current EvidenceProfile exposes distinct counts for reviews, computational reproductions, independent replications, non-supporting outcomes, inconclusive outcomes, critiques, conflicts, corrections, and withdrawals. Attestation currently records evidence IDs and an optional protocol reference; EvidenceArtifact records a content hash, media type, locator, license, and availability.

The present source_key(actor, organization) collapses to the acting organization when one is present and otherwise falls back to actor ID. Reproduction and replication counts therefore use a single organization-or-actor key. Review counts use actor identity separately. The projection also excludes withdrawn attestations from active support and rejects duplicate active attestations by the same actor, uniqueness family, and protocol reference.

This baseline is useful as a v1 compatibility view, but it cannot distinguish dataset lineage, laboratory identity, collection site, instrument, method family, funding correlation, or execution provenance. A different organization presenting a mirror of the same dataset can currently look like an independent source if the acting-organization IDs differ. Conversely, independent teams inside one organization are collapsed for organization-or-actor source counts.

The current assessment already carries policy_id and policy_version. The proposal is to preserve the current policy lineage and add a separate v2 independence profile; do not silently reinterpret historic v1 counts.

## 3. Normative concepts

### 3.1 A source is not a string

A source identity is a resolved, typed object:

| Dimension | Canonical identity evidence | What it answers |
|---|---|---|
| Person / actor | Credential-registry subject with continuity across key rotation | Is this the same accountable researcher? |
| Organization | Governed organization identifier and authority binding | Are the sources organizationally independent? |
| Laboratory / team | Registry-backed or institutionally attested lab/team identifier | Are the experiments conducted by distinct operational groups? |
| Dataset lineage | Content-addressed provenance closure over source data, mirrors, transformations and derived datasets | Is this genuinely new data, or a new copy/derivative of the same data? |
| Protocol family | Canonical protocol identifier plus immutable version digest | Are the procedures comparable, and which version was used? |
| Preregistration | Persistent preregistration record and immutable version | Was the hypothesis/analysis plan fixed before observing the result? |
| Method family | Canonical method/analysis/software family and version | Are computational methods independent or merely reruns of the same pipeline? |
| Execution provenance | Signed run identity bound to inputs, code, parameters, environment, seed and outputs | Was there a distinct, reproducible execution? |
| Instrument / platform | Stable instrument identity and calibration provenance | Do measurements share a common measurement system? |
| Site / cohort | Canonical collection site, cohort or population identifiers | Are observations independent across location or population? |
| Funding / sponsor | Governed funder identity and disclosed support relationship | Is there a material shared-interest correlation? |

The raw identifier, its authority, and the proof that binds it must be retained. The resolver must not infer a canonical identity merely because two free-form strings differ.

### 3.2 Provenance is a graph

Represent datasets and activities using explicit relationships, not a flat list of URIs. At minimum, the graph needs relations equivalent to:

- derived_from: a new artifact is derived from a source artifact;
- mirror_of / same_origin_as: copies or mirrors share a lineage;
- used: an activity consumed an entity;
- generated_by: an entity was produced by an activity;
- was_associated_with: an activity involved an agent, team, instrument or organization.

Resolve transitive lineage with deterministic graph traversal. If two artifacts share a root ancestor, they do not become independent datasets merely because the file hashes or repository locations differ. If lineage assertions conflict, the lineage result is conflicted, not guessed.

### 3.3 Independence is profile-specific

There is no universally correct independence formula across disciplines. Mycelix should expose named, versioned profiles and publish their required dimensions:

| Profile | Distinct dimensions | Presence requirements | Meaning |
|---|---|---|---|
| reviewer_diversity | actor | verified actor identity | Distinct accountable people, regardless of organizational diversity |
| organization_independence | actor + organization | verified actor and organization | Distinct people acting for distinct governed organizations |
| empirical_replication | actor + organization + lab + dataset lineage | actor, organization, lab, dataset lineage, protocol and preregistration | Separate groups collect genuinely independent data under an identified protocol |
| computational_reproduction | actor + organization + execution provenance | actor, organization, dataset, method, execution and protocol | A separate governed group independently reproduces an analysis; the same source dataset may be appropriate |

A shared protocol is normally expected in an empirical replication; protocol equality is not evidence of dependence by itself. For computational reproduction, the same dataset is often intentionally reused, while the execution and responsible group must be distinct. A separate future computational_reimplementation profile can require method-family diversity.

### 3.4 Four-valued dimensions; three-valued decisions

Each dimension returns one of:

- established: the required identifiers and provenance demonstrate the dimension is distinct;
- not_established: the known records demonstrate that it is not distinct;
- unknown: required evidence is absent or unverified;
- conflicted: signed or authoritative records disagree.

A profile decision returns:

- qualified: all required dimensions are established and all required attributes are present;
- not_qualified: a required dimension is known not to be distinct;
- indeterminate: a required dimension is unknown or conflicted.

Missing metadata must never be treated as a distinct source. A profile must carry explicit reasons and the event/evidence references that support its decision.

## 4. Proposed Rust-oriented domain types

This is a design contract, not an implementation claim. The production model should remain Rust-first and strongly typed.

    enum DimensionStatus {
        Established,
        NotEstablished,
        Unknown,
        Conflicted,
    }

    enum ProfileDecision {
        Qualified,
        NotQualified,
        Indeterminate,
    }

    struct ProvenanceNode {
        id: CanonicalUri,
        kind: ProvenanceNodeKind,
        authority: Option<AuthorityReference>,
        content_digest: Option<ContentDigest>,
    }

    struct ProvenanceEdge {
        subject: CanonicalUri,
        predicate: ProvenanceRelation,
        object: CanonicalUri,
        asserted_by: EventReference,
    }

    struct IndependenceAssessment {
        profile_id: String,
        policy_id: String,
        policy_version: String,
        decision: ProfileDecision,
        dimensions: BTreeMap<IndependenceDimension, DimensionStatus>,
        reasons: Vec<AssessmentReason>,
        supporting_events: Vec<EventReference>,
        provenance_graph_digest: ContentDigest,
    }

Use domain-specific newtypes for IDs, digests, event references, and policy identities. The resolver must consume already-verified registry and event inputs; it must not perform network discovery from inside the deterministic projection reducer.

## 5. Compatibility and migration

1. Freeze v1 output and its DEFAULT_EVIDENCE_POLICY_ID / DEFAULT_EVIDENCE_POLICY_VERSION for historical replay.
2. Add v2 provenance descriptors through a versioned event/predicate extension or a new typed event. Do not alter canonical bytes of existing event payloads.
3. Rebuild v2 projections from source events plus the explicit v2 policy version. Preserve unknown/conflicted states.
4. Dual-run v1 and v2 on the same histories. Publish a decision-difference report; do not make v2 silently replace v1.
5. Maintain old assessments and their policy digests. A corrected provenance assertion creates a new event and a new assessment; it does not erase the previous decision.
6. Keep funding, maturity, and allocator authorization outside this profile. An independence assessment is an evidence-quality signal, not a verdict of truth or a disbursement instruction.

## 6. Corpus and first conformance targets

The machine-readable corpus is in cases.json; its Draft 2020-12 contract is cases.schema.json. It has 18 research fixtures covering:

- repeated attempts by the same actor/team;
- multiple people in one organization;
- different organizations sharing mirrored or derived data;
- genuinely independent data collection;
- computational re-analysis of the same data;
- different organizations sharing a lab, or one organization operating multiple labs;
- missing organization identity or dataset lineage;
- conflicting lineage assertions;
- unverified actor aliases and credential-backed key rotation;
- shared site/instrument correlation;
- withdrawals and corrections.

These are fixture expectations, not executed Rust results. The next implementation stage must add a pure resolver oracle and execute these cases against ClaimProjection without introducing an alternate event-signing system.

### Release gates

| Benchmark | Initial gate |
|---|---:|
| Known synthetic fixture outcomes | 100% exact agreement |
| False qualification on correlated-source adversaries | 0 / at least 1,000 deterministic mutations |
| Missing required metadata | 100% propagated as indeterminate, never qualified |
| Conflicting lineage resolution | 100% propagated as indeterminate |
| Repeated attempt / replay amplification | 0 count increase |
| Key rotation / alias continuity | 100% registry-resolved identity continuity |
| Policy replay | 100% deterministic output for fixed event, provenance and policy digests |
| Compatibility | Historical v1 output remains byte-for-byte or semantically stable under its original policy version |
| Performance | Measure 1, 10, 100, 1,000 and 10,000 sources; report p50/p95/p99, allocations and graph size; target O(V+E) lineage resolution |

A zero-false-qualification target on designed adversarial fixtures is a release gate, not proof that every real-world experiment is independent. Publish coverage and indeterminate rates alongside qualified counts.

## 7. Industry and standards positioning

This RFC composes with established standards instead of replacing them:

- [W3C PROV-O](https://www.w3.org/TR/prov-o/) supplies a general provenance model built around entities, activities and agents, with relations for use and generation.
- [RO-Crate 1.3](https://www.researchobject.org/ro-crate/specification.html) packages research data and contextual metadata, including people, organizations, software, equipment and workflows, using JSON-LD.
- [DataCite Metadata Schema 4.7](https://schema.datacite.org/) provides interoperable resource identification and related-identifier relations; its 2026 release adds RAiD and SWHID as related identifier types.
- [FAIR principles](https://www.go-fair.org/fair-principles/) establish findability, accessibility, interoperability and reusability, including rich metadata and provenance.

These standards make research objects and their provenance more interoperable. Mycelix's proposed additional layer is the explicit, versioned **independence decision**: which dimensions are required for this scientific activity, how source correlations are resolved, what remains unknown, and exactly which signed inputs produced the result. It must reuse external identifiers and provenance vocabularies wherever possible.

## 8. Known implementation blocker

The DeSci core currently has a separate, tracked build-integrity problem: lib.rs declares and re-exports scientific_credentials, but the module source is absent from the inspected tree. The build blocker is tracked in [#5189](https://github.com/Luminous-Dynamics/mycelix/issues/5189), with a migration-safe resolution contract in [PR #4968](https://github.com/Luminous-Dynamics/mycelix/pull/4968). The independent-policy design can be reviewed now, but runtime qualification must wait until the authentic credential registry boundary is restored. No placeholder module should be introduced to make tests appear green.

## 9. Open review questions

1. Which profiles should be normative per domain (field experiment, clinical study, computational analysis, observational study, simulation)?
2. Which sources of correlation are material in each domain: organization, lab, dataset lineage, cohort/site, instrument, method, funding, or other dimensions?
3. Which identifiers can be resolved from existing registries and which need signed local attestations?
4. What level of provenance completeness is mandatory before a source can affect a maturity assessment?
5. Should the system expose profile-specific qualified counts at all, or only per-dimension assessments and supporting graph paths?
