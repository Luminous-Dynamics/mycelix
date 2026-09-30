# Integral Interop 1 — Semantic Identity Conformance Target

Status: ReferenceModelOnly / research fixture

This document defines a deliberately narrow interoperability target between the public Integral Collective data model and the Mycelix Integral D6S/D6X/D6W reference model.

## Scope

The fixture is based on the public Integral OAD/COS/ITC pseudocode specifications, not an assumption about an implementation. Integral currently describes OAD as the prerequisite for COS, with certified design data feeding production planning, labor decomposition, and material accounting. COS emits candidate labor claims; ITC is the verification boundary for official LaborEvent records.

The first conformance object is an OAD-style certified design version containing stable design/version identifiers, a production-step decomposition, a normalized bill of materials, lifecycle/material metadata, and certification status.

The fixture intentionally does not claim that these pseudocode structures are already ratified wire schemas.

## Semantic boundary

OAD DesignVersion -> OAD LaborProfile/MaterialProfile -> COS production task definition -> ITC LaborEvent

D6S supplies deterministic identity for semantic records.
D6X selects only dependencies required by a named closure profile.
D6W consumes the resulting closure identity.
Runtime addresses, retrieval attempts, observations, and qualification context remain D6X resolution evidence and do not become semantic identity merely because they were used to retrieve a dependency.

## Conformance properties

1. Equivalent JSON object member ordering produces identical D6S-CANON-1 bytes.
2. Array order remains semantically significant where Integral's model uses ordered lists such as production_steps.
3. Map/member insertion order does not alter identity.
4. An irrelevant candidate object does not alter the selected D6X closure identity.
5. A selected material quantity, production step, or certified-version commitment mutation changes the closure identity.
6. A runtime retrieval reference can change without changing semantic closure identity.
7. A missing selected dependency cannot silently collapse into a smaller complete closure.
8. The fixture can be reproduced outside Rust from its published canonical bytes and domain-separated SHA-256 contract.

## Important non-goals

This does not establish authority of any Integral decision, truth or causal validity of a design, certification validity, production safety, economic-policy correctness, Holochain runtime equivalence, or interoperability with a future Integral implementation.

The purpose is narrower: prove that externally defined semantic records can cross the D6S/D6X boundary without implementation-defined identity drift.

## Source basis

The public Integral OAD specification defines DesignSpec, DesignVersion, MaterialProfile, LaborStep, LaborProfile, CertificationRecord, and OADValuationProfile. Its OAD model explicitly includes production_steps and bill_of_materials. The public COS specification defines COSTaskDefinition with required_materials_kg, and says COS emits candidate labor claims while ITC verifies and records official LaborEvent objects.

This fixture should be replaced or version-bumped if Integral ratifies different schemas in its authoritative specifications repository.
