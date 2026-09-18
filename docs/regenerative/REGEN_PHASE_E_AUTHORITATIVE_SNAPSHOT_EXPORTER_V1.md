# REGEN Phase-E Authoritative Snapshot Exporter v1

Status: architecture/preregistration only

Program: Luminous-Dynamics/mycelix#940

Parent qualification profile: #1587 / `64b6e7d9d5d0e854c06442915a1b9f510794068c`

Bridge executable contract: #1572

Cross-repository ownership: #1523

## 1. Purpose

Freeze the missing producer-side boundary between authoritative/adopted Mycelix domain state and the pure canonical Phase-E bridge.

The exporter answers one narrow question:

> which exact already-authoritative objects and evidence states were selected, under which coverage/currentness rules, to construct this one Phase-E assessment envelope?

It does not make a domain record authoritative, choose policy by model preference, invent service floors, perform resilience simulation, authenticate the final transport, or authorize physical action.

## 2. Governing separation

```text
domain record exists
!= record is authoritative for this proposition
!= record is current enough for this purpose
!= source coverage is complete
!= record belongs in adopted Phase-E assessment
!= bridge payload is complete
```

The exporter must preserve these distinctions rather than collapse them into `snapshot_valid=true`.

## 3. Evidence cut, not fictional global snapshot

Mycelix spans independent domains and distributed stores. V1 MUST NOT imply that all sources were read at one globally atomic instant unless a specific owning source actually provides and qualifies such a primitive.

The exported object is therefore an explicit **AssessmentEvidenceCut**.

Conceptually:

```rust
pub struct AssessmentEvidenceCut {
    pub cut_id: ExactRef,
    pub selection_profile_ref: ExactRef,
    pub service_profile_refs: Vec<ExactRef>,
    pub source_cuts: Vec<SourceCut>,
    pub unresolved_coverage: Vec<CoverageGap>,
}
```

A cross-domain cut can contain source observations from different admissible times/currentness windows while retaining that fact exactly.

## 4. Source-cut identity

Every contributing source must expose a source-specific cut record carrying at minimum:

```text
source_domain
source_profile_ref
source_subject_scope_ref
coverage_class
coverage_evidence_ref
observed/read basis
currentness profile ref
currentness evaluation result/evidence
selected object refs
selection rule ref
unresolved coverage
```

Mutable branch names, `latest`, UI labels, or unversioned query names are not sufficient source-cut identities.

## 5. Coverage classes

V1 distinguishes source coverage explicitly rather than treating every query as complete.

Initial conceptual classes:

```text
ExactVersionedArtifact
OwningRecordSetWithQualifiedEnumeration
OwningSnapshotPrimitive
BoundedQueryObservation
PartialReplicaObservation
ExternallyAssertedCoverage
UnresolvedCoverage
```

The names describe evidence shape, not trust rank.

A local DHT query, cache, index, replica, or search result MUST NOT be upgraded to closed-world source coverage merely because it returned successfully.

## 6. Closed-world qualification rule

The exporter may claim source completeness only when the owning domain has a separately qualified theorem that the exact source primitive enumerates the complete relevant set for the declared subject/scope/profile.

```text
query returned N records
!= no other relevant records exist
```

When that theorem does not exist, the source remains partial or unresolved and the Phase-E envelope must preserve the limitation.

## 7. Selection profile is immutable

The exporter operates under one exact `selection_profile_ref` that freezes:

- required service revisions;
- required domain/source roles;
- required source coverage class per role;
- currentness policy per role;
- hard inclusion/exclusion semantics;
- unresolved-state behavior;
- object-count/size limits;
- canonical bridge profile version.

Changing any material selection rule creates a new profile revision and new evidence lineage.

## 8. Adoption is not inferred

The exporter MUST receive exact adopted service/profile/policy references from their owning authority paths.

It may verify their shape and owning-domain evidence through qualified adapters. It may not infer adoption from:

- popularity;
- newest timestamp;
- Symthaea recommendation;
- governance-like naming;
- model confidence;
- dashboard selection;
- local configuration without an owning authority theorem.

```text
recommended profile != adopted profile
```

## 9. Currentness is purpose-specific

A structurally valid object is not necessarily current enough for Phase-E use.

Every currentness decision binds:

```text
object/evidence identity
purpose/profile identity
evaluation time basis
freshness/currentness rule
result
evidence
```

The exporter must not replace `unknown time`, `stale`, `indeterminate`, or `currentness not evaluated` with current.

## 10. No single global freshness timestamp

Different source roles may have different qualified currentness rules.

V1 therefore forbids a shortcut such as:

```text
exported_at - record_timestamp < X
=> entire evidence cut is fresh
```

without source-specific semantics supporting it.

`exported_at` is export provenance, not evidence freshness.

## 11. Cross-source temporal skew

The evidence cut records enough source timing/currentness information to expose cross-source skew.

A service profile from one revision, inventory observation from another time window, and authority state from another source are not silently presented as one simultaneous observation.

A later model may reject, bound, or explicitly analyze skew. The exporter does not hide it.

## 12. Exact object selection

Selected objects are retained by immutable exact references and content identities where available.

The exporter must not pass only semantic labels such as:

```text
water-profile=current
energy-source=local
service-plan=approved
```

The bridge payload receives exact bounded projections plus a manifest sufficient to trace every authoritative input back to its exact selected object/evidence lineage.

## 13. Projection is lossy and must be declared

The Phase-E bridge intentionally projects only model-relevant fields. Projection therefore does not mean the canonical payload is a full copy of the authoritative object.

For every adapter, a frozen projection profile should state:

```text
source object/profile
fields/propositions consumed
fields omitted
unit/basis mapping
unknown/unresolved mapping
hard validation rules
```

An omitted source field cannot later be claimed as verified by the bridge merely because the containing source object was referenced.

## 14. Unknown preservation

Adapters MUST preserve epistemic uncertainty conservatively.

```text
missing field
!= zero
!= false
!= unavailable
!= not applicable
```

Only an owning-domain rule or frozen adapter mapping may convert a source state into bridge `Known`, `Unresolved`, `Unknown`, or `NotApplicable`.

## 15. Unit and basis mapping

No adapter may perform implicit unit/basis conversion.

Any required conversion must be either:

- already established by an exact authoritative/derived evidence object; or
- performed by a separately qualified deterministic conversion profile whose inputs, formula/profile and output identity are retained.

The exporter must never use a display-layer conversion as scientific identity.

## 16. Duplicate and conflict semantics

Multiple source objects claiming the same semantic role do not default to newest-wins or last-write-wins.

The selection profile must define one of:

```text
ExactSingleRequired
ExplicitOwningReducer
ConflictIsUnresolved
QualifiedSupersessionLineage
```

Ambiguous competing authoritative candidates fail closed or remain unresolved.

## 17. Supersession requires evidence

A later timestamp or higher revision string alone does not prove supersession.

A supersession path must be established by the owning domain's qualified identity/history semantics or an exact adopted registry/profile.

Historical evidence remains historical rather than being rewritten.

## 18. Negative/absence evidence

Absence claims require stronger evidence than positive record retrieval.

Examples:

```text
no unresolved permission
no other dependency
no active conflicting policy
no additional reserve commitment
```

MUST NOT be derived from a source that has not qualified the relevant closed-world enumeration/negative-query theorem.

If absence cannot be established, the exported state is unresolved rather than favorable.

## 19. Capacity and reserve anti-double-counting

The exporter must preserve exact source identities and commitments needed to prove that one physical stock/capacity/reserve is not exported into multiple incompatible model roles as independently available quantity.

Where an upstream domain already owns reservation/partition semantics, the exporter consumes those exact states rather than reconstructing allocation from labels.

## 20. Authority and physical state remain distinct

A resource may be physically present but unauthorized, or authorized but unavailable.

Adapters preserve these as separate propositions.

```text
physical availability != legitimate authority
legitimate authority != physical availability
```

The exporter cannot manufacture one from the other.

## 21. Ecology/quality/safety remain hard states

A Phase-E selection profile cannot convert violated or unresolved hard ecology, rights, quality, or safety gates into successful bridge availability because resilience utility is high.

Those states are inherited from their owning qualified domains/adapters.

## 22. Symthaea has no exporter mutation authority

Symthaea may propose:

- missing dependencies;
- alternative evidence to collect;
- candidate substitutions;
- suspicious conflicts;
- informative next observations.

Those outputs return as proposal/counterexample evidence.

They cannot silently alter the source cut, adopted profile, authoritative graph, or exported envelope used by the run that produced them.

## 23. Export freeze-before-run

One scientific/model run consumes one immutable exporter result.

```text
source cut
-> validated exporter receipt
-> canonical bridge payload
-> Symthaea import
-> model run
```

New records arriving after export create a new evidence cut/export lineage. They do not mutate the historical run's inputs.

## 24. Export receipt

A future exporter should emit a machine-readable receipt binding at minimum:

```text
exporter ProductHead/profile
selection_profile_ref
assessment cut identity
all source-cut identities
coverage classifications
currentness results/profile refs
selected object manifest
projection profile identities
unresolved coverage gaps
bridge profile identity
result/non-claims
```

The receipt is evidence of export-process conformance, not authentication or source truth by itself.

## 25. Exporter qualification campaign

The first campaign should include positive and adversarial fixtures for:

- complete exact versioned sources;
- partial DHT/replica observation that must not become complete;
- conflicting candidate records;
- unresolved supersession;
- stale source under one purpose but acceptable under another;
- unknown timestamps;
- source timing skew;
- omitted mandatory source role;
- duplicate physical stock exported into two roles;
- authority present / resource absent;
- resource present / authority unresolved;
- hard ecology/quality failure;
- changed selection profile after export;
- late-arriving evidence creating a new lineage;
- Symthaea proposal unable to mutate frozen export.

## 26. Coverage matrix

The exporter qualification receipt should expose each required role as a matrix rather than one boolean:

```text
role
owning domain/profile
coverage class
coverage theorem identity
currentness state
projection profile
selected object count
unresolved gaps
```

A dashboard may summarize this, but the machine theorem stays plural and inspectable.

## 27. Fail-closed export result

A selection profile may define some roles as optional, but required roles cannot disappear silently.

A future exporter result should distinguish at least:

```text
ExportReady
ExportReadyWithDeclaredUnresolvedInputs
ExportBlocked
```

Whether unresolved inputs are admissible for a particular simulation is campaign/profile policy. The exporter itself does not relabel them as known.

## 28. Relationship to bridge qualification

The exporter and bridge codec prove different things:

```text
exporter theorem:
selected authoritative/evidence inputs conform to frozen selection/coverage/currentness rules

bridge theorem:
validated semantic envelope is canonically encoded/decoded/committed
```

An end-to-end producer theorem requires both exact qualified identities.

## 29. Relationship to authentication

Source action signatures, DHT authorship, Xenia sessions, Q002 signatures, institutional credentials, or other authentication mechanisms remain owned by their respective systems.

The exporter may bind qualified authentication/authority evidence. It does not treat a content digest as authentication.

## 30. Deliberate non-claims

Exporter qualification establishes no universal global snapshot, no global DHT completeness, no truth of source observations, no universal freshness policy, no legitimacy of an institution merely from its identifier, no model correctness, no real-world resilience, no emergency policy, no procurement decision, and no physical-action authority.

Its narrow intended proposition is:

> for one exact selection profile and one exact evidence cut, the qualified exporter can demonstrate which authoritative/evidence objects it selected, what source coverage/currentness evidence justified those selections, which gaps remained unresolved, and which exact projection was supplied to the separately qualified canonical bridge.
