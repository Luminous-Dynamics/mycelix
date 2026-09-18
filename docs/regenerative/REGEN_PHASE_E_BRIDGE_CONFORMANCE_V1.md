# REGEN Phase-E Cross-Repository Bridge Conformance v1

Status: preregistration / conformance-boundary hardening only

Program: Luminous-Dynamics/mycelix#940

Parent ownership contract: #1523 / `b80afbf44ea378d14ce4949971be8888be95331d`

Related Mycelix semantics: REGEN-040..047, especially #1431 / REGEN-042 and #1519 / REGEN-042A

Related Symthaea execution profile: Luminous-Dynamics/symthaea#3817 and staged REGEN-042S successors

## 1. Purpose

The Phase-E ownership contract prevents Mycelix and Symthaea from becoming competing authorities. This successor freezes the next question:

> how do we demonstrate that a cross-repository adapter preserved the meaning of an authoritative Mycelix snapshot when Symthaea consumes it?

The bridge itself is a scientific boundary and must therefore be versioned, bounded, canonical, replayable, and independently testable.

Core theorem:

```text
exact authoritative snapshot
+ frozen bridge schema
+ canonical encoding
+ content commitment
+ producer conformance oracle
+ consumer conformance oracle
+ shared rejection corpus
= reviewable meaning-preserving bridge instance
```

not:

```text
bridge decodes
= upstream facts are true
= model is valid
= receipt is authenticated
= recommendation is authorized
= physical action is permitted
```

## 2. Bridge schema is not an internal Rust layout

The interchange contract MUST be versioned independently from Mycelix and Symthaea internal types.

```text
Mycelix internal structs
!= bridge wire schema
!= Symthaea internal model structs
```

A refactor that preserves the bridge bytes/semantics need not create a new bridge revision. A semantic bridge change does.

The bridge must not use `Debug`, incidental serde field order, pointer layout, compiler ABI, hash-map iteration order, or another unstable internal representation as its scientific identity.

## 3. Version identity

Every payload MUST bind an explicit protocol identity at minimum:

```text
protocol_id
schema_version
canonicalization_version
```

Unsupported major/new semantic versions fail explicitly.

An older consumer MUST NOT silently parse a newer payload as if unknown fields or enum variants did not exist when those additions may alter meaning.

```text
unsupported schema
!= old schema with ignored meaning
```

## 4. Domain separation

Canonical commitments must use an explicit domain/version prefix so bridge bytes cannot be confused with another object class.

Conceptually:

```text
mycelix.regen.phase-e.assessment-envelope/v1\0
+ canonical_payload_v1
```

The exact bytes are implementation work, but the domain/version separation is normative.

A digest valid for a state commitment, qualification receipt, model receipt, or unrelated JSON object must not be replayable as a bridge commitment merely because the raw payload bytes happen to match.

## 5. Assessment envelope identity

A v1 assessment envelope should bind enough exact identity to make hidden re-resolution impossible.

Conceptually:

```text
AssessmentEnvelopeV1 {
    envelope_id,
    schema_version,
    canonicalization_version,
    producer_repository,
    producer_commit,
    snapshot_revision,
    service_profile_refs,
    dependency_graph_ref,
    evidence_snapshot_ref,
    authority_profile_refs,
    ecology_profile_refs,
    quality_safety_profile_refs,
    campaign_ref,
    assessment_horizon,
    time_basis,
    randomness_policy_ref,
    object_manifest,
    payload_commitment,
}
```

This list is a semantic target, not yet a claim that these exact field names are final.

## 6. Exact references, not mutable names

A scientific envelope must not identify authoritative input solely by a mutable branch, tag, human label, URL, or `latest` alias.

Where Git-backed identity is used, exact commits/blobs or content digests must be available.

Where domain records are not Git-backed, the bridge must carry an immutable revision identifier plus the relevant content/evidence commitment where the owning system supports one.

```text
name = "current-water-profile"
```

alone is insufficient.

## 7. Content-addressed envelope

Every accepted envelope must have a commitment computed over the exact canonical bytes of the semantic payload.

The commitment is evidence of byte identity, not truth or authenticity.

```text
matching digest
=> matching canonical payload bytes under the declared scheme
```

not:

```text
matching digest
=> payload was authorized
=> source evidence was true
```

Authentication remains a separate proposition, compatible with later REGEN-Q002/Xenia-backed signing.

## 8. Manifest closure

Every semantic reference used by the model must be either:

1. included in the envelope/object bundle with an exact commitment; or
2. represented as an explicit immutable external reference whose resolution policy is frozen outside the scientific run.

The bridge must not contain dangling semantic references that the consumer silently resolves from mutable live state.

A closure check should answer:

```text
which exact object did this reference mean for this run?
```

without network access.

## 9. No hidden live lookups

Once the envelope is frozen, Symthaea must not perform a live Mycelix lookup that changes the semantic input while retaining the same envelope/run identity.

```text
frozen envelope A + model M -> run A
updated authority/evidence -> new envelope B -> run B
```

The model may report that fresh/current authoritative state is unavailable; it may not manufacture freshness by re-querying mid-run.

## 10. Snapshot freshness remains upstream state

A freshly created envelope can contain stale evidence.

Therefore:

```text
fresh serialization
!= fresh evidence
```

Currentness/freshness outcomes required by adopted profiles must remain explicit input semantics and survive the bridge.

## 11. Unknown-state preservation

The bridge must preserve the epistemic distinction among states such as:

```text
Known(value)
Unresolved(reason)
Unknown
NotApplicable
Unavailable
Failed
```

where those distinctions exist upstream.

No adapter may coerce an unresolved/unknown value into zero, false, available, empty list, or a default enum variant merely because the transport library expects a concrete value.

## 12. Missing field != negative fact

Absence has no generic semantic meaning.

```text
field omitted
!= false
!= zero
!= none exists
!= not required
```

If optionality is meaningful, the schema must freeze that meaning explicitly.

Semantic unknowns should not be represented only through transport-level field omission when that would make them indistinguishable from schema evolution or parser behavior.

## 13. Quantity semantics

A numeric bridge field is incomplete without quantity semantics.

At minimum, relevant quantities preserve:

```text
value representation
unit identity
basis identity where applicable
resolution/uncertainty class where applicable
support/scope where applicable
```

The bridge must not silently perform conversions such as:

```text
wet mass -> dry mass
stock -> flow
power -> energy
nominal capacity -> usable capacity
modeled quantity -> observed quantity
```

## 14. No unqualified floating-point scientific identity

The v1 resilience bridge should prefer exact bounded integer/fixed-decimal/rational representations for quantities that participate in conservation, capacity, service-floor, timing, or comparison identity.

If a floating-point value is ever admitted, its encoding, finiteness rules, signed-zero/NaN/infinity policy, rounding semantics, and canonical byte representation must be frozen first.

A convenient language `f64` must not become the implicit wire theorem.

## 15. Time semantics

The bridge must distinguish the relevant clocks/indices rather than flatten them into one timestamp.

Potential fields include:

```text
observation time
snapshot/revision time
campaign start
model tick/time basis
assessment horizon
currentness evaluation time
validity interval
```

Wall-clock timestamp presence does not itself establish trusted time.

Campaign ticks must not be silently interpreted as Unix seconds or vice versa.

## 16. Half-open interval convention

Where REGEN-042S uses half-open campaign intervals, the bridge profile must carry the same semantic convention explicitly.

```text
[start, end)
```

must not be translated into an inclusive interval by another library.

An instantaneous event remains distinct from an empty interval.

## 17. Adopted floor != scenario demand

The bridge must carry adopted service requirements separately from modeled/scenario demand where both are present.

```text
adopted floor
!= scenario demand
!= delivered service
```

A demand-shock field cannot alias the adopted minimum service requirement.

This protects the Mycelix authority boundary and the Symthaea S1B model theorem simultaneously.

## 18. Authority/profile references remain opaque to the model core

Symthaea may validate the shape and exact identity of upstream authority/ecology/quality references and may execute a separately delegated deterministic gate evaluator.

The bridge must not convert a reference into `authorized=true` merely because it arrived from Mycelix.

Likewise, the model cannot create authority by emitting a feasible path.

## 19. Dependency identity preservation

A Mycelix dependency node/edge identity must survive translation exactly.

Symthaea may create internal indexes for efficient execution, but model receipts and counterexamples must be able to echo the upstream dependency identity without lossy reverse lookup.

```text
internal index 42
```

alone is not a sufficient cross-repo evidence identity.

## 20. Failure-domain preservation

Shared-failure-domain membership is part of resilience semantics and must survive adapter translation without deduplication or flattening that changes overlap.

Two providers that share one source/corridor/grid/software/skill domain cannot emerge from the bridge as independent merely because the consumer stores providers separately.

## 21. Deterministic ordering is representation, not semantic priority

Canonical encoding requires deterministic ordering of unordered sets/maps.

That ordering must not create a physical, institutional, dispatch, or shock-priority order unless the schema explicitly says so.

```text
canonical sort order
!= execution priority
```

## 22. Duplicate rejection occurs before map collapse

A producer or consumer that first inserts elements into a map and thereby overwrites duplicate IDs can hide invalid input.

Duplicates with identity semantics must be detected before canonical map materialization.

At minimum the first conformance corpus should attack duplicate:

- service IDs;
- dependency IDs;
- stock IDs;
- failure-domain IDs;
- profile/reference IDs where uniqueness is required;
- object-manifest entries.

## 23. Bounded resource profile

The bridge must define structural limits before adversarial or accidental input size becomes a denial-of-service path.

A future executable profile should bound at least:

```text
payload bytes
object count
services
dependencies
edges/fanout
stocks
failure domains
string/identifier bytes
reference-list sizes
```

Exceeding a bridge limit is invalid input, not a modeled resilience failure.

No silent truncation/pruning is allowed to fit a limit.

## 24. Producer conformance oracle

Mycelix should own a small implementation-independent producer oracle or known-answer fixture generator for the bridge contract.

For each accepted fixture it should freeze:

```text
semantic fixture
canonical bytes
canonical SHA-256 commitment
expected object manifest
expected schema/version identity
```

The oracle should be simple enough to inspect independently from production adapter code.

## 25. Consumer conformance oracle

Symthaea must independently demonstrate that it consumes the same fixture as the same semantics.

Where a canonical encoder exists on the consumer side, decoding then canonical re-encoding an accepted fixture must produce the exact expected canonical bytes.

Where the execution model intentionally transforms the input into internal state, the model-state receipt must at minimum bind the exact original bridge commitment and pass semantic field-by-field known-answer checks.

## 26. Shared negative corpus

The same rejection fixtures should be exercised in both repositories where applicable.

Initial negative cases should include at least:

1. unknown protocol ID;
2. unsupported schema major/version;
3. unknown canonicalization version;
4. digest mismatch;
5. duplicate identity before map materialization;
6. dangling object-manifest reference;
7. over-budget payload;
8. invalid/non-canonical identifier;
9. unit mismatch;
10. basis mismatch;
11. quantity overflow;
12. hidden/non-finite float if floats are forbidden;
13. invalid/reversed campaign horizon;
14. interval convention violation;
15. missing explicit unresolved state;
16. service-floor/scenario-demand aliasing;
17. mutable/non-exact upstream revision where exact identity is required;
18. unsupported newer enum/effect kind;
19. object bytes inconsistent with declared commitment;
20. parser acceptance followed by semantically different canonical re-encoding.

A fixture rejected by one side but silently accepted by the other is a bridge conformance failure.

## 27. Anti-downgrade rule

A consumer that does not support a new semantic feature must not coerce the payload into an older representation and continue under the old schema identity.

It should return an explicit unsupported-schema/unsupported-feature result.

A migration to an older schema, if ever supported, must be a separately versioned transformation with its own evidence—not hidden parser behavior.

## 28. Capability negotiation occurs before evidence execution

Runtime protocol negotiation must not mutate the meaning of a frozen scientific run after its identity is established.

If producer and consumer cannot agree on one exact bridge profile, no joint evidence theorem exists for that run.

## 29. Model receipt binds exact bridge input

Every Symthaea Phase-E model receipt should bind at least:

```text
bridge protocol/schema version
canonicalization version
input commitment
object-manifest commitment or equivalent
model revision
limits profile
campaign revision
```

The receipt must not identify only a human-friendly scenario name.

## 30. Input commitment != model-state commitment

The cross-repo envelope commitment and Symthaea internal canonical model-state commitment are separate objects.

```text
bridge input commitment
!= model-state commitment
```

The import receipt should bind both and prove which transformation implementation connected them.

This lets reviewers distinguish bridge corruption from later model-state bugs.

## 31. Import receipt

Before shock execution, Symthaea should be able to emit an import/conformance receipt conceptually containing:

```text
ImportReceipt {
    bridge_commitment,
    schema_version,
    adapter_revision,
    model_revision,
    resulting_model_state_commitment,
    preserved_reference_census,
    conformance_assertions,
    unresolved_items,
}
```

A successful import is not yet a successful resilience campaign.

## 32. Counterexamples retain bridge identity

Any counterexample, minimal witness, missing-dependency candidate, or invariant failure returned to Mycelix must bind the exact bridge/input commitment from which it was discovered.

Otherwise later authoritative revisions can make the witness ambiguous.

## 33. No reverse authority inference

A consumer-side conformance PASS cannot establish that the upstream record was legitimately adopted.

```text
bridge well-formed
!= authority valid
```

Similarly, a signature/authentication PASS cannot establish the scientific truth of the content.

## 34. Authentication remains separable

The v1 conformance theorem is intentionally compatible with detached receipt/input authentication.

Potential future composition:

```text
canonical bridge commitment
+ Q002/Xenia-backed authentication evidence
+ issuer/key lifecycle evidence
```

But the base bridge parser/semantic validator must remain independently testable without network access or possession of private keys.

## 35. Privacy minimization

Cross-repo scientific envelopes should carry only information required for the declared analysis.

Opaque/revision identifiers are preferred where Symthaea does not need personally identifying or otherwise sensitive source content.

A model does not gain authority to ingest broader personal/domain data merely because it can technically parse it.

## 36. Offline replay

A retained accepted envelope plus its referenced immutable bundle should be sufficient to replay the import/conformance step offline.

Network access, mutable Holochain queries, live databases, or external APIs do not belong inside the deterministic conformance theorem.

## 37. Producer and consumer qualification remain distinct

Mycelix producer qualification may prove:

- exact schema implementation;
- canonical byte generation;
- closure/limits validation;
- known-answer commitments;
- negative-corpus rejection.

Symthaea consumer qualification may prove:

- exact parser/schema support;
- semantic preservation;
- limits enforcement;
- known-answer import/model-state mapping;
- negative-corpus rejection.

One PASS does not manufacture the other.

## 38. Joint conformance theorem

Only after both sides independently pass should an integration campaign claim:

```text
exact Mycelix producer
+ exact bridge fixture bytes
+ exact Symthaea consumer
+ identical expected commitment
+ accepted shared positive corpus
+ rejected shared negative corpus
= bridge conformance PASS for that exact pair
```

This theorem remains narrower than model validity or real-world resilience.

## 39. Version migration creates new evidence lineage

Changing any semantic field meaning, canonicalization rule, quantity/time representation, unknown-state grammar, or compatibility behavior creates a new bridge version or explicitly qualified migration.

Historical receipts keep their original bridge identity.

They are not silently reinterpreted under the newest schema.

## 40. First executable tranche

The first implementation should remain deliberately small:

```text
Mycelix:
  bridge schema/types
  canonical encoder
  semantic validator
  4-6 accepted known-answer fixtures
  shared rejection corpus

Symthaea:
  independent decoder/validator
  import mapping into immutable model state
  exact input-commitment echo
  independent canonical/semantic oracle checks

Integration:
  byte-identical positive vectors
  rejection parity
  unsupported-version test
  unknown/unit/time preservation tests
```

Do not begin with Holochain networking, streaming federation, live synchronization, adaptive models, signatures, dashboards, or physical control.

## 41. Initial positive fixtures

The first accepted fixture family should include at least:

1. one service + one direct dependency;
2. one service with explicit unresolved dependency;
3. primary + fallback sharing one failure domain;
4. stock-backed provision with protected and committed reserve state;
5. adopted floor distinct from larger scenario demand;
6. deterministic REGEN-042A shock-campaign reference with half-open horizon semantics.

Each fixture should have frozen canonical bytes and a known-answer SHA-256 commitment.

## 42. Metamorphic invariants

Where semantics are unordered, these changes should preserve canonical identity:

- input map insertion order;
- equivalent ordering of set members;
- non-semantic whitespace in source fixture representations before canonicalization.

These changes must alter identity when semantic:

- service/dependency ID;
- adopted floor;
- scenario demand;
- unit/basis;
- unknown/resolution state;
- campaign revision;
- horizon;
- failure-domain membership;
- evidence/profile revision;
- schema/canonicalization version.

## 43. Deliberate non-claims

REGEN Phase-E bridge conformance v1 establishes no real service sufficiency, no disaster forecast, no hazard probability, no scientific truth of imported evidence, no valid rights/ecology/quality determination, no model validity beyond the tested import semantics, no policy/adoption authority, no emergency power, no procurement authority, no authentication unless separately proven, and no physical-action authority.

Its proposition is narrow:

> an exact versioned, bounded, content-addressed bridge can be independently qualified so Mycelix and Symthaea agree on the meaning and identity of the frozen Phase-E input they exchange, without allowing transport or adapter behavior to manufacture authority, freshness, certainty, or scientific truth.
