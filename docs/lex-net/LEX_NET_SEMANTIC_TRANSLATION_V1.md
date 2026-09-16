# LEX-NET Semantic Translation Fidelity v1

Status: executable research contract; no legal, recognition, authority, or external-effect authority claimed.

Qualified parent: `4dadc07eecf8eab67d50d8526dd7e6d3015de9a7` (LEX-NET-001 R2 exact-head qualification PASS).

## Purpose

LEX-NET-017 defines a deterministic translation boundary for mapping evidence between external standards, vocabularies, schemas, jurisdictional profiles, and Mycelix envelopes without silently changing meaning.

The governing theorem is:

```text
syntactic transform
!= semantic equivalence
!= factual truth
!= legal equivalence
!= recognition
!= authority equivalence
```

A successful parser or schema conversion is not enough. Every qualified transform must expose what was preserved, re-expressed, narrowed, widened, aggregated, decomposed, derived, dropped, left partial, left ambiguous, preserved as conflict, or unsupported.

## Scope

This tranche freezes a pure translation relation oracle and golden corpus. It performs no network access, registry lookup, legal analysis, AI inference, recognition, authority grant, capability issuance, or external effect.

The translator consumes exact source/target/mapping/profile inputs. It never infers a stronger semantic relation merely because target output is syntactically valid or round-trips to equivalent bytes.

## Translation receipt minimum

A translation receipt binds, where meaningful:

- source profile identity/version and source evidence commitment;
- target profile identity/version;
- exact mapping identity/version/commitment;
- exact mapping qualification state;
- exact source and target claim/field selectors;
- field-level semantic relation records;
- preserved units, currency, precision, timezone, jurisdiction, identifier scheme, qualifiers, reservations, and cardinality;
- unmapped source information;
- synthesized or derived target information with derivation provenance;
- assumptions/defaults;
- conflicts/ambiguities;
- explicit overall semantic relation;
- exact target-use policy and whether that policy permits the relation;
- explicit nonclaims.

The receipt is evidence about a transform. It is not a `RecognitionReceipt`, `AuthorityGrant`, capability, legal opinion, or external-effect receipt.

## Semantic relation classes

LEX-NET-017 freezes these exact relation classes:

- `Lossless`
- `RepresentationOnly`
- `Narrowing`
- `Widening`
- `Aggregation`
- `Decomposition`
- `Derived`
- `Dropped`
- `Partial`
- `Ambiguous`
- `ConflictPreserving`
- `Unsupported`

A transform may carry field-level relations more specific than its overall relation, but a lossy or ambiguous transform may never be reported as `Lossless`.

## Translation dispositions

The v1 oracle emits exactly:

- `Translated` — the relation is deterministically established and explicitly permitted for the exact target use;
- `Refused` — the transform is unsupported, violates a hard semantic invariant, or uses a relation not permitted for the exact target use;
- `Indeterminate` — the required semantic relation cannot be deterministically established from the frozen mapping evidence.

`Translated` does not mean semantic identity. Its receipt still reports the exact relation class.

## ST-001 — unknown is not false

`unknown`, `false`, `zero`, empty, absent, and not-applicable are distinct states unless the exact mapping contract proves a specific equivalence. A transform that collapses unknown into a positive or negative assertion is refused.

## ST-002 — syntax validity is not semantic equivalence

Schema-valid target output proves only that target syntax accepted the representation. It does not establish that source and target claims mean the same thing.

## ST-003 — round-trip equality is representation evidence only

Round-trip reconstruction can support a `RepresentationOnly` claim when the semantic contract independently establishes equivalence. Byte/value round-trip success alone cannot upgrade a widening, narrowing, ambiguity, or dropped-context transform into `Lossless`.

## ST-004 — contextual dimensions are preserved or disclosed

Units, currency, precision, timezone/offset, jurisdiction, identifier schemes, qualifiers, reservations, exclusions, code-list identity, cardinality, and applicable profile/version are semantic context. Dropping them silently is forbidden.

## ST-005 — narrowing and widening are different

A target that represents fewer source states is `Narrowing`; a target that asserts or permits more states is `Widening`. Neither is `Lossless`, and target-use policy decides whether the exact relation is permitted.

## ST-006 — many-to-one and one-to-many mappings are explicit

Aggregation and decomposition are first-class relations. They must bind the exact source and target selectors and cannot hide collisions, fan-out assumptions, or information loss.

## ST-007 — derivation requires provenance

A derived target value must bind the deterministic derivation rule and exact source inputs. Model confidence, reputation, similarity, or an AI-generated guess cannot substitute for a qualified deterministic derivation.

## ST-008 — ambiguity and conflicts survive translation

When multiple source interpretations remain possible, the result is `Ambiguous` or `Indeterminate`; it is not guessed. Conflicting source facts may be preserved as `ConflictPreserving` evidence rather than collapsed by last-write-wins.

## ST-009 — profile and mapping versions are exact

Draft, newer, older, jurisdiction-specific, or vendor-specific profile variants may not be silently interpreted as another version. Unsupported versions are explicitly refused.

## ST-010 — target-use policy gates lossy relations

A transform relation can be deterministically correct yet still unsuitable for a particular downstream use. The exact target-use policy must explicitly permit non-lossless relations; permissive defaults are forbidden.

## ST-011 — translation grants no stronger institutional status

Translation never establishes factual truth, issuer trust, legal equivalence, treaty applicability, local recognition, local authority, or external-effect authority.

## ST-012 — evaluator is pure over frozen inputs

The reference oracle performs no network access, ambient-clock read, AI inference, mutable-registry lookup, or external side effect. New external facts require a new frozen input and receipt.

## Hard refusal examples

The v1 corpus refuses, at minimum:

- `unknown -> false`;
- a missing source field defaulted into a stronger positive target claim;
- incorrect or unqualified unit/currency conversion;
- timezone loss that can alter event ordering;
- removal of an identifier scheme where collisions become possible;
- removal of a restrictive qualifier or reservation;
- use of an unsupported profile version;
- unqualified AI-proposed field equivalence;
- a schema-valid output falsely labeled semantically equivalent;
- round-trip success used to hide state collapse.

## Receipt relationship

A `TranslationReceipt` is a typed evidentiary stage. It may later be consumed by quarantine and recognition logic, but it cannot be relabeled as a stronger receipt type.

```text
OriginVerificationReceipt
    -> TranslationReceipt
    -> QuarantineReceipt
    -> RecognitionReceipt
```

Each arrow requires its own qualified transition. Translation does not skip quarantine or recognition.

## Explicit nonclaims

A LEX-NET-017 PASS does not establish:

- factual truth of source claims;
- semantic equivalence beyond the exact frozen relation;
- legal equivalence;
- regulatory acceptability or compliance;
- treaty or convention applicability;
- local recognition;
- local authority or an `AuthorityGrant`;
- a capability lease;
- external-effect authority;
- production readiness.

It proves only that the exact frozen synthetic translation corpus is classified and gated according to this contract.
