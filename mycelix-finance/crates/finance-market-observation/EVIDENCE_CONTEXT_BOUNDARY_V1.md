# FIN-MKT-002A Evidence Context Boundary V1

## Purpose

This note freezes the interpretation context for `source_evidence_commitment` in FIN-MKT-002A.

A bare 32-byte value has no context-free meaning. In V1, every source-evidence commitment is interpreted under the exact `ProviderObservationRefV1.observation_profile` already bound into the observation identity.

```text
source_evidence_commitment
+ exact observation_profile
= evidence commitment interpretation context
```

The observation profile therefore owns, at minimum, the reviewed rules for:

- provider observation-ID namespace and scoping;
- source-payload selection/redaction rules;
- canonical evidence byte construction;
- digest/hash algorithm and parameters;
- any provider-version/schema assumptions needed to interpret the evidence;
- whether independently captured envelopes with identical normalized semantics are valid alternate evidence under that same profile.

## Profile drift

If any authority-significant evidence-commitment rule changes, the observation profile revision and/or digest must change.

Because the exact observation profile is part of the observation identity:

```text
changed evidence canonicalization/hash profile
-> changed observation profile ref
-> changed observation identity
```

No implementation may silently reuse the same observation profile while changing the meaning of `source_evidence_commitment`.

## Same-profile alternate evidence

Within one exact observation profile, independently captured evidence can legitimately produce a different source-evidence commitment while supporting the same normalized semantics.

That case is intentionally represented as:

```text
same identity
+ same semantic commitment
+ different evidence-binding commitment
-> SameSemanticsDifferentEvidence
```

This is not a semantic conflict.

## Semantic conflict

If normalized FIN-MKT semantics differ under the same observation identity:

```text
same identity
+ different semantic commitment
-> ConflictingReuse
```

Additional evidence cannot erase or resolve that conflict by arrival order.

## No credential binding

Evidence commitment inputs must exclude reusable bearer credentials, OAuth tokens, session cookies, API secrets, or private signing material.

A protected adapter may retain raw provider evidence according to its storage/privacy profile, but portable FIN-MKT observations carry only the profile-bound commitment.

## Claim boundary

Binding evidence under an exact observation profile establishes deterministic provenance interpretation only.

```text
profile-bound evidence commitment
!= provider truth
!= currentness
!= account ownership
!= financial authority
!= settlement
```
