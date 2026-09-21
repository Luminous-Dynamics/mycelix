# mycelix-evidence-core

`mycelix-evidence-core` is the dependency-light semantic waist for EPI-001.

This first tranche intentionally establishes **identity only**.

```text
EvidenceId
!= artifact authenticity
!= source reliability
!= observation validity
!= assertion entailment
!= claim support
!= evidence independence
!= scientific qualification
!= truth
!= application authority
```

## V1 role identities

- `ArtifactId`
- `ObservationId`
- `AssertionId`
- `ClaimId`
- `HypothesisId`
- `SourceId`
- `DerivationId`
- `EvidenceRelationId`
- `AssessmentId`

All use the exact profile `mycelix:epistemic-identity:v1` and bind the role tag into canonical identity bytes, so equal namespace/local-id strings in two semantic roles are not the same protocol identity.

## Canonical framing

The canonical commitment preimage is an ordered sequence of length-prefixed fields:

```text
profile
role
namespace
local-id
```

Field names and values are length-prefixed separately. Serde/JSON field order is therefore not identity authority.

The exposed SHA-256 commitment is only an identity commitment over those exact canonical bytes:

```text
identity commitment
!= signature
!= content digest
!= evidence validity
```

## Wire admission

Serde uses a closed `EvidenceIdWireV1` structure. Deserialization revalidates:

- profile;
- exact semantic role;
- namespace/local-id lexical bounds;
- lowercase 64-hex commitment shape;
- commitment equality with recomputed canonical bytes;
- unknown wire fields are rejected.

## Deliberate scope

EPI-001 contains no Holochain, network access, scraping, AI evaluation, source reputation, evidence-relation inference, scoring, truth classification, currentness, or action authority.

Those require separate exact subjects.