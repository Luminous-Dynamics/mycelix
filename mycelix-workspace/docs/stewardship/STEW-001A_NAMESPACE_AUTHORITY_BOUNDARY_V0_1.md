# STEW-001A — Namespace Authority Boundary v0.1

Status: preregistration / authority firewall

## Purpose

Freeze the distinction between a syntactically valid stewarded-subject identifier and authority to speak for the namespace encoded in that identifier.

STEW-001 deliberately admits opaque namespaced identifiers such as `community:example:work-1`. That is structural identity only. Without an explicit authority boundary, later code could accidentally read a namespace-shaped string as evidence that the caller represents the named community, institution, archive, creator, or protocol.

## Core theorem

```text
well-formed identifier
!= namespace registration
!= namespace control
!= stewardship authority
!= cultural authority
!= access authority
```

A subject identifier may name a namespace without establishing any right to control that namespace.

## Namespace claims

A future executable theorem should represent a namespace authority claim separately from `StewardedSubjectIdV1`:

```text
NamespaceAuthorityClaimV1
├── namespace
├── claimant
├── scope
├── basis
├── evidence_refs
├── valid_from
└── valid_until
```

The claim is evidence-bearing state, not an automatic authority grant.

## No string-derived authority

The following are forbidden inference patterns:

```text
subject_id starts_with "community:foo:"
-> caller represents community foo
```

```text
subject_id contains DID X
-> DID X approved the subject
```

```text
namespace resembles Local Contexts / museum / archive / DOI / ORCID syntax
-> external authority accepted the record
```

Namespace syntax and namespace authority are independent propositions.

## External namespaces

For external namespaces, Mycelix should preserve the external identifier and verification evidence rather than minting a look-alike identifier that could be mistaken for an external authority's own record.

Examples include DIDs, ORCID, DOI, ARK, Handle, Local Contexts identifiers, archival identifiers, and other community or institutional namespaces.

```text
Mycelix reference to external identifier
!= external issuer attestation
```

Where an external issuer provides cryptographically or otherwise verifiable evidence, that evidence may be recorded separately and validated under a versioned interoperability profile.

## Namespace lifecycle

Namespace authority may be:

- active;
- delegated;
- jointly administered;
- contested;
- expired;
- revoked;
- unresolved.

Revocation of namespace authority does not rewrite historical identifiers or provenance.

```text
future namespace authority revoked
!= historical identifier erased
```

## Collision and squatting

A first-seen identifier must not automatically create exclusive namespace authority.

```text
first writer
!= namespace owner
```

Likewise, generic reputation, token stake, MATL score, governance voting weight, or compute contribution must not automatically resolve namespace disputes.

## Relationship to STEW-004 / STEW-005

Namespace authority is narrower than stewardship authority. A community may control a namespace without having stewardship rights over every object named inside it, and a legitimate steward may reference an object without controlling the surrounding namespace.

```text
NamespaceAuthority
!= StewardshipAuthority
```

Contested namespace authority should use the same no-automatic-winner discipline intended for STEW-005.

## Qualification direction

A later executable child should demonstrate at minimum:

```text
valid canonical ID alone
-> no authority result

valid namespace claim + admitted evidence
-> scoped namespace authority candidate

expired/revoked claim
-> no current authority

conflicting claims
-> contested/unresolved
-> no automatic winner
```

## Deliberate non-claims

This document establishes no namespace ownership, trademark right, cultural legitimacy, community representation, stewardship, copyright, access right, external-issuer endorsement, DID control, or legal authority.

It only freezes the rule that identifier syntax must never silently manufacture authority.
