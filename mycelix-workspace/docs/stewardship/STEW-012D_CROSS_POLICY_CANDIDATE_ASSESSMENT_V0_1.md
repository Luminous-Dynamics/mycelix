# STEW-012D — Cross-Policy Candidate Assessment v0.1

## Status

Draft executable theorem profile.

## Purpose

STEW-012D is the first executable implementation of the STEW-012C cross-policy boundary. It compares action classifications produced by multiple STEW-012B policy-authority binding candidates for one exact STEW-001 representation without converting any structural candidate into verified authority, authorization, or final denial.

## Core theorem

```text
candidate says Permit
!= issuer authority verified
!= constraints satisfied
!= duties satisfied
!= authorized

candidate says Prohibit
!= issuer authority verified
!= globally denied

candidate says DeniedUnspecified
!= globally denied
```

The symmetry is intentional. Unverified positive authority must not grant access, but unverified negative authority must not create censorship either.

## Exact-target discipline

Every candidate assessed together must bind a STEW-003 policy whose exact target equals the requested exact representation. Target mismatch is rejected as malformed composition rather than silently filtered.

## Closed structural states

The v1 aggregate vocabulary is descriptive only:

- `NoCandidates`
- `UniformPermissionCandidates`
- `DivergentPermissionCandidates`
- `UniformProhibitionCandidates`
- `UniformUnspecifiedDenialCandidates`
- `MixedNonPermissionCandidates`
- `MixedCandidateDispositions`

These states describe the shape of candidate classifications. They establish no policy applicability, issuer authority, precedence, conflict resolution, or runtime permission.

Permission candidates are structurally uniform when their unresolved constraint and duty reference **sets** are equal after canonical sorting. Reference order therefore cannot manufacture a false conflict.

## No implicit reducer

STEW-012D does not implement newest/oldest wins, majority vote, reputation/MATL/stake weighting, institutional primacy, global prohibition dominance, most-restrictive/least-restrictive wins, or first/last writer wins.

## Runtime consequence

A caller may use this theorem to conclude that more evaluation is required. It may not use `UniformPermissionCandidates` as authorization or `UniformProhibitionCandidates` as final denial until later layers independently establish authority, applicability, precedence/composition, currentness, and unresolved constraint/duty satisfaction.

## Symthaea boundary

```text
model observes candidate agreement
!= authority agreement proven

model observes candidate disagreement
!= model may choose a winner
```

## Deliberate non-claims

No policy applicability, issuer authority, admission legitimacy, mandate validity, currentness truth, precedence, legal right, cultural authority, community consent, access authorization, final denial, AI-use permission, duty satisfaction, constraint satisfaction, or runtime capability is established.
