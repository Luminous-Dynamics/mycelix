# MYC-CAP-002F1 — Designated-current service evidence v1

Status: executable candidate; qualification is exact-head CI evidence, not this document.

## Purpose

Prevent a historically valid public-service snapshot from authorizing investor distributions indefinitely.

This profile answers one narrow question:

> Is this exact MYC-CAP-002F service receipt current relative to one supplied designated-current registry lineage, with no applicable invalidation/revocation event?

It does not evaluate the underlying service covenant. MYC-CAP-002F remains the authority for `ELIGIBLE/BLOCKED/NO_ACTIVE_CLAIM`.

## Core separation

```text
historical service validity
!= service evidence currentness
!= distribution eligibility
```

A snapshot may remain historically valid while currentness becomes `STALE`, `PENDING`, or `REVOKED`.

## No wall-clock authority

v1 implements only designated-current semantics.

It deliberately contains no timestamps, expiry dates, local clock reads, or age arithmetic.

```text
uses_local_wall_clock = false
```

Bounded-age semantics require a separately qualified time/epoch source and belong in a future profile.

## Designation semantics

The supplied designation binds:

- exact registry ID;
- exact project;
- exact currentness profile SHA-256;
- exact designated measurement ID;
- exact semantic SHA-256 of the service-gate receipt;
- registry epoch;
- designation state;
- authority/evidence references.

`ACTIVE` can yield `CURRENT` only when both the measurement ID and receipt digest match.

A different designated snapshot makes the older supplied receipt `STALE`; it does not rewrite its historical service result.

## Invalidation lineage

Within one designation, these event classes are monotonic:

```text
MaterialInvalidation -> at least STALE
PendingRemeasurement -> at least PENDING
RevokeEvidence       -> REVOKED
```

There is deliberately no `ClearInvalidation` operation in v1. Recovery requires a new measurement/designation lineage rather than silently re-authorizing the old evidence.

Events for another measurement do not invalidate this one.

## State precedence

When multiple conditions apply:

```text
REVOKED > PENDING > STALE > CURRENT
```

All applicable blockers remain visible in the receipt.

## Claim boundary

The receipt always fixes:

```text
claim_modified = false
```

Currentness failure cannot create principal, preferred return, default interest, impairment, or debt forgiveness.

## Exact parent

The executable child is stacked on exact MYC-CAP-002F candidate subject:

```text
45361d938d9a56f66e6eab360b04d773f54b1c8c
```

The profile also binds the service-gate profile SHA-256:

```text
18dc049978d41bfda3e2d5c48280ccaa37f7e03d5a04cbedc50af80db49e0937
```

## Fixture commitments

Canonical example file SHA-256:

```text
dc2d0dc4ed404251347110cf270d100aed4ed469c121ec771aaaf4854533444d
```

Frozen receipt file SHA-256:

```text
eb9d9c3306652a35e0948cb36a9d89f86a22223038d151010a0e5f1edd849e38
```

Currentness profile semantic SHA-256:

```text
b36fef31bdbe9b43458fbc4ce4a07b539037e37db7803f8b247fa07d81224f5a
```

Designation semantic SHA-256:

```text
abc54685ec6d568b382a2e5c95804928983379bbad608c57d01d42c07dde581f
```

Supplied service receipt semantic SHA-256:

```text
347cf7cc038ba918cade9e1e210871ddb6039440d9af5ea78c3745594acc8969
```

## Nonclaims

PASS would establish only designated-current/invalidation semantics for the supplied evidence lineage. It would not establish measurement truth, legal distribution restrictions, regulatory authority, optimal measurement cadence, external signature authenticity, infrastructure adequacy, or currentness of the designation registry itself outside the supplied qualified evidence.
