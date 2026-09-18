# MYC-CAP-002C1 — Claim registry currentness v1

Status: executable candidate over hosted-qualified claim registry; local preflight only until exact-head hosted qualification succeeds.

## Purpose

Separate conserved holder state at an exact claim-registry operation tip from proof that this exact tip is the currently designated holder snapshot.

```text
qualified claim-registry state
!= latest designated registry state

CURRENT holder snapshot
!= payment authority
!= commons-asset ownership
```

## Exact qualified input

Claim-registry subject:

`9f940f915e6aa2bf253f859045d104e80406dd4c`

Hosted qualification run:

`35140287981` — SUCCESS.

Canonical claim-registry receipt semantic SHA-256:

`ec4431b5e4ee5ab8c91a4b2a24285d617c9c37d81d65473c997a2633a0b8b628`

Claim-registry profile SHA-256:

`6011726fcb6af452a3c8d8d8027c000e9d324a2776a7739e4f22eb76de3ceb40`

Operation-history SHA-256:

`db65331686b023f8683cff201a608354c3a492d8baf483f5fa8c2b7099b92a92`

Operation-chain tip SHA-256:

`9a9a2c7a462f9a1b449891a63b7516f7e13460caaf1fda69d707a00938ebaa05`

## Frozen currentness profile

Profile semantic SHA-256:

`bf6c6ebbe015f646b2c581bb353ec4363c1f6af9965bd86796ac1de2f273fd75`

Designation semantic SHA-256:

`629d1b78611a101594c496c76137da28f32f571270f21e16652353cc4cbce786`

Registry:

`registry:capital-claim-currentness`

## States

`CURRENT | STALE | PENDING | REVOKED`

Precedence:

`REVOKED > PENDING > STALE > CURRENT`.

Closed event vocabulary:

`SupersedeTip | PendingReconciliation | MaterialInvalidation | RevokeEvidence`.

No local wall-clock time creates authority.

## Positive theorem

A positive result requires the exact qualified claim-registry subject and canonical receipt/profile/history/tip plus an ACTIVE designation naming those exact commitments and no stronger invalidation event.

Then only:

`claim_registry_currentness_state = CURRENT`.

The receipt also exposes a deterministic `holder_snapshot_sha256` over active claim total, active slices and holder totals. That digest is evidence identity only.

## Canonical commitments

Holder snapshot semantic SHA-256:

`48de5ec9d27ac4b74b2ef37a6e2584e52bb6a254f46f7f8638adf8f28a05a316`

Case file SHA-256:

`a644246d060ba4189c27f7d9d98ddab78e53e774c8a8e407fda2ba625fa5866e`

Receipt file SHA-256:

`c3fb63cfbf22475a0b7e5be7df65f19f35988784541c44a9b792d4290bb94762`

Receipt semantic SHA-256:

`ed6bbf29fa6dccab29d33bf6258ebeae137d81b473cd9a62e23fe4b61d410a4f`

Canonical empty event-history semantic SHA-256:

`4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945`

## Local preflight

The stdlib suite passes **31/31** locally, covering exact qualified subject/receipt/profile/history/tip binding, conservation and asset-boundary checks, stale/pending/revoked precedence, chain integrity, substitution attacks, authority contamination, and deterministic replay.

## Authority boundary

Every receipt fixes:

```text
payment_authority_established = false
asset_title_authority_established = false
constitutional_authority_established = false
operator_authority_established = false
uses_local_wall_clock = false
```

## H5 boundary

C1 proves only claim-registry snapshot currentness. It does **not** prove that the parent Return Envelope financial checkpoint is current. H5 must separately establish parent-financial currentness before using C1 as a current financial/holder dimension.

## Nonclaims

Even hosted PASS would not establish payment entitlement, legal negotiability, securities-law compliance, market value, tax treatment, lender priority, external holder identity/signature authenticity, commons-asset title, operator authority, constitutional governance authority, or currentness of the parent Return Envelope.
