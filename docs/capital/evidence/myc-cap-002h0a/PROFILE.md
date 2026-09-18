# MYC-CAP-002H0A — Read-only H1/H2/H3 dimension projections v1

Status: local executable preflight complete; hosted qualification required for the exact published subject.

## Purpose

Provide a typed, read-only composition envelope over already-qualified H1, H2 and H3 receipts without rewriting or superseding those subjects.

Core rule:

```text
projection may normalize representation
projection may not improve evidence
projection may not add currentness
projection may not add authority
```

## Exact qualified source subjects

H1 stewardship governance: `cb6f2e88d81a7ff0fb0adcf36888f44c4fe8ed69`

Canonical H1 receipt semantic SHA-256: `ef3e4b8772d4476fe24568cd8c503db7dec192418cd4719a052d6b941d523544`

H2 public-service distribution: `7ba66c70be2df27b1d5deaa77a88ffdda96baa35`

Canonical H2 receipt semantic SHA-256: `bd78cf22a9899d0a816de80600d8f375e1d089f07b9ba64f3146b0bd32318e83`

H3 handback readiness: `f9d88b41a750d2bddfa8fa2bf26727f04a4998a6`

Canonical H3 receipt semantic SHA-256: `be1c02a199688e8aef4c6054abfef216cfffe830730f090fdb1be1db406e3045`

Projection profile semantic SHA-256: `9900b467a7f29393b30b61db85361649b5138644da323efd9b3129ebcda751e7`

## Projection mappings

H1:

```text
semantic_state = CURRENT
currentness_state = CURRENT
currentness_owner = INTEGRATED_SOURCE_THEOREM
authority_ceiling = GOVERNANCE_STATE_ONLY
```

H2:

```text
ELIGIBLE_CURRENT -> semantic ELIGIBLE / currentness CURRENT
BLOCKED_SERVICE  -> semantic BLOCKED_SERVICE / currentness CURRENT
NO_ACTIVE_CLAIM  -> semantic NO_ACTIVE_CLAIM / currentness CURRENT
STALE            -> semantic NOT_SEPARATELY_EXPOSED / currentness STALE
PENDING          -> semantic NOT_SEPARATELY_EXPOSED / currentness PENDING
REVOKED          -> semantic NOT_SEPARATELY_EXPOSED / currentness REVOKED
```

H3:

```text
semantic_state = exact H3 handback_readiness_state
currentness_state = NOT_ESTABLISHED
currentness_owner = H3A_REQUIRED
authority_ceiling = READINESS_ONLY
```

H3 currentness is never inferred from hosted qualification time or local wall clock.

## Bundle semantics

The canonical bundle requires exact project equality and exact source subject/receipt bindings for all three dimensions.

Every projection carries the original source subject, source receipt semantic digest, source receipt version, source nonclaims, normalized blockers, currentness ownership and an explicit closed authority ceiling.

Every projection and bundle fixes execution, payment-transfer, legal-title and constitutional authority false.

## Local preflight

The stdlib suite passes **26/26** locally. It covers H1/H2/H3 positive mappings, conservative H2 stale/pending/revoked decomposition, H3 currentness non-amplification, source-schema closure, authority contamination, exact frozen source binding, cross-project rejection, blocker preservation, authority ceilings and deterministic bundle replay.

Canonical case file SHA-256: `b91a0f931bfca4b3265972487b96ce5edfcfc434d280d60f7a41ea169daf919b`

Canonical receipt file SHA-256: `ad5b127cc15acafbde312b25caf482c2fcf934ab8f3c640417ca327cf7b60490`

Canonical receipt semantic SHA-256: `8e1b96d2fa041dc3f7eecda0ee9ff57f51552d2998c8d13f342ce05a5c4684ab`

Source-identity bundle SHA-256: `2ca7cc63a49f7c5dcb8f27eef780970987b79f8236b179619f04233670e3a10d`

## Nonclaims

PASS would not establish H1/H2/H3 for any new subject, H3 currentness, project transition readiness, custody acceptance/currentness, legal title, payment-transfer authority, execution authority, constitutional transition authority, or democratic legitimacy. It would establish only faithful typed projections of the exact qualified source receipts bound by the qualification workflow.
