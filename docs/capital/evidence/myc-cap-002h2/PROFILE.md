# MYC-CAP-002H2 — Current public-service distribution composition v1

Status: executable candidate; local preflight 22/22 PASS. Hosted qualification must execute on the exact review subject.

## Purpose

Compose the hosted-qualified MYC-CAP-002F service gate and MYC-CAP-002F1 service-currentness receipts into one narrow #1020 dimension:

```text
public_service_distribution_state
```

Core separation:

```text
historically ELIGIBLE != currently eligible
CURRENT evidence != service covenant PASS
current service eligibility != payment authority
```

## Qualified inputs

Exact F subject:

`92fe8408b52ce1b63808201e9f70ab5280a6f489`

Hosted F run:

`35136881114`

Exact F1 subject:

`caf653560310c9d47b036ef88a2207bff6d4b06f`

Hosted F1 run:

`35138437073`

The positive pair must agree on project, measurement ID, service-gate profile, service receipt semantic digest, and service eligibility. F1 must name the exact qualified F subject.

## States

```text
ELIGIBLE_CURRENT
BLOCKED_SERVICE
STALE
PENDING
REVOKED
NO_ACTIVE_CLAIM
UNSUPPORTED
```

Currentness states take precedence over historical eligibility. F1 `STALE`, `PENDING`, or `REVOKED` therefore prevents current eligibility without rewriting the historical F receipt.

## Permanent stale-subject regression

The superseded F subject:

`45361d938d9a56f66e6eab360b04d773f54b1c8c`

must be rejected even if all other service bytes appear compatible. Git parentage and semantic subject identity are separate claims and both must be correct.

## Authority boundary

Every H2 receipt fixes:

```text
claim_modified = false
execution_authority_established = false
payment_authority_established = false
legal_distribution_authority_established = false
```

H2 does not authorize a transfer of funds. It establishes only the current state of the frozen service/distribution covenant relative to qualified currentness evidence.

## Local preflight

The stdlib suite passes **22/22** locally, including current/blocked/stale/pending/revoked/no-claim states, exact F subject binding, project/measurement/profile/receipt/eligibility substitution, claim mutation, local-clock authority, raw-input substitution, authority-amplifying unknown fields, deterministic blocker ordering, strong nonclaims, and deterministic replay.

Canonical fixture file SHA-256:

`14cedf973bcfa09d7e7d2fbf1bfbcfc64649a0949f503a0f058177b0d243e410`

Canonical receipt file SHA-256:

`9b072799ca1a89a5a0098e94bfd97f0b1cb9a23d8ee3cacb28114eebb2a013ba`

## Nonclaims

PASS would not establish fair tariffs under law, measurement authenticity, utility/regulatory compliance, infrastructure engineering adequacy outside F, solvency, payment authorization, handback readiness, legal distribution entitlement, democratic legitimacy, or execution authority.
