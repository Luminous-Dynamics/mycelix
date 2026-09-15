# MYC-CAP-002F — Public-Service Distribution Gate v1

Status: executable candidate; qualification is exact-head CI evidence, not this document.

## Purpose

This profile proves a narrow distribution condition for Capital-to-Commons infrastructure:

> a valid investor claim may remain outstanding while discretionary distributions are blocked because the project is below a frozen public-service covenant.

The theorem is intentionally separate from Return Envelope accounting.

## Core separation

```text
valid financial claim
!= current distribution eligibility
!= acceptable public service
```

The gate never edits the parent claim. It only emits current distribution eligibility and exact blockers from supplied evidence.

## Frozen dimensions

v1 supports six bounded dimensions:

- service coverage;
- service reliability / uptime;
- affordability / tariff ceiling;
- reserve coverage;
- deferred-maintenance ceiling;
- continuity/resilience state.

The profile freezes thresholds; the verifier does not invent fairness or engineering requirements.

## Eligibility semantics

If the parent financial receipt has a positive remaining claim:

```text
no blockers -> ELIGIBLE
one or more blockers -> BLOCKED
```

If the parent remaining claim is zero:

```text
NO_ACTIVE_CLAIM
```

The result contains an ordered blocker set so multiple simultaneous failures remain visible.

## No hidden claim growth

The receipt fixes:

```text
claim_modified = false
```

Blocking distribution is not an impairment and does not add default interest, penalty return, management fees, or recoverable capital. Any future claim modification must be an explicit event in the financial theorem under its own qualified profile.

## Evidence boundary

The snapshot binds:

- exact project;
- exact service profile SHA-256;
- exact parent transition receipt SHA-256;
- measurement identity and epoch;
- bounded metric values;
- authority and evidence references.

Those references are pointers only. v1 does not authenticate the external measurement source.

## Deliberate exclusions

Separate profiles should own:

- tariff law / utility regulation;
- low-income or lifeline tariff policy;
- geographic entitlement law;
- engineering sufficiency beyond these metrics;
- time-series currentness/staleness;
- outage causality;
- service penalties that intentionally impair investor claim;
- emergency/public-support treatment;
- handback readiness;
- legal enforcement of distribution blocks.

## Nonclaims

PASS establishes only that the supplied current snapshot satisfies or fails one frozen distribution covenant. It does not establish legally fair tariffs, regulatory compliance, engineering adequacy, solvency, democratic legitimacy, handback readiness, or the truth of external measurements.
