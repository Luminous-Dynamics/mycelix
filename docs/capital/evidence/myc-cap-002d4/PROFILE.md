# MYC-CAP-002D4 — Physical handback readiness v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Qualify a narrow physical-infrastructure handback theorem without pretending that financial completion or reserve sufficiency proves physical fitness.

Core separation:

```text
financially satisfied
!= physical condition acceptable
!= handback reserve sufficient
!= handover accepted
!= legal title transfer
```

## Frozen physical model

v1 is profile-driven and finite. Each component record freezes:

- component ID;
- whether it is required;
- minimum residual-life units.

The assessment supplies exactly one record per profile component with:

- `condition_state`;
- integer `residual_life_units`;
- evidence reference.

Allowed states are exactly:

```text
PASS
FAIL
NOT_ASSESSED
NOT_APPLICABLE
```

Required components cannot be `NOT_APPLICABLE`. Non-required components must be `NOT_APPLICABLE`.

## Integer residual-life authority

Residual life uses bounded non-negative integers in the profile-selected unit.

A required component marked `PASS` still blocks readiness when:

```text
observed_residual_life_units < min_residual_life_units
```

No floating-point authority math is used.

## Independent-assessment boundary

The supplied outgoing operator and assessor references must differ.

This proves only separation of supplied references. It does not establish licensed-engineer status or external identity authenticity.

## Deferred maintenance

The profile freezes a maximum allowed deferred-maintenance amount.

If:

```text
deferred_maintenance_units > max_deferred_maintenance_units
```

the result is `REMEDIATION_REQUIRED`.

Known maintenance burden therefore cannot disappear merely because financing is complete.

## Handback reserve

The profile freezes an exact required reserve amount.

If no stronger blocker applies and:

```text
reserve_balance_units < required_handback_reserve_units
```

the result is `RESERVE_DEFICIENT`.

Reserve sufficiency is independent from condition fitness.

## State precedence

```text
condition FAIL
or residual-life deficit
or deferred-maintenance excess
    -> REMEDIATION_REQUIRED

else any required NOT_ASSESSED
    -> ASSESSMENT_INCOMPLETE

else reserve deficient
    -> RESERVE_DEFICIENT

else
    -> PHYSICAL_CONDITION_ACCEPTABLE
```

All blockers remain explicit.

## Financial orthogonality

The receipt carries parent financial state and remaining claim for context only.

Both are valid states:

```text
CLAIM_ACTIVE + PHYSICAL_CONDITION_ACCEPTABLE

RETURN_ENVELOPE_SATISFIED + REMEDIATION_REQUIRED
```

Financial state never promotes physical readiness.

## Authority boundary

Every receipt fixes:

```text
handover_accepted = false
legal_transition_complete = false
```

A physical PASS is therefore evidence input to later handover composition, not handover itself.

## Deterministic commitments

Canonical fixture SHA-256:

`044cd50acb036813e1861de06b0e3221b957cc8a4c7a87d3d05ffb941e627289`

Frozen receipt SHA-256:

`8631c6bd583a4a02115de694325b8e58cbec7fcf9575bb2289e5fc7b7eb3f955`

Verifier source SHA-256:

`43e9a37be78f03181047dee295bb4339e1a19bc8b40ca83339343ec82f8f7d41`

Regression-suite SHA-256:

`ab425ce7aaf53ea078b9406c6307bd2c5cbdb7394b148f92b046b3c94848e0cb`

Parent transition receipt semantic SHA-256:

`60b7217980c615eab453eae25b74b27e75fc67cb336ea60d140f089341d50565`

Physical profile semantic SHA-256:

`a318f2cf7f086779d61162301e38425d2aa8afed694884b25faa567a9ab8f5fc`

## Local preflight

The stdlib suite passes **18/18** locally, covering:

- positive physical condition;
- component condition failure;
- residual-life deficit despite `PASS`;
- incomplete assessment;
- reserve deficiency;
- deferred-maintenance excess;
- operator self-assessment rejection;
- required `NOT_APPLICABLE` rejection;
- unknown component rejection;
- negative/float residual-life rejection;
- negative reserve rejection;
- project/profile/parent substitution;
- zero-claim financial satisfaction remaining orthogonal to physical failure;
- injected legal-title field rejection;
- deterministic receipt reconstruction.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only the frozen component/residual-life/deferred-maintenance/reserve theorem for supplied evidence. It would not establish statutory inspection compliance, engineering fitness outside this profile, legal title, democratic legitimacy, future performance, or handover acceptance.
