# MYC-CAP-002D1 — Digital Operational Sovereignty v1

Status: executable candidate; qualification is exact-head CI evidence, not this document.

## Purpose

Qualify whether a steward can independently operate/recover a digital or cyber-physical public infrastructure system without treating financial payoff or legal title as handback readiness.

## Orthogonal state

This theorem keeps:

```text
financial state
operational-sovereignty readiness
handover acceptance
legal transition
```

separate.

`CLAIM_ACTIVE` may coexist with `OPERATIONAL_TRANSFER_READY`.
`RETURN_ENVELOPE_SATISFIED` may coexist with `REMEDIATION_REQUIRED`.

## Frozen dimensions

v1 has twelve explicit dimensions:

```text
reproducible_deployment
administrator_recovery
trust_root_rotation
secrets_migration
backup_restore
data_schema_export
sbom_inventory
observability_runbooks
disaster_recovery_exercise
operator_replacement_exercise
operational_documentation
continuity_plan
```

Each dimension is one of:

```text
PASS
FAIL
NOT_ASSESSED
NOT_APPLICABLE
```

Unknown dimensions or states fail closed.

## Requiredness

The profile freezes which dimensions are required.

A required dimension cannot be `NOT_APPLICABLE`.
A non-required dimension must be `NOT_APPLICABLE`.

This prevents the assessment from declaring a hard requirement irrelevant after seeing the result.

## Independent-assessment boundary

The outgoing operator and assessor references must be distinct in v1.

This proves only separation of the supplied references. It does not authenticate those identities or prove institutional independence outside the supplied evidence.

## Deterministic readiness

For required dimensions:

```text
any FAIL         -> REMEDIATION_REQUIRED
else any NOT_ASSESSED -> ASSESSMENT_INCOMPLETE
else             -> OPERATIONAL_TRANSFER_READY
```

FAIL takes precedence over incomplete assessment.

## No automatic handover

The receipt always fixes:

```text
handover_accepted = false
legal_transition_complete = false
```

Automated operational readiness is never promoted into an acceptance or title-transfer claim.

## Parent financial context

The profile binds the exact MYC-CAP-002A transition receipt semantic SHA-256 and subject SHA.

The parent financial state and remaining claim are copied as orthogonal context only. They do not change operational readiness.

## Fixture commitments

Canonical example file SHA-256:

```text
0e031c3f90b7b2e448210c2812479794308257a52df14432c681d8dcfe0210d8
```

Frozen receipt file SHA-256:

```text
379320dbcdc65a00c843b96be97a2b4caf06df57b722689baa2a93bb1dc41e7d
```

Operational-sovereignty profile semantic SHA-256:

```text
50a8af7e97ffa2c603d722f2ce39c51be6790116a9739a54a32e590d7554a853
```

Assessment semantic SHA-256:

```text
bc352c751d4b913a6781185f14d57dbcd47b0d2693a2e704a1ae97e5d9fcb1aa
```

Parent transition receipt semantic SHA-256:

```text
60b7217980c615eab453eae25b74b27e75fc67cb336ea60d140f089341d50565
```

## Nonclaims

PASS establishes only the frozen digital operational-readiness theorem for supplied evidence. It does not establish legal title transfer, physical asset condition, engineering fitness outside the checklist, regulatory/cybersecurity certification, external assessor identity authenticity, democratic legitimacy, or future performance.
