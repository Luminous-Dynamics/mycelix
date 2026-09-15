# MYC-CAP-002D2 — Authorized handover acceptance v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

This profile qualifies a narrow custody-acceptance theorem over one exact MYC-CAP-002D1 operational-sovereignty readiness receipt.

It answers:

> Given a current exact readiness receipt, a frozen authority profile, a complete or explicitly waived custody inventory, and continuity PASS, is this handover acceptance event valid for **operational custody**?

It does **not** establish legal title, constitutional stewardship, democratic legitimacy, property-law compliance, or future operating condition.

## Core authority separation

```text
OPERATIONAL_TRANSFER_READY
!= HANDOVER_ACCEPTED

HANDOVER_ACCEPTED
!= LEGAL_TITLE_TRANSFER

LEGAL_TITLE_TRANSFER
!= CONSTITUTIONAL_STEWARDSHIP
```

v1 supports only:

```text
legal_effect_mode = CUSTODY_ONLY
```

Any other legal-effect mode fails closed.

## Exact readiness/currentness binding

The acceptance profile binds the exact MYC-CAP-002D1 readiness profile digest, one named readiness registry, and one named readiness-designation authority.

The supplied designation binds:

- registry ID;
- integer registry epoch;
- project;
- exact acceptance-profile digest;
- exact readiness receipt digest;
- designation state;
- authority/evidence references.

Only `ACTIVE` is accepted. A different designated receipt is stale. `PENDING` and `REVOKED` fail closed.

v1 contains no wall-clock or expiry arithmetic. Time-based currentness requires a separately qualified time/epoch profile.

## Acceptance authority

The frozen profile names:

- acceptance authority;
- waiver authority;
- outgoing custodian;
- incoming custodian.

The event cannot substitute any of those identities.

Reference equality is only a typed boundary; v1 does not authenticate external signatures or legal identity.

## Custody inventory

v1 recognizes exactly twelve custody inventory items:

- deployment bundle;
- administrator recovery material;
- trust-root transition material;
- secrets-migration package;
- backup/restore package;
- data/schema export package;
- SBOM inventory;
- observability/runbooks;
- disaster-recovery package;
- operator-replacement package;
- operational documentation;
- continuity plan.

Each item is exactly `PRESENT` or `MISSING`.

Requiredness and waivability are frozen by profile, not chosen by the acceptance event.

## Waivers

A missing required inventory item blocks acceptance unless that exact requirement is profile-waivable and has one explicit typed waiver.

A waiver binds:

- unique waiver ID;
- exact requirement;
- waiver authority;
- reason reference;
- remediation owner;
- remediation epoch/reference;
- retention/security reference.

A waiver cannot cover a present item, cannot cover a non-waivable requirement, and cannot be duplicated.

Waivers preserve the historical readiness receipt; they do not retroactively change `OPERATIONAL_TRANSFER_READY`.

## Continuity

The acceptance event requires:

```text
continuity_state = PASS
```

Anything else fails closed.

## Output states

Without waivers:

```text
handover_acceptance_state = CUSTODY_ACCEPTED
inventory_acceptance_state = COMPLETE
```

With valid explicit waivers:

```text
handover_acceptance_state = CUSTODY_ACCEPTED_WITH_WAIVERS
inventory_acceptance_state = ACCEPTED_WITH_WAIVERS
```

Every receipt fixes:

```text
operational_custody_accepted = true
legal_title_transition_established = false
constitutional_stewardship_transition_established = false
```

## Deterministic commitments

Candidate fixture SHA-256:

`54c64c9534490325a89638a6326c15d1890355857eb274c3cf11e28344754bd4`

Frozen receipt SHA-256:

`0e65c240598b0a4640101046501f3386f020582a142b863f967604f513d56b65`

Verifier source SHA-256:

`246d59b1ceab1276862f5d44f9722f4c9814262db8a4005807aca6017f391759`

Regression-suite SHA-256:

`b9030843e011da19c0b5fdc2204241d9dcc2aa328a9087fc39b38bd0bf21b760`

Readiness receipt semantic SHA-256:

`050e9db5299cc7eabe2c862fa5202c1e712e12a1a074bab784f9e35690889367`

Acceptance profile semantic SHA-256:

`3689d61db89b1467ca0d0b73b00d220c70c9f9cd3e52cd6a5329052cae7aceb5`

## Local preflight

The stdlib suite passes **21/21** locally, covering:

- clean custody acceptance;
- valid explicit waiver;
- readiness not ready;
- stale, pending, and revoked readiness designation;
- readiness registry substitution;
- readiness-designation authority substitution;
- wrong acceptance authority;
- wrong incoming custodian;
- missing non-waivable inventory;
- missing waivable inventory without waiver;
- waiver over a present item;
- unauthorized waiver;
- duplicate waiver requirement;
- continuity failure;
- project/profile substitution;
- contaminated readiness receipt;
- injected legal-title authority;
- deterministic receipt reconstruction.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only the frozen custody-acceptance theorem for supplied evidence. It would not establish:

- legal title transfer;
- constitutional stewardship transition;
- democratic legitimacy;
- regulatory/property-law compliance;
- physical asset condition outside the parent readiness scope;
- authenticity of external authority identities/signatures;
- current post-acceptance condition after later material events;
- future performance or service quality.
