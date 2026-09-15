# MYC-CAP-002D3 — Custody continuity and successor state v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Preserve an immutable historical handover acceptance while deriving the **current operational-custody state** from one ordered, profile-bound event lineage.

Core separation:

```text
historical CUSTODY_ACCEPTED
!= current custody CURRENT forever

current custody regression
!= retroactive falsification of historical acceptance
```

## Exact custody genesis

The profile binds both:

- exact accepted-custody Git subject `5bef6d4cf9cc1a09f43bbe09846edc6c0be19c10`;
- exact semantic acceptance-receipt SHA-256 `7a073b1b012c60ac645c9cc98c00d78392a2adadbf2474b0cd39df7a7c03b832`.

The accepted receipt must establish operational custody while continuing to state that legal-title and constitutional-stewardship transitions are not established.

This prevents a receipt from another implementation lineage from silently becoming the custody genesis.

## Event lineage

Every event binds:

- exact project;
- exact custody profile SHA-256;
- contiguous sequence;
- unique event ID;
- previous-event SHA-256;
- typed event kind;
- blocker/successor fields;
- frozen authority role;
- evidence reference.

Mutation, insertion, deletion, reordering, project substitution, profile substitution, and duplicate event IDs fail closed.

## v1 event vocabulary

```text
GenesisCurrent
MaterialRegression
VerifiedCure
SuspendCustody
ResumeCustody
SuccessorAcceptance
TerminateCustody
```

Unknown events fail closed.

### Genesis

The first and only first event must be `GenesisCurrent` under the frozen custody authority.

### Material regression

A `MaterialRegression` adds one profile-allowed blocker under the regression authority. Duplicate active blockers are rejected.

The v1 profile freezes blocker vocabulary rather than allowing arbitrary incident prose to become state authority.

### Verified cure

A `VerifiedCure` may remove only one currently active exact blocker and must come from the frozen cure authority.

There is no generic `clear=true` event.

Historical regression and cure events remain in the committed lineage after recovery.

### Suspension / resumption

Suspension is separately authorized and does not erase active blockers.

Resumption requires:

```text
suspended == true
AND
active_blockers == []
```

Therefore suspend/resume cannot be used as a blocker-clearing shortcut.

### Successor acceptance

A successor may supersede the current lineage only when:

```text
suspended == false
AND
active_blockers == []
```

and under the frozen successor authority.

The successor custodian must differ from the initial custodian and an exact successor-acceptance receipt digest is required.

v1 binds that digest but does not independently replay/authenticate the successor receipt; that remains an explicit nonclaim and future strengthening point.

`SUPERSEDED` is terminal for this lineage.

### Termination

`TerminateCustody` is separately authorized and terminal.

No event may follow `SUPERSEDED` or `TERMINATED`.

## Current state derivation

State precedence is:

```text
SUPERSEDED / TERMINATED
    -> terminal state

else suspended
    -> SUSPENDED

else active blocker set non-empty
    -> DEGRADED

else
    -> CURRENT
```

The receipt exposes both the original lineage custodian and an explicit `current_custodian_ref`:

- initial custodian for CURRENT/DEGRADED/SUSPENDED;
- successor custodian for SUPERSEDED;
- null for TERMINATED.

## Canonical history

The checked-in positive fixture is deliberately non-trivial:

```text
GenesisCurrent
-> MaterialRegression(BACKUP_RESTORE_FAILED)
-> VerifiedCure(BACKUP_RESTORE_FAILED)
-> CURRENT
```

The active blocker set is empty, but the incident remains permanently committed to `event_history_sha256` and the chain tip.

## Authority boundary

Every receipt preserves:

```text
historical_handover_accepted = true
legal_title_transition_established = false
constitutional_stewardship_transition_established = false
```

Current custody state is operational evidence, not property-law or constitutional-governance authority.

## Deterministic commitments

Canonical fixture SHA-256:

`d097318f48564a51bd58bf71062a1a271721be1307ecb9e8f5d0564660e72c9c`

Frozen receipt SHA-256:

`6ced4e4a9ec5ab1d26c5e22dc7a2d0dd9788b52795555f67a3e1d52d37123d6d`

Verifier source SHA-256:

`5afedd1120d6a860fcba8d2a54fd6dd4d53cf356139038d2d68a017c40e574e7`

Regression-suite SHA-256:

`2288dfa2248313451f76b2d4963ca03bad4b4f3c70ee90bf9e76ee1b747150db`

Custody profile semantic SHA-256:

`881808eba6b08983a6eee434a98a2c508313ae3b885af37b8926246725fcb400`

Accepted-custody receipt semantic SHA-256:

`7a073b1b012c60ac645c9cc98c00d78392a2adadbf2474b0cd39df7a7c03b832`

## Local preflight

The stdlib suite passes **24/24** in the repository's actual checked-in path layout, covering:

- current genesis;
- degradation on material regression;
- verified cure back to current;
- cure of a non-active blocker rejection;
- duplicate/unknown blocker rejection;
- suspension preserving blockers;
- resume-with-blocker rejection;
- cure-then-resume;
- successor supersession;
- successor while blocked rejection;
- same-custodian successor rejection;
- terminal-lineage immutability;
- termination;
- wrong authority;
- project/profile/event-chain substitution;
- duplicate event IDs;
- acceptance subject validation;
- accepted-receipt mutation rejection;
- authority-contaminated acceptance rejection;
- injected legal-title event field rejection;
- deterministic reconstruction.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only current custody-state derivation under this frozen profile and supplied evidence. It would not establish:

- legal title;
- constitutional stewardship legitimacy;
- democratic legitimacy;
- physical engineering fitness;
- service quality;
- cybersecurity certification;
- external authority/signature authenticity;
- independent authenticity of a referenced successor acceptance receipt;
- future custody health.
