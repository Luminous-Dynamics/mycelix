# STEW-010A — Reciprocity Duty Execution Phase Profile v0.1

## Purpose

STEW-010A adds execution timing semantics **over** STEW-010 reciprocity obligations without changing the frozen obligation theorem.

STEW-010 already says what benefit is asserted to be owed, to whom, and on what basis. It does not say when fulfillment is required relative to the governed use. STEW-010A makes that distinction explicit so a future authorization engine does not accidentally require a future post-use duty to be already fulfilled—or defer a genuine precondition until after access has been granted.

## Core theorem

```text
obligation exists
!= duty timing established
!= phase basis valid
!= duty satisfied
!= authorization granted
```

## Execution phases

Each exact STEW-010 benefit clause receives exactly one v1 phase:

- `Precondition` — satisfaction must be independently established before the governed use may positively proceed;
- `Concurrent` — the duty must be bound/enforceable as part of the governed use, but is not represented as already completed;
- `PostUse` — the duty becomes an outstanding tracked obligation after the governed use;
- `Ongoing` — the duty must be bound at use and remain tracked across the continuing relationship.

The phase itself does not prove that an enforcement mechanism, schedule, beneficiary acceptance, or satisfaction evidence exists.

## Exact coverage

A `ReciprocityDutyProfileV1` embeds the complete STEW-010 obligation and must assign every exact benefit clause exactly once.

```text
missing benefit assignment -> REJECT
invented benefit assignment -> REJECT
duplicate benefit assignment -> REJECT
```

The assignment includes an opaque `phase_basis_ref`. Carrying that reference does not prove the referenced authority or interpretation is valid.

## Pre-use handling projection

The profile may structurally project each phase to one required handling mode:

```text
Precondition -> MustBeSatisfiedBeforeUse
Concurrent   -> MustBeBoundAtUse
PostUse      -> MustBeActivatedForTracking
Ongoing      -> MustBeBoundAndTracked
```

This projection describes what a later decision engine must evaluate. It is not itself evidence that the requirement was satisfied.

## Reciprocity receipt boundary

STEW-011 receipts remain evidence reports:

```text
ReportedFulfilled != verified satisfaction
```

A future satisfaction/admission theorem must evaluate receipts, beneficiary acceptance where required, disputes, and currentness before a `Precondition` can be treated as satisfied.

## Deliberate non-claims

No obligation legitimacy, phase-authority validity, beneficiary identity, trigger satisfaction, payment, benefit delivery, receipt verification, beneficiary acceptance, cultural consent, legal discharge, runtime authorization, or enforcement correctness is established.
