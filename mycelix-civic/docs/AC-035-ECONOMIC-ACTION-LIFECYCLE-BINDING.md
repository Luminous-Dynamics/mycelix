# AC-035 — Economic Action Lifecycle Binding

## Purpose

AC-030 made the integrity scope of an economic action explicit.

AC-035 prevents that scope from silently drifting as the action moves through its lifecycle.

The stable action reference is preserved across immutable lifecycle revisions:

`Planning -> Tendering -> Awarded -> Contracted -> Implementation -> Completed/Terminated`

The lifecycle model is intentionally append-only. A previous revision is never edited.

## Scope continuity

Ordinary lifecycle updates must retain the active scope ID.

Changing scope requires `amend_scope()`, which:

- validates the new AC-030 scope;
- requires the same stable action reference;
- requires a new scope ID;
- records the predecessor scope ID;
- records a new lifecycle revision;
- is forbidden after completion or termination.

This turns scope drift into an explicit, reviewable amendment.

## Revision continuity

Every revision after the initial planning record must point to the immediately preceding revision.

A forked predecessor is rejected.

Revision timestamps cannot move backward.

Terminal actions cannot receive later revisions.

## Why this matters

Without lifecycle binding, a system can correctly authorize planning under one integrity scope, then execute or pay under a narrower replacement scope.

That is a time-of-check/time-of-use class of governance failure:

`scope checked at T1 -> scope silently changed -> action executed at T2`

AC-035 binds the identity and scope across the interval.

## Research basis

OCDS models a contracting process as a single process spanning tendering, awarding, contracting and implementation, joined by a common contracting-process identifier. It also represents changes through new immutable releases rather than editing historical releases.

References:

- https://standard.open-contracting.org/latest/en/primer/how/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/schema/identifiers/
- https://standard.open-contracting.org/latest/en/guidance/map/amendments/

AC-035 uses the same general systems principle—stable identity across lifecycle stages plus append-only change history—without importing OCDS policy requirements into Mycelix.

## Invariants

### Stable identity

The action reference cannot change during its lifecycle.

### Scope continuity

The active scope cannot change through an ordinary update.

### Explicit amendment

Every scope change produces a new revision and identifies the predecessor scope.

### Linear history

Every revision points to the exact immediately previous revision.

### Monotonic time

Lifecycle timestamps never move backward.

### Terminal finality

Completed and terminated actions cannot be mutated through further lifecycle revisions.

## Validation

The reference implementation tests:

- normal lifecycle progression;
- silent scope change rejection;
- explicit scope amendment;
- preservation of old scope history;
- predecessor-fork rejection;
- stage regression rejection;
- terminal immutability;
- timestamp monotonicity;
- action identity preservation;
- malformed serialized lifecycle rejection.

## Architectural result

The execution model now has a durable chain:

`attested scope -> lifecycle-bound scope -> integrity assessment -> execution history`

This is the foundation needed to bind later payment, procurement, and completion records to the same economic action without allowing the authorization context to drift.
