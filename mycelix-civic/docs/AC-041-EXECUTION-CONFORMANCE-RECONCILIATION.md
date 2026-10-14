# AC-041 — Execution Conformance Reconciliation

## Purpose

AC-039 proves that an execution receipt was authorized under the current action lifecycle.

AC-041 asks the next question:

**Did the observed execution conform to the explicit execution constraint?**

The constraint is deliberately explicit rather than inferred from a generic economic amount. It carries:

- stable action identity;
- lifecycle revision;
- scope identity;
- scope fingerprint;
- execution kind;
- exact expected quantity;
- native quantity unit;
- supporting evidence.

## Exact-match reference policy

The reference implementation uses exact quantity matching.

It classifies observed execution as:

- `Conformant`;
- `UnderQuantity`;
- `OverQuantity`;
- `UnitMismatch`;
- `KindMismatch`;
- `MissingObservedQuantity`;
- `AuthorizationMismatch`.

There is intentionally no implicit tolerance.

A deployment may later add explicit, governed tolerances for domains where exact equality is not appropriate. Those tolerances must be policy, not hidden arithmetic.

## Non-conformance is evidence

A receipt that is under, over, or otherwise non-conformant is still retained.

Reconciliation does not rewrite the receipt or manufacture an approval state.

This creates a clean separation:

`authorization -> execution evidence -> conformance observation -> policy response`

The reconciliation result tells downstream policy that the execution differed; it does not itself determine whether the difference was legitimate.

## Authorization freshness

The constraint must still match the current lifecycle revision and scope fingerprint.

Therefore a stale constraint cannot be used to reconcile a later execution.

This prevents:

`old authorization -> new execution -> historical constraint chosen after the fact`

## Research basis

OCDS models implementation transactions and milestones as part of the same contracting process, and its implementation guidance shows progress and payment information being published over time. Contract amendments are explicit changes rather than silent replacement of prior data.

References:

- https://standard.open-contracting.org/latest/en/primer/how/
- https://standard.open-contracting.org/latest/en/guidance/map/milestones/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/guidance/map/amendments/

AC-041 applies the identity and explicit-change principles without importing procurement-specific policy.

## Validation

Tests cover:

- exact quantity conformance;
- under-execution;
- over-execution;
- missing observed quantity;
- unit mismatch;
- stale lifecycle constraints;
- duplicate reconciliation identifiers.

## Architectural result

The economic chain now reaches outcome verification:

`policy -> scope -> fingerprint -> lifecycle -> integrity decision -> execution receipt -> reconciliation`

The system can distinguish:

**authorized**

from

**actually executed as authorized**.

That distinction is essential for any later payment, procurement, restoration, or civic-audit integration.
