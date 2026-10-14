# AC-044 — Reconciliation Ledger Structural Integrity

## Purpose

AC-041 introduced immutable execution reconciliation records.

AC-043 made their chronology explicit.

AC-044 makes the persisted reconciliation ledger itself auditable and rejectable when its serialized state is malformed.

## Validation rules

The ledger validator enforces:

- every reconciliation record is structurally valid;
- reconciliation identifiers are unique;
- reconciliation timestamps are monotonic.

The validator does not invent the truth of a reconciliation result. It validates the stored record envelope only. Recomputing conformance requires the original execution receipt and execution constraint.

## Live mutation

`reconcile()` validates the current ledger before appending a new record.

This prevents a corrupted in-memory/persisted history from becoming the base for additional apparently valid reconciliation events.

Rejected validation leaves the ledger unchanged.

## Research basis

OCDS emphasizes versioned change histories and publication of implementation updates over the lifetime of a contracting process. That makes the integrity of the history itself important, not just the current snapshot.

References:

- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/guidance/map/milestones/
- https://standard.open-contracting.org/latest/en/primer/how/

## Security contribution

The post-execution chain now protects:

`receipt -> reconciliation -> reconciliation history`

rather than treating reconciliation history as trusted storage outside the integrity model.
