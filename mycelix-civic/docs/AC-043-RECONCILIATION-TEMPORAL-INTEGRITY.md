# AC-043 — Reconciliation Temporal Integrity

## Purpose

AC-041 records whether an execution receipt conforms to an explicit execution constraint.

AC-043 ensures that reconciliation itself has a valid temporal position.

## Rules

A reconciliation timestamp cannot precede the execution receipt it reconciles.

Reconciliation history is also monotonic: a later ledger entry cannot be timestamped before the previous reconciliation.

This produces a simple chronology:

`authorization <= execution <= reconciliation`

## Why this matters

Without the ordering rule, a reconciliation could appear in history before the event being reconciled.

That weakens audit chronology and can create confusing or exploitable ordering around corrections.

The rule does not assume synchronized wall clocks beyond the timestamp domain already used by the reference model. It simply rejects explicit contradictions.

## Atomicity

Temporal validation occurs before the reconciliation record is appended.

A rejected timestamp therefore leaves reconciliation history unchanged.

## Research basis

OCDS describes implementation data as a sequence of updates over the life of a contracting process and uses release dates to represent changes over time. Its implementation examples distinguish planned milestones from later events such as actual completion and payment.

References:

- https://standard.open-contracting.org/latest/en/guidance/map/milestones/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/primer/how/

Mycelix uses the same general chronological principle without importing OCDS publication semantics.

## Validation

Added coverage for:

- reconciliation before execution;
- backwards reconciliation timestamps;
- unchanged history after rejected temporal input.

## Security contribution

AC-043 protects the temporal ordering of the audit trail itself, extending the existing monotonic-time guarantees from action lifecycle into post-execution reconciliation.
