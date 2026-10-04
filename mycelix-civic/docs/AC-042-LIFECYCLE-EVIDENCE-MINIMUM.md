# AC-042 — Lifecycle Evidence Minimum

## Purpose

The economic lifecycle is intended to be an auditable history, not merely a valid state machine.

AC-042 requires every lifecycle revision to carry at least one evidence reference.

## Rule

A lifecycle revision is invalid when:

- its evidence list is empty; or
- any evidence reference is empty/blank.

This applies to the initial revision, ordinary updates, scope amendments, completion, and termination.

## Why this matters

A state transition can be structurally correct while having no attributable basis:

`Implementation -> Completed`

is not sufficient by itself.

The lifecycle should also be able to answer:

`completed because of which evidence?`

AC-042 keeps the answer attached to the immutable revision.

## Research basis

OCDS explicitly models implementation updates, transactions, milestones and documents as part of a contracting process, and its release history is designed to make changes inspectable over time.

References:

- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/guidance/map/milestones/
- https://standard.open-contracting.org/latest/en/schema/reference/

Mycelix does not import OCDS evidence requirements. The shared engineering principle is that lifecycle change should carry enough provenance to be independently inspected.

## Validation

A regression test now rejects creation of a lifecycle with zero evidence references.

Existing evidence-content validation remains unchanged.

## Security contribution

AC-042 prevents an attacker or faulty integration from creating an apparently valid lifecycle transition that is impossible to audit back to any declared evidence.
