# AC-038 — Lifecycle Transition Semantic Hardening

## Purpose

AC-035 establishes an append-only economic action lifecycle.

AC-036 binds lifecycle revisions to the content fingerprint of the active scope.

AC-038 closes the remaining lifecycle-state ambiguity: stage and revision kind must agree, and serialized history must obey the same transition rules as live updates.

## Rules

Normal updates cannot directly create terminal states.

- `Completed` requires `Completion` kind.
- `Terminated` requires `Termination` kind.

Scope amendments cannot simultaneously move the action to another lifecycle stage.

Every adjacent historical revision is checked with the same stage-transition function used for live mutation.

## Why this matters

Without this coupling, a serialized or manually constructed record could describe:

`Implementation -> Terminated`

while labeling the event as an ordinary `Update`.

That weakens the semantic meaning of the lifecycle history even if the stage itself looks correct.

AC-038 makes the state machine explicit at both creation and validation boundaries.

## Validation

The reference implementation now checks:

- terminal-stage/change-kind agreement;
- scope amendments preserve lifecycle stage;
- historical stage transitions are valid;
- malformed serialized lifecycle structures are rejected;
- existing scope and fingerprint invariants remain intact.

## Research basis

OCDS models contracting as a lifecycle with distinct stages and uses release tags to distinguish the nature of changes, including updates and amendments. Its change-history model keeps earlier releases immutable.

References:

- https://standard.open-contracting.org/latest/en/primer/how/
- https://standard.open-contracting.org/latest/en/guidance/map/amendments/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/

Mycelix does not import OCDS procurement law. The relevant engineering lesson is that lifecycle state and change semantics should be explicit and independently auditable.

## Security contribution

The lifecycle boundary now protects:

`identity -> scope -> scope contents -> predecessor chain -> stage transition -> change semantic`

rather than treating those as loosely related fields.
