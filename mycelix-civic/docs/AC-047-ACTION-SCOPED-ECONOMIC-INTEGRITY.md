# AC-047 — Action-Scoped Economic Integrity

## Purpose

AC-047 separates global impact-ledger visibility from action-local economic
closure.

A shared impact ledger can contain observations for many economic actions.
Action-level finalization must not inherit unresolved state that belongs to a
different action.

## Invariant

For an action scope identified by `action_ref`, the scoped integrity decision
and finalization exposure contain only impacts and restoration obligations whose
causal impact record has that same action reference.

Global ledger methods remain available for genuinely global governance decisions.

## Implementation

The impact ledger now provides:

- `exposure()` for the complete ledger;
- `exposure_for_action(action_ref)` for action-local closure;
- `gate()` for global impact governance;
- `gate_for_action(action_ref, purpose)` for action-local governance.

The existing scoped integrity path now uses `gate_for_action` rather than the
global gate.

AC-045/AC-046 finalization now consumes `exposure_for_action` rather than the
global exposure snapshot.

## Why this matters

Without the distinction, a shared ledger creates unintended coupling:

`action:A` can be clean while `action:B` has an unresolved depletion, yet
A's finalization could be blocked merely because both records live in one
ledger.

That is not a fail-closed safety property; it is a scope error.

AC-047 therefore preserves the stronger invariant:

**shared evidence, isolated decision scope.**

## Obligation handling

Restoration obligations do not carry a duplicated action reference. Their
action scope is derived through their referenced impact record.

This avoids introducing a second mutable copy of action identity. An obligation
is action-local when its referenced impact is action-local.

An orphaned obligation is not silently attributed to an action by guesswork.

## Relationship to prior hardening

AC-025 composes substrate and reciprocity decisions.

AC-030 attests the required dimensions and known-impact coverage of a scope.

AC-045 creates the finalization boundary.

AC-046 binds reconciliation records to exact execution evidence and recomputes
conformance.

AC-047 ensures those controls are evaluated against the correct action-local
impact state rather than unrelated global records.

## Non-goals

AC-047 does not claim that every future impact will be discovered automatically.
The existing known-impact coverage semantics remain explicit.

It also does not remove or hide global unresolved impacts. They remain available
to system-wide governance and analysis.

## Tests

The reference tests cover:

- action-scoped exposure excludes unrelated actions;
- action-local gating remains blocked by that action's own unresolved impact;
- scoped integrity ignores unrelated action impacts;
- finalization remains Ready when the only unresolved impact belongs to another
  action.

