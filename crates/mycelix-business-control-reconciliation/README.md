# Mycelix Business Control Reconciliation

`mycelix-business-control-reconciliation` compares canonical read-only campaign replay against an independently issued control-total statement for one exact preregistered scope and time window.

The motivating example is a POS provider export plus a separately issued end-of-day control report containing the provider's transaction count and quantity/value total for that exact window.

## What a match establishes

A match establishes that, for the registered connector/input/metric/scope/unit/window:

- canonical replay produced the same unique source-event count as the control statement; and
- the preregistered aggregation produced the same fixed-point total as the control statement.

The evidence binds the contract, source document digest, campaign, replay, selected observation set, count, value and reconciliation decision.

## What a match does not establish

Provider agreement with itself is not physical truth. This crate does not prove:

- that the control document came from the claimed provider;
- that the provider observed every real-world transaction;
- that the mapped metric has the intended accounting/economic meaning;
- that the provider's own ledger is correct; or
- that any business action is authorized.

It also does **not** let one daily match narrow limitations for a week, month, or full pilot.

## Scoped limitation evidence

An exact match can emit two possible evidence-backed transitions, but each transition is cryptographically bound to the contract's exact `scope`, `window_start`, `window_end`, contract digest and reconciliation digest:

- `limitation:upstream-export-completeness-unverified:v1` -> `limitation:control-source-external-reality-unverified:v1`; and
- `limitation:aggregation-semantic-authority-unverified:v1` -> `limitation:control-metric-semantic-authority-unverified:v1`.

`limitation:control-source-authenticity-unverified:v1` remains independently unresolved.

These are **scoped transition candidates**, not global limitation edits. A later coverage theorem must prove that matching control windows cover the entire qualification interval without gaps before a qualification-wide limitation may be narrowed.

## Fail-closed behavior

Count and value mismatches remain first-class evidence and emit no transition. Reconciliation also rejects connector drift, undeclared source inputs, malformed statements, early control statements, mutated replay contents, incompatible units/scales, duplicate observation identities and arithmetic overflow.

A fabricated undeclared input therefore cannot reconcile `0 == 0` and acquire evidentiary force.

## Evidence evolution

Prior qualification evidence is immutable. This crate only creates a narrower scoped evidence claim. It never rewrites an earlier report or claims broader temporal coverage than the control contract establishes.

## Safety

This crate is read-only. It creates no institutional authority and contains no provider credentials or mutation commands.
