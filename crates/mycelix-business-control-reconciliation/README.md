# Mycelix Business Control Reconciliation

`mycelix-business-control-reconciliation` compares canonical read-only campaign replay against an independently issued control-total statement.

The motivating example is a POS provider export plus a separately issued end-of-day control report containing the provider's transaction count and quantity/value total for an exact preregistered window.

## What a match establishes

A match establishes that, for the registered connector/input/metric/scope/unit/window:

- canonical replay produced the same unique source-event count as the control statement; and
- the preregistered aggregation produced the same fixed-point total as the control statement.

The evidence binds the contract, source document digest, campaign, replay, selected observation set, count, value, decision, and limitation transitions.

## What a match does not establish

Provider agreement with itself is not physical truth. This crate does not prove:

- that the control document came from the claimed provider;
- that the provider observed every real-world transaction;
- that the mapped metric has the intended accounting/economic meaning;
- that the provider's own ledger is correct; or
- that any business action is authorized.

Accordingly a matched reconciliation narrows:

- `limitation:upstream-export-completeness-unverified:v1` to `limitation:control-source-external-reality-unverified:v1`; and
- `limitation:aggregation-semantic-authority-unverified:v1` to `limitation:control-metric-semantic-authority-unverified:v1`.

It also keeps `limitation:control-source-authenticity-unverified:v1` explicitly unresolved.

There are no self/no-op limitation transitions.

## Fail-closed behavior

Count and value mismatches remain first-class evidence and cannot narrow limitations. Reconciliation also rejects connector drift, malformed statements, early control statements, mutated replay contents, incompatible units/scales, duplicate observation identities, and arithmetic overflow.

## Evidence evolution

`apply_control_reconciliation_limitations` creates a new digest-bound limitation state. It never rewrites the earlier hospitality report. The broad limitations remain part of the historical evidence; the new record explains exactly which evidence justified replacing them with narrower limitations.

## Safety

This crate is read-only. It creates no institutional authority and contains no provider credentials or mutation commands.
