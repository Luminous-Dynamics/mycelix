# AC-024 — Commons Balance Overflow Hardening

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-020 hardened the arithmetic of the constitutional commons reserve ratio.

AC-024 closes the next corruption path: state-transition overflow.

Before this tranche, repeated contributions or compost receipts could use unchecked addition on u64 balances.

That creates a dangerous distinction:

- arithmetic is correct for ordinary values;
- arithmetic silently corrupts state at the numeric boundary.

AC-024 makes those transitions checked.

## Contribution path

A contribution now calculates all successor balances before mutating the pool:

- inalienable reserve;
- available balance;
- member cumulative contribution.

If any addition would overflow, the operation returns a structured CommonsResult error and leaves the pool unchanged.

This creates an all-or-nothing mutation boundary.

## Compost path

The existing receive_compost method keeps its historical unit-returning API for compatibility.

A new try_receive_compost method exposes an explicit Result error path.

The historical method calls the checked path and panics on impossible/corrupt overflow rather than silently wrapping the balance.

This is deliberate fail-closed behavior: a corrupted monetary state should abort the operation rather than continue with an invalid balance.

## No partial mutation invariant

The adversarial fixture sets the reserve to u64::MAX and attempts another contribution.

The expected behavior is:

- contribution rejected;
- reserve unchanged;
- available balance unchanged;
- member contribution total unchanged;
- activity timestamp unchanged.

The same property is tested for compost overflow.

## Why this belongs in the anti-corrosion series

A governance invariant can be theoretically correct and still fail if arithmetic at its implementation boundary is unsafe.

The resulting progression is:

AC-017: substrate state
AC-020: exact reserve arithmetic
AC-024: checked balance transitions

The system should preserve invariants both semantically and numerically.

## Policy preservation

AC-024 does not change:

- the 25% constitutional reserve target;
- the reserve tolerance;
- compost allocation proportions;
- SAP monetary semantics.

It only prevents numeric overflow from violating those rules.

## Future hardening

The next finance-level arithmetic pass should audit:

- total_sap overflow semantics;
- demurrage computation at maximum balances;
- annual mint-cap counters;
- compost distribution arithmetic;
- collateral totals;
- aggregate treasury balances;
- any multiplication before division involving u64 amounts.

Every such transition should have explicit overflow semantics and no partial mutation.

