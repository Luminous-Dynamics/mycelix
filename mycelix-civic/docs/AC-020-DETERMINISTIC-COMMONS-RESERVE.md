# AC-020 — Deterministic Commons Reserve Accounting

Status: implementation candidate; execution qualification not yet established.

## Purpose

Mycelix already contains an explicit anti-corrosion mechanism: a constitutional minimum inalienable reserve for commons pools.

AC-020 hardens the arithmetic underneath that invariant without changing the intended policy.

The change replaces floating-point reserve splitting and ratio validation with integer arithmetic.

## Why this matters

A constitutional invariant should not depend on floating-point representation when the underlying quantity is discrete SAP units.

The prior contribution path computed the reserve portion using a floating-point multiplication. The hardened path uses the exact fraction:

reserve = amount / 4

The remainder stays in the circulating zone.

For validation, the historical tolerance of 0.1 percentage points is represented exactly as:

reserve / total >= 249 / 1000

This removes dependence on floating-point comparison for the safety check.

## New observable

CommonsPool now exposes reserve ratio in basis points:

- 2,500 bps = 25%
- 0 bps = empty pool

This gives AC-017 a deterministic representation that can be incorporated into a substrate account without introducing floating-point state.

## Policy preservation

AC-020 does not silently change the constitutional target.

The intended minimum remains 25%.

The existing validation tolerance is preserved mathematically rather than approximated through floating point.

This distinction is important: implementation hardening should not quietly become policy revision.

## Relationship to AC-017

AC-017 represents substrate state.

AC-020 provides a deterministic bridge from an existing Mycelix commons reserve invariant into that substrate model.

Conceptually:

commons reserve -> exact ratio -> substrate financial state -> discretionary-action gate

The reserve itself remains protected by existing CommonsPool behavior; AC-020 makes its arithmetic more suitable as machine-checkable substrate evidence.

## Next hardening

A subsequent finance pass should address additive overflow across repeated contributions and compost receipts while preserving API compatibility or explicitly versioning any changed return types.

That is intentionally kept separate from AC-020 so the reserve-arithmetic change remains small and auditable.
