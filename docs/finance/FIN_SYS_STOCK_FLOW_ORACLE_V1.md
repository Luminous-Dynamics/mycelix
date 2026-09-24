# FIN-SYS-003A synthetic stock-flow accounting oracle v1

Issue: #3074
Parent: #3072
Status: synthetic conformance corpus only; no accounting, policy, or empirical authority.

## Purpose

Freeze a small independent corpus that prevents financial-system simulations from obtaining attractive outcomes by violating accounting identities or by relabelling financing/revaluation events as income or cash flow.

The 2025 System of National Accounts is an interoperability/reference framework for the broader program. This V1 corpus deliberately uses a much smaller closed synthetic profile.

## Core identity

For every declared financial position:

```text
closing
=
opening
+ transactions
+ revaluations
+ other_changes
```

For `ClosedSynthetic` fixtures, each declared financial instrument must also have matched aggregate asset and liability positions/changes across the included sectors.

## Semantic non-equivalences

```text
borrowing != current income
revaluation != operating cash flow
writeoff != repayment
principal repayment != interest expense
secondary equity purchase != issuer capital formation
scenario profitability != accounting validity
```

## Corpus separation

- `fixtures/stock_flow_v1/public.json` contains solver/verifier-visible synthetic accounting records.
- `fixtures/stock_flow_v1/oracle.json` contains evaluator-only expected dispositions.
- `scripts/verify_stock_flow_v1.py` reconstructs the V1 identities independently of any future scenario engine.

A future solver benchmark must exclude oracle bytes from solver-visible input.

## V1 fixtures

Valid controls:

- loan creation with matched loan + deposit positions;
- principal repayment;
- asset revaluation with no cash-flow claim;
- debt writeoff represented as an other change.

Hostile controls:

- borrowing relabelled as current income;
- liability erased without a transaction/revaluation/other-change;
- revaluation relabelled operating cash flow;
- one-sided financial asset in a closed synthetic universe.

## Dispositions

V1 evaluator outputs are drawn from a closed vocabulary including:

```text
ReconciledUnderProfile
StockFlowIdentityViolation
CounterpartViolation
FlowClassificationViolation
ConsolidationViolation
InsufficientData
UnsupportedProfile
```

No scalar accounting-quality score.

## Numeric discipline

All synthetic financial numbers are finite decimal strings. No NaN, infinity, or sentinel number denotes missingness.

## Claim ceiling

A PASS establishes only that these exact synthetic records satisfy or violate the frozen V1 accounting/flow-class rules for the expected structural reason. It does not establish empirical macroeconomic accuracy, SNA certification, legal/tax accounting, profitability, welfare, sustainability, causal truth, or superiority of any financial-system design.