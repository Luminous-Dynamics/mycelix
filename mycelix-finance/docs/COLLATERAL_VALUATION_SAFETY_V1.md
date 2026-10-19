# Collateral Valuation Safety v1

**Status:** implementation safety contract and source-review finding; not an oracle integration claim.

## Decision

Missing, malformed, zero, non-finite, or internally inconsistent collateral valuation must not be converted into either a Healthy status or a Liquidation status. In particular, zero is a value only when the valuation source explicitly reports zero under a defined asset/currency contract; an absent or unreachable oracle is unknown evidence.

The collateral-health handler therefore refuses to persist a health snapshot without a positive decoded valuation. The canonical CollateralHealthStatus::from_ltv classifies NaN, infinities, and negative ratios as Invalid, which is a calculation outcome and must not be stored as an economic health status. The integrity validator rejects unpriced values, recomputes the reported LTV from obligation_amount / current_value, and checks the status against that ratio.

## Multi-collateral consistency

For each multi-collateral position, the coordinator and integrity validator enforce:

- a bounded component manifest;
- a non-empty, unique asset_id for each component;
- a recognized CollateralAssetClass;
- strictly positive valuation and a finite weight in (0, 1];
- weights summing to 1 within a declared numerical tolerance;
- the sum of component valuations equalling aggregate_value without integer overflow;
- each component weight matching that component's share of aggregate value;
- a diversification bonus recomputed by the shared compute_diversification_bonus function;
- effective_ltv and status matching the validated obligation, aggregate value, and bonus.

These checks establish internal consistency, not truth of the prices. A self-consistent manifest can still contain invented valuations when valuation evidence is not authenticated.

## Integration boundary found during review

The bridge coordinator calls price_oracle/get_collateral_value. The current public mycelix-finance/zomes/price-oracle/coordinator/src/lib.rs exports get_consensus_price and other oracle functions, but does not export get_collateral_value. Consequently, this handler must currently fail closed when that call is unavailable; the old behavior turned an unavailable result into a zero price, then a synthetic 999.0 LTV and Liquidation label.

Do not add a wrapper that simply renames a TEND-denominated get_consensus_price result to a collateral value. Before enabling this path, the valuation contract must specify and enforce:

1. asset identity and unit;
2. valuation amount and denomination, with an explicit conversion profile when denominations differ;
3. source/consensus record identity and digest;
4. observation time, freshness ceiling, and behavior for a degraded/fallback consensus;
5. authenticated source/provider profile and threshold for sufficient independent evidence;
6. missing/invalid/stale outcome semantics as Unknown, not zero;
7. how a revaluation supersedes previous observations without erasing history.

Likewise, the multi-collateral manifest currently proves the relationships between values supplied by the caller; it does not by itself prove that those values were issued by the oracle. A follow-on adapter should bind each component to an authenticated, currency-correct, fresh valuation receipt. Until then, this work must not be described as a complete collateral-oracle integration.

## Regression contract

The test suite should continue to reject:

- NaN, positive/negative infinity, or negative LTV;
- status inconsistent with the recorded ratio;
- an absent/zero valuation presented as a liquidation sentinel;
- aggregate values that disagree with the component sum;
- invalid asset classes, duplicate/empty asset identities, zero component values, or malformed weights;
- a claimed diversification discount that is not produced by the shared rule;
- an effective LTV/status inconsistent with the validated component manifest;
- over-limit component JSON on both create and update paths.

A missing external value must fail with an explicit error. It must never be silently translated to Healthy, zero, or Liquidation.
