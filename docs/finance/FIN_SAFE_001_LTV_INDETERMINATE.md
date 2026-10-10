# FIN-SAFE-001 — Indeterminate collateral health must not masquerade as healthy or liquidation

## Problem

The canonical shared classifier currently uses ordinary floating-point comparisons in `CollateralHealthStatus::from_ltv`. `NaN` makes every comparison false and therefore falls through to `Healthy`; the resilience suite currently documents that behavior. By contrast, the bridge coordinator treats non-finite LTV as the liquidation string.

Both policies are unsafe in different ways:

- `NaN -> Healthy` can permit risk-increasing behavior when valuation is unknown.
- `NaN/unavailable -> Liquidation` conflates telemetry failure with evidence of insolvency and could trigger an unjustified destructive action.

## Safety theorem

An invalid or unavailable valuation is **Indeterminate**, not Healthy and not evidence of Liquidation.

While collateral health is Indeterminate:

1. risk-increasing operations must fail closed;
2. automated liquidation must not proceed solely from the indeterminate reading;
3. the system should request/retry revaluation and preserve the last independently valid observation as historical evidence only;
4. UI/API surfaces must distinguish `Indeterminate` from real threshold states;
5. recovery from Indeterminate requires a new valid finite valuation, not a caller-provided status override.

## Fixed checked-assessment contract

Use a checked assessment layer that separates threshold state from evidence quality:

- `Known(CollateralHealthStatus)` for a valid finite observation;
- `Indeterminate(reason)` for missing valuation, zero valuation denominator, non-finite ratio, or negative ratio.

`Indeterminate` is **not** another collateral-risk tier. It is an epistemic state saying that the current observation cannot support a threshold conclusion.

### Valid finite threshold semantics

Preserve the existing strict-boundary policy deliberately:

- `ltv <= 0.80` -> `Known(Healthy)`;
- `0.80 < ltv <= 0.90` -> `Known(Warning)`;
- `0.90 < ltv <= 0.95` -> `Known(MarginCall)`;
- `ltv > 0.95` -> `Known(Liquidation)`.

A zero denominator must be rejected **before division**. Do not manufacture `+Inf` and then classify that sentinel as liquidation evidence.

## Authorization rule

A collateral-health assessment is evidence, not authority.

- `Known(Liquidation)` may satisfy a health-evidence prerequisite for a later policy decision, but does not itself authorize seizure/default/liquidation.
- `Indeterminate(_)` cannot satisfy that prerequisite.
- `Indeterminate(_)` also cannot relax collateral constraints, authorize new risk, or be silently replaced with the last valid observation as if that observation were current.

This is fail-closed in both directions: uncertainty cannot justify either risk expansion or destructive enforcement.

## Canonical implementation requirements

The implementation must have exactly one security-relevant threshold classifier. Bridge/coordinator code may convert its typed result for serialization or presentation, but must not duplicate the numeric threshold ladder.

Required indeterminate reasons should be stable enough for wire/API clients to distinguish at least:

- `MissingValuation`;
- `ZeroValuation` (zero denominator);
- `NonFiniteRatio`;
- `NegativeRatio`.

If a future caller has a more specific oracle/telemetry failure, it may preserve that richer provenance separately, but it must still map into the canonical checked assessment without inventing a threshold status.

## Recovery theorem

Recovery from uncertainty is monotonic with respect to evidence history:

1. an indeterminate observation is recorded as indeterminate;
2. a later independently valid finite observation is classified normally;
3. the later valid observation becomes current health evidence;
4. the earlier uncertainty event remains historical evidence and is not rewritten or erased.

A caller-provided status string is never a substitute for step 2.

## Implementation sequence

1. introduce the checked assessment in the smallest shared Finance crate already consumed by security-critical coordinators;
2. add malformed/zero-denominator/negative-ratio and exact-boundary tests there;
3. replace bridge-local threshold copies with that classifier;
4. preserve `Indeterminate` + reason across wire/API serialization;
5. audit risk-increasing and liquidation-adjacent callers;
6. migrate/deprecate the legacy unchecked `CollateralHealthStatus::from_ltv` only with an exact whole-file change and its own regression tests;
7. add recovery and cross-surface conformance tests.

This sequencing deliberately avoids a risky blind rewrite of the large historical `types/src/lib.rs`: security-critical consumers can move first to the checked API while the legacy compatibility helper remains explicitly non-authoritative until its exact migration is qualified.

## Acceptance tests

- `NaN`, `+Inf`, `-Inf`, missing valuation, zero valuation denominator, negative ratios, and other malformed inputs classify as typed Indeterminate;
- invalid telemetry cannot produce Healthy;
- invalid telemetry alone cannot produce an executable liquidation authorization;
- valid finite LTV values preserve the documented threshold behavior, including explicit tests for 0.80, 0.90, and 0.95 boundaries;
- bridge coordinator and shared checked classification are conformance-tested against one canonical implementation;
- serialized API/wire behavior distinguishes Indeterminate from threshold-derived Liquidation and preserves the reason;
- risk-increasing operations reject an indeterminate current assessment;
- liquidation-adjacent operations reject an indeterminate current assessment as evidence of liquidation;
- recovery test: `Indeterminate -> new valid finite observation -> Known(status)` without erasing the earlier uncertainty event.

## Non-claim

This theorem does not define default, insolvency, seizure, or liquidation authority. It only separates valuation uncertainty from actual collateral-health evidence so those later decisions cannot be driven by malformed telemetry.
