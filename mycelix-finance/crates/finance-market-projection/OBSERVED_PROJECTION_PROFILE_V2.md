# FIN-MKT-002B Observed Market Projection V2

## Purpose

FIN-MKT-002B projects only what is established by one exact supplied control-event frontier plus one positive FIN-MKT-002B0 observed execution frontier.

```text
latest observed control event
!= provider current state

observed effective executions
!= complete fill history

observed target gap
!= authoritative residual quantity
```

The projector does not consume raw fills or raw fill adjustments.

## Inputs

```text
CanonicalMarketOrderIntentV1
ProjectionScopeV2
[CanonicalMarketEventObservationV1]
ObservedExecutionFrontierV1
```

`ProjectionScopeV2` binds the exact provider profile and event-observation profile. The supplied execution frontier must bind the same immutable intent commitment and provider profile.

Provider-order-reference evidence is merged monotonically:

```text
None + None       -> None
None + Some(X)    -> Some(X)
Some(X) + None    -> Some(X)
Some(X) + Some(X) -> Some(X)
Some(X) + Some(Y) -> conflict
```

## Control projection

Event observations are first reduced by observation identity:

```text
same identity + same semantics
-> one semantic event

same identity + different semantics
-> conflict
```

For one event, its mapped posture may be reported directly as latest observed control evidence.

For multiple events, V2 selects a latest event only when every event has the same non-empty chronology profile and an explicit unique provider sequence. Input order, lexical timestamps, arrival order and semantic commitment ordering are never chronology authority.

Otherwise control posture remains `ChronologyIndeterminate` or `Conflicted`.

`CancelAcceptedObserved` remains `CancelAcknowledged`, not terminal canceled. `ReplaceAcceptedObserved` remains `ReplacementAcceptedSuccessorUnresolved`.

## Execution posture

The positive execution frontier determines only observed execution evidence:

```text
Resolved + no effective executions + no historical execution identities
-> NoExecutionObserved

Resolved + effective executions + no adjustments
-> ExecutionObserved

Resolved + had adjustments
-> AdjustedExecutionObserved

AdjustmentResolutionIndeterminate
-> AdjustmentResolutionIndeterminate
```

Historical observations are not erased merely because a bust/correction removes an execution from the effective set.

## Unit-target progress

For unit-target intents, every effective execution must have the exact same unit profile and asset as the target. The projector checked-sums atomic units and returns:

```text
ObservedNone
ObservedBelowTarget
ObservedAtTarget
ObservedOverTargetConflict
```

The arithmetic gap is explicitly named `unobserved_target_gap_under_this_frontier` and is not an authoritative residual quantity.

## Notional-target progress

V2 deliberately does not derive notional progress from generic quantity × execution-price arithmetic.

```text
unit execution + execution price
!= canonical executed notional
```

Therefore:

```text
no effective executions
-> NotionalNoEffectiveExecutionObserved

one or more effective executions
-> NotionalTargetProgressIndeterminate
```

until a separately qualified provider/instrument/value theorem exists.

## Positive-type boundary

`MarketOrderObservedProjectionV2` is a positive derived theorem. Its fields are private, it has no unchecked public constructor, and it is serializable but not deserializable. Downstream code may inspect immutable getters only.

```text
projection-shaped bytes
!= MarketOrderObservedProjectionV2
```

Reconstruction requires rerunning this projector from the positive intent, event observations and positive execution frontier.

## Core duplicate-execution regression

The defining cross-layer regression is:

```text
fill observation A: execution E, 40 units
fill observation B: execution E, 40 units
        ↓
002B0: one effective execution E, 40 units
        ↓
002B: observed progress = 40 units
```

Never 80 units.

## Claim ceiling

A future exact-head PASS establishes only deterministic observed-frontier control/execution/progress projection under this exact profile. It does not establish provider currentness, frontier completeness, authoritative residual quantity, settlement/finality, position ownership, best execution, financial authority, suitability, tax/accounting correctness, compliance or autonomous Symthaea authority.
