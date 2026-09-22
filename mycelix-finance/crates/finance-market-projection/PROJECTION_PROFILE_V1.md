# FIN-MKT-002B Observed Projection V1

## Scope

FIN-MKT-002B projects an exact supplied control-event frontier together with one positive FIN-MKT-002B0 observed execution frontier for one immutable FIN-MKT-001 market-order intent.

```text
control observations
+
ObservedExecutionFrontierV1
-> deterministic observed projection
```

It does not ingest raw fill or adjustment observations. FIN-MKT-002B0 owns provider-scoped economic-execution de-duplication and adjustment resolution first.

```text
fill observation envelope
!= economic execution

002B raw fill aggregation
= forbidden
```

The projection remains evidence-local:

```text
latest observed event in supplied frontier
!= provider current state

observed effective execution sum
!= exhaustive execution sum

observed target gap
!= authoritative residual quantity
```

## Exact coordinates

A projection binds:

- exact FIN-MKT-001 intent commitment;
- exact projection provider profile;
- exact event-observation profile;
- the exact positive `ObservedExecutionFrontierV1` consumed by the projector.

The execution frontier itself retains its exact provider/fill-observation/adjustment-observation scope.

The projector rejects:

- an execution frontier for another intent commitment;
- an execution frontier under another provider profile;
- event subjects outside the immutable intent/account/instrument lineage;
- event observation profiles outside the selected event role;
- incompatible non-empty provider-order references across event and execution evidence.

## Positive-type boundary

`MarketOrderObservedProjectionV1` is a positive derived theorem.

Its fields are private, it has no unchecked public constructor, and it is serializable for inspection but deliberately not deserializable.

```text
JSON shaped like a projection
!= MarketOrderObservedProjectionV1
```

Reconstruction requires the positive FIN-MKT-002B0 frontier plus the exact control-event observations and rerunning the projector.

Any future portable signed/attested projection receipt remains owned by FIN-MKT-002B5 / #2768.

## Control posture

Event duplicate/conflict reduction follows FIN-MKT-002A:

```text
same identity + same semantics
-> one semantic event

same identity + different semantics
-> conflict
```

A single semantic event may be projected directly.

For multiple semantic events, latest-observed control posture is derived only when every event has:

- the exact same non-empty chronology profile;
- an explicit provider sequence;
- a unique sequence value.

Input vector order, arrival order, lexical provider timestamps and semantic-hash order never establish chronology.

Current conservative mappings remain:

```text
CancelAcceptedObserved
-> CancelAcknowledged
!= terminal cancellation

ReplaceAcceptedObserved
-> ReplacementAcceptedSuccessorUnresolved
!= successor intent identified
```

Exact predecessor-successor replace lineage remains owned by #2761.

## Execution posture

Execution posture derives only from the positive FIN-MKT-002B0 resolution.

```text
Resolved + no effective executions + no adjustments
-> NoExecutionObserved

Resolved + effective executions + no adjustments
-> ExecutionObserved

Resolved + adjustment evidence
-> AdjustedExecutionObserved

AdjustmentResolutionIndeterminate
-> AdjustmentResolutionIndeterminate
```

A bust/correction may remove an execution from the effective frontier without claiming that the historical execution never occurred.

## Dimension-safe progress

### Unit target

For `QuantitySpecV1::Units`, every effective execution must exactly match the target unit profile and target asset ID.

The projector sums only the de-duplicated effective economic executions supplied by FIN-MKT-002B0 with checked arithmetic.

```text
two observation envelopes
+ same admitted provider execution identity
+ 40 units economics each
-> 40 observed units
!= 80 observed units
```

The frontier-local comparison is:

```text
ObservedNone
ObservedBelowTarget
ObservedAtTarget
ObservedOverTargetConflict
```

`unobserved_target_gap_under_this_frontier` is arithmetic over supplied evidence only and is not authoritative residual quantity without a separate completeness/currentness theorem.

### Notional target

V1 does not derive executed notional by generic quantity-times-price arithmetic.

```text
unit execution + execution price
!= canonical executed notional
```

Therefore:

```text
notional target + no effective execution
-> NotionalNoEffectiveExecutionObserved

notional target + effective execution(s)
-> NotionalTargetProgressIndeterminate
```

A later provider/instrument-specific value theorem may unlock exact notional progress.

## Provider execution-ID dependency

FIN-MKT-002B inherits FIN-MKT-002B0's dependency on qualified provider execution identity.

FIN-ADAPT-EXECID-001 / #2774 requires each adapter to prove the semantics and uniqueness scope of the upstream execution ID before fill observations may feed FIN-MKT-002B0.

Adapters may not synthesize an execution identity from order ID, quantity, price, timestamp or proximity heuristics.

## Boundedness

V1 admits at most 4096 control-event observations per projection call. FIN-MKT-002B0 independently bounds its execution frontier construction.

All event counters and unit aggregation use checked conversion/arithmetic. Bounded arbitrary input must produce a typed result/error rather than panic.

## Dependency boundary

The crate is pure Rust semantics. It contains no Robinhood/Alpaca SDK, FIX engine, MCP, HTTP/WebSocket, Holochain, database, wall clock, OAuth/token/secret, broker credential or Symthaea runtime dependency.

## Claim ceiling

A future PASS establishes only deterministic, bounded projection over:

- the exact supplied control-event frontier; and
- one exact positive FIN-MKT-002B0 execution frontier.

It does not establish provider truth beyond supplied evidence, frontier completeness/currentness, provider current state, authoritative residual quantity, position ownership, settlement/finality, best execution, financial authority, suitability, legal/regulatory compliance, tax/accounting correctness or autonomous Symthaea authority.
