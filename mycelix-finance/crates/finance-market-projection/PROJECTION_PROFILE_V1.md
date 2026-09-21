# FIN-MKT-002B Observed-Frontier Projection V1

## Scope

This profile deterministically projects an exact supplied FIN-MKT-002A observation frontier for one immutable FIN-MKT-001 order intent.

It deliberately does **not** claim that the supplied frontier is complete or current.

```text
latest observed event in frontier
!= provider current state

observed fill sum
!= exhaustive fill sum

observed target gap
!= authoritative residual quantity
```

## Exact projection scope

A projection binds one exact:

- FIN-MKT-001 intent;
- provider profile;
- event observation profile;
- fill observation profile;
- adjustment observation profile.

The observation-profile roles are separate on purpose:

```text
event evidence semantics
!= fill evidence semantics
!= adjustment evidence semantics
```

Every observation must match the intent's account/instrument/intent commitment and the exact provider/profile role selected for its observation class.

`provider_order_ref` may be absent on early observations and present later, but all present values inside one immutable intent frontier must agree. Multiple incompatible provider-order references fail closed.

## Duplicate/conflict reduction

Before projection, observations sharing one identity are reduced under FIN-MKT-002A semantics:

```text
same identity + same semantics
-> one semantic observation

same identity + different semantics
-> reject as conflict
```

Different evidence bindings for the same identity/semantics do not create duplicate control transitions or duplicate fills.

## Latest observed control posture

One unique event may be projected directly.

For multiple semantic events, V1 uses ordering only when every event has:

- the exact same non-empty chronology profile;
- an explicit provider sequence;
- a unique sequence value.

Then the greatest sequence establishes the latest **observed** control posture inside this frontier.

Otherwise the posture is chronology-indeterminate/conflicted.

Input vector order, arrival order and lexical provider timestamps never establish order.

V1 maps cancel acknowledgment separately from terminal cancellation:

```text
CancelAcceptedObserved
-> CancelAcknowledged
!= terminal Canceled
```

The current 002A profile has no terminal `CanceledObserved` event; FIN-MKT-002A3 tracks that future vocabulary extension.

Likewise:

```text
ReplaceAcceptedObserved
-> ReplacementAcceptedSuccessorUnresolved
```

until a separately qualified predecessor-successor lineage theorem exists.

## Fill correction / bust resolution

Fill adjustments name fill semantic commitments.

V1 resolves references only against admitted fills in the supplied frontier.

Missing references do not become zero, harmless history or implicit rejection. They make execution projection indeterminate.

Correction topology rules:

- one prior fill may have at most one effective outgoing correction-or-bust edge;
- one replacement fill may have at most one correction predecessor;
- cycles fail closed;
- exact duplicate relations are idempotent;
- unresolved references withhold execution progress.

A correction removes its predecessor from the effective observed-fill set and points to its replacement. A bust removes its referenced fill. Chained corrections are supported when the graph is acyclic and fully resolved.

## Dimension-safe observed progress

### Unit-target orders

For `QuantitySpecV1::Units`, each effective observed fill must use the exact target unit profile and exact target asset ID.

Then V1 computes:

```text
effective_observed_atomic_units
```

with checked arithmetic and compares only the **observed frontier sum** with the target:

```text
ObservedNone
ObservedBelowTarget
ObservedAtTarget
ObservedOverTargetConflict
```

For `ObservedBelowTarget`, V1 may expose:

```text
unobserved_target_gap_under_this_frontier
```

but that value is not an authoritative residual quantity without a separate completeness theorem.

### Notional-target orders

V1 never derives notional execution by generic `quantity * price` arithmetic.

```text
unit fill + execution price
!= canonical executed notional
```

Therefore:

```text
notional target + effective observed fills
-> NotionalTargetProgressIndeterminate
```

A later qualified provider-reported executed-notional/conversion theorem may unlock exact notional progress.

## Frontier completeness/currentness firewall

This profile projects supplied evidence only.

It does not establish:

- snapshot/history pagination completeness;
- stream continuity;
- absence of unseen fills/events;
- provider currentness;
- authoritative residual quantity.

A future completeness/currentness admission should be provider-evidence-driven and should not be extracted generically until at least two independent providers demonstrate the same theorem.

## Dependency boundary

The crate is pure Rust semantics and contains no Robinhood/Alpaca SDK, FIX engine, MCP, HTTP/WebSocket, Holochain, database, wall clock, OAuth/token/secret or Symthaea runtime dependency.

## Claim ceiling

A future V1 PASS establishes only deterministic, bounded, dimension-safe projection over the exact supplied observation frontier.

It does not establish provider truth beyond supplied evidence, provider current state, complete fill history, financial authority, buying power, position ownership, settlement/finality, best execution, suitability, legal/regulatory compliance, tax/accounting correctness or autonomous Symthaea authority.
