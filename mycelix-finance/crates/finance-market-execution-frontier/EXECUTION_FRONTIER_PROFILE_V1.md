# FIN-MKT-002B0 Observed Execution Frontier V1

## Scope

This profile resolves provider-neutral FIN-MKT-002A fill observations into unique observed economic executions before FIN-MKT-002B projects order progress.

```text
fill observation envelope
!= economic execution
```

A provider may expose one economic execution through multiple observation IDs, polling snapshots, stream envelopes, or independently captured evidence records.

FIN-MKT-002B0 therefore owns only:

- exact scope validation;
- fill-observation duplicate/conflict reduction;
- provider-scoped execution de-duplication;
- fill-adjustment alias resolution;
- effective observed execution frontier construction.

It does not own control-state projection, notional arithmetic, frontier completeness/currentness, financial authority, position ownership, settlement, or portable attestation.

## Exact scope and positive-result binding

The resolver consumes one exact positive `CanonicalMarketOrderIntentV1` plus:

```text
ExecutionFrontierScopeV1 {
    provider_profile,
    fill_observation_profile,
    adjustment_observation_profile,
}
```

Every supplied fill/adjustment must bind the exact intent/account/instrument lineage and exact provider/profile role.

The positive `ObservedExecutionFrontierV1` carries both:

```text
exact intent commitment
exact ExecutionFrontierScopeV1
```

so the result cannot silently lose the context that gives provider execution refs meaning.

```text
same execution ref/economics under scope A
!= same execution identity theorem under scope B
```

Within one scope:

- no non-empty provider-order ref may conflict with another non-empty ref;
- `None` may coexist with one exact later `Some(X)` provider-order ref;
- provider order IDs remain scoped evidence, not global identity.

## Observation identity first

FIN-MKT-002A's existing observation theorem remains authoritative:

```text
same observation identity + same semantic commitment
-> one observation semantic claim

same observation identity + different semantic commitment
-> observation identity conflict
```

Different evidence bindings for the same observation identity/semantics do not create another fill observation semantic claim.

## Economic execution identity

After observation reduction, V1 groups fills by exact `provider_execution_ref` **inside the already-validated exact intent/provider/fill-observation-profile scope**.

The fill observation profile owns the reviewed interpretation of `provider_execution_ref` on that provider surface.

```text
provider execution ref text
!= globally unique execution identity
```

The same text under another account, instrument, provider, intent, or fill observation profile is outside this execution frontier and must not be merged.

## Economic execution semantics

Two observations with one execution ref describe the same economic execution only when V1's required economic fields are compatible.

Exact-match fields:

```text
executed quantity unit profile
executed quantity asset ID
executed quantity atomic amount
execution price profile
execution price quote asset ID
execution price atomic amount
```

Optional venue is monotonic enrichment:

```text
None + None       -> None
None + Some(X)    -> Some(X)
Some(X) + None    -> Some(X)
Some(X) + Some(X) -> Some(X)
Some(X) + Some(Y) -> conflict
```

This permits later evidence to identify a venue without creating a second execution while refusing incompatible non-empty venues.

Observation chronology and source-evidence binding are deliberately not economic execution identity.

Therefore:

```text
same execution ref + compatible exact economics + different observation IDs
-> one observed economic execution

same execution ref + different quantity/price or incompatible venue
-> execution identity conflict
```

V1 does not heuristically merge different execution refs by quantity, price, time, venue, or proximity.

## Observation aliases

Each unique economic execution retains the sorted set of supporting FIN-MKT-002A fill semantic commitments.

This supports exact adjustment alias resolution and diagnostics while preserving:

```text
more observation envelopes
!= more economic executions
```

## Adjustment alias resolution

FIN-MKT-002A r3 corrections/busts reference fill semantic commitments, which are observation-envelope-bound.

V1 creates an alias map:

```text
fill semantic commitment -> provider-scoped economic execution
```

Then correction/bust topology is resolved over economic executions.

A correction:

```text
prior fill semantic ref -> prior execution
replacement fill semantic ref -> replacement execution
```

A bust:

```text
prior fill semantic ref -> prior execution
```

If a referenced fill semantic commitment is missing from the supplied frontier, adjustment resolution is **indeterminate**. Missing refs never become zero quantity, harmless history, or already-busted state.

If prior and replacement refs collapse to the same economic execution, the relation conflicts and fails closed.

V1 also fails closed on:

- two incompatible outgoing correction/bust relations from one execution;
- two different predecessor executions targeting one replacement execution;
- correction cycles;
- observation identity conflicts;
- execution identity conflicts.

Duplicate semantic adjustment relations are idempotent after alias resolution.

## Effective execution frontier

When adjustment resolution is complete, the effective observed executions are exactly the unique executions with no outgoing correction/bust relation.

Historical observations are never erased.

```text
Bust(A)
-> A absent from effective observed execution set
!= A never occurred

Correction(A -> B)
-> A absent, B effective unless B is itself adjusted
!= historical A deleted
```

Each effective execution exposes:

- exact provider execution ref;
- exact executed quantity;
- exact execution price;
- best available compatible optional venue under V1 enrichment;
- sorted supporting fill semantic commitments;
- supporting observation count.

## Result model

Conceptually:

```text
ObservedExecutionFrontierV1 {
    intent_commitment,
    scope,
    provider_order_ref_observed,
    unique_fill_observation_count,
    unique_execution_count,
    unique_adjustment_count,
    resolution,
}

resolution =
    Resolved {
        had_adjustments,
        effective_executions,
    }
  | AdjustmentResolutionIndeterminate
```

The result is an in-process positive value only. It is not a signed or portable projection receipt. FIN-MKT-002B5 / #2768 governs any later frontier/projection commitment.

## Boundedness

V1 admits at most 4096 total fill + adjustment observations per invocation.

All counters use checked conversion. No arbitrary bounded input may panic.

## Relationship to later projection

FIN-MKT-002B should consume this execution frontier rather than raw fill envelopes.

For a unit-target order, later progress code may aggregate the exact quantity of effective executions only after exact target-unit/profile compatibility checks.

For a notional-target order, FIN-MKT-002B1 / #2760 still applies:

```text
unit execution + execution price
!= canonical executed notional
```

## Required source/adversarial coverage

V1 source tests must include:

1. same execution ref + same economics + two observation IDs -> one execution;
2. same execution ref + alternate evidence -> one execution;
3. same execution ref + different quantity -> conflict;
4. different execution refs + identical economics -> two executions;
5. `venue None -> Some(X)` enriches one execution;
6. two incompatible non-empty venues under one execution ref -> conflict;
7. positive result binds exact intent commitment + scope;
8. correction referencing one alias of a duplicated execution -> one corrected execution;
9. correction prior/replacement collapse to same execution -> conflict;
10. missing adjustment ref -> indeterminate;
11. bust removes execution only from effective frontier;
12. correction chain resolves to terminal replacement;
13. cycle -> conflict;
14. provider-order-ref conflict -> fail closed;
15. arbitrary bounded frontier -> typed result/error, never panic.

## Claim ceiling

A future PASS establishes only deterministic provider-profile-scoped observed-execution de-duplication and adjustment resolution over the supplied frontier.

It does **not** establish:

- provider truth beyond supplied evidence;
- provider/current order state;
- frontier completeness or currentness;
- complete fill history;
- authoritative residual quantity;
- financial authority or buying power;
- position ownership;
- settlement/finality;
- best execution;
- investment suitability;
- legal/regulatory compliance;
- tax/accounting correctness;
- autonomous Symthaea authority.
