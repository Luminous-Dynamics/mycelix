# FIN-MKT-002B0 Positive-Type Boundary V1

## Purpose

`ObservedExecutionFrontierV1` is a positive derived theorem produced only by the FIN-MKT-002B0 resolver.

A downstream caller must not be able to construct or mutate a positive execution frontier merely because it can name the Rust type.

```text
struct-shaped bytes
!= resolved execution frontier
```

## Construction rule

The following positive types have private fields and no public unchecked constructor:

```text
ObservedExecutionFrontierV1
EffectiveObservedExecutionV1
```

They are returned only by `resolve_observed_execution_frontier_v1` after exact scope, observation identity, execution identity, adjustment topology and boundedness checks succeed.

Read access is provided through immutable getters.

## Wire rule

Positive execution-frontier values may implement `Serialize` for diagnostics/UI transport, but deliberately do **not** implement `Deserialize`.

```text
serialized positive value
!= portable reconstruction input
!= attested receipt
```

A caller receiving JSON or another wire representation must reconstruct from the underlying FIN-MKT-002A observations and run the resolver again; it cannot deserialize bytes directly into the positive theorem.

FIN-MKT-002B5 / #2768 owns any future portable frontier/projection commitment or signed receipt.

## Input types

`ExecutionFrontierScopeV1` remains caller-constructible because it is a request/profile coordinate, not a positive theorem.

Likewise FIN-MKT-002A canonical observations remain the qualified/reconstructable evidence inputs consumed by the resolver.

## Mutation rule

No setter or mutable reference to positive fields is exposed.

A new observation frontier requires a fresh resolver invocation. A caller cannot append an execution, change counts, replace the bound intent/scope, or switch an indeterminate adjustment result to `Resolved` in place.

## Claim ceiling

This boundary prevents API-level theorem forgery inside safe Rust. It does not itself establish that upstream evidence is true/current/complete, nor does it create a cryptographic attestation, financial authority, settlement, suitability, compliance, or autonomous Symthaea authority.
