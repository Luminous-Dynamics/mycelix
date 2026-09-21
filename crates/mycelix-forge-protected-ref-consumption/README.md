# FORGE-009E — Atomic Protected Ref Consumption

This crate couples protected-ref compare-and-swap with durable one-time request consumption in one provider transaction subject.

## Why CAS alone is insufficient

FORGE-009D closes the immediate stale-tip race:

```text
expected old ref == current ref
→ old -> new CAS
```

But CAS alone does not make a merge request permanently one-shot. If some later authorized operation returns the protected ref to the original old object, the same old request could become mechanically replayable.

FORGE-009E therefore adds a deterministic per-request consumption marker.

## Atomic transaction subject

`AtomicProtectedRefConsumptionIntentV1` is derived only from a positive `ProtectedRefTransitionIntentV1` and binds:

```text
target ref:
    expected_base -> proposed_revision

consumption marker:
    absent -> proposed_revision

same transaction subject
```

The marker is deterministic from the exact `ProtectedMergeRequestId`:

```text
refs/mycelix/forge/consumed/<digest-algorithm>/<request-digest-hex>
```

The marker value is the exact proposed revision. The intent has no `Deserialize` implementation.

## Provider observation

`AtomicProtectedRefConsumptionObservationV1` records:

- provider identity;
- exact atomic intent;
- provider transaction id;
- target-ref old/new arguments and observations;
- canonical consumption-marker ref;
- whether that marker existed before the transaction;
- requested and observed marker value;
- exact merge nonce;
- provider-native receipt commitment;
- applied / stale / already-consumed / rejected outcome.

The observation is deserializable and therefore non-authoritative.

## Positive receipt

`verify_atomic_protected_ref_consumption_v1(...)` requires an independent `AtomicProtectedRefTransactionVerifierV1` and fails closed unless:

```text
provider == verifier identity
observation.intent == exact atomic intent
target ref == exact target ref
target expected old == exact expected base
target observed old == exact expected base
target requested new == exact proposed revision
target observed new == exact proposed revision
marker ref == exact canonical marker ref
marker existed before == false
marker requested value == exact proposed revision
marker observed value == exact proposed revision
merge nonce == exact request nonce
outcome == Applied
provider-specific verifier accepts same-transaction evidence
```

Only then can Forge construct `ProviderVerifiedAtomicProtectedRefConsumptionReceiptV1`.

## Intended Git realization

A later Git adapter can realize the transaction with `git update-ref --stdin` transaction commands conceptually equivalent to:

```text
start
update <target-ref> <proposed> <expected-base>
create <consumption-marker-ref> <proposed>
prepare
commit
```

The concrete adapter is intentionally deferred until a sealed merge-execution authorization type exists. This tranche defines and verifies the transaction evidence contract but exposes no mutation API.

## Claim boundary

A positive FORGE-009E receipt establishes, relative to the named provider verifier, that the target transition and durable request-consumption marker belonged to one provider transaction.

It does **not** establish:

- M0 OfflineEvidence qualification;
- merge authorization;
- project trust in the provider identity;
- trusted time;
- release authorization;
- permanent immutability of either ref;
- a globally atomic read snapshot for concurrent readers.

The replay theorem is narrower and useful:

```text
same request id
→ same canonical marker ref
→ marker already exists
→ same atomic create precondition cannot succeed again
```

provided the marker namespace itself is protected from unauthorized deletion/rewriting.

Protection of the `refs/mycelix/forge/consumed/**` namespace must therefore be part of the repository execution/policy profile before this receipt is relied upon for durable replay prevention.

## Tests

Focused tests cover:

- deterministic namespaced marker derivation;
- exact target + marker transaction becoming positive;
- pre-existing marker rejection before provider verification;
- stale target rejection;
- marker-ref substitution rejection;
- raw observation serde round-trip remaining non-authoritative;
- public constructor surface requiring a positive FORGE-009D transition intent.

## Next

The next execution-policy tranche should bind the protected consumption-marker namespace into the repository policy/executor profile. After M0/source/authority qualification is joined, the concrete Git executor can consume that sealed authorization and execute the exact multi-ref transaction.
