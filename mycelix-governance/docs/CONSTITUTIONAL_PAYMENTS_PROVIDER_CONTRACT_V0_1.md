# MYC-CONST-003D1D-F1A — Inert Payments Provider Operation Contract

F1A is the pure semantic successor to F0P0. It does not modify the Finance payments zome, Treasury, Governance routing, DHT entries, balances, receipts, or provider capability minting.

## Identity

One provider operation is derived from one exact F0 effect intent plus one exact provider profile:

```text
provider_operation_key = H(
  domain/version,
  provider_profile_id,
  provider_profile_commitment,
  F0 execution_id,
  F0 request_commitment
)
```

Wall clock, attempt IDs, historical `Payment.id`, and receipt IDs are excluded. Thus transport retry cannot mint a new semantic provider operation merely because time changed.

The historical payments `Payment.id` remains a provider audit-record identity. A future live adapter may associate it with the deterministic provider operation, but must not treat the timestamp ID itself as the idempotency key.

## Provider observation model

Every provider-origin observation binds the exact:

- provider operation key;
- F0 execution ID;
- F0 request commitment;
- observation ID.

Outcomes are deliberately limited to:

```text
KnownSuccess
KnownNoEffect
UnknownOutcome
```

`KnownSuccess` carries payment/receipt evidence. `KnownNoEffect` carries explicit no-effect evidence. `UnknownOutcome` carries provider uncertainty evidence. No transport exception is automatically promoted into `KnownNoEffect` by this pure model.

## Conflict rules

- identical observation ID + identical observation => idempotent `ExistingSame`;
- identical observation ID + changed observation => integrity halt;
- success plus no-effect evidence => integrity halt;
- multiple incompatible success receipts => integrity halt;
- unknown evidence after a terminal result is historical-only and cannot demote the terminal state.

## Registry rules

The pure registry enforces one provider operation per F0 `execution_id`.

Submitting the exact same provider intent returns `ExistingSame`. Changing the provider-profile commitment for the same F0 execution produces an integrity conflict rather than a second provider operation.

## Activation boundary

F1A has no persistent provider intent, no payments-zome index, no authoritative provider query, no live Governance route, and no replay-capability minting path.

The next production-facing successor must qualify, separately:

1. the exact value/authority/capacity join;
2. durable provider intent before dispatch;
3. a payments operation-key index;
4. duplicate/conflict/concurrency semantics;
5. authoritative query outcomes;
6. verified machine-readable receipts;
7. lost-ack and concurrent-duplicate behavior;
8. provider capability qualification;
9. live Governance routing.

A provider capability may only be minted after those provider-specific semantics are exact-head qualified.

## Non-claims

F1A does not establish live payments-provider behavior, persistence, provider truth, qualified replay, public-fund authority, qualified SAP values, physical exactly-once execution, external settlement finality, or deployment currentness.
