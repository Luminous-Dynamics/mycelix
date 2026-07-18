# Finance Settlement and Reputation Integrity v1

## Scope

This patch separates three concepts that must not be collapsed into one local status field:

1. **Marketplace delivery state** — the buyer confirms that the transaction reached `Delivered`.
2. **Economic finality** — Finance returns a matching `Completed` payment record.
3. **Reputation evidence** — immutable events are projected from exact delivered or arbitration records.

Marketplace therefore remains `Delivered` after settlement. It does not write a locally forgeable `Completed` transaction revision. The settlement view is recovered from Finance using the stable transaction root.

## Retry-safe Finance contract

Marketplace derives the reference `marketplace_tx:{stable_transaction_root}` and authoritative payer, recipient, amount, and SAP currency from the resolved transaction. The Finance patch uses `(source_happ, reference)` as the idempotency key, persists intent before moving value, and exposes `verify_payment_status_remote` for response-loss recovery.

A retry that sees a completed record returns that record without transferring again. A visible `Processing` record fails closed because the caller cannot prove whether an interrupted transfer completed. This is deliberate **at-most-once retry behavior**, not a claim of atomic rollback across DHT writes and token movement.

## Reputation model

Direct MATL score mutation is disabled. Reputation is projected from immutable, integrity-validated events:

- buyer-authored fulfillment evidence bound to the exact `Delivered` transaction revision;
- arbitration-winner evidence bound to the exact `ArbitrationResult`;
- arbitration-loser evidence bound to that same result.

Each event has a deterministic semantic key. Identical retries return the existing event; conflicting records under one semantic key fail closed. The derived ratio is an explainable summary and is never used as an integrity authorization weight or a Byzantine-tolerance claim.

## UI boundary

The Leptos transaction page can inspect settlement status and evidence-derived seller history. The settlement action is compiled behind the opt-in `finance-settlement` feature and remains disabled in the default build until the Marketplace and patched Finance roles are bundled and the live scenario passes.

## Required live evidence

The conductor scenario must prove:

1. a buyer and seller reduce one transaction root to a single `Delivered` leaf;
2. delivery projection creates one seller fulfillment event;
3. repeating delivery or event projection does not create another semantic event;
4. settlement and a response-loss retry return the same Finance payment identity;
5. settlement status is recoverable by stable reference;
6. the Marketplace transaction remains `Delivered`;
7. seller derived reputation counts the fulfillment once;
8. arbitration finalization creates exactly one winner and one loser event;
9. direct score mutation remains rejected.

The scenario is evidence only when executed against configured conductors with the patched Marketplace and Finance roles. Bundling or type-checking the scenario is not execution evidence.
