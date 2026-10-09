# constitutional-effect-ledger-sqlite

This crate is the first concrete durable implementation of the constitutional
same-action fence contract.

It is intentionally **host-side**. The Holochain zome layer must not treat DHT
links as a global lock. The deployment boundary is the shared SQLite store (or
another store with equivalent linearizable, durable, conflict-detecting
transactions).

## Transaction theorem

Every mutating operation uses:

`BEGIN IMMEDIATE -> validate persisted state -> mutate attempt/replay/fence -> COMMIT`

Admission installs all three roots in one transaction:

`native replay binding + attempt record + ActionKey fence`

Terminal resolution also commits as one unit:

`owner/token checks + exact evidence binding + attempt transition + fence close/release`

Any error before COMMIT rolls back the complete mutation set.

## Durable collision keys

The schema gives durable uniqueness to:

- `attempt_identity`
- `action_key_digest`
- `native_replay_identity`

These are deliberately different namespaces.

## Deployment boundary

All boundary instances that can reach the same effecting target must use the same
SQLite database for the property to hold. Two independent database files are two
independent collision domains. Filesystem locking and crash durability are therefore
deployment prerequisites.

This adapter does not authenticate terminal evidence or grant execution authority.
A separate authority/evidence verifier must do that before terminal resolution.

## Verifier affirmation

Terminal verification is represented as a typed `VerifiedTerminalOutcomeV1`
proof. The proof carries the exact attempt identity, operation identifier,
native replay identity, action-key digest, and verification purpose that were
evaluated. The effect boundary checks all of those bindings again before it
commits or releases the durable attempt/fence state.

The type is constructed from the evaluated attempt rather than from caller-
supplied identity fields. A verifier that returns a terminal outcome for a
different attempt, action, or purpose is therefore rejected at the boundary
and the occupied fence remains held.


## Final provider-entry claim

The boundary performs a durable read of DISPATCH_PENDING, then installs a
single-winner provider-entry claim in another BEGIN IMMEDIATE transaction before
calling the provider. Reconciliation refuses to advance a DISPATCH_PENDING
attempt while that claim exists, eliminating the read-to-provider-entry race
without holding a SQLite transaction across the remote provider call.

Provider adapters receive a ProviderEntryPermitV1 instead of a bare mutable
attempt. The permit carries a frozen attempt projection and a provider
idempotency key derived from native replay identity plus provider scope. The
operation identifier is deliberately not an input to this derivation.

A claim is consumed atomically with the transition to INVOKED. If the process
fails after claim creation, the claim remains durable and reconciliation cannot
guess that provider entry did not occur. An explicitly authorized claim
recovery moves the attempt to INDETERMINATE and removes the claim in one durable
transaction; there is no unsafe lease timeout.
