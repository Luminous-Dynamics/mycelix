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
