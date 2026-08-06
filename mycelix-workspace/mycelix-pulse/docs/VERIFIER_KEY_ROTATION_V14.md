# Verifier Key Rotation — Runtime Protocol v14

Protocol v14 separates possession of hardware-backed authority keys from proven
continuity when those keys are replaced in service.

For each accepted proof contract, Pulse publishes:

- the custody and rotation protocol identifiers;
- exact custody and rotation policy hashes;
- the active ceremony and rotation identities and epochs;
- whether routine retiring and replacement quorums were authenticated;
- whether compromise recovery used a separately role-bound recovery quorum;
- whether the replacement threshold was at least the retiring threshold;
- whether the retiring keys were disabled inside the bounded handoff window;
- the externally pinned rotation checkpoint;
- stable fail-closed blockers.

## Routine rotation

Routine changes require threshold signatures from both the retiring and
replacement ceremonies over the same canonical rotation record. A replacement
ceremony's predecessor hash is necessary but not sufficient.

## Compromise recovery

A compromised retiring ceremony is not asked to authorize its successor.
Instead, the replacement ceremony and a dedicated `custody_recovery` authority
ceremony must each satisfy their threshold over the exact rotation. The record
also binds the compromise notice and recovery record. Recovery cannot lower the
replacement threshold.

## Release and reservation binding

A production release attestation and future sender-proof reservation challenge
must bind the exact rotation policy, rotation record/checkpoint, and sequence.
Those hashes remain absent in the current implementation.

## Current runtime truth

The full DNA advertises protocol v14 with no pinned rotation policy, no verified
rotation continuity, no pinned rotation checkpoint, and proof acceptance
disabled. Append-only DHT schemas are evidence containers, not authorization.
