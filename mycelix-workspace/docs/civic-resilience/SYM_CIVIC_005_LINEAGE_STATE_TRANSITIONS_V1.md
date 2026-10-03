# SYM-CIVIC-005 — temporal invalidation and lineage state transitions v1

Status: synthetic research provenance qualification only

Parent: SYM-CIVIC-004 result/claim binding / `dc847acae5c52383c797266e676f901db8b17e71`

Tracking issue: #3847

## Purpose

Test provenance as a temporal state machine rather than a collection of static validity fields.

The research lineage is:

`ExecutionEvidence -> ResultArtifact -> ScientificClaim`

Validity propagates downstream. Historical receipts are append-only; replacement versions create new identities and require explicit derivation and a new binding.

## State rules

- `VALID` may transition to `INVALIDATED` or `SUPERSEDED`.
- `INVALIDATED` is terminal for that entity identity. "Restoring" an underlying source never resurrects the old identity.
- `SUPERSEDED` is terminal for the old identity.
- A replacement result is a new entity identity.
- A downstream claim is valid only when every bound upstream identity is valid.
- Mutable `latest` pointers never establish provenance and cannot repair an invalidated binding.
- Event identity is immutable and replay is idempotent only when the replay is byte-equivalent.
- Conflicting reuse of an event identifier is rejected.
- Temporal ordering is by canonical effective time, then immutable event identifier; stale events cannot roll a terminal state backward.
- Invalidation receipts and historical claims are append-only. A changed receipt is a new artifact, not an in-place correction.

## Adversarial corpus

T-01 source invalidation after claim qualification
T-02 result mutation after claim qualification
T-03 result supersession without new claim binding
T-04 source restoration cannot resurrect old identity
T-05 mutable latest pointer rebound after invalidation
T-06 replacement result without derivation
T-07 valid replacement with explicit derivation/new binding
T-08 stale out-of-order invalidation cannot roll state backward
T-09 conflicting duplicate/replayed invalidation event
T-10 mutated invalidation receipt
T-11 historical claim receipt rewritten in place
T-12 stale downstream cache survives invalidation
T-13 exact replay is idempotent
T-14 canonical event ordering is arrival-order invariant
T-15 valid append-only replacement history

The qualifier derives dispositions from state-transition semantics; fixture files do not contain expected verdicts.

## Qualification ceiling

PASS establishes only that this synthetic corpus detects the declared temporal lineage failures and preserves append-only provenance semantics.

It does not establish real-world scientific truth, causal effect, public safety, clinical validity, civic legitimacy, or deployment readiness.

No runtime implementation is proposed.
