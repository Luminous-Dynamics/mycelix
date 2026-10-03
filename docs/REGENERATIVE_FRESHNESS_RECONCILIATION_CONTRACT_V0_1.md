# Regenerative Freshness Reconciliation Contract v0.1

## Purpose

Define the receiver-side state contract for freshness handles carried by regenerative evidence.

This document complements the regenerative evidence envelope. It does not add a mutable freshness cursor to the envelope and does not implement transport, cryptographic verification, authority, or physical inference.

The core separation is:

**envelope = evidence/reference**

**receiver state = freshness acceptance memory**

**verifier = authenticity/trust evaluation**

**authority = lifecycle decision**

**Symthaea physics = physical-model evidence**

## Receiver acceptance state

A receiver maintains state scoped to the configured freshness policy and source:

- schema_version
- policy_id
- policy_fingerprint
- expected freshness scheme
- expected freshness source
- minimum accepted epoch
- bounded acceptance window
- highest accepted epoch
- retained marker digests by epoch
- conflict-latched state

The highest accepted epoch is a receiver cursor. It is not a global statement about the Epoch Bell, vehicle state, or physical truth.

## Acceptance outcomes

The receiver should distinguish:

- Advanced: the marker is strictly newer than the highest accepted epoch and moves the cursor;
- AcceptedWithinWindow: the marker is older than the cursor but inside the configured reordering window and does not move the cursor;
- Duplicate: the exact marker digest was already retained and the operation is idempotent;
- Conflicted: the same epoch has different marker digests;
- Rollback: the marker is older than the configured acceptance window;
- Quarantined: marker identity or policy binding is invalid;
- BlockedByConflict: a prior same-epoch conflict has latched the receiver state.

An older marker inside the window is historical freshness evidence. It must not silently become the current cursor.

## Same-epoch conflict

Two different marker digests for the same epoch are preserved rather than overwritten.

This is intentional. A receiver must not manufacture a canonical winner from contradictory freshness evidence.

The conflict is latched in the receiver state. Local receipt of a later marker does not clear the conflict. Clearing or replacing conflicted state belongs to an explicit higher-level resynchronization or authority mechanism.

This prevents a late, apparently newer message from laundering an earlier sequencing conflict.

## Bounded reordering

The acceptance window is a policy parameter measured in epoch distance.

For highest accepted epoch H and incoming epoch E:

- E > H: advance;
- E = H and exact digest is retained: duplicate;
- E = H and digest differs: conflict;
- E < H and H-E <= window: retain without advancing;
- E < H and H-E > window: rollback/stale evidence.

Window size is deployment-specific and should account for marker emission variability, distribution latency, evidence-path latency, and expected in-flight reordering.

The receiver should not substitute wall-clock age for epoch acceptance when the selected freshness scheme is designed to avoid local-clock dependence.

## Offline replica reconciliation

Offline replicas may receive different subsets and in different orders.

Reconciliation is deterministic set union over retained marker-digest observations, followed by the shared policy's pruning rule.

For replicas A and B:

1. verify that both states bind to the same freshness policy;
2. choose the maximum observed highest accepted epoch;
3. union retained marker digests by epoch;
4. apply the shared bounded-window pruning rule;
5. if any retained epoch has multiple digests, enter Conflicted;
6. otherwise produce the merged state.

No replica wins because it was synchronized later, stored more records, or appears more authoritative locally.

Reconciliation is therefore commutative and order-independent for equivalent policy state.

## Replay and rollback

A replay of an exact retained marker is idempotent.

A marker outside the acceptance window is rollback/stale evidence. It may still be stored in an append-only evidence system, but it must not advance the receiver freshness cursor.

A future freshness marker is not automatically proof of valid freshness: authenticity, source identity, and marker semantics remain separate verifier concerns.

## Policy substitution

Receiver state is bound to the exact freshness policy through:

- policy identity;
- policy fields;
- a domain-separated policy fingerprint.

Applying a state under a changed freshness scheme, source, minimum epoch, or acceptance window is quarantined rather than silently reinterpreted.

This is important for offline operation because a stale policy cache must not become a hidden downgrade path.

## Evidence-envelope relationship

The Mycelix evidence envelope should continue to carry:

- freshness_scheme
- freshness_source_id
- freshness_epoch
- freshness_marker_digest
- freshness_policy_fingerprint

The envelope should not carry the mutable highest-accepted epoch as though that value were evidence.

Receiver-side freshness state belongs to the consumer and may differ between replicas until reconciliation.

## Security boundary

Freshness agreement is not authority.

Marker authenticity is not physical truth.

Replica convergence is not proof that either replica was initially correct.

A merged freshness state therefore remains an input to the verifier/authority pipeline:

**freshness marker -> receiver acceptance state -> verifier result -> authority lifecycle decision**

The physical qualification pipeline remains separate:

**sensor evidence -> corroboration -> physics consistency -> intervention/recovery verification**

Neither path should silently collapse into a generic trust score.

## Relation to current RATS work

The IETF RATS Epoch Markers draft specifies receiver-side state and explicit acceptance policy for replay/rollback detection and in-flight reordering. This contract adopts that architectural separation while keeping the implementation transport-neutral.

The acceptance window and receiver cursor are deployment state, not a new freshness wire format.

