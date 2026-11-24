# FIN-EFFECT-003 — Authority epoch transition and durable revocation v0.1

**Status:** bounded TLA+ qualification model and synthetic adversarial corpus; not a production witness protocol.

## Purpose

Extend the PCOE payment / claim-admission model from PR #6595. A fixed membership set is not enough: replacing it can destroy the quorum-intersection argument that prevents two incompatible authority histories from certifying the same economic effect.

This model uses a deliberately small overlap between two membership sets and requires **both** the old and new membership quorums to certify one identical transition digest. It does not claim that Raft's joint consensus can be copied directly into a Byzantine witness protocol. The finite model is a candidate construction whose assumptions and boundaries must be reviewed before implementation.

## Transition protocol represented

`StableOld → JointPrepared → JointCertified → StableNew`

- In `StableOld`, only the old authority can admit new effects.
- `JointPrepared` freezes new owner-effect admission while the exact transition digest is collected.
- One logical witness vote is journaled by logical identity. If the identity belongs to both sets, its one signature is eligible in both membership calculations, but it is never counted twice inside either set.
- `JointCertified` requires the old quorum **and** the new quorum over the same digest.
- `StableNew` is entered only after the joint certificate. The old epoch is then closed to new admissions.
- Owner effects admitted before the barrier remain historical facts; transition does not rewrite or erase them.
- New-epoch effects cannot be admitted before activation. Existing effects remain visible to the shared singleton-effect key, preventing a second conflicting effect after handover.

The example configuration uses old set `{w1,w2,w3,w4}`, new set `{w2,w3,w4,w5}`, one Byzantine identity `w4`, and threshold 3 in each set. The two sets overlap in three logical identities. The quorum-intersection expression is checked separately for each set. This is one finite instance, not a proof for arbitrary membership sizes or Byzantine models.

## Negative controls

The pinned TLC workflow expects each unsafe mutation to violate its named invariant:

1. old-side quorum accepted without new-side quorum → `JointCertificateRequiresBothQuorums`;
2. new-side quorum accepted without old-side quorum → same invariant;
3. direct old-to-new switch → `NewEpochActivationRequiresJointCertificate`;
4. rollback of an honest witness vote journal plus digest rewrite → `HonestNonEquivocation`;
5. old-epoch effect after the transition barrier → `OldEpochClosedAfterActivation`;
6. new-epoch effect before activation → `NewAdmissionRequiresActivation`.

The safe configuration checks all safety invariants together. A deliberate mutant is considered caught only when TLC exits unsuccessfully and the expected invariant name appears in its log.

## Invariants

- per-set quorum intersection under the declared fault bound;
- one immutable transition digest after preparation;
- honest witness non-equivocation across old/new membership;
- both quorums required for a transition certificate;
- no conflicting transition certificates;
- new epoch activation requires a joint certificate;
- old epoch closed after activation;
- no new-epoch admission before activation;
- no conflicting or duplicate owner effects across epochs;
- spendability only after an owner effect.

## Corpus and coverage boundary

`../pcoe-authority-epoch-transition-corpus.v0.1.json` defines 16 synthetic adversarial cases and has a Draft 2020-12 schema. The corpus is not yet an executed performance benchmark. Four cases are deliberately marked not-in-model: key alias resolution, trusted snapshot rollback proof, key revocation/compromise timing. Partial cases cover stale certificates and concurrent duplicate transitions. These are explicit next implementation/recovery obligations, not silently assumed successes.

## What is not proven

This model assumes:
- authenticated logical witness identities and correct set membership;
- an externally verified Byzantine bound;
- correct cryptography and canonical transition digest encoding;
- durable vote-journal semantics in the safe model;
- a trustworthy checkpoint that carries all prior admitted singleton effects and journal roots into the new epoch;
- explicit governance over the transition proposal and policy revision.

It does not model cryptographic signatures, actual key ownership, key aliases, disk flush ordering, crash-consistent storage, restore/rollback detection, revocation propagation, network partitions as messages, arbitrary-`n` quorum derivation, epoch reconfiguration at scale, Holochain DHT completeness, source-chain forks, owner zomes, external payment rails, or legal finality. It does not prove that a transition service currently exists in Mycelix.

A critical follow-up is to implement an append-only transition certificate and journal-recovery verifier, then run crash/restart/rollback tests against the selected storage and Holochain runtime. No production spending permission may rely on this abstract model alone.

## Economic OS composition

This is an authority-lifecycle primitive, not a payment-only primitive. See the separate Economic OS effect-control-plane contract proposed in issue #6600: operation identity and immutable intent bind the economic obligation; per-owner effect identities distinguish each actual mutation; admission authorities control who can claim a singleton effect; owner adapters commit only their local effects; evidence profiles determine what can be reconciled or called spendable. Currency identity, unit profiles, policy/jurisdiction versions, accounting classifications and external settlement finality remain domain-owned and must not be collapsed into the effect identity.

## Benchmark targets (not results)

- 100 identical concurrent transition requests → one transition identity, one certificate and one active epoch;
- 100 distinct payloads under one transition identity → 100% conflict/hold, zero digest rewrite;
- 0 certificates accepted with only one side's quorum;
- 0 old-epoch new admissions after the barrier; 0 new-epoch admissions before activation;
- 0 conflicting spendable effects across the declared fault model;
- 0 honest double-signs across restart, rotation and restore once the durable journal verifier exists;
- report p50/p95/p99 transition latency, liveness during partitions, time spent in `Indeterminate`, checkpoint bytes, journal flush/fsync cost, recovery time, key-revocation propagation and operator effort.

Safety and liveness are separate axes. A safe pause during quorum loss is not a liveness pass, and a fast unsafe transition is a safety failure.

## Industry comparison

- **Raft joint consensus** requires majorities of both old and new configurations during the joint phase and disallows unilateral decisions by either configuration. It is a relevant membership-change comparator, but Raft's crash-fault model is not a Byzantine witness proof.
- **CometBFT/Tendermint-style validator updates** are height-bound and versioned by consensus state. This motivates explicit activation/checkpoint heights rather than wall-clock lease expiry; it is not a direct replacement for Mycelix's owner/effect semantics.
- **Temporal** persists orchestration history and retries Activity attempts, while explicitly recommending idempotent external side effects. Durable workflow history is not by itself an external payment uniqueness theorem.
- **AWS transactional outbox** atomically couples a local state update and outbox record, but downstream delivery can be at least once; consumers still need idempotency and reconciliation.
- **BIS Project Agorá** is a materially more mature institutional comparator for multi-currency programmable settlement: its 2026 real-value testing involved 28 institutions/central banks, about CHF 800,000 and 17 scenarios. Mycelix should not claim to outperform that. Our near-term contribution is a reusable evidence/identity/authority layer spanning community economic events, owner effects and adapters—not a wholesale settlement network.

The differentiation target is not a bigger TPS number in isolation. It is lower **unqualified-effect risk** with explicit finality boundaries, deterministic replay, controlled cross-domain mapping and a public, reproducible adversarial evaluation.

