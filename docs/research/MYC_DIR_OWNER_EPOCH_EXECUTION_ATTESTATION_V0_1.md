# Mycelix Directory Fabric: owner-epoch execution attestation v0.1

**Profile:** MYC-DIR-OWNER-EPOCH-EXECUTION-ATTESTATION-0.1  
**Status:** migration, SQL adversarial test, bounded TLA+ model and PostgreSQL/NATS integration source added; exact-head CI and TLC execution pending  
**Related:** PostgreSQL receipt-verifier work, tools/myc-dir-fabric-reference/postgres/001_effect_owner.sql, 002_worker_claim_fencing.sql, and the PostgreSQL receipt verifier.

## 1. Paradigm shift: persist the authority coordinates at the effect boundary

A message delivery, worker log or later read of the current claim is not enough to reconstruct which authority state was actually accepted when an irreversible effect committed.

This increment makes the effect owner write an immutable execution attestation in the same transaction as the protected mutation and the append-only effect receipt.

The attestation preserves:
- owner epoch;
- accepted resource generation;
- exact worker ID;
- worker claim epoch;
- worker claim proof commitment;
- claim clock profile and validity interval;
- logical operation and exact resource scope;
- idempotency, request, decision and payload commitments;
- PostgreSQL transaction ID.

No worker log or later lookup of the current claim table substitutes for this historical record.

## 2. Migration 003

Adds tools/myc-dir-fabric-reference/postgres/003_owner_epoch_execution_attestation.sql.

### Owner epoch

Adds owner_epoch BIGINT NOT NULL DEFAULT 1 to effect_fence_state.

advance_effect_owner_epoch:
1. locks the owner fence row exclusively;
2. requires a strictly greater owner epoch;
3. requires nonzero epoch-authority and physical-fencing receipt commitments;
4. advances the resource generation at the same time;
5. writes an append-only effect_owner_epoch_transitions row;
6. returns the accepted (owner_epoch, generation).

The tuple is ordered lexicographically. The transition function can skip epochs after recovery, but the external authority must prove the chosen epoch is above its independent high-water.

**Important:** SQL checks that receipt commitments are nonzero and stores them. It does not authenticate their signatures or physically fence a former primary. The control plane must validate the epoch authority and physical fencing receipts and prevent the old primary from accepting writes before invoking the transition.

### Execution attestation

Adds effect_execution_attestations, with one immutable row per effect_commits.effect_id.

apply_owner_epoch_claim_fenced_balance_delta is the strict queue-backed entry point:
1. locks and checks the exact current worker claim;
2. locks and checks the exact current owner epoch/resource generation;
3. calls the existing claim-fenced mutation inside the same transaction;
4. writes the execution attestation before commit;
5. returns the new attestation identity and accepted authority coordinates.

Both the epoch-transition and execution-attestation tables have append-only UPDATE, DELETE and TRUNCATE guards. The effect_commits table also receives a TRUNCATE guard.

Migration 003 also adds `owner_epoch` to `worker_claim_state` and introduces `install_worker_claim_for_owner_epoch`. It locks claim then owner-fence, matching the effect path lock order. When the owner epoch changes, the worker claim epoch must strictly increase; the strict effect entry point rejects a claim installed under a different owner epoch. The prior entry points remain reference functions. Production application roles must be granted only the epoch-aware installer and strict mutation entry point; do not grant worker roles direct execution access to older mutation functions.

## 3. Replay semantics

An exact idempotent replay must not create another effect or overwrite the historical attestation.

The new wrapper requires:
- the current command to present the current owner epoch;
- the current worker claim to be valid;
- the existing effect row to match the exact resource, operation, idempotency, request, decision and payload commitments;
- the original execution attestation to exist;
- the original effect position not to be newer than the replay command position.

A replay returns the original effect ID and original execution coordinates. Legacy effects created before migration 003 have no attestation and fail closed through this strict entry point until a separate reconciliation process can reconstruct or classify them.

## 4. Independent PostgreSQL verifier

The NATS adapter's PostgresEffectCommitVerifier now joins:
- effect_commits;
- effect_execution_attestations;
- the matching effect_owner_epoch_transitions row for non-initial owner epochs.

The canonical receipt commitment now includes:
- effect ID and execution-attestation ID;
- resource scope and command digests;
- owner epoch and accepted generation;
- worker ID, claim epoch, claim proof commitment;
- claim clock profile and validity interval;
- database transaction ID;
- epoch-authority and physical-fencing receipt commitments.

The verifier rejects a missing attestation, missing epoch-transition record, read-only/recovery server, future owner position, worker/claim-field mismatch, digest substitution and canonical digest mismatch.

It verifies the presence and binding of external epoch/fencing receipt commitments. Their signatures and the external primary-fencing mechanism still require independent verification. A SQL boolean such as pg_is_in_recovery() = false cannot rule out two writable primaries.

## 5. Database adversarial test

Adds tools/myc-dir-fabric-reference/postgres/tests/003_owner_epoch_execution_attestation.sql.

The script runs in a disposable transaction and checks:
- initial owner epoch 1 and generation 1;
- initial worker claim and exact execution-attestation contents;
- stale owner epoch rejected;
- owner epoch advance to 2 also advances resource generation to 2;
- old worker claim rejected after reassignment;
- exact replay after an epoch transition returns the original effect ID/attestation/worker coordinates;
- a new effect under epoch 2 stores the new owner and worker-claim coordinates;
- an owner-epoch-1 worker claim cannot execute under epoch 2 even when the command supplies current epoch/generation;
- crossing to owner epoch 2 requires claim epoch 2 (reusing claim epoch 1 rejects);
- an old claim is rejected after a later claim epoch;
- repeated/non-advancing owner epoch rejected;
- execution attestations and owner-epoch transitions reject UPDATE/DELETE/TRUNCATE; the effect ledger also rejects TRUNCATE.

The workflow applies migrations 001, 002 and 003, then executes this SQL test script before Rust/NATS integration tests.

## 6. PostgreSQL → NATS integration

The real-service integration fixture now:
1. creates a fence domain and protected balance;
2. advances the test owner to epoch 2, generation 2, with test authority/fencing commitments;
3. installs a worker claim;
4. invokes apply_owner_epoch_claim_fenced_balance_delta;
5. commits under synchronous_commit = on;
6. queries both append-only ledgers through CA-verified TLS;
7. verifies the epoch-transition evidence is included in the canonical receipt;
8. rejects wrong scope/request/decision/payload, older owner epoch and older resource generation;
9. uses the verified receipt capability to AckSync the exact JetStream message.

The test receipt commitments are synthetic fixtures. They exercise persistence and binding, not real epoch-authority signature validation or physical node fencing.

## 7. Qualification limits

- No migration, SQL test, Rust test or new integration result is claimed until the exact-head workflow passes.
- Owner epoch and claim proof digests are evidence fields; the external claim issuer and epoch authority must authenticate their receipts.
- The SQL owner cannot stop a disconnected former primary from accepting writes. Production needs a single-writer lease / physical fencing authority and primary-promotion protocol.
- A writable primary SELECT does not establish that an asynchronous promotion candidate contains the commit. Synchronous replication, standby acknowledgement and failover policy require separate tests.
- Legacy effects without migration-003 attestations fail closed in the strict path.
- Owner epoch advance also increments resource generation, but restore safety still requires an independent high-water outside the rollback domain.
- The current test service is one PostgreSQL node and one NATS node. No clustered failover, performance or SOTA claim is made.

## 8. Next research gates

1. TLA+ model refinement for (owner_epoch, resource_generation), worker claims, exact replay, backup rollback, former-primary fencing and promotion.
2. Two-primary split-brain and restore/failover tests with real fencing.
3. Production least-privilege role that can read receipt tables but cannot mutate them.
4. Separate durable terminal-rejection/no-effect/reconciliation ledgers.
5. Independent history-oracle reconstruction of authorization, claim, owner epoch, effect receipt and NATS ACK.
6. Open-loop database benchmark: 10 repetitions, p50/p95/p99 commit and verifier latency, WAL bytes/effect, attestation write overhead and recovery time.

The performance corpus is preregistered; no measured result is claimed.


## 9. Bounded state-machine model

Adds `specs/owner-epoch/OwnerEpochExecutionAttestation.tla` and a bounded TLC configuration. The model covers:
- owner epoch, resource generation and independent lease-generation high-water;
- old/current primary write capability and physical-fencing actions;
- restore to a stale backup, quarantine, former-primary fencing and epoch re-promotion;
- worker claims bound to owner epoch with monotonic claim epoch;
- effect plus execution-attestation written in one transition;
- exact replay returning the original attestation.

The branch inventory is 23 reference-contract unit tests, 16 adapter-core unit tests, 4 PostgreSQL receipt/attestation unit tests, 3 ignored PostgreSQL/NATS integration tests and 20 SQL-level adversarial assertions. The TLA+ module and configuration are authored source, not a model-check result. No TLC execution is claimed. The SQL and Rust tests still require exact-head CI. The benchmark corpus is 52 scenarios / 37 metrics and remains preregistered, not executed.
