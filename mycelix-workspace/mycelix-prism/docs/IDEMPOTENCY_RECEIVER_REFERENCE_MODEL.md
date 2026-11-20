# Prism receiver-side idempotency reference model v0.1

**Status:** test-only deterministic receiver model with 31 test functions. It is not an HTTP server, ACID database, consensus/replication system, or production idempotency implementation.

## Why the receiver must be tested independently

A client-side retry classifier cannot prove that a resource actually deduplicates operations. The receiver is the other half of the safety contract. A server that stores an idempotency key separately from the business mutation can still produce duplicates if failover preserves one and loses the other.

The model at prism-egress/tests/idempotency_receiver_model.rs makes a deliberately strong assumption: the business effect and the completed idempotency result are committed together as one atomic transaction. If that assumption is false in production, the client must not claim the same guarantee.

## Receiver contract

The key namespace is scoped by workload, principal, origin and opaque key digest. The completed entry binds:
- logical operation ID;
- workload and principal identity;
- origin and method;
- exact canonical request digest;
- completion result digest and server revision;
- receiver-owned expiry deadline.

The client supplies the key, but it does **not** choose or extend the server's retention period. A new request reserves a private one-use owner token. A concurrent duplicate with the same key and fingerprint is returned as ConflictInProgress (the default contract for a duplicate arriving before original completion); a same-key/different-request operation is rejected. The owner commits the side effect and result together. After commit, duplicates return the cached completion without applying another effect.

A crash before transaction commit discards the in-flight reservation and has no effect. A crash after commit but before the response preserves both the effect and the replayable result. Safe failover preserves both. The deliberately unsafe negative control keeps the effect but loses the idempotency record; replay then applies the effect a second time. This is the exact split-brain failure the production receiver must rule out.

## Concurrency and duplicate semantics

The model's conflict-on-in-progress path corresponds to the Idempotency-Key Internet-Draft's recommendation that a concurrent duplicate receive a resource-conflict response. It does not mean the original operation completed. A caller must treat the conflict as pending/reconciliation, not success.

A stronger receiver may coalesce concurrent duplicates onto the in-flight original. That capability may enable safe hedging only when the server proves that all copies join one effect. PEAB treats this as a separate contract, not as an assumption shared by all idempotency servers.

## Executable coverage

The 31 tests cover:
- first request atomic effect/result commit;
- completed duplicate cache replay and exact result/revision preservation;
- concurrent duplicate conflict and 128-way duplicate control;
- same-key/different-fingerprint and same-key/different-method rejection;
- new attempt ID for same logical operation;
- operation, workload, principal and origin key scoping;
- abort, invalid owner token, stale owner token and commit-at-most-once;
- crash before commit and crash after commit/before response;
- durable failover versus the deliberately unsafe missing-dedup-state control;
- receiver-owned retention, exact expiry, attempted retention extension and timestamp overflow;
- 128 generated sequential replays that return one cached result without incrementing the effect count.

All 31 tests are source-present only until exact-head compilation and execution.

## Production requirements and limitations

A production implementation should:
1. create a unique idempotency key reservation in a database transaction or equivalent serializable primitive;
2. bind the key to authenticated tenant/principal, authorized workload where relevant, resource origin, method and a canonical request fingerprint;
3. atomically commit the business effect and the durable result record;
4. return a documented conflict/pending result for a concurrent duplicate, or prove atomic coalescing;
5. retain the key and result across every failover, restore and replica promotion inside the published retention period;
6. reject same-key/different-fingerprint reuse and avoid disclosing another principal's cached result;
7. own and publish the retention deadline; clients must not extend it by resending;
8. use a transactional outbox and downstream inbox/idempotency contract for effects outside the database transaction (payments, email, queues, remote services). A local database transaction cannot make an independent external side effect atomic by itself.

The model's in-memory map mutations are atomic only at the abstraction level of a single function call. It does not test database isolation, fsync, WAL recovery, replication lag, consensus, failover snapshots, outbox delivery or power loss. Those are explicitly planned PEAB scenarios, not results.

## Standards references

RFC 9110 §9.2.2 defines HTTP method idempotence by intended effect; it does not define receiver persistence or exactly-once external effects. The active Idempotency-Key Internet-Draft version -08 (22 October 2026; work in progress, not a published RFC) places key lifecycle and expiry responsibility on the resource and distinguishes completed duplicates from concurrent duplicates.

- https://www.rfc-editor.org/rfc/rfc9110.html#section-9.2.2
- https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-08
