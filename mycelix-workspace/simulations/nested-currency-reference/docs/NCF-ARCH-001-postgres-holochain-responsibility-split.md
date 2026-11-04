# NCF-ARCH-001 — PostgreSQL authority + Holochain evidence

**Status:** architecture decision proposal for review; PostgreSQL SQL implementation and Holochain integration require exact-head runtime qualification.  
**Date:** 2026-10-28  
**Related:** [PostgreSQL profile](../postgres/README.md), [PostgreSQL↔Holochain contract](../holochain/NCF-POSTGRES-HOLOCHAIN-CONTRACT.md), [NCF-1 reference model](../README.md).

## Decision

Use **PostgreSQL as the authoritative shared coordinator for online monetary mutations** and **Holochain as the agent-centric source-chain / DHT evidence and validation layer**. Do not build the production economic architecture around SQLite.

SQLite's recent NCF-ALLOC-7 through NCF-ALLOC-10 experiments were local transaction-semantics probes: replay binding, crash boundaries, checkpoint behavior, and a deterministic SQLITE_FULL rollback. They do not establish the architecture or qualify PostgreSQL/Holochain. Retain those tests only as narrow regression references where they cover a distinct invariant; no additional SQLite-only frontier is planned.

## Responsibility boundary

### PostgreSQL — one shared transactional authority domain

The PostgreSQL transaction domain is responsible for:
1. canonical instrument identity and immutable profile revisions;
2. shared idempotency-key and canonical-work uniqueness;
3. community/instrument gross caps, agreement-direction caps, and preallocated reservation-slice accounting;
4. request payload and verifier-evidence binding;
5. canonical obligation and immutable receipt creation;
6. settlement and exact retry idempotency; and
7. a transactional outbox row for subsequent Holochain publication.

An acceptance response may be returned only after the transaction commits. Database uniqueness and row locks protect writers using this same PostgreSQL authority. A uniqueness constraint does not coordinate isolated PostgreSQL instances.

The current SQL profile uses READ COMMITTED with explicit lock ordering, row locks, advisory locking for canonical work, and unique constraints as final guards. Do not switch isolation levels on intuition alone: benchmark the same operation histories under READ COMMITTED and SERIALIZABLE, including complete transaction retries for SQLSTATE 40001 and deterministic handling for unique-constraint conflicts.

For durable acceptance, run with fsync=on and synchronous_commit=on. Record the server version, isolation level, durability settings, schema/functions SHA-256, workload hash, raw pgbench logs, and post-run ledger reconciliation.

### Holochain — authored provenance and replicated validation evidence

Holochain is responsible for agent-authored signed claims, manifest/profile references, action/link provenance, deterministic integrity validation, DHT distribution, and the observed validation-receipt state.

The Holochain WASM zome must not connect directly to PostgreSQL. A Rust bridge/dispatcher consumes committed outbox rows, authors the stable event identity into Holochain, then records the returned action hash and any receipt observations. The dispatcher uses at-least-once delivery and retries with the same immutable event ID after ambiguous acknowledgements.

The systems are not one distributed SQL transaction. Never imply that SQL commit atomically commits a DHT action. Never infer global financial settlement from a Holochain action hash or validation receipt count: those are distinct evidence states and must remain distinct in the state machine.

## Required state machine

Do not overload a single "accepted" flag. Store/query distinct lifecycle states:
- sql_committed
- holochain_authored
- dht_validation_pending
- dht_receipts_threshold_met
- settlement_evidence_pending
- settled
- disputed / reversed

Every transition binds the same immutable obligation ID, request ID, canonical-work digest, instrument/profile revision, amount, direction, reservation, epoch, and verifier receipt digest. A timeout never authorizes minting a new obligation ID. Resolve ambiguous outcomes through the original idempotency key.

A receipt threshold is a configurable health/evidence gate, not a BFT quorum or permanent-availability guarantee. Record the observation time and identity of the source/validation receipts; a count alone does not establish signer independence or current peer availability.

## Offline / partition behavior

Safety and unrestricted offline liveness cannot both be promised for globally unique work under an arbitrary network partition without a coordination mechanism.

Before disconnection, allocate signed disjoint gross-exposure slices by agreement, community, instrument, direction, shard, and epoch. Do not reclaim another shard's unused allocation while the old owner may still be live. For canonical-work duplication, route each work identity to a deterministic home shard and bind the home shard/author into the manifest, or specify a different qualified coordination/quorum/dispute protocol. A non-home shard must not claim it has proved global uniqueness merely because its local DHT view contains no matching action.

If a home shard is unreachable, reject/defer the affected new obligation rather than silently accepting a globally duplicate claim. Reconciliation after reconnection can publish evidence and initiate an explicit dispute/reversal path, but cannot erase the fact that two offline parties may already have provided services.

## Qualification plan — no weakening of gates

### Gate A: exact PostgreSQL 18.6 path

Run the existing isolated PostgreSQL workflow and require all of:
- sequential smoke: initial admission, exact replay, payload conflict, canonical alias, caps/slices, integer-unit validation, settlement replay, outbox retry/dispatch, DHT-receipt threshold/high-water behavior, immutable profile identity, and application-role direct-write denial;
- independent PostgreSQL session races: distinct cap contention, same canonical work under different request IDs, and same request ID with different payloads;
- final accounting: canonical obligations, accepted request bindings, cap allocation/outstanding, edge allocation/outstanding, slice use, and outbox rows reconcile exactly;
- pgbench drives ncf.admit_obligation, not a generic counter query; successful log rows, stdout transaction count, and SQL accepted-obligation count match; failed or malformed rows fail the campaign;
- p50/p95/p99 latency, tps, client/thread counts, server settings, exact source hashes, and environment are retained.

A zero-job workflow record is not a test result. A parser unit-test PASS is not a PostgreSQL SQL PASS. Do not claim a database test passed until the job actually executes and its raw log/reconciliation artifact is available.

### Gate B: concurrency isolation and throughput

On the same PostgreSQL major/minor and workload, run at least five repetitions at 1, 4, 16, 64, and 128 clients (after stable smoke). Compare READ COMMITTED plus current lock discipline against SERIALIZABLE with full transaction retry. Separate:
- successful new admissions;
- idempotent replays;
- policy rejections;
- serialization/deadlock retries;
- database/storage errors; and
- outbox lag.

A higher TPS result loses if it accepts a conflicting request binding, overdraws a cap, creates a second canonical obligation, or acknowledges an uncommitted transaction. Compare to generic pgbench/TPC-B or settlement networks only using matched transaction semantics, durability, workload, hardware, and failure model.

### Gate C: Holochain integration

Implement the manifest/obligation/settlement entry types in the existing Mycelix Finance hApp rather than building an unrelated parallel currency zome. Use the repository's supported Sweettest generation and 3–5 conductors/agents. At minimum test:
- accepted SQL obligation followed by authored claim;
- dispatcher crash after Holochain authoring but before outbox acknowledgement;
- stable event replay with no second SQL debit;
- wrong issuer, stale epoch, mutated amount/instrument/direction/manifest, invalid settlement, and duplicate settlement ID rejection;
- dependency unavailable/unresolved validation that retries instead of permanently rejecting prematurely;
- PostgreSQL unavailable while a Holochain action is present, and Holochain unavailable after SQL commit;
- DHT receipt count below threshold and later threshold satisfaction;
- PostgreSQL restart with pending/published outbox rows; and
- three agents submitting the same canonical-work digest, including non-home shard rejection.

Receipts must bind commit SHA, DNA hash, integrity/coordinator WASM hashes, conductor version, agent count, operation-history IDs, and artifact hashes. Planned Sweettest scenarios are not pass evidence.

## Benchmark targets

These are qualification targets, not current results:
- **Safety:** zero cap overshoots, duplicate debits, payload rebinding acceptances, settlement double-release, or SQL/outbox partial commits across all deterministic negatives.
- **Concurrency:** at least 1,000 synchronized multi-session race repetitions per race class with zero safety violations.
- **Replay:** exact retries return the original byte-equivalent receipt; changed immutable fields reject without mutation.
- **Durability:** injected termination before commit leaves no partial accepted state; ambiguous commit recovers through the same key; real PostgreSQL crash/PITR/failover tests are separately required.
- **Holochain reconciliation:** every committed outbox event converges to one canonical event identity under at-least-once delivery; receipt state never masquerades as settlement finality.
- **Performance:** publish 1/4/16/64/128 client curves, p50/p95/p99, retries/errors, outbox age, WAL/DB growth, reconciliation time, and workload/hardware; repeat each point at least five times. First target stable linear/near-linear improvement until the measured lock/IO bottleneck—not an arbitrary TPS headline.
- **Scale:** 1 million unique obligations plus replay, conflicting requests, outbox delivery faults, and reconciliation; then 10 million after the memory/storage envelope is recorded. Preserve every safety gate.

## External comparison rule

The PostgreSQL function benchmark is not comparable to payment-rail TPS without a matched semantic workload and durability contract. BIS Project FuSSE reported 10,000 TPS in a controlled proof of concept, explicitly as architecture exploration rather than a production performance benchmark. Our competitive claim should be stronger failure semantics and independently verified conservation first, then a transparent like-for-like performance curve—not a naked number.

## Current truth

The PostgreSQL SQL profile, pgbench workload, and Holochain contract are committed research. The parser/runner regressions have local unit tests, but PostgreSQL 18.6 integration, Rust adapter execution, and Holochain Sweettest remain unqualified until their respective jobs run with non-empty logs and exact-head receipts. No production, cross-host uniqueness, offline double-spend, key security, or external-settlement finality claim is made.
