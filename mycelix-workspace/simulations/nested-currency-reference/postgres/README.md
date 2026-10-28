# NCF PostgreSQL transactional-admission profile

**Profile status:** executable PostgreSQL SQL and CI tests are defined; the first live PostgreSQL run is pending because this editing environment has no `psql`, PostgreSQL server, Docker, or Podman. The dedicated GitHub Actions workflow uses PostgreSQL 18.6. No PostgreSQL PASS is claimed until that workflow has a completed, exact-HEAD result.

## Why PostgreSQL now

The earlier SQLite increments were cheap single-host experiments for checking transaction assumptions. They are not the production architecture. This profile makes PostgreSQL the shared-coordinator transaction engine for online admission, gross-exposure accounting, reservation consumption, settlement, and the transactional outbox. The associated Holochain path records signed claims and validation evidence. SQLite can remain a tiny reference adapter test; no additional SQLite-only frontier is planned unless it diagnoses an invariant specific to SQLite.

PostgreSQL 18.6 is a supported current minor release as of 28 October 2026. PostgreSQL 19 is still at release-candidate stage on this date, so this harness deliberately targets stable 18.6 rather than the development release. See the [official PostgreSQL release notes](https://www.postgresql.org/docs/release/) and [versioning policy](https://www.postgresql.org/support/versioning/).

## What the SQL implements

- Immutable instrument identity and immutable, versioned policy profiles.
- Gross cap accounting in exact instrument units.
- Reservation slices tied to an agreement, direction, community, shard, and epoch.
- Per-request payload/verifier-evidence binding.
- Canonical-work uniqueness, where a repeated canonical digest returns the original obligation receipt without a second debit.
- Atomic update of cap usage, reservation-slice usage, canonical obligation, request binding, and Holochain outbox intent.
- Idempotent settlement that releases outstanding usage exactly once.
- A restricted application role: application sessions may read and call the audited `SECURITY DEFINER` functions, but do not have direct table-write permissions.
- Independent PostgreSQL sessions racing cap admission, canonical-work aliases, and conflicting request IDs.

The function contract expects the calling service to independently verify source evidence and supply the verifier version and receipt digest. SQL enforces transactionality and bindings; it does not perform cryptographic proof verification or establish that real-world work occurred.

## Run on PostgreSQL 18.6

Create a disposable PostgreSQL 18.6 database and set `DATABASE_URL`. The test workflow uses an isolated PostgreSQL service container.

```sh
psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 -f 001_schema.sql
psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 -f 002_functions.sql
psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 -f 900_smoke_test.sql
psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 -f 910_concurrent_setup.sql
bash run-concurrency-tests.sh
```

From the repository root, prefix the paths with `mycelix-workspace/simulations/nested-currency-reference/postgres/`. The scripts intentionally assert:

1. First admission succeeds.
2. Exact retry returns the original receipt.
3. A request ID rebound to a different immutable payload rejects without mutation.
4. The same canonical work under a second request ID returns replay with no second cap debit.
5. A claim beyond the available cap/slice rejects without mutation.
6. Settlement releases cap/slice usage exactly once; exact settlement retry returns replay.
7. Outbox rows are atomic with the economic transaction.
8. Concurrent distinct 60-unit requests against a 100-unit cap yield one accepted and one rejected request.
9. Concurrent identical work under different IDs yields one accepted and one replay.
10. Concurrent different payloads using one request ID bind only one payload; the losing transaction's changes roll back.
11. The final counter, reservation usage, obligation units, request bindings, and outbox rows reconcile.

## Isolation and locking choices

The online profile runs under PostgreSQL's default `READ COMMITTED` isolation and uses explicit row locks plus unique constraints and a per-canonical-work advisory transaction lock. The lock order for new claims is **agreement → community/instrument cap → reservation slice**; settlement uses the same order before locking the obligation. Unique constraints remain final guards; the design does not rely only on a preflight `SELECT`.

This is an initial correctness-oriented choice, not a claim that READ COMMITTED is always faster or safer than SERIALIZABLE. A matched benchmark should compare both profiles. If SERIALIZABLE is chosen, the application must retry the **entire** aborted transaction on SQLSTATE `40001`; PostgreSQL documents that serializable transactions can abort and sometimes still encounter unique-constraint conflicts. See [PostgreSQL transaction isolation](https://www.postgresql.org/docs/current/transaction-iso.html) and [explicit locking](https://www.postgresql.org/docs/current/explicit-locking.html).

Keep `fsync=on` and `synchronous_commit=on` for durable acceptance unless the product explicitly weakens that guarantee. PostgreSQL's official docs warn that turning synchronous commit off can return success before a transaction is crash-safe. See [WAL configuration](https://www.postgresql.org/docs/current/runtime-config-wal.html).

## Important architectural split: online SQL vs offline federation

This migration is the **shared PostgreSQL coordinator** profile: its uniqueness and exposure caps apply to writers using the same database transaction domain. It is not a distributed database and should not be replicated as if each disconnected node could enforce the same global cap independently.

For partition-safe/offline operation, use a separate profile:

- Preallocate disjoint, signed reservation slices before disconnection.
- Each offline coordinator may consume only its own slice.
- A disconnected shard must not claim it can check global canonical-work uniqueness across other unreachable shards.
- Route a canonical work identity to a deterministic home shard, or accept that duplicate claims discovered after reconnection require explicit dispute/reversal policy. Do not claim both globally unique work and unrestricted offline acceptance without a coordination or identity/routing mechanism.
- Do not reclaim another shard's unused slice until a separately qualified fencing/rebalancing protocol proves the prior owner cannot still spend it.

The Holochain DHT distributes and validates authored evidence; it is not a cross-agent serializable SQL transaction. Holochain validation receipts indicate that validators received, validated, stored, and began serving DHT operations; they are a health signal, not a global financial commit or settlement-finality certificate. See the [DHT operation model](https://developer.holochain.org/build/dht-operations/) and [validation receipt semantics](https://developer.holochain.org/build/validation-receipts/).

## Outbox / Holochain contract

An accepted obligation and its outbox event commit in the same PostgreSQL transaction. A worker dispatches outbox events to Holochain and records the returned action hash and any observed validation-receipt progress. The worker must retry using the same stable event/obligation ID if it crashes after Holochain accepts the action but before PostgreSQL records the reply.

Do not confuse these states:

- `db_committed`: the PostgreSQL transaction committed the obligation and outbox.
- `holochain_authored`: the action was authored on an agent's source chain.
- `validation_receipts_observed`: the configured DHT validation receipts were observed.
- `settled`: the settlement policy and evidence have accepted the settlement.

They are separate facts. The outbox is an at-least-once dispatch mechanism, not distributed two-phase commit. Holochain duplicate authored actions can be harmless only if stable event identities and zome validation make the economic claim idempotent; they must never create a second SQL debit.

## Claim ceiling

The current schema/functions and CI tests are a PostgreSQL implementation attempt. They are not yet live-tested in this environment, do not yet qualify the existing Mycelix Finance Holochain zomes, and do not prove offline double-spend resistance, key security, legal status, real-world work delivery, or external settlement finality. See the companion [PostgreSQL↔Holochain contract](../holochain/NCF-POSTGRES-HOLOCHAIN-CONTRACT.md).
