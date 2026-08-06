# PostgreSQL Canonical Authority Backend

> **v0.9 upgrade precondition:** drain or explicitly publish every pending schema-v1 outbox row before applying schema v2. Those rows predate signed delivery envelopes and cannot be retroactively signed without changing their historical acceptance semantics. Delivered v1 rows may remain as historical transport records.

## Scope

The PostgreSQL backend is the production-oriented persistence boundary for:

- canonical scientific events;
- independently signed authority receipts;
- credential and threshold-governance histories;
- publication-outbox messages;
- signed checkpoint-mirror observations; and
- governed database epochs, recovery reconciliations, and delivery acknowledgements.

A scientific append uses one serializable transaction. The transaction locks the
claim stream, verifies the expected sequence and both hash chains, persists the
event and receipt, and creates the outbox message before commit. Direct writes to
the PostgreSQL event log fail closed; callers must use the governed append path.

Schema v4 includes the earlier credential/governance and database-epoch
cutovers, then adds durable authority-write fencing state. Proposal execution can
commit an optional credential mutation, the governance execution, and both
signed publication messages atomically, but only while a current externally
signed lease authorizes the exact operation and connected PostgreSQL identity.

## Configuration

```text
DESCI_SCIENTIFIC_EVENT_BACKEND=postgres
DESCI_POSTGRES_URL=postgres://mycelix:<password>@postgres:5432/mycelix_desci
DESCI_POSTGRES_MAX_CONNECTIONS=16
DESCI_POSTGRES_ACQUIRE_TIMEOUT_SECONDS=10
DESCI_POSTGRES_AUTO_MIGRATE=false
DESCI_REQUIRE_AUTHORITY_WRITE_FENCING=true
DESCI_AUTHORITY_DEPLOYMENT_ID=mycelix-desci-production
DESCI_AUTHORITY_WRITE_LEASE_FILE=/run/authority/current-write-lease.json
DESCI_AUTHORITY_WRITE_LEASE_TRUST_FILE=/app/config/authority-write-lease-trust.json
DESCI_AUTHORITY_WRITE_LEASE_CLOCK_SKEW_SECONDS=30
DESCI_REQUIRE_AUTHORITY_DATABASE_EPOCH=true
DESCI_MIN_DATABASE_EPOCH_ACK_ORGANIZATIONS=2
```

A controlled deployment job should apply migrations 0001 through 0004 while
holding the schema advisory lock and presenting a current trusted lease with the
`schema_migration` scope. Startup then requires the exact v4 marker and fails
closed if it is absent.

If runtime migration is deliberately enabled, the API verifies the lease,
deployment, actual PostgreSQL system identifier, timeline, and—when present—the
exact current database epoch **before any migration DDL executes**. A bootstrap
lease is accepted only before the epoch schema exists.


## External write fencing

Every PostgreSQL authority mutation verifies a short-lived signed lease. The
lease must match the configured deployment, connected primary identifier,
`pg_control_system()` system identifier, current timeline, durable monotonic
generation, and requested operation scope. After the first governed database
epoch it must also match that exact epoch number and hash.

The database stores the highest accepted generation and signed-lease hash. A
lower generation or a different lease at the same generation is rejected. Lease
expiration is strict; configured clock skew may permit early activation but
never extends `expires_at`.

Outbox claim, publish, and failure transitions are fenced too. A process may not
continue draining or mutating the queue after losing its lease merely because it
already holds database credentials.

See [Externally Signed PostgreSQL Write Fencing](AUTHORITY_WRITE_FENCING.md).

## Transaction invariants

For one scientific append, PostgreSQL commits or rolls back all of:

1. the canonical signed event;
2. its exact event hash and actor-scoped idempotency key;
3. the independently signed authority receipt;
4. receipt-chain linkage; and
5. a durable outbox message containing the event, append receipt, and authority
   receipt.

Before the SQL insert, the backend rebuilds the complete candidate claim
projection. This prevents a direct backend caller from bypassing domain-state
rules even if it possesses an otherwise valid signed receipt.

## Outbox delivery

Set `DESCI_AUTHORITY_OUTBOX_WEBHOOK_URL` to enable delivery. Workers lease rows
with `FOR UPDATE SKIP LOCKED`, include the outbox UUID and a deterministic
idempotency key in every request, and mark a row published only after a 2xx
response. Delivery is at least once; receivers must deduplicate.

A missing webhook does not discard records. They remain durable and pending.
`DESCI_MAX_PENDING_AUTHORITY_OUTBOX` and
`DESCI_MAX_AUTHORITY_OUTBOX_AGE_SECONDS` can turn backlog count or oldest age
into readiness gates. Zero disables a gate. Monitor delivery attempts and last
error as additional incident signals.

## Replay and readiness

Startup reconciliation verifies:

- every scientific signature;
- every claim projection and event hash chain;
- every authority-receipt signature and event binding;
- every receipt-chain hash and immediately adjacent sequence; and
- absence of unsafe unattested SQL events.

Readiness fails if PostgreSQL is selected but unavailable, if reconciliation
finds invalid authority history, or if the current lease is absent, expired,
untrusted, stale, scope-incompatible, or bound to another deployment, primary,
database system identifier, timeline, or epoch.

## Operational limits

- SQLx was added in this tranche, but `Cargo.lock` must be regenerated and
  committed with the pinned Rust toolchain before merge.
- File-backed deployments remain supported as a single-process reference path;
  they are not safe to mix with PostgreSQL authority in one deployment.
- Outbox publication does not prove that an external subscriber retained data;
  external mirrors and checkpoint witnesses provide separate evidence.
- The repository does not ship the independent lease issuer, promotion
  orchestrator, or database isolation mechanism. Those remain deployment
  responsibilities.
- PostgreSQL backup, restore, point-in-time recovery, stale-primary, and strict
  lease-expiration drills still require deployment-specific automation and
  validation.

See also [Transactional SQL Credential Governance](TRANSACTIONAL_SQL_CREDENTIAL_GOVERNANCE.md), [Signed Authority Delivery](SIGNED_AUTHORITY_DELIVERY.md), [Authority-Write Fencing](AUTHORITY_WRITE_FENCING.md), [Failover/PITR Validation](POSTGRES_FAILOVER_PITR_VALIDATION.md), and [PostgreSQL Recovery Runbook](POSTGRES_RECOVERY_RUNBOOK.md).

## Governed database epochs

Schema v3 adds hash-chained primary epochs, exact recovery reconciliation, and
independent acknowledgement of signed epoch publications. Every cooperating SQL
authority writer acquires a shared epoch barrier; state capture and epoch commit
acquire its exclusive form. PostgreSQL deployments should keep
`DESCI_REQUIRE_AUTHORITY_DATABASE_EPOCH=true`. See
[Governed Authority Database Epochs](AUTHORITY_DATABASE_EPOCHS.md) and the
[Epoch Recovery Runbook](POSTGRES_EPOCH_RECOVERY_RUNBOOK.md).
