# MYC-INT-006M — PostgreSQL conventional service baseline

Status: preregistered architecture/profile. No PostgreSQL implementation is qualified by this document.

Tracks #3185 and refines the conventional arm of MYC-INT-006B without changing the runtime-neutral I0 protocol.

## Decision

PostgreSQL is a SQL database, not an alternative to SQL.

The benchmark needs four distinct roles:

```text
A0  SQLite embedded semantic control
A1  PostgreSQL conventional service baseline
B   Mycelix + Holochain reference runtime
C   Hybrid PostgreSQL + Mycelix/Holochain where justified
```

These roles answer different questions and must not be collapsed into one `SQL` candidate.

```text
SQLite control
!= PostgreSQL service baseline
!= Holochain runtime
!= hybrid architecture
```

## Source basis

Observed 2026-09-27 from upstream project documentation:

- PostgreSQL press FAQ reports PostgreSQL 18 as the current stable major release; the PostgreSQL support/versioning page lists 18.6 as the current supported 18.x minor. PostgreSQL 19 is still prerelease/beta at this observation point.
- PostgreSQL 18 concurrency documentation describes MVCC and Serializable Snapshot Isolation.
- PostgreSQL 18 documentation describes physical/streaming replication, logical replication, standby/failover facilities, row-level security, backup/restore/PITR and server monitoring.
- PostgreSQL 18 includes native UUIDv4/UUIDv7 generation.
- PostgreSQL `LISTEN` registrations are session-scoped and asynchronous notification is not a substitute for a durable application delivery ledger.
- PostgreSQL `jsonb` is a normalized binary representation: it does not preserve object-key order, whitespace or duplicate object keys.
- SQLite's own guidance recommends a client/server database when many concurrent writers must write without taking turns; SQLite allows only one writer at a time per database file.

Primary upstream references:

- https://www.postgresql.org/support/versioning/
- https://www.postgresql.org/docs/18/mvcc-intro.html
- https://www.postgresql.org/docs/18/applevel-consistency.html
- https://www.postgresql.org/docs/18/high-availability.html
- https://www.postgresql.org/docs/18/logical-replication.html
- https://www.postgresql.org/docs/18/ddl-rowsecurity.html
- https://www.postgresql.org/docs/18/sql-listen.html
- https://www.postgresql.org/docs/18/datatype-json.html
- https://www.postgresql.org/docs/18/datatype-uuid.html
- https://sqlite.org/whentouse.html
- https://sqlite.org/isolation.html

These implementation facilities are useful capabilities, not Mycelix semantic primitives.

## Role A0 — SQLite semantic control

Retain the existing SQLite/Python candidate as the smallest executable conventional control.

Its purpose is to answer:

> Can an ordinary local implementation preserve the I0 semantic invariants without distributed-system machinery?

Properties:

- embedded;
- no database server;
- deterministic clean-room execution;
- simple state inspection;
- low operational noise;
- useful for mutation/conformance controls.

Nonclaim:

```text
SQLite I0 PASS
!= realistic production conventional-service benchmark
```

In particular, do not infer multi-writer throughput, HA behavior, operational burden, backup/recovery characteristics or networked service behavior from A0.

## Role A1 — PostgreSQL conventional service baseline

PostgreSQL becomes the realistic conventional comparison arm after the neutral protocol is exact-qualified.

It answers:

> How does a competent multi-user client/server implementation behave under the same semantic workload and operational tests?

Initial environment generation should pin an exact supported PostgreSQL 18.x minor in the benchmark environment manifest. At the 2026-09-27 observation point, 18.6 is the current supported 18.x minor. Do not use `latest` as an evidence identity.

```text
postgres_major=18
postgres_minor=<exact qualified minor>
configuration=<committed profile>
```

A future PostgreSQL 19 generation, after a stable release, is a new benchmark environment rather than a silent refresh.

## Semantic ownership rules

Database mechanisms must remain implementation details behind the neutral protocol.

```text
PostgreSQL row primary key
!= Mycelix SemanticRef

PostgreSQL transaction ID
!= semantic event identity

PostgreSQL commit
!= external effect completion

PostgreSQL commit
!= desired real-world outcome

replication acknowledgement
!= local governance authorization

row-level security policy
!= standing
!= expertise
!= governance authority

logical replication
!= Mycelix federation

LISTEN/NOTIFY notification
!= durable delivery receipt
```

The database may enforce implementation invariants. It may not manufacture semantic authority merely because a row exists or a transaction committed.

## Initial relational ownership model

Do not begin by mapping every domain object to a large universal table.

Use a narrow shared relational spine and explicit typed tables/projections where useful.

The implementation should distinguish at least:

```text
semantic identity
source/schema identity
current lifecycle state
historical/superseded state
provenance references
transport/admission identity
local authority state
external-effect attempt
external-effect receipt
outcome observation
```

Exact table names remain implementation details, but a first conventional profile may use conceptual stores equivalent to:

```text
semantic_subject
source_object
provenance_edge
decision_history
authorization_history
implementation_attempt
implementation_receipt
outcome_observation
inbox
outbox
translation_receipt
```

Avoid one mutable `objects` row whose latest JSON value silently replaces history.

## Identity rules

Store semantic identity separately from database-generated convenience identifiers.

A local surrogate UUID may be useful for indexing/joins, including PostgreSQL UUIDv7 where appropriate, but:

```text
postgres_uuid
!= source object ID
!= SemanticRef
```

Recommended pattern:

```text
local_row_id        implementation convenience
semantic_namespace  protocol identity
semantic_name       protocol identity
semantic_version    protocol identity/source version
source_id           source-owned identity where applicable
source_schema_ref   exact external schema identity
```

Use unique constraints to reject duplicate semantic admission where the protocol requires uniqueness.

## Idempotency and delivery

Cross-process effects require durable state.

The PostgreSQL profile should use explicit inbox/outbox semantics when delivery is benchmarked:

```text
local transaction
  ├─ persist semantic transition
  └─ persist outbox intent
            ↓
       external worker
            ↓
       effect attempt
            ↓
       effect receipt
```

and on reception:

```text
transport delivery
      ↓
 durable inbox admission
      ↓
 dedupe semantic identity
      ↓
 local semantic processing
```

Required theorem:

```text
transport retries > 1
+ semantic attempt = 1
=> logical effect count <= 1
```

where the operation is defined as idempotent/exactly-once logically.

PostgreSQL unique constraints can help enforce this implementation property, but the semantic identity being constrained must come from the protocol rather than from an auto-generated row key.

## LISTEN / NOTIFY rule

PostgreSQL notifications can reduce wake-up latency, but they are not the durable queue.

Use:

```text
durable outbox/inbox rows
+ optional LISTEN/NOTIFY wake-up
```

not:

```text
LISTEN/NOTIFY alone
= delivery ledger
```

A restarted or disconnected worker must be able to recover pending work from durable database state.

## Transaction isolation profile

Preregister isolation instead of relying on implicit defaults.

### Read Committed

Appropriate for bounded ordinary queries/projections where the operation does not require a transaction-wide stable snapshot.

### Repeatable Read

Useful when one operation requires a stable snapshot but full serial execution semantics are unnecessary.

### Serializable

Use for carefully selected invariants where concurrent transactions must behave as though executed serially.

PostgreSQL Serializable transactions can abort with serialization failures; application code must treat bounded retry as an explicit behavior, not an invisible implementation detail.

Required measurements:

- serialization failure count;
- deadlock count;
- retry count;
- total attempts per semantic operation;
- end-to-end latency including retry;
- final semantic disposition.

Important distinction:

```text
transaction retry
!= duplicate semantic event
```

The same semantic operation retried after serialization failure must retain one stable semantic identity.

## Race-condition fixtures

Add PostgreSQL-specific execution profiles for:

1. duplicate delivery from two concurrent workers;
2. authorization expires while an implementation attempt is being admitted;
3. two concurrent attempts to supersede the same historical decision;
4. delivery persisted before caller timeout;
5. two concurrent foreign-authority import attempts;
6. source-state update racing a derived-summary read;
7. concurrent translation receipt creation for the same source/version pair.

The database may serialize or reject transactions. The semantic result must remain visible rather than being inferred from which transaction happened to commit first.

```text
last commit wins
!= legitimate conflict resolution by default
```

## JSON / JSONB policy

Use relational columns for fields whose identity/type participates directly in semantic or authority invariants:

- semantic namespace/name/version;
- source schema identity;
- lifecycle state;
- authority scope/status;
- provenance references;
- time validity;
- supersession/correction refs;
- stable dedupe keys.

JSONB is appropriate for extensible payloads and queryable domain projections when byte fidelity is not itself the evidence.

But PostgreSQL JSONB normalizes JSON representation and does not preserve duplicate object keys, whitespace or object-key order.

Therefore:

```text
JSONB value
!= exact source bytes
```

If exact external bytes matter for source commitment/provenance, retain their hash and, where policy requires, the exact captured bytes/artifact through the existing evidence lineage rather than reconstructing them from JSONB.

## Row-level security

RLS can be valuable defense in depth for application/database access boundaries.

Use it for questions like:

```text
may this database role read/write this row?
```

Do not use it as the ontology for:

```text
who has governance standing?
who is an expert?
who may authorize an institutional effect?
```

Those remain explicit Mycelix/domain semantics.

RLS bypass/owner behavior must be included in security qualification if RLS is part of the profile.

## Replication and high availability

PostgreSQL provides physical/streaming and logical replication facilities plus standby/failover mechanisms. These are useful operational capabilities, but the benchmark must distinguish data replication from semantic federation.

```text
row replicated to another PostgreSQL server
!= foreign institution accepted the claim
!= foreign authority recognized locally
```

Measure separately:

- synchronous vs asynchronous replication configuration;
- commit latency impact;
- standby lag;
- failover duration;
- potential acknowledged-write loss under the selected profile;
- read behavior on standbys;
- logical-replication behavior where used.

Do not assume Serializable semantics extend transparently to read-only hot standbys/logical replicas; PostgreSQL documents caveats here.

## Backup, recovery and upgrades

A1 must execute operational tasks rather than list features.

At minimum qualify:

```text
clean initialization
schema migration
least-privilege app role creation
backup
restore to clean host
PITR profile where enabled
minor-version upgrade
major-version upgrade rehearsal
replication/failover profile where enabled
lock/block investigation
serialization/deadlock diagnosis
runtime-neutral export
```

The environment manifest records the exact database version and configuration for each result generation.

## Performance comparison

PostgreSQL performance should be measured as its own conventional service profile, not compared to SQLite numbers as though they represent the same operating model.

Measure at least:

- single-worker semantic admission latency;
- concurrent-worker admission throughput;
- p50/p95/p99 transaction latency;
- duplicate-delivery contention;
- Serializable retry rate;
- indexed semantic-ref lookup;
- provenance traversal/query projection;
- outbox drain latency;
- backup/restore duration;
- storage growth;
- CPU/RSS;
- replication lag/overhead where enabled.

No aggregate winner score.

## Hybrid role

PostgreSQL may also be useful in a future hybrid Mycelix profile as:

- source-owned local operational state;
- read/query projection;
- analytics/reporting projection;
- search/index store;
- durable conventional service boundary.

But a projection is not a second semantic owner.

```text
Holochain/Mycelix semantic object
      ↓ projection
PostgreSQL query row

query row
!= new canonical authority
```

Conversely, where PostgreSQL owns a source fact locally, the Mycelix/Holochain side should carry an explicit projection/federation object rather than pretending to become the source owner.

## Evaluation matrix

The target conventional/distributed comparison becomes:

| Candidate | Role | Primary question |
|---|---|---|
| A0 SQLite | embedded semantic control | Is the protocol implementable simply and deterministically? |
| A1 PostgreSQL | production-style conventional service baseline | How does a competent client/server relational implementation behave? |
| B Mycelix + Holochain | distributed/federated candidate | How does the same semantics behave under agent-centric distributed persistence/federation? |
| C Hybrid | mixed ownership/projection candidate | Which responsibilities combine cleanly without creating two semantic owners? |

Each consumes the same neutral benchmark generation once its prerequisite protocol is qualified.

## Gating

The architecture/profile work can proceed before runner capacity is available.

Executable PostgreSQL candidate work is gated on successful exact MYC-INT-006K neutral-protocol qualification.

The Holochain implementation remains separately gated on its outstanding Mycelix/Holochain runtime and semantic qualifications.

## Follow-on PRs

After 006KQ PASS:

1. `MYC-INT-006N` — PostgreSQL schema + migration fixture for I0;
2. `MYC-INT-006O` — PostgreSQL oracle-blind conformer using the neutral protocol;
3. `MYC-INT-006P` — PostgreSQL concurrency/idempotency race corpus;
4. `MYC-INT-006Q` — PostgreSQL backup/restore/PITR operations qualification;
5. `MYC-INT-006R` — cross-conformer report: SQLite control vs independent Go control vs PostgreSQL service baseline, without an overall winner.

Only after those conventional/control lines are understood should PostgreSQL-specific findings be used to refine the eventual Holochain/hybrid comparison.

## Nonclaims

This profile does not select PostgreSQL for Integral, does not replace Mycelix/Holochain, does not treat database ACL/RLS as governance authority, and does not claim PostgreSQL is superior to any other candidate architecture.