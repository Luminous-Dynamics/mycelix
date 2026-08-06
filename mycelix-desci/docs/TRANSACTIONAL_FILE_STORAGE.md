# Transactional Reference File Storage

**Status:** durable reference adapter; not a distributed database

The original file adapters used process-local locks and atomic rename. That protected a single process from partial writes but allowed two independently started processes to overwrite one another from stale in-memory state.

The v0.8 adapters add process-independent optimistic transactions for the small append-only JSON journals and scientific event streams.

## Write protocol

For each replacement:

1. create an exclusive sidecar lock with `create_new`;
2. fsync lock metadata and its directory;
3. reread the durable journal;
4. compare it with the caller's expected prefix;
5. reject a stale caller rather than merge or overwrite;
6. write a unique temporary file;
7. fsync the temporary file;
8. atomically rename it over the target;
9. fsync the parent directory;
10. verify lock ownership before removing the lock.

A second process that opened the journal before another writer committed receives an optimistic-concurrency error.

## Lock recovery

Locks are deliberately not reclaimed automatically. PID reuse, container namespaces, and delayed storage can make automatic stale-lock detection unsafe. After a crash, an operator must:

1. prove no writer is active;
2. inspect the target journal and lock metadata;
3. replay or verify the journal;
4. remove only the corresponding sidecar lock;
5. restart one writer.

## Scope

The transaction helper now protects:

- scientific credential registry;
- credential-governance journal;
- each canonical scientific event stream.

When `DESCI_SCIENTIFIC_EVENT_BACKEND=postgres`, canonical scientific events,
authority receipts, and publication-outbox rows now commit in one serializable
PostgreSQL transaction. The file receipt journal remains the reference path for
`file` mode and should still have only one cooperating writer.

## Limitations

- The lock protocol assumes all writers honor it.
- Atomic rename and directory fsync semantics depend on a local filesystem with normal POSIX behavior.
- Network filesystems and object stores require a different adapter.
- In file mode, a credential mutation plus its governance execution spans two journals and is crash-recovered rather than committed as one database transaction.
- File adapters are suitable as deterministic reference implementations and small deployments, not high-throughput horizontal scaling.

The stronger production path is the unified PostgreSQL authority adapter documented in
[PostgreSQL Canonical Authority Backend](POSTGRES_AUTHORITY_BACKEND.md) and
[Transactional SQL Credential and Governance Authority](TRANSACTIONAL_SQL_CREDENTIAL_GOVERNANCE.md).
In PostgreSQL mode, credential mutations, threshold-governance executions, authority
receipts, and signed publication outbox rows use serializable transactions. The file
adapters remain the deterministic reference and offline-bootstrap path.
