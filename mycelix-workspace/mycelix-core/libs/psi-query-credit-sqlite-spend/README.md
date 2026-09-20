# PSI-002B3B2A — SQLite WAL/FULL Spend Backend

Status: **source constructed / backend-specific / not compile-qualified / not a query-credit theorem**

This crate adds a concrete persistent replay store above PSI-002B3B1.

It pins `rusqlite = 0.39.0` with bundled SQLite and requires a runtime profile of:

```text
journal_mode = WAL
synchronous = FULL
busy_timeout = 5000 ms
schema_version = 1
```

The constructor sets these values and reads them back before admitting the store.

## Store identity

Every database persists one explicit `store_instance_id`. Opening the same database with a different expected identity fails closed.

This prevents accidental database substitution. It does **not** prove that only one physical database exists for a logical store identity.

## Atomic spend table

The primary key is the canonical B3B1 spend-key SHA-256. The table stores only:

- spend-key SHA-256;
- policy SHA-256;
- token SHA-256;
- requested identifier count.

Raw Privacy Pass token bytes are never accepted by this API and therefore cannot be persisted by this backend.

## Commit ordering

`consume_once` opens an IMMEDIATE transaction, attempts one primary-key-protected insertion, and commits before constructing the positive type.

```text
insert
-> commit
-> positive type
```

Never:

```text
positive type
-> best-effort persistence
```

A crash after commit but before the caller receives the positive may consume the token without yielding a visible credit. That is a fail-closed availability loss, not a replay opening.

## Positive ceiling

`SqliteFullSyncConsumedQueryTokenV1` may report:

```text
sqlite_atomic_single_use_established = true
sqlite_full_sync_profile_established = true
```

It must report false for:

```text
privacy_pass_token_cryptographically_verified
global_store_uniqueness_established
hardware_power_loss_durability_established
query_credit_granted
anonymous_rate_limit_established
enumeration_resistance_established
application_authority_granted
```

The `FULL` profile means the SQLite connection was configured and read back with SQLite's FULL synchronous setting. It is not a claim about dishonest storage hardware, filesystem bugs, VM snapshots, or fault-injection results.

## Source corpus

The committed eight-case corpus covers:

- exact WAL/FULL profile and first commit;
- replay rejection after close + reopen;
- two independent connections racing one database with exactly one winner;
- distinct tokens each committing once;
- wrong expected store identity rejection;
- schema-version mismatch rejection;
- persisted schema retaining only digest/audit fields rather than raw token bytes;
- strict positive authority ceiling.

## Next boundary

This backend is still not sufficient to mint a real query credit. The eventual join must consume a qualified RFC 9578 token-verification positive bound to the exact B3A challenge/policy and a qualified admitted replay-store profile.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= SQLite fault-injection qualified
!= hardware durability established
!= Privacy Pass token valid
!= query credit granted
```

A separate exact-source qualifier is required.