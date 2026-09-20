# PSI-002B3B2A r2 — Nonce-Bound SQLite WAL/FULL Spend Backend

Status: **source constructed / backend-specific / not compile-qualified / not a query-credit theorem**

This replaces the unqualified r1 SQLite subject before execution.

R2 consumes the nonce-bound `QueryCreditSpendKeyV2` from B3B1 r2. Whole-token artifact identity no longer partitions durable replay state.

## Exact backend profile

```text
rusqlite = 0.39.0
journal_mode = WAL
synchronous = FULL
busy_timeout = 5000 ms
schema_version = 1
```

The store sets and reads back the runtime profile before admission, after schema initialization, and inside each spend transaction.

## Durable row minimization

The v1 SQLite schema is deliberately reduced to:

```text
spend_key_sha256
policy_sha256
token_nonce_sha256
requested_identifier_count
```

It does not persist raw tokens or whole-token SHA-256 values.

The canonical `spend_key_sha256` already binds the exact policy/challenge/service/issuer/token-type/key/epoch plus the nonce digest.

## Commit ordering

```text
IMMEDIATE transaction
-> recheck WAL/FULL/schema profile
-> primary-key-protected insert
-> commit
-> construct positive type
```

A crash after commit but before positive delivery can consume a token without yielding a visible credit. That is fail-closed for replay safety but may lose availability.

## Same-nonce theorem

Because the v2 spend key ignores the whole-token artifact digest:

```text
same nonce + different whole-token digest
-> same spend key
-> second SQLite insertion is replay
```

## Positive ceiling

`SqliteFullSyncConsumedQueryTokenV2` may establish only the exact SQLite store/profile transition. It keeps false Privacy Pass crypto, nonce crypto binding, challenge crypto binding, global-store uniqueness, hardware power-loss qualification, safe compaction, real query credit, anonymous rate limiting, enumeration resistance and application authority.

## Source corpus

Nine tests cover:

- exact WAL/FULL profile and first commit;
- replay after close + reopen;
- same nonce / different token artifact still replaying;
- two-connection same-file race with one winner;
- distinct nonces each committing;
- wrong store identity;
- schema-version mismatch;
- minimized persisted columns;
- strict authority ceiling.

## Compaction boundary

Deleting spent rows is not automatically safe. PSI-002B3B2B / #2469 owns proof that the exact old token acceptance surface is closed before replay state can be compacted.

## Qualification boundary

A fresh exact-source qualifier is required. No evidence transfers from superseded #2463/#2466.