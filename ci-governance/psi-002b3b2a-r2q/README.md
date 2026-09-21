# PSI-002B3B2A r2Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
e48e3bc6e9cb203084d2a5aad61c5ead1ada515d
```

This qualifier covers only the exact nonce-bound SQLite WAL/FULL replay-store source and its registered tests.

## Bound source

The lock binds the exact B3A r2, B3B1 r2 and SQLite r2 crate blobs.

## SQLite ratchets

The qualifier requires:

```text
rusqlite = 0.39.0 exactly
journal_mode = WAL set + read back
synchronous = FULL set + read back
busy_timeout = 5000 ms set + read back
schema version = 2
TransactionBehavior::Immediate
uniqueness-protected nonce-bound insert
commit before positive construction
```

The persisted spend schema must contain `token_nonce_sha256` and must not contain a whole-token digest column or raw token bytes.

The same-nonce/different-token-artifact replay regression and close/reopen replay regression are mandatory.

## Offline execution

The qualifier reconstructs exact B3A r2, B3B1 r2 and SQLite r2 sibling crates and runs:

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback is permitted. Missing bundled-SQLite/rusqlite cache is a truthful FAIL.

## Claim ceiling

Even a successful execution remains false for Privacy Pass token verification, cryptographic nonce binding, global store uniqueness, hardware power-loss durability, safe compaction, real query credit, rate limiting, enumeration resistance, production admission and application authority.
