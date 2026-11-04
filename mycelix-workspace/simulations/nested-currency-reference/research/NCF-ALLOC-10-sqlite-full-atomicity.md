# NCF-ALLOC-10 — atomic admission under SQLITE_FULL

**Status:** four local Python tests pass; database-size-limit research only.  
**Source SHA-256:** b472cef5ac7a241c91ab08bc42c8c4a69e9281969970599237a8b6ccf06820ac  
**Repository Git blob:** 2916f463e9092ffad77c4e6afe1ee488f52b84e6 (verified byte-for-byte).  
**Runtime:** Python 3.13.5 / SQLite 3.46.1 / local file-backed WAL.

## Research question

A financial operation can fail while the process remains alive. An application must distinguish policy rejection from a storage error and must not leave a partially applied obligation when a write fails after a liability counter has already been mutated.

The harness deliberately orders one transaction as: validate cap, update used-capacity, then insert the obligation receipt. It sets PRAGMA max_page_count to current page_count + 1 on the same connection, then attempts a 10 MiB receipt. SQLite returns SQLITE_FULL (extended code 13) during the later insert. The application rolls back the whole transaction.

## Results

| Case | Expected | Result |
|---|---|---|
| Storage capacity failure after cap update | No obligation, no counter increase | Pass: SQLITE_FULL, zero rows, used_units = 0, integrity ok |
| Restore database growth and retry same request | Accept once, next retry is idempotent replay | Pass: one 60-unit obligation, counter 60, retry replay |
| Existing accepted obligation followed by a full-database attempt | Previous obligation remains; failed one absent | Pass: prior 20-unit obligation preserved, later 40-unit retry accepted after capacity restored |
| Policy cap rejection vs storage full | Different result classes, neither mutates state on rejection | Pass: policy decision rejected; database limit separately returns storage_error / SQLITE_FULL |

Four tests passed with ResourceWarning promoted to error. Final ledger states had no duplicate work digests, counter conservation held, and SQLite integrity_check was ok.

## Paradigm shift

The core contract should be outcome-typed:

- Accepted(receipt) only after the complete durable transaction commits.
- PolicyRejected(reason) for deterministic rules such as cap exceeded.
- StorageFailed(code, retry_class) for SQLITE_FULL, SQLITE_BUSY, SQLITE_IOERR, and similar operational failures.
- OutcomeUnknown(idempotency_key) when the caller cannot know whether commit occurred; retry/query by the same immutable key rather than inventing a new obligation.

Never collapse storage failures into policy rejection, and never acknowledge acceptance before the complete state transition is durable. The PRAGMA max_page_count experiment is a repeatable size-capacity fault, not a real physical disk-full or filesystem error.

## Limits

- This is local Python/SQLite application-transaction behavior using a deterministic database-size cap.
- It does not simulate physical disk exhaustion, I/O faults, filesystem faults, kernel crash, or power loss.
- Runtime is SQLite 3.46.1, so the separate WAL-reset engine gate remains open. The NCF-ALLOC-9 --require-fixed-sqlite guard correctly returns NOT_QUALIFIED on this runtime.
- No Rust, Holochain, multi-host, cryptographic-verifier, or production settlement qualification is claimed.

## Next steps

1. Run the same error-classification contract against the Rust storage adapter and ensure SQL errors map to typed retryable/non-retryable outcomes.
2. Add true filesystem-level ENOSPC / EIO, process termination, and ambiguous-commit tests in a disposable VM or filesystem test harness.
3. Verify that after restart every ambiguous outcome is resolved via the same idempotency key and original receipt.
4. Add exact-head Rust + Holochain multi-agent qualification; retain fail-closed behavior if storage or verification state is indeterminate.
