# Prism durable dispatch journal reference model v0.1

**Status:** executable, test-only model of append ordering, flush visibility, token issuance and crash replay. It is not a production file journal, operating-system durability test, cryptographic receipt format, or verified storage guarantee.

## The critical invariant

**The transport adapter must not be able to begin handing request bytes to a socket until it holds an ephemeral, one-use DispatchToken returned after the exact DispatchMarked record has crossed the durable flush boundary.**

The token is not reconstructed from a journal replay. This closes an important crash window:

- Crash before the dispatch marker is flushed: recovery sees only the durable Authorized state. The transport could not have started because no token was issued. A fresh attempt may be authorized under current policy.
- Crash after the dispatch marker is flushed: the operation is conservatively DispatchMayHaveOccurred. The original in-memory token is gone, and recovery must not manufacture another one. Query authoritative remote status or hold unknown.
- Crash after the transport begins writing but before the next receipt is flushed: the durable DispatchMarked state still correctly means may-have-dispatched. It does not falsely claim that the request was not sent.

A durable DispatchMarked record therefore acts as an irrevocable one-attempt dispatch boundary, not as a reusable permit.

## Model

The Rust model is prism-egress/tests/durable_journal_model.rs and currently contains 25 deterministic test functions.

Each record binds:
- monotonically increasing journal sequence;
- attempt ID and logical operation ID;
- workload and principal;
- exact origin and request digest;
- application policy and network-enforcement generations;
- optional logical idempotency-key digest;
- record kind, previous-record digest and record digest.

Record kinds are Intent, Authorized, DispatchMarked, RequestWriteStarted, ResponseObserved, and RemoteReconciled. The reference state machine rejects invalid transitions, identity/scope drift, out-of-order records, sequence gaps, prior-digest mismatch and payload tampering.

The durable_len watermark represents the durable prefix. flush() advances it only after validating the appended sequence. recover_after_crash() discards the volatile suffix and reconstructs the durable prefix. This is a model of a durability contract, not an implementation of fsync or a proof that a particular file system or storage device honors it.

The model uses a small deterministic checksum solely to exercise chain and tamper-detection logic. It is deliberately **not cryptographic**. A production journal should use a domain-separated cryptographic digest (the egress crate already depends on BLAKE3), and must separately analyze who can rewrite the journal, rollback, key custody, file permissions, crash consistency, directory-entry durability, storage-controller caches and filesystem/device guarantees. A hash chain alone does not authenticate records against an attacker who can rewrite the whole log.

## Required production API shape

The next production implementation should make the unsafe sequence unrepresentable through private types and a narrow transport adapter:

1. begin_attempt(scope) durably appends Intent.
2. authorize_attempt(scope, current_policy) durably appends Authorized after all application and Nixward policy checks.
3. authorize_dispatch(scope) verifies the durable prefix, appends and flushes DispatchMarked, then returns a private, non-Clone DispatchToken tied to the attempt and exact scope.
4. TransportAdapter::begin_request_write(token) consumes that token. No other public function may write request bytes.
5. Transport and response observations append later journal facts. They never retroactively alter DispatchMarked.
6. Startup replay validates the entire durable chain and produces a recovery state. It never recreates a DispatchToken for an attempt whose durable state already includes DispatchMarked.

The token should include attempt identity, exact request and origin digests, workload/principal, policy/network generations, a marker sequence, and a process-local unforgeable nonce. A process-local nonce is not a substitute for durable history; it complements the durable marker and cannot be replayed after restart.

Group commit is permissible only when token issuance waits for the flush that makes the exact marker durable. It is not permissible to send and fsync later. The transport adapter must not accept a raw boolean such as authorized=true.

## PEAB crash and storage tests

Before claiming production durability, run crash-injection tests against the actual journal implementation:

- terminate before append, after append but before flush, during flush, after successful flush and before token issuance, after token issuance and before first write, during partial request write, after write and before the next journal record, after response and before response flush;
- repeat with process kill, host crash or power-loss simulation, storage-full, short write, fsync error, directory creation/rename, rotation, torn final record and recovery during concurrent writers;
- confirm no unflushed DispatchMarked can issue a token, no replayed DispatchMarked can issue a second token, and no corrupt prefix is silently truncated into an apparently valid success state;
- compare one-fsync-per-record with bounded group commit. Group commit may improve throughput only if no transport write starts before the batch containing its own dispatch marker is durably acknowledged.

Record fsync/fdatasync p50/p95/p99, marker-to-token latency, token-to-first-byte latency, throughput, write amplification, bytes per attempt, WAL growth, rotation cost, crash recovery scan time, invalid-tail handling, and request latency under HTTP/1, HTTP/2 multiplexing, concurrency, slow storage and full-disk fault injection.

Hard gates: **zero writes without a durable token; zero token recreation after replay; zero retry from a dispatched unknown outcome; zero silent acceptance of a corrupt journal prefix.** These remain hard gates even if a faster unsafe configuration has better throughput.

## Important limitations

The 25 tests run only the deterministic Rust model. They do not execute fsync, kill a process, test power loss, validate storage hardware, integrate with Reqwest, or establish that production code has a one-use adapter boundary. All new PEAB scenarios stay not_run_exact_head until the exact PR head is compiled and executed. No performance number is claimed.
