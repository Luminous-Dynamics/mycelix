# FIN-ECO-006 durable append contract v1

**Issue:** [#5873 — FIN-ACC-JOURNAL-DURABLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5873)  
**Semantic owner:** [#1123 — FIN-ECO-006](https://github.com/Luminous-Dynamics/mycelix/issues/1123)  
**Reference implementation:** [PR #5820](https://github.com/Luminous-Dynamics/mycelix/pull/5820)  
**Generic commit classifier convergence:** [#1089](https://github.com/Luminous-Dynamics/mycelix/issues/1089)  
**Status:** research contract / storage-independent; no durable backend qualification is claimed.

## 1. Objective and non-goals

The FIN-ECO-006 reference kernel projects an already-qualified economic source through an exact accounting policy into an immutable, balanced journal entry. Its current `JournalBook` is in-memory. It proves local append/replay/reversal semantics for the lifetime of that process, not durability across process or machine failure.

This contract defines the next theorem: a storage adapter can persist the exact result, enforce the same uniqueness and reversal invariants under concurrent writers, recover after crashes, and allow an independent verifier to reconstruct the persisted book.

It does **not** add a second accounting engine, new economic authority, payment coordinator, generic workpaper scheduler, or new commit-classification algebra. No real database adapter is implemented by this document.

Keep these propositions distinct:

```text
qualified journal entry     != durably appended journal entry
commit error                != definitely not committed
durability classification  != retry authorization
durable journal             != complete books or closed period
balanced entry              != proof of economic authority or settlement truth
```

## 2. Existing semantic contract that storage must preserve

PR #5820 is the semantic reference for this tranche. The durable adapter must preserve its exact meanings, not reinterpret them:

- canonical entry bytes bind schema version, ledger-profile digest, projection-policy digest, source-event identity and commitment, reporting entity and period, effective time, evidence frontier, reversal target and canonical postings;
- posting amounts are exact atomic units; floating point and implicit FX are absent;
- debit and credit totals are checked with exact arithmetic and must both equal the qualified source amount;
- one entry uses one asset-unit profile;
- posting semantic roles, account IDs, normal sides, account schema revisions and reporting entities match the pinned ledger/policy;
- `journal_id` is the digest of the canonical entry bytes;
- `source_application_key` is derived by the reference kernel from the ledger-profile digest, reporting entity and source-event ID;
- exact resubmission is idempotent; same source key with changed source commitment or journal identity is a conflict;
- a reversal is a new entry, targets an existing original, preserves the original, obeys source reversibility, matches exact account/unit/opposite-side/semantic-role identities, and cannot cumulatively exceed any original posting.

The storage adapter MUST call or independently implement these semantic checks. A successful SQL INSERT, Holochain commit, or matching display amount does not itself qualify a journal entry.

## 3. Append request and result model

The semantic append request is an immutable bundle:

```text
JournalAppendRequestV1 {
    ledger_profile_digest,
    projection_policy_digest,
    source_application_key,
    source_commitment,
    journal_id,
    canonical_journal_bytes,
    reversal_target?, 
}
```

The adapter validates the supplied digests by recomputing them from the canonical bytes and their declared inputs. It does not trust caller-supplied IDs or digests without recomputation.

The observable result vocabulary is:

| Result | Meaning |
|---|---|
| `AppendedExact` | The exact entry and all required indexes/reversal effects are positively observed as durably committed. |
| `IdempotentReplayExact` | The same source key, source commitment, journal ID and canonical bytes already exist. |
| `SourceProjectionConflict` | The source key already exists with a different commitment or journal identity. |
| `JournalIdentityConflict` | The journal identity exists but the canonical bytes do not match. |
| `ReversalConflict` | The target, source reversal permission, posting identity, effective-time or cumulative reversal constraint fails. |
| `IndeterminateCommit` | The adapter cannot positively establish either the exact predecessor state or exact successor state after an ambiguous persistence outcome. |
| `StorageIntegrityFailure` | Stored bytes, indexes or commitments are malformed, missing, inconsistent or fail independent reconstruction. |

These are observations about storage state, not permission to execute another financial action. `IdempotentReplayExact` is available only for a byte-for-byte/commitment-exact replay of the same semantic append.

## 4. Atomicity and linearization point

A successful append has one documented linearization point: the durable transaction/CAS boundary for the selected storage profile.

The following must be committed atomically (or through a separately proven equivalent protocol):

1. immutable canonical journal record;
2. unique journal identity and canonical-byte digest;
3. unique source-application key mapped to source commitment and journal identity;
4. exact ledger-profile and projection-policy commitments;
5. ordered append sequence / predecessor frontier, when required by the declared ledger profile;
6. reversal target linkage;
7. cumulative reversed-amount updates for every affected original posting;
8. durable head/checkpoint update required by the adapter.

A reader must never observe a partial state in which, for example, the journal row exists but the source uniqueness key or reversal counters do not, unless the adapter marks that state as explicitly pending/indeterminate and recovery is required before further append.

A read-then-write uniqueness check is insufficient. The selected backend must enforce uniqueness at the transaction boundary, using database unique constraints/serializable transactions, an explicitly scoped single-writer log, or another reviewed mechanism with equivalent concurrency semantics.

### Source uniqueness

The reference `source_application_key` is scoped to one ledger profile, reporting entity and source-event identity. Under that key:

- same source commitment + same journal ID + identical canonical bytes => exact idempotent replay;
- changed source commitment, policy-bound journal ID, entity/period/postings or canonical bytes => conflict;
- no timestamp-only or caller-random identity may bypass the key.

The durable adapter must not silently weaken this scope. A policy or ledger-profile revision change is not a replay of the previous journal projection.

### Reversal race

Concurrent partial reversals must update each original posting's cumulative reversed amount atomically with the reversal journal append. A separate read of current reversed totals followed by a write is race-prone and is not sufficient.

For each original posting identity:

```text
previous_reversed + new_reversal <= original_posting_amount
```

Use checked exact integer arithmetic. A rejected/indeterminate reversal must not increment the cumulative total.

## 5. Commit uncertainty and recovery

Use the generic exact-state commit classification already owned by #756/#759 and its Finance convergence issue #1089. Do not copy its classifier or infer the result from exception strings.

For the exact append subject:

- **Committed:** the exact successor record, uniqueness mapping, indexes and required reversal state are positively observed;
- **DefinitelyNotCommitted:** the exact predecessor state is positively observed and the intended successor is positively absent under the selected store identity;
- **IndeterminateCommit:** neither exact state is positively established, or contradictory/partial state is observed.

The append adapter maps these to `AppendedExact` / `IdempotentReplayExact`, a retryable storage observation subject to the caller's existing policy, or `IndeterminateCommit` / `StorageIntegrityFailure`. The adapter MUST NOT grant retry authority.

For an ambiguous result:

1. stop writes dependent on the unresolved frontier;
2. reopen/re-read using the exact canonical store identity and the adapter's qualified read path;
3. reconstruct the relevant predecessor/successor state and source/reversal indexes;
4. pass only exact observations to the shared classifier;
5. keep unresolved outcomes indeterminate and require reconciliation; never create a new source identity to escape the ambiguity.

A process-local token, missing return value, absent log line, generic COMMIT error or caller timeout is not positive proof of non-commit.

## 6. Storage profiles and deployment boundaries

The contract is backend-neutral. Concrete storage profiles require their own exact threat model and qualification.

- **PostgreSQL:** candidate for a multi-writer service profile, with unique constraints, transaction isolation/locking that closes source-key and reversal races, durable commit assumptions, backup/restore and independent reconstruction.
- **Holochain:** candidate for a source-chain/DHT evidence profile only when the authority and single-writer/uniqueness scope are explicit. DHT publication alone is not assumed to supply a globally atomic multi-writer accounting transaction.
- **SQLite WAL/FULL:** candidate for a host-local reference profile, only after the exact SQLite/rusqlite version, native ABI, filesystem, locking, flush, reopen, corruption and failure assumptions are tested. WAL/FULL alone is not a universal power-loss guarantee.

The first implementation should be chosen after deployment and threat modelling. No backend is selected or qualified by this document.

The generic commit-classification kernel may retain its historical crate name during exact convergence; do not fork, rename or rewrite its behavior while claiming inherited qualification. Follow #1089's exact tree/parent/manifest rules.

## 7. Independent reconstruction

A separate, read-only verifier must reconstruct the journal book from persisted canonical records and the declared durable frontier. It must independently recompute:

- canonical journal IDs and byte digests;
- ledger and policy commitments bound by each entry;
- source-application uniqueness and source-commitment consistency;
- posting semantics, account/entity/unit bindings, and exact debit/credit equality;
- reversal targets, opposite-side semantic identity and cumulative reversal totals;
- append order/predecessor continuity for profiles that claim ordering;
- unresolved, partial, corrupted or indeterminate append attempts.

It must not import the storage adapter's reducer or trust a producer-generated index/checkpoint without checking it against canonical source records. A matching row count or checksum over an unchecked index is not reconstruction.

## 8. Adversarial conformance corpus

Freeze candidate-visible request inputs separately from expected evaluator outcomes. Public reference vectors are not a blind model holdout.

Minimum persistence cases:

| ID | Fault / interleaving | Required outcome |
|---|---|---|
| JDP-001 | First exact append | `AppendedExact` |
| JDP-002 | Same request replayed sequentially | `IdempotentReplayExact` |
| JDP-003 | Same source key, changed source commitment | `SourceProjectionConflict` |
| JDP-004 | Same journal ID, changed canonical bytes | `JournalIdentityConflict` |
| JDP-005 | Crash before transaction begins | Exact predecessor remains; no append claimed |
| JDP-006 | Crash after record insert but before source index | Transaction atomicity or explicit recovery-required state; never partial success |
| JDP-007 | Crash after reversal counter update before commit | No partial reversal total becomes authoritative |
| JDP-008 | Commit succeeds, response is lost | Reopen identifies exact successor; replay is idempotent |
| JDP-009 | Commit returns error, exact predecessor is observed | `DefinitelyNotCommitted` observation; no new economic identity |
| JDP-010 | Commit returns error, neither exact state is observed | `IndeterminateCommit`; block append/retry at that frontier |
| JDP-011 | Two processes race same exact source append | At most one append; other observes exact replay |
| JDP-012 | Two processes race incompatible projections under one source key | At most one append; loser observes conflict |
| JDP-013 | Two partial reversals race at the cumulative limit | Combined reversals never exceed original posting |
| JDP-014 | Same request retries after later successor append | Historical retry remains idempotent; head never moves backward |
| JDP-015 | Canonical bytes or index corrupted | `StorageIntegrityFailure` / quarantine; never repair by guessing |
| JDP-016 | Restore/reopen under mismatched store identity | No silent continuation under a different state history |

Add property/metamorphic checks: exact byte replay, changed-byte conflict, posting-order canonicalization, crash at every transaction boundary, concurrent schedule permutations, and independent reconstruction after every accepted sequence.

## 9. Benchmarks and release gates

Hard gates (not weighted into an aggregate):

- 100% exact idempotent replay for frozen exact duplicates;
- zero duplicate append under same-key concurrency;
- zero incompatible source projections accepted under one key;
- zero reversal overrun under concurrent partial reversals;
- 100% exact-state outcome classification across seeded before/after-commit windows;
- zero `IndeterminateCommit` surfaced as append success or definitely-not-committed;
- 100% independent reconstruction equality after restart/reopen for supported profiles;
- zero rewriting/deletion of committed journal records;
- zero authorization or retry authority emitted by the storage layer.

Operational measures are separate: throughput under contention, p50/p95/p99 append latency, write amplification, recovery time, storage/retention cost, and fully loaded cost per independently reconstructible entity-period.

Targets are aspirations until tested on the exact backend, build, schema, hardware, filesystem, fault model and workload.

## 10. Ownership and claim ceiling

- #1123 / PR #5820 owns journal semantics and in-memory reference behavior.
- #1089 owns Finance convergence of generic durable commit classification.
- #756/#759 own generic commit uncertainty classification.
- #5803/#5805 own the close/workpaper DAG, not journal storage.
- #2525/#2529 own external-effect coordinator recovery, not accounting journal authority.
- #5744 owns the SAP payment-saga effects, not the general ledger store.

A future PASS applies only to the exact adapter, storage profile, deployment assumptions, fault model, corpus, verifier and commit. It does not establish complete books, period close, legal/tax/accounting-standard compliance, audit assurance, provider settlement, economic authority or production suitability.
