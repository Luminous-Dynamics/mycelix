# CIV-013 Durable Adapter Conformance v1

**Profile ID:** `civ-013-durable-adapter-v1`  
**Status:** experimental reference profile; not production qualification.  
**Source adapter revision:** [Symthaea commit `d76ddc77f4ac325b75450241f705d548c7835e77`](https://github.com/Luminous-Dynamics/symthaea/commit/d76ddc77f4ac325b75450241f705d548c7835e77)  
**Golden vectors:** [civ-013-durable-witness-conformance-v1.json](civ-013-durable-witness-conformance-v1.json)  
**Manifest:** [civ-013-durable-witness-conformance-v1-manifest.json](civ-013-durable-witness-conformance-v1-manifest.json)  
**Independent checker:** [verify_civ013_durable_witness_conformance_v1.py](../../scripts/integral/verify_civ013_durable_witness_conformance_v1.py)  
**Tracking issue:** [#4916](https://github.com/Luminous-Dynamics/mycelix/issues/4916)

## Why this is a separate profile

The Mycelix `civ-013-model-smoke-v1` profile models record/commit-marker state in memory. The SQLite adapter has a different durable encoding and local commit boundary. This profile makes that distinction intentional and testable; the model profile is not a substitute for it.

This profile is a portable test contract. Passing its hashes and abstract transition vectors does not mean that the Rust SQLite tests ran, that an operating system honored sync requests, or that a real independent anchor exists.

## Canonical durable record encoding

All integers are unsigned and big-endian. Variable byte strings use an unsigned 64-bit byte length prefix. Text fields are UTF-8 and must be non-empty. Hashes are SHA-256 over the concatenated bytes below.

A record encodes, in order:

1. Domain separator `mycelix-civ013-durable-record-v1\0`.
2. Protocol version as u16.
3. Generation as u64.
4. Length-prefixed UTF-8 `log_id`.
5. Length-prefixed UTF-8 `policy_version`.
6. 32-byte `anchor_digest`.
7. Receipt sequence as u64.
8. Optional receipt digest: `0x00` for absent, or `0x01` followed by 32 digest bytes.
9. Optional predecessor digest using the same option encoding.

The computed SHA-256 is the record identity. The record's `digest` field itself is not included in its preimage. Protocol version 1 is the only version accepted by these vectors.

The encoding uses u64 fields, but the current SQLite adapter persists generation and receipt sequence in SQLite's signed 64-bit `INTEGER` columns. Consequently the adapter rejects values above `i64::MAX` (2^63 - 1), even when those values fit the canonical u64 encoding. DA022 and DA023 qualify that adapter-specific boundary; they do not change the canonical encoding.

## Canonical durable fork-evidence encoding

Fork evidence encodes, in order:

1. Domain separator `mycelix-civ013-durable-fork-v1\0`.
2. Length-prefixed UTF-8 `log_id`.
3. Generation as u64.
4. 32-byte first record digest.
5. 32-byte conflicting record digest.
6. Optional previous fork-evidence digest.

The two record digests must be distinct. Including `log_id` in the hashed bytes is deliberate: the same conflict payload copied to another log must not retain the same evidence digest.

## Local commit and external anchor are separate

The adapter represents **local durable acceptance** through an SQLite transaction that changes a prepared record to accepted and advances `witness_meta`. It does not use the model profile's separately hashed commit-marker value. The external anchor is then a separate trust boundary:

```text
validate exact predecessor + input
  -> persist Prepared in SQLite
  -> compare-and-advance external anchor
  -> atomically mark Accepted + advance local head
  -> return record to caller
```

Recovery behavior:

- If local accepted head and external anchor match, return the accepted head.
- If the external anchor identifies the exact one-generation prepared successor, finalize that candidate and return it.
- If the anchor is unavailable, belongs to another log, is behind the accepted local head, or disagrees at the same generation, fail closed.
- If a caller resumes late after another recovery has accepted its exact record and a later successor has committed, return idempotent success only for that exact accepted historical record. Never move the head backwards.
- Receipt sequence cannot decrease. A different digest at the same sequence is equivocation. Sequence zero requires no digest; positive sequence requires one.
- A prepared candidate conflicting with another candidate at the same generation is a conflict, not an instruction to overwrite the existing candidate.

The profile's transition vectors are an independent abstract reference for these outcomes. They do not emulate SQLite's locking, transaction commits, process crashes, or filesystem behavior.

## Vector inventory

| Vector | Expected result | Property |
|---|---|---|
| DA001–DA002 | VALID_RECORD_DIGEST | Canonical durable record encoding for bootstrap and successor |
| DA003–DA005 | VALID_FORK_DIGEST | Fork evidence encoding, log scoping, and prior-evidence chaining |
| DA006–DA007 | StalePredecessor | Wrong expected predecessor digest or generation |
| DA008–DA010 | ReceiptRollback / ReceiptTailEquivocation / InvalidInput | Receipt-tail monotonicity and shape |
| DA011–DA013 | AnchorUnavailable / ExternalAnchorMismatch | Missing anchor, wrong-log anchor, and same-generation digest mismatch |
| DA014–DA015 | RollbackDetected / RecoverPreparedSuccessor | Anchor is ahead by multiple generations versus exact prepared successor recovery |
| DA016–DA017 | PreparedCandidateConflict / IdempotentAcceptedHistory | Competing prepared candidate and late finalization after a later successor |
| DA018–DA020 | RejectCorruptForkEvidence / RejectRecordDigest / RejectForkDigest | Fork-order and tampering rejection |
| DA021 | RejectCorruptForkEvidence | A single fork-evidence history cannot splice entries from different log IDs, even when each entry's digest is valid |
| DA022–DA023 | RejectSqliteIntegerRange | Adapter rejection of generation or receipt sequence above SQLite's signed-integer maximum |
| DA044 | RejectBeforeStateMutation | An oversized receipt sequence is rejected during input validation, before any anchor read, prepared/accepted row write, or external-anchor change; SQLite integrity remains `ok` |
| DA045 | RejectBeforeStateMutation | An unrepresentable successor generation is rejected before recovery performs any anchor read or local state mutation; SQLite integrity remains `ok` |
| DA046 | ConsistentSnapshotMaintained | Integrity queries observe one WAL snapshot while a concurrent writer commits, then a fresh snapshot verifies the successor |
| DA024 | ExternalAnchorMismatch | Equal-generation recovery fails closed when the local accepted-head digest is absent |
| DA025 | IdempotentAcceptedHistory | Late finalization confirms that the metadata pointer matches the accepted current-head row |
| DA026 | CorruptCurrentHeadMetadata | Late finalization rejects a well-formed but incorrect metadata head digest |
| DA027 | CorruptCurrentHeadRecord | Late finalization rejects field tampering in the accepted head row even when the metadata pointer still matches its stored digest |
| DA028 | CorruptCurrentHeadRecord | Same-generation retry rejects current-head field tampering when the metadata pointer still matches its stored digest |
| DA029 | RollbackDetected | A post-prepare anchor already ahead by more than the candidate generation is treated as missing/rollback history, not a same-generation fork |
| DA030 | RollbackDetected | A pre-prepare observation of an anchor ahead by more than one generation rejects without writing a candidate or false fork evidence |
| DA031 | CorruptForkEvidence | A valid but truncated fork-evidence prefix is rejected when the persisted count/tail commitment identifies a longer history |
| DA032 | CorruptForkEvidence | A modified per-log fork-evidence tail digest is rejected even when the row chain itself is valid |
| DA033 | CorruptForkEvidence | An append attempted after tail corruption is rejected without changing the persisted evidence row count |
| DA034 | MigrateLegacyForkMetadata | A valid legacy fork chain is validated and atomically seeded with its count/tail commitment; schema version advances only after success |
| DA035 | CorruptForkEvidence | A corrupted legacy fork chain blocks migration and leaves schema version unchanged |
| DA036 | CorruptForkEvidence | Startup-style semantic integrity validation detects a truncated fork tail before the store can be used |
| DA037 | RecoverExactPreparedSuccessor | Abrupt child exit after anchor commit leaves the predecessor accepted and exact successor prepared; a fresh process recovers it without fork evidence |

## Late-finalization metadata binding

The adapter permits idempotent success for both an exact same-generation retry and a historical candidate after another process recovered its prepared candidate and advanced to a later generation. That exception must not treat a correctly sized digest as proof of a correct head pointer or a well-shaped stored row as a self-consistent record. Before returning success for a historical candidate, the adapter checks that the metadata head row exists, is accepted, and its digest matches the record row at the metadata generation; it then validates the accepted history and recomputes record digests in the same transaction snapshot. DA025–DA027 capture valid pointer matching, pointer mismatch, and field tampering under an unchanged digest column. Any mismatch is corruption, not idempotent success.

## Fork-evidence schema migration and startup validation

The database now uses `PRAGMA user_version=2` to distinguish new databases from earlier schemas without the fork-tail commitment. Opening a legacy database validates each pre-existing fork chain before atomically backfilling its count/tail commitment. A malformed legacy chain fails migration, and the new schema version is not published. DA034 and DA035 cover valid migration and corrupted legacy history.

Normal `integrity_check()` validates per-log semantic histories as well as SQLite's physical structure; DA036 covers the startup-style path that must reject a truncated fork tail.

## Fork-evidence tail commitment

Fork evidence is append-only within the SQLite adapter, but validating only the rows that remain cannot detect deletion of the final row: the shortened prefix can still be a valid hash chain. The local schema therefore stores a per-log `witness_fork_meta` commitment with the evidence count and tail digest. Each append updates the evidence row and this commitment in the same `BEGIN IMMEDIATE` transaction. Reads fail closed if the row count or current chain tail disagrees with the commitment.

DA031 deletes the last evidence row while leaving the count/tail commitment untouched; DA032 leaves all evidence rows in place but mutates the persisted tail digest. Both must return `CorruptForkEvidence`. Appending further evidence to a chain that already fails validation is also rejected before a new row is written. DA033 models that no-write requirement explicitly.

This is local truncation/corruption detection, not a tamper-proof external anchor. An actor who can rewrite or restore both the evidence rows and their local commitment can still roll them back together; anti-rollback for evidence history needs an independent trust domain.

## Per-log fork chain boundary

Each SQLite fork-evidence chain is queried and validated for one exact `log_id`. The hash preimage binds the log ID, but hash validity alone does not prove that a sequence of individually valid records belongs to one chain. DA021 checks a cross-log splice and requires rejection.

## Bootstrap authorization boundary

The API requires the caller to authenticate and authorize the initial policy version and initial checkpoint digest before calling `initialize()`. The adapter does not authenticate that caller or infer the underlying checkpoint's semantic truth. A generation-zero external anchor is an initial position, not by itself proof that an arbitrary caller-selected first checkpoint is trusted.

## What this profile does not establish

It does not establish:
- that the named Symthaea test suite passed on that exact revision;
- actual filesystem/SQLite crash consistency or power-loss behavior;
- operational correctness of any implementation of `IndependentAnchor`;
- signatures, key lifecycle, witness quorum, independent operation, transparency monitors, or global non-equivocation;
- anti-rollback when the database and external anchor share a rollback domain;
- truth of the log's underlying claims or governance legitimacy.

SQLite documents that WAL with `synchronous=FULL` performs an additional WAL sync at each transaction commit to improve power-loss durability, while that guarantee still depends on the VFS and storage stack honoring synchronization requests: https://www.sqlite.org/pragma.html. `BEGIN IMMEDIATE` acquires write intent up front and can return `SQLITE_BUSY` if another writer is active: https://www.sqlite.org/lang_transaction.html. These documented behaviors support the adapter design; they are not evidence that this particular crate has completed its required tests.


## Subprocess recovery split-state contract

DA037 describes the source-level integration test's decisive pre-recovery observation, not hardware durability. After the first child exits with code 86, the independent test anchor must be at generation two while local metadata still accepts generation one and generation two remains prepared (status zero); no fork evidence is written. A distinct recovery process then succeeds at generation two with the exact externally anchored digest. The fixture uses a second SQLite file on the same host and must not be described as a production independent trust domain.


## SQLite WAL-reset fix floor

The producer workspace pins rusqlite 0.40.2 with `bundled` and `fallible_uint`, resolving libsqlite3-sys 0.38.2 and bundled SQLite 3.53.2. SQLite's official WAL documentation identifies the WAL-reset corruption bug through 3.51.2 and lists 3.51.3 as fixed; because this adapter deliberately uses WAL and concurrent connections, the source checks `rusqlite::version_number()` at connection-open time and fails closed below 3.51.3. It has separate unit tests for the actually linked runtime and for rejecting a simulated 3.51.2 runtime through the same extracted version-validation function used by connection opening.

- DA039 models acceptance of SQLite version number 3051003 (3.51.3).
- DA040 models rejection of version number 3051002 (3.51.2).

Sources: [SQLite WAL documentation, §11](https://www.sqlite.org/wal.html#the_wal_reset_bug), [SQLite 3.51.3 release notes](https://www.sqlite.org/releaselog/3_51_3.html), [rusqlite 0.40.2 documentation](https://docs.rs/crate/rusqlite/0.39.0).

These vectors express the adapter's version policy. Only the source crate's executed runtime-version test proves the linked library reports a version at or above the minimum; the profile itself remains reference-vector conformance, not proof that Actions or Rust tests ran.

## Missing persistent anchor state

DA038 specifies that absence of a persistent test-anchor row is not equivalent to a provisioned generation-zero state. The test anchor now returns unavailable for a missing row and only returns genesis when a row was explicitly provisioned. This is a test-fixture invariant that prevents a missing anchor from silently resetting anti-rollback state; it does not qualify a production anchor.


| DA039 | AcceptSqliteRuntimeVersion | A bundled SQLite 3.51.3 runtime meets the WAL-reset-fix minimum |
| DA040 | UnsupportedSqliteVersion | A runtime at SQLite 3.51.2 is rejected before the adapter opens a store |


## SQLite WAL-reset fixed-version floor

The durable adapter now pins `rusqlite 0.40.2` and `libsqlite3-sys 0.38.2`, whose bundled SQLite version is 3.53.2 according to the [upstream rusqlite 0.40.2 README](https://github.com/rusqlite/rusqlite/blob/v0.40.2/README.md). SQLite's [WAL documentation](https://www.sqlite.org/wal.html) states that the WAL-reset corruption bug affects releases through 3.51.2 and is fixed in 3.51.3 and later; see also the [3.51.3 release notes](https://www.sqlite.org/releaselog/3_51_3.html).

The adapter refuses to open a store unless `rusqlite::version_number()` is at least `3051003` (SQLite 3.51.3), and its runtime regression checks that the linked engine is exactly the pinned bundled release 3.53.2 as well as above the fix floor. The public integrity check now requires an explicit `&Transaction<'_>` helper and uses one deferred read transaction for SQLite integrity, foreign-key, log-enumeration, and semantic-history checks. Its unit regression calls the helper by concrete type from the free test function, avoiding an invalid `Self` reference outside an `impl`. DA039 binds the current 3.53.2 bundle; DA040 injects 3.51.2 into the version gate and expects rejection. This separates the exact reproducible build input (3.53.2) from the minimum acceptable runtime floor (3.51.3). The Mycelix Actions workflow checks the exact pinned source's constant, runtime-gate call, and positive/negative regression names in addition to verifying the `rusqlite`/`libsqlite3-sys` versions in `Cargo.lock`. None of these static/reference checks substitutes for an exact-head Rust test PASS.


| DA041 | UnanchoredForkEvidenceErasure | Coordinated deletion of local fork rows and same-database tail metadata is outside the current claim because the external anchor tracks accepted-head state only |

| DA042 | ExternalAnchorMismatchAndForkEvidenceRecorded | Recovery seeing a same-generation, different external-anchor digest records the competing pair locally, leaves accepted head unchanged, creates no prepared candidate, still fails closed, and repeated identical retries do not duplicate the fork event |
| DA043 | ExternalAnchorMismatchWithoutForkEvidence | A same-generation mismatch at generation zero remains an anchor mismatch but creates no fork row because genesis is not a record |

## Fork-evidence trust boundary and follow-on

The current count/tail commitment is in the same SQLite database as the fork-evidence rows. It detects suffix truncation, interior/order/digest inconsistency, and metadata mismatch when some part of the committed state survives. It **cannot** prove that no fork was ever recorded if an actor coherently deletes both `witness_fork_evidence` and `witness_fork_meta`, because the current `IndependentAnchor` commits only accepted-record generation/digest and does not retain the fork-evidence frontier.

Therefore the profile does not qualify fork evidence against a hostile actor able to rewrite the entire local database. A production follow-on should add an independent per-log monotonic fork frontier (count + tail digest bound to the accepted-head position) and explicitly model the fork-record/anchor-update crash, retry, and recovery interleavings. That should be a separately reviewed operation (for example, `compare_and_record_fork` or a signed append-only witness event), not an assumed same-generation accepted-head advance. No production anchor implementation is included here.


The adapter makes fork-report append idempotent for an exact `(log_id, generation, first_digest, conflicting_digest)` tuple after validating the existing chain and tail commitment. DA042 explicitly repeats the same non-genesis recovery mismatch twice, requires the returned error to remain `ExternalAnchorMismatch`, and requires exactly one local fork event. The checker rejects generation zero and malformed numeric fields for this record-level fork case. This prevents retry loops from inflating the local evidence chain; it still does not make that chain externally anchored.


## Genesis recovery mismatch boundary

DA043 covers generation zero specifically: if the external anchor claims genesis but carries a nonzero digest, recovery returns `ExternalAnchorMismatch` and fails closed without accepting a record, preparing a candidate, or inserting fork evidence at generation zero. The fork-evidence schema models conflicts between actual record generations only.


## Receipt-sequence integer boundary

DA044 is backed by a Rust regression that supplies `i64::MAX + 1` on an otherwise valid bootstrap request. It expects the explicit `InvalidInput("receipt sequence exceeds SQLite INTEGER range")` outcome, zero external-anchor reads, unchanged genesis anchor state, no accepted or prepared local records, and a successful physical/semantic integrity check. This is stronger than relying on SQLite's non-negative column constraint to reject a narrowed negative integer after the operation reaches persistence.

## Generation integer boundary

DA045 is backed by a Rust regression that supplies `i64::MAX` as the expected generation. Its successor would exceed SQLite's signed-integer range, so the adapter returns `GenerationOverflow` before calling recovery or reading the independent anchor. The regression asserts zero anchor reads, unchanged genesis anchor state, zero accepted/prepared records, and successful physical/semantic integrity checking.


## Consistent-snapshot integrity validation

DA046 covers two legitimate database views, not corruption. The integrity reader establishes a WAL read snapshot at generation one; a second connection commits generation two; all integrity, foreign-key, log-enumeration, and semantic-history checks within the original transaction must still see generation one and return `ok`. After that read transaction ends, the public integrity check takes a fresh snapshot and sees generations one and two, also returning `ok`. This prevents a healthy concurrent commit from producing a mixed-view false corruption report; it is not a power-loss durability claim.
