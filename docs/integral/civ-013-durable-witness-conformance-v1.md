# CIV-013 Durable Adapter Conformance v1

**Profile ID:** `civ-013-durable-adapter-v1`  
**Status:** experimental reference profile; not production qualification.  
**Source adapter revision:** [Symthaea commit `f3def874aa868c5cb1e0997f601096bc33b0df45`](https://github.com/Luminous-Dynamics/symthaea/commit/f3def874aa868c5cb1e0997f601096bc33b0df45)  
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
| DA024 | ExternalAnchorMismatch | Equal-generation recovery fails closed when the local accepted-head digest is absent |
| DA025 | IdempotentAcceptedHistory | Late finalization confirms that the metadata pointer matches the accepted current-head row |
| DA026 | CorruptCurrentHeadMetadata | Late finalization rejects a well-formed but incorrect metadata head digest |
| DA027 | CorruptCurrentHeadRecord | Late finalization rejects field tampering in the accepted head row even when the metadata pointer still matches its stored digest |
| DA028 | CorruptCurrentHeadRecord | Same-generation retry rejects current-head field tampering when the metadata pointer still matches its stored digest |

## Late-finalization metadata binding

The adapter permits idempotent success for both an exact same-generation retry and a historical candidate after another process recovered its prepared candidate and advanced to a later generation. That exception must not treat a correctly sized digest as proof of a correct head pointer or a well-shaped stored row as a self-consistent record. Before returning success for a historical candidate, the adapter checks that the metadata head row exists, is accepted, and its digest matches the record row at the metadata generation; it then validates the accepted history and recomputes record digests in the same transaction snapshot. DA025–DA027 capture valid pointer matching, pointer mismatch, and field tampering under an unchanged digest column. Any mismatch is corruption, not idempotent success.

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
