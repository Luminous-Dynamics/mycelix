# Durable journal checkpointing and compaction: design gate

Status: **design only; not implemented or qualified**.

This note specifies the evidence and failure cases required before changing the
journal's on-disk recovery format. It is deliberately separate from the current
J1/J2 reader and J3 writer. Do not infer checkpoint support from the existence
of this document or from passing the current journal tests.

## 1. What the measurements must answer first

The current journal appends at most three lifecycle records per effect identity:
begin, optional indeterminate, and acknowledgement. Streaming replay avoids a
second full-log byte buffer and reuses the record buffer, but the live index still
contains every distinct identity and its request/provider/status/receipt/evidence
bindings.

Therefore, compaction cannot make the exact identity index constant-memory, and
the maximum record-count reduction for a fully transitioned effect is bounded.
Run journal-replay-scale on a documented host before implementing compaction.
Collect repeated measurements at 10k, 100k, and 1m identities; separate generation
time from open/replay time and peak RSS; record runtime, OS/kernel, filesystem,
storage type, commit SHA, journal byte size, and repetitions. Do not pick a
compaction threshold from a single run.

A compacted representation that preserves every effect entry may reduce bytes
but will not remove the per-entry map cost. Consider a separate bounded external
index only if memory measurements show that the map itself is the limiting cost;
that is a larger query/index design, not log compaction.

## 2. Non-negotiable invariants

1. **No unresolved effect disappears.** Pending and Indeterminate entries must
   survive compaction with the exact request digest and provider-profile binding,
   including None for legacy J1/J2 records.
2. **Acknowledgement is a permanent identity tombstone.** Acknowledged identities
   must survive in the checkpoint so begin_effect cannot return Started for a
   previously acknowledged ID and accidentally dispatch it again.
3. **No evidence upgrades.** Legacy records must not acquire provider-profile or
   source-evidence bindings they did not record. An absent digest remains absent.
4. **No cross-profile transitions.** Checkpoint serialization/replay must retain
   and validate request, provider, receipt, and source-evidence bindings exactly
   as the J3 journal does today.
5. **No silent repair.** Truncated, corrupt, unknown-version, duplicate-ID, or
   inconsistent checkpoint content must fail closed. Do not skip a bad suffix and
   do not fall back to an older checkpoint if a newer committed checkpoint is
   corrupt without an explicit, externally reviewed recovery procedure.
6. **The currently committed journal always remains recoverable**
   through every crash boundary. A temporary file is never the sole authoritative
   copy until a durable commit point selects it.
7. **Single-writer exclusivity remains enforced.** Checkpoint creation, commit,
   active-file replacement, and descriptor reopen all occur under the existing
   lock; stale-lock recovery remains explicit rather than automatic.
8. **Durability assumptions are stated per platform.** The implementation must
   validate same-filesystem rename semantics, sync the temporary snapshot, sync
   the directory after namespace changes, and handle file-descriptor replacement
   explicitly. Do not silently assume identical Windows and Unix semantics.

## 3. Candidate format

If measurement justifies implementation, introduce a new explicit checkpoint
format rather than rewriting J1/J2/J3 records ambiguously. The checkpoint must
have a header with a format version and declared entry count, followed by one
canonical row per identity in deterministic identity order. Each row preserves:

- canonical effect ID;
- exact request digest;
- provider-profile digest or explicit None;
- Pending, Indeterminate, or Acknowledged status;
- receipt digest only for acknowledged rows;
- source-evidence digest when historically recorded, otherwise explicit None.

The reader must reject duplicate IDs, impossible status/receipt combinations,
non-canonical fields, unexpected trailing fields, entry-count mismatch, and
trailing/truncated records. A count catches simple truncation but is **not** a
cryptographic integrity or authenticity mechanism. Current digest fields are
caller-supplied and the journal is not tamper-evident; keep that claim ceiling.

## 4. Crash-safe commit protocol to prove before coding

The protocol must be accompanied by a state-transition diagram and executable
crash-injection tests. At minimum, reason about these boundaries:

1. Snapshot temporary file created; no bytes written.
2. Partial snapshot write.
3. Snapshot fully written but not synced.
4. Snapshot synced, directory entry not yet durable.
5. Snapshot installed but commit selector not durable.
6. New checkpoint selected durably; old journal still present.
7. Active journal replacement/truncation begun.
8. New active journal synced, but its directory entry not yet durable.
9. New active journal installed and reopened for append.
10. Process exits after durable state change but before caller observes success.

At each boundary, a restart must choose exactly one valid recovery path and
must preserve all effect identities and states. It must never append to an old
inode after an atomic rename has installed a new path. All post-rename appends
must go through a descriptor reopened against the active pathname.

Do not adopt a two-file snapshot-plus-log scheme until the authority/commit
selector and recovery for partially completed rotation are precisely defined.
A safer generation directory plus one atomically replaced current-generation
manifest may be easier to reason about, but adds format and cleanup complexity.
Immutable generation artifacts may be retained until garbage collection has
proof that no live selector references them; premature deletion is forbidden.

## 5. Required verification before merge

- Golden replay tests for J1, J2, J3, and the checkpoint format.
- Round-trip test asserting every JournalEntry field is identical before and
  after snapshot/replay, including legacy missing bindings.
- Duplicate identity and conflicting receipt/evidence rejection.
- Pending and Indeterminate survive checkpoint and remain non-dispatching.
- Acknowledged IDs remain idempotent and cannot be reused to dispatch.
- Crash injection at every boundary in section 4, using child processes and
  abrupt termination where appropriate.
- Corrupt-header, incorrect-entry-count, duplicate-row, invalid status, invalid
  digest, missing final newline, truncated checkpoint, and appended-garbage tests.
- Disk-full/write failure simulation where feasible; after a possibly partial
  write, the live instance must be poisoned and must require reopen/reconcile.
- Unix permission/symlink tests and documented native ACL/rename tests for any
  supported non-Unix platform.
- Scale comparison before/after on the same host, including open/replay latency,
  peak RSS, disk bytes, and effect-count correctness for repeated runs.
- Exact-head formatting, unit/integration tests, and strict Clippy results.
- README and PR claim ceiling updated only after those observations exist.

## 6. Decision gate

Proceed to implementation only if measured replay time or disk overhead warrants
the new recovery protocol. If memory pressure comes from the live identity map,
compaction is not the remedy: it retains identities to preserve no-redispatch
semantics. Explore a disk-backed exact index or a typed external-effect store
rather than deleting acknowledged identities or weakening recovery guarantees.

Current status: streaming replay is present in the code; this design note alone
does not mean checkpointing, compaction, benchmarks, or production recovery have
been implemented or proven.
