# CIV-014 Fork-Witness Conformance v1

**Profile ID:** `civ-014-fork-witness-v1`  
**Status:** experimental reference profile; not production qualification.  
**Producer:** [Symthaea source commit `84cbae9e16823f7947e4ce6adbd20ac9167562df`](https://github.com/Luminous-Dynamics/symthaea/commit/84cbae9e16823f7947e4ce6adbd20ac9167562df)  
**Adapter source:** `crates/domains/symthaea-civ-witness/src/lib.rs`  
**Protocol source:** `crates/domains/symthaea-civ-witness/src/fork_witness.rs`  
**Golden/reference vectors:** [civ-014-fork-witness-conformance-v1.json](civ-014-fork-witness-conformance-v1.json)  
**Manifest:** [civ-014-fork-witness-conformance-v1-manifest.json](civ-014-fork-witness-conformance-v1-manifest.json)  
**Independent checker:** [verify_civ014_fork_witness_conformance_v1.py](../../scripts/integral/verify_civ014_fork_witness_conformance_v1.py)

## Claim ceiling

The checker computes canonical event/receipt hashes independently, evaluates frozen state-transition vectors, verifies exact producer-source/blob pins, and emits a reference-conformance result. The workflow also runs the exact pinned Rust crate's format check and unit-test suite.

A passing run means only **reference-profile agreement plus exact-head crate test execution**. It does not prove that a real external witness is independent, durably retains events, authenticates receipts, resists administrative rollback, or survives hardware/power loss. The profile does not qualify production anti-erasure or anti-rollback.

## Event wire encoding

All digests are 32-byte SHA-256 outputs represented in JSON as 64 lowercase hexadecimal characters. Unsigned integers use fixed-width big-endian encoding. Byte strings use an unsigned 64-bit big-endian byte length followed by raw UTF-8 bytes.

The event bytes are concatenated in this exact order:

1. ASCII domain `mycelix-civ014-fork-event-v1` followed by one NUL byte.
2. Schema version as u16 big-endian (schema version is 1).
3. Length-prefixed UTF-8 log ID.
4. Witness epoch as u64 big-endian.
5. Previous event count as u64 big-endian.
6. Previous event digest (32 bytes).
7. Accepted-head generation as u64 big-endian.
8. Accepted-head digest (32 bytes).
9. Fork generation as u64 big-endian.
10. First competing record digest (32 bytes).
11. Conflicting record digest (32 bytes).

The event identity is SHA-256 of these bytes. Scope and frontier validation additionally require the log/epoch to match, the previous count/digest to equal the expected frontier, an all-zero digest only for the empty frontier, two distinct competing record digests, a positive fork generation, and a generation no greater than the next accepted-head generation.

## Append receipt encoding

Receipt bytes concatenate:

1. ASCII domain `mycelix-civ014-fork-receipt-v1` followed by one NUL byte.
2. Length-prefixed log ID.
3. Witness epoch as u64 big-endian.
4. Previous frontier count as u64 big-endian.
5. Previous frontier tail digest (32 bytes).
6. Exact event digest (32 bytes).
7. Resulting frontier count as u64 big-endian.
8. Resulting frontier tail digest (32 bytes).

A valid receipt binds the same log and epoch, the exact event, the exact previous frontier, and exactly one frontier-step advance whose tail equals the event digest. Receipt hashing does not authenticate the witness; production receipts still require a defined authentication and trust model.

## Durable local handoff model

The producer's schema-v3 journal has a local pending/anchored row state and a local frontier. The expected protocol is:

1. Validate the event and read the independently retained frontier.
2. Commit a local pending event before remote append.
3. Append idempotently at the exact expected frontier.
4. If the response is ambiguous, look up the stable event identity and validate the exact receipt.
5. Confirm the receipt's event is still retained and the witness frontier is not regressed.
6. Finalize the local event and advance its local frontier atomically.

A pending event is not a success receipt. If the witness is unavailable, the retained payload is missing, the frontier regresses/changes, or a surviving local prefix conflicts, recovery must fail closed and preserve the evidence it still has.

Full-prefix restoration reads and checks each remote event payload and receipt before local writes, rechecks remote frontier stability, then restores the local event chain/frontier in one transaction. It is capped at 100,000 events per operation and also honors the caller's smaller limit. A count/tail-only witness cannot reconstruct erased event bodies.

## Frozen vectors

FW026 makes the local pending-row invariant explicit: even if the first remote lookup is unavailable, no success is reported and the exact payload remains in a recoverable pending state. FW027 requires that retrying a locally anchored earlier event after a later successor return the existing receipt without modifying the frontier. FW028 requires a locally anchored event missing from the remote history to fail closed rather than re-append over an externally regressed frontier.

The fixture contains 28 ordered IDs, FW001–FW028. They cover canonical event/receipt encodings, event and receipt tampering, empty/non-empty frontier rules, ambiguous append readback, retry after local pending persistence, remote commit before local finalization, missing remote payload/receipt, remote frontier change during restore, restore work bounds, restoration of an entirely erased local journal, preserving a conflicting surviving local prefix, remote rollback, local orphan detection, and the invariant that recording fork evidence does not advance accepted-head generation.

The checker is a separate standard-library Python implementation. It does not import Rust-produced expected outcomes or access SQLite. The Actions workflow executes both the independent checker and `cargo test -p symthaea-civ-witness --lib` against the exact pinned Symthaea commit.

## Reproducibility and status discipline

The manifest pins the producer commit and Git blob IDs for both Rust source files, workspace manifest, lockfile, crate manifest, toolchain, fixture, checker, specification, and workflow. The checker recomputes Git blob IDs from the file bytes and verifies the detached producer checkout commit. The workflow records the exact profile SHA, source SHA, runner/tool versions, per-step outcomes, and source/asset digests in an uploaded evidence artifact.

Static pins are identity evidence, not execution evidence. Queued, pending, skipped, cancelled, stale-head, or artifact-unavailable runs are not PASS. Even a successful reference profile leaves production backend, witness independence, signer/key lifecycle, event retention/restore policy, quorum/failure behavior, signed checkpoints/consistency proofs, caller-path integration, and power-loss qualification open.

## Research direction

Certificate Transparency v2 (RFC 9162) uses signed tree heads and Merkle consistency proofs to make prefix consistency independently verifiable; Sigstore Rekor similarly documents signed checkpoints and inclusion/consistency verification. These are useful design comparisons, not automatic requirements for CIV-014. A production design should choose explicitly between a simpler externally witnessed append receipt and signed frontier checkpoints with consistency proofs, then define key ownership, rotation, witness epoch semantics, retention, fork handling, and verifier obligations. A digest in the current reference protocol is not a signature or proof of administrative independence.
