# CIV-013 durable-adapter conformance v1

Status: frozen fixture proposal; exact-head CI is required.

This profile is distinct from the civ-013-model-smoke-v1 profile. It pins canonical byte encoding for the SQLite durable adapter and records expected adapter outcomes. The checker does not execute the Rust transition reducer and does not imply storage or operational guarantees.

## Source and profile identity

- Profile: civ-013-durable-adapter-v1
- Producer: Symthaea adapter commit 19b58d053073a121c8f2cd5378dcd4644cf036ec
- Durable record domain: mycelix-civ013-durable-record-v1 followed by one NUL byte
- Fork evidence domain: mycelix-civ013-durable-fork-v1 followed by one NUL byte
- Hash: SHA-256.
- Protocol version: unsigned 16-bit integer, big-endian.
- Generation and receipt sequence: unsigned 64-bit integers, big-endian.
- Strings: unsigned 64-bit byte length, big-endian, followed by UTF-8 bytes.
- Optional digest: 0x00 for None; 0x01 followed by exactly 32 digest bytes for Some.

A record digest covers protocol version, generation, log ID, policy version, anchor digest, receipt sequence/digest, and predecessor record digest. It does not hash SQLite status or metadata rows. Local acceptance is the SQLite transaction that marks the exact prepared row accepted and conditionally moves the metadata head pointer from the exact predecessor.

Fork evidence covers log ID, generation, both competing record digests, and the previous fork-evidence digest. The chain is scoped per log ID.

## Fixture inventory

- DA001-bootstrap-record: bootstrap record hash, both optional digests absent.
- DA002-successor-record: successor record hash with receipt and predecessor digests present.
- DA003-first-fork-evidence: first fork hash with no prior evidence.
- DA004-linked-fork-evidence: fork hash linked to the first.

The independent Python checker reconstructs canonical bytes and checks all four frozen SHA-256 results. It does not import the Rust crate, invoke its reducer, or label adapter outcomes as passing tests.

Thirteen DA005-DA017 state expectations cover bootstrap, idempotent retry, stale predecessor, receipt-tail rollback/equivocation, anchor outages, prepared-state recovery, anchor-ahead/local-history-behind, digest mismatch, competing prepared candidates, and late finalization. These are declarative expectations. The adapter's Rust tests and exact-head CI must independently exercise them.

## Operational state ordering

1. Validate inputs and recover local state against the external anchor.
2. Persist exactly one prepared candidate using an immediate SQLite transaction and exact-predecessor checks.
3. Compare-and-advance the independent anchor from the exact predecessor to the exact candidate.
4. In a local SQLite transaction, mark the candidate accepted and conditionally move the accepted head pointer.
5. A crash after step 2 but before step 3 keeps the accepted predecessor; retry can reuse only the identical prepared candidate.
6. A crash after step 3 but before step 4 can be recovered only if the external anchor matches the exact prepared one-step successor.
7. The returned Record is not a cryptographic signature or quorum proof.

## Claim ceiling

This profile provides golden canonical-byte conformance and declared state expectations only. It is not proof that Rust adapter tests passed, SQLite survives power loss, an independent anchor is implemented, signatures are authentic, witnesses are operationally independent, a quorum is safe, or production anti-rollback is qualified.
