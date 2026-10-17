# Pulse recipient receipt V2 contract

## Why V2 is additive

The current `RecipientReceipt` V1 transcript signs the message ID, recipient agent bytes, receipt kind, and occurrence timestamp. It does not explicitly bind the receipt to an envelope commitment, record an independent receipt-event/idempotency identity, or sign the envelope protocol/profile. V1 remains available for compatibility; this patch does not reinterpret or rewrite V1 signatures.

V2 signs a separate, domain-separated transcript containing:

- receipt protocol version;
- stable 32-byte `receipt_event_id`;
- logical `MessageId` and recipient agent bytes;
- observed envelope version and exact cipher-suite identifier;
- 32-byte envelope commitment;
- receipt kind and occurrence timestamp.

The signature bytes are not included in their own signing transcript.

## Commitment algorithm

For the exact canonical signing transcript produced by the selected V1 or V2 envelope type:

```text
preimage = "mycelix-pulse/envelope-commitment/v1\\0"
           || u32be(len(canonical_signing_bytes))
           || canonical_signing_bytes
commitment = SHA-256(preimage)
```

The WASM-safe types crate defines the exact preimage bytes and intentionally does not add a second hash implementation. The caller must compute SHA-256 with the approved implementation. A receipt verifier must independently recompute the commitment from the exact envelope it validated. A caller-supplied commitment alone proves nothing.

## Retry / conflict contract

A sender/recipient implementation must persist `receipt_event_id` before its first attempt and reuse it for every retransmission of that *same signed receipt*. Replays with the same event ID and same canonical signing bytes are duplicate observations of one logical receipt. Reuse of that event ID with any changed signed field—including message ID, recipient, envelope commitment, profile, kind, or timestamp—is an integrity conflict and must fail closed.

The type-level transcript makes those differences observable. It does **not** implement an atomic durable deduplication store, serialize concurrent coordinator calls, prevent duplicate DHT entries, or prove end-to-end exactly-once delivery. Those are separate integration obligations tracked in [#5062](https://github.com/Luminous-Dynamics/mycelix/issues/5062) and [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091).

## Required qualification

- 100% pass rate for frozen V1/V2 known-answer transcript vectors.
- 100% detection of critical mutants that omit the event ID, recipient, envelope commitment, envelope version, suite, kind, or timestamp from the V2 signing transcript.
- Zero acceptance of unknown receipt versions, unsupported envelope versions, or version/suite mismatches.
- Zero advancement to `Delivered` from transport acknowledgement alone.
- Zero same-ID/different-transcript retries accepted as an ordinary duplicate.
- Report p50/p95/p99 transcript construction, commitment hashing, signature verification, and durable dedupe-store latency separately on named hardware.
- Tie all evidence to exact source SHA, build/toolchain, test vector corpus digest, and pinned cryptographic profile. Queued, skipped, stale-head, or missing CI evidence is not a pass.

These are targets, not results. The patch does not claim the Holochain coordinator or supported UI currently uses this V2 receipt contract; migration/wiring and runtime tests are still required before any implementation claim.
