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

The shared `validate_envelope_binding` helper recomputes the commitment from the candidate envelope's own canonical transcript and checks the message ID, recipient, envelope version, and suite against the receipt. It is a context/binding check, not a signature verifier.

The V2 receipt signature profile is fixed by this protocol version: **both** a 64-byte Ed25519 signature under the recipient's Holochain AgentPubKey and a 3309-byte ML-DSA-65 signature under the recipient key bundle bound by the envelope are required. Both sign the same canonical receipt transcript; neither signature field is included in that transcript. Length checks alone do not authenticate either signature, and the source-chain action signature does not replace these application-level checks. A verifier must confirm both signatures, verify the exact envelope commitment, and bind the ML-DSA key to the recipient's validated V2 key bundle. A future signature profile requires an explicit protocol-version/profile change; it must not silently fall back to Ed25519-only. The V2 receipt profile accepts only the V2 hybrid-PQC envelope suite; presenting a legacy V1 envelope under a V2 receipt is a context mismatch and must fail closed.

## Commitment algorithm

For the exact canonical signing transcript produced by the selected V1 or V2 envelope type:

```text
preimage = "mycelix-pulse/envelope-commitment/v1\\0"
           || u32be(len(canonical_signing_bytes))
           || canonical_signing_bytes
commitment = SHA-256(preimage)
```

The WASM-safe types crate defines both the exact preimage and SHA-256 result using the constrained `sha2 0.10` dependency (`default-features = false`), matching the existing `keys_types` dependency profile. The shared helper computes the digest; a receipt verifier must still recompute the commitment from the exact envelope it validated. A caller-supplied commitment alone proves nothing.

## Deterministic receipt-event identity

For the coordinator profile, a deterministic event ID can be derived as:

```text
preimage = "mycelix-pulse/receipt-event-id/v2\\0"
           || message_id[32]
           || u32be(len(recipient_agent)) || recipient_agent
           || receipt_kind_u8
receipt_event_id = SHA-256(preimage)
```

Receipt-kind codes are `Observed=0x01`, `Delivered=0x02`, and `Read=0x03`.
The V2 canonical encoder enforces that `receipt_event_id` equals this derived identity; caller-supplied arbitrary IDs are rejected before signing or verification. The shared helper also rejects recipient identities whose raw Holochain AgentPubKey encoding is not exactly 39 bytes. The exact preimage is frozen in
`test-vectors/receipt-v2-event-id-preimage-delivered.hex`. The corresponding
SHA-256 known answer is recorded in
`test-vectors/receipt-v2-event-id-delivered.sha256`. Do not include the
timestamp, route, attempt number, or envelope commitment in this identity
derivation: otherwise an ordinary retry or conflicting transcript could be
mistaken for a new logical receipt. The commitment itself remains signed and
is compared as part of replay classification.

## Frozen canonical-byte vector

The crate test `v2_receipt_matches_frozen_canonical_byte_vector` consumes
`crates/mail-leptos-types/test-vectors/receipt-v2-delivered.hex`. It freezes
the exact output of `RecipientReceiptV2::canonical_signing_bytes` for the
deterministic test fixture below (signature bytes are intentionally excluded
from the signing transcript):

- protocol version: `2`
- receipt event ID: SHA-256 of the deterministic event-ID preimage, `4a177a8f2100fa6fc2ef4f5ca51cefbd7863f166bb303730673d0a72a039da1f`
- message ID: 32 bytes of `0x20`
- recipient: 39 bytes of `0x21`
- envelope version: `2`
- suite: `x25519+ml-kem-768-hkdf-sha256-aes256gcm-agent-ed25519+ml-dsa-65`
- envelope commitment: 32 bytes of `0x22`
- kind: `Delivered` (`0x02`)
- timestamp: signed 64-bit big-endian `35`

The commitment hash helper is also tested with a frozen known-answer fixture: SHA-256 of the domain-separated commitment preimage for the three-byte transcript `abc` must equal the contents of `test-vectors/envelope-commitment-abc.sha256`. This is a primitive/encoding fixture, not a real envelope.

The vector is a wire-format conformance fixture, not a signature-validity vector:
it does not contain a private key, signature verification result, or live
Holochain receipt. Any change to these bytes requires an explicit protocol
versioning decision and review, not a silent fixture update.

## Retry / conflict contract

V2 requires `receipt_event_id` to be deterministically derived from the immutable semantic identity, not a caller-chosen or random value. The canonical encoder rejects a receipt whose ID does not match the derived value. A sender/recipient implementation may persist the derived ID before its first attempt and reuse it for every retransmission of that *same signed receipt*. The shared preimage helper `canonical_receipt_event_id_preimage` defines the exact encoding: SHA-256 over a domain-separated encoding of message ID, recipient agent bytes, and receipt kind. It deliberately excludes timestamp, transport route, attempt number, and envelope commitment, so conflicting content for the same logical message/recipient/kind reuses the same identity and is detected as a conflict. Replays with the same event ID and same canonical signing bytes are duplicate observations of one logical receipt. Reuse of that event ID with any changed signed field—including message ID, recipient, envelope commitment, profile, kind, or timestamp—is an integrity conflict and must fail closed.

The type-level transcript makes those differences observable. The profile also requires both Ed25519-agent and ML-DSA-65 receipt signatures; shape validation is a separate helper so a signer can calculate the transcript before either signature exists. It does **not** implement an atomic durable deduplication store, serialize concurrent coordinator calls, prevent duplicate DHT entries, or prove end-to-end exactly-once delivery.

The pure `classify_receipt_v2_replay` helper compares an incoming receipt with
the prior receipt found by the same event ID. It returns `ExactReplay` for
identical signed transcript bytes and `ConflictingReuse` for changed signed
content; an event-ID mismatch is an error. The caller must verify both
signatures before acting on the classification. The durable store must make
lookup/reservation atomic, and must retry a Holochain `chain head moved`
outcome by re-reading the current source-chain state before deciding whether
the operation is a replay. This helper by itself does not eliminate concurrent
or persistence races. Those are separate integration obligations tracked in [#5062](https://github.com/Luminous-Dynamics/mycelix/issues/5062) and [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091).

## Required qualification

- 100% pass rate for frozen V1/V2 known-answer transcript vectors.
- 100% detection of critical mutants that omit the event ID, recipient, envelope commitment, envelope version, suite, kind, or timestamp from the V2 signing transcript.
- Zero acceptance of unknown receipt versions, unsupported envelope versions, or version/suite mismatches.
- Zero V1 envelopes accepted under the V2 receipt profile.
- Zero receipts with arbitrary/non-derived event IDs, non-39-byte recipient identities, or a commitment/recipient/message ID that does not match the exact supplied envelope.
- Zero V2 receipt accepted with only one valid signature: both the recipient AgentPubKey Ed25519 signature and the envelope-bound recipient-bundle ML-DSA-65 signature must verify over the exact same canonical transcript.
- 100% detection of critical signature mutants: missing/replaced Ed25519 signature, missing/replaced ML-DSA signature, wrong recipient key bundle, changed envelope commitment, or verification against a different transcript.
- Zero advancement to `Delivered` from transport acknowledgement alone.
- Zero same-ID/different-transcript retries accepted as an ordinary duplicate.
- Report p50/p95/p99 transcript construction, commitment hashing, signature verification, and durable dedupe-store latency separately on named hardware.
- Tie all evidence to exact source SHA, build/toolchain, test vector corpus digest, and pinned cryptographic profile. Queued, skipped, stale-head, or missing CI evidence is not a pass.

These are targets, not results. The contract crate checks signature lengths but does not itself verify cryptographic signature validity. The patch does not claim the Holochain coordinator or supported UI currently uses this V2 receipt contract; migration/wiring and runtime tests are still required before any implementation claim.
