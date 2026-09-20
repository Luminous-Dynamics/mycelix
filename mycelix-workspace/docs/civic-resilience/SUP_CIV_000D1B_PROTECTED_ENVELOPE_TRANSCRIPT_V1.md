# SUP-CIV-000D1B — Protected-Envelope Canonical Transcript v1

Status: independent canonical-byte reference only. Tracks #2347 and stacks directly on SUP-CIV-000D1A exact head `3e2d22b711311d91b3fbde8844817f932d5f8226`.

This tranche freezes **bytes, framing, canonical ordering and strict decode behavior**. It does not select production cryptographic algorithms, key generation, storage topology, recipient authorization, or an envelope hash/ID algorithm.

## Governing boundary

```text
CanonicalEnvelopeBytes
!= RustSerdeBytes
!= JSON
!= CBOR implementation defaults
!= EnvelopeHash
!= CryptographicValidity
```

A product implementation must independently reproduce these bytes. The Python reference is not a library to import into production.

RFC 9180 leaves application message encoding to the application and requires an unambiguous encoding of the values needed by the application. V1 therefore uses a deliberately small map-free binary framing rather than inheriting serializer-specific map ordering, duplicate-key behavior, text normalization, or field-name semantics.

## Integer and byte rules

All integers are unsigned **big-endian**.

```text
u16 = 2 bytes
u64 = 8 bytes
commitment = exactly 32 bytes, all-zero forbidden
```

Variable byte strings are length-prefixed. No float, signed integer, UTF-8 field value, host-endian number, map, set serializer, or implicit optional encoding participates in the normative transcript.

### V1 bounds

```text
MAX_RECIPIENTS        = 64
MAX_WRAPPED_DEK_LEN   = 4096 bytes
MAX_METADATA_BYTES    = 524288 bytes
MAX_CIPHERTEXT_LEN    = 2^63 - 1 bytes
```

The metadata envelope binds ciphertext **artifact commitment + declared byte length** instead of embedding arbitrary-size ciphertext.

## Domain separation

Exact byte prefixes include a final NUL byte:

```text
PayloadAadV1
"mycelix/protected-envelope/payload-aad/v1\0"

RecipientWrapContextV1
"mycelix/protected-envelope/recipient-wrap/v1\0"

ProtectedEnvelopeV1
"mycelix/protected-envelope/full/v1\0"
```

The three transcript domains are not interchangeable.

```text
PayloadAadV1 != RecipientWrapContextV1 != ProtectedEnvelopeV1
```

## PayloadAadV1

Layout:

```text
MAGIC_AAD
u16 schema_version = 1
u16 aead_profile_id
[32] payload_subject_commitment
u64 payload_version
u64 key_epoch
```

Notably absent:

```text
envelope_revision
recipient set
recipient key identities
wrapped DEKs
previous-envelope ref
ciphertext commitment
```

This is deliberate. `AddRecipientCurrentEpoch` and `RewrapRecipientKey` may modify the envelope/wrap metadata while preserving the existing ciphertext and DEK. If `envelope_revision` or the recipient set were part of the payload AAD, those operations would falsely require payload re-encryption.

The reference corpus freezes:

```text
same payload_version + same key_epoch + same AEAD profile
+ envelope_revision change only
-> identical PayloadAadV1 bytes
```

and:

```text
key_epoch changes
-> PayloadAadV1 changes
```

## RecipientWrapContextV1

Layout:

```text
MAGIC_WRAP
u16 schema_version = 1
u16 aead_profile_id
u16 recipient_key_profile_id
u16 wrap_profile_id
[32] payload_subject_commitment
u64 payload_version
u64 key_epoch
[32] recipient_subject_commitment
[32] recipient_key_identity_commitment
```

This context is intended to prevent a wrapped DEK from acquiring meaning merely because the opaque wrapped bytes decrypt under some key.

```text
WrappedDekBytes
!= correct payload
!= correct epoch
!= correct recipient
!= correct key identity
!= current authorization
```

Actual KEM/wrap algorithm mappings and how this context is supplied to them remain 000D1C work.

## ProtectedEnvelopeV1

Layout:

```text
MAGIC_FULL
u16 schema_version = 1

u16 commitment_profile_id
u16 aead_profile_id

[32] payload_subject_commitment
u64 payload_version
u64 envelope_revision
u64 key_epoch

[32] ciphertext_artifact_commitment
u64 ciphertext_len

u8 previous_tag
  0 -> no previous commitment
  1 -> [32] previous_envelope_commitment
  other -> reject

u16 recipient_count

repeat recipient_count:
  [32] recipient_subject_commitment
  [32] recipient_key_identity_commitment
  u16 recipient_key_profile_id
  u16 wrap_profile_id
  u16 wrapped_dek_len
  [wrapped_dek_len] wrapped_dek
```

All version/epoch values must be non-zero. `ciphertext_len` must be `1..2^63-1`.

## Canonical recipient ordering

The encoder first rejects duplicate recipient subjects and duplicate recipient key identities.

It then sorts lexicographically by:

```text
(
  recipient_subject_commitment,
  recipient_key_identity_commitment,
  recipient_key_profile_id,
  wrap_profile_id
)
```

The **strict decoder does not normalize incoming bytes**. Encoded recipient records must already be in strictly increasing canonical order.

```text
reverse input objects
-> encoder canonicalizes
-> same canonical bytes

reverse already-encoded recipient records
-> decoder rejects NonCanonicalRecipientOrder
```

This prevents two wire/storage encodings from representing the same V1 envelope.

## Counter separation inherited from 000D1A

```text
payload_version
!= envelope_revision
!= key_epoch
```

Examples:

```text
recipient rewrap
 -> envelope_revision changes
 -> payload_version unchanged
 -> key_epoch unchanged

replace protected payload
 -> payload_version changes
 -> ciphertext commitment changes

fresh-DEK forward exclusion / compromise rotation
 -> key_epoch changes
 -> ciphertext commitment changes
```

The codec does not infer the validity of those lifecycle transitions; 000D1A owns that theorem.

## Profile-ID boundary

The reference vectors use only test-reserved IDs in:

```text
0xF000..0xFFFF
```

Their numerical presence has **no production cryptographic meaning**.

```text
TestProfileId != ProductionCryptoProfile
```

000D1C must allocate and qualify production AEAD/KEM/wrap/commitment mappings separately, and production profiles must reject the test-reserved IDs.

## Vector authority

The committed vector corpus contains:

```text
12 positive exact-byte vectors
22 negative strict-decode vectors
```

Positive vectors cover all three domains and include:

- one-recipient envelope;
- canonical two-recipient envelope;
- reverse input producing identical canonical bytes;
- previous-envelope reference;
- envelope-revision-only change;
- fresh key epoch;
- max u64 counters / max admitted ciphertext length;
- recipient-wrap context separation.

Negative cases include:

- magic/schema errors;
- zero profile IDs;
- zero fixed commitments;
- zero counters;
- zero/overflow ciphertext length;
- bad optional tag;
- zero recipient count;
- truncation;
- duplicate subject;
- duplicate key;
- noncanonical recipient order;
- zero recipient key/wrap profiles;
- zero/overflow wrapped-DEK length;
- trailing bytes;
- metadata size overflow.

The JSON records SHA-256 for each positive byte vector only as a fixture-integrity convenience:

```text
VectorSHA256 != RuntimeEnvelopeId
```

No production hash algorithm is selected here.

## Why not deterministic CBOR in V1?

RFC 8949 provides deterministic CBOR rules, but a CBOR profile would still need to freeze:

- allowed data model;
- duplicate-map-key rejection;
- map ordering profile;
- integer encodings;
- forbidden tags/types;
- unknown-field behavior;
- decoder behavior for nonpreferred encodings.

Those are reasonable choices for a broad interchange protocol, but this object is small and closed. A map-free transcript gives us a smaller parser and a smaller qualification surface. A later version may adopt a standard serializer only through a new version/domain, never by silently changing V1.

## Qualification theorem

A successful exact-head qualifier may establish:

```text
exact source parent
+ exact four-path child
+ exact vector-file hash
+ Python syntax
+ stdlib-only reference encoder/decoder
+ 12 positive vectors reproduce exactly
+ 22 negative vectors reject with exact classes
+ decode/re-encode identity for admitted full envelopes
+ reverse-input canonicalization
+ encoded-order strictness
+ no production/build/runtime files changed
= SUP-CIV-000D1B canonical transcript evidence
```

It does not qualify parent 000D1A, and parent PASS does not qualify these bytes.

## Next boundary

```text
000D1A semantic lifecycle
        ↓
000D1B canonical transcript + vectors   <- this tranche
        ↓
000D1C production primitive profiles
        ↓
000D1D independent Rust encoder/decoder + crypto binding
        ↓
000D2 storage topology
        ↓
000D3 accountable executable composition
```

I am splitting primitive selection from Rust implementation: choosing exact algorithms/suite IDs is itself a reviewable theorem and should not be hidden inside a first Rust implementation.

## Nonclaims

000D1B does not establish cryptographic security, key entropy, production algorithm choice, KEM/AEAD correctness, recipient authorization, storage currentness/availability, metadata privacy, legal compliance, erasure, retroactive knowledge revocation, municipal authority, Johannesburg readiness, or deployment readiness.
