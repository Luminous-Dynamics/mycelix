# Mycelix Finance reservation commitment format v1

This document specifies the language-neutral canonical byte encoding used by
`mycelix-finance-reservation-core`.

It defines deterministic commitments only. It does **not** authenticate Finance
issuance, establish Holochain finality, serialize concurrent mutation, or grant
Business/financial authority.

## Cryptographic profile

Both v1 commitments use SHA-256 over the canonical bytes defined below.

No text rendering, JSON serialization, locale-specific representation, native
integer representation, or floating-point value participates in either hash.

## Primitive encodings

- `u8`: one byte.
- `u16`: two bytes, unsigned big-endian.
- `u32`: four bytes, unsigned big-endian.
- `u64`: eight bytes, unsigned big-endian.
- `Digest32`: exactly 32 raw bytes; no hex/base64 wrapper.
- UTF-8 string/reference: `u32 byte_length || utf8_bytes`.
- ordered set of references: `u32 count || item_0 || ... || item_n`, where each
  item uses the reference encoding above and items are sorted by canonical
  `ReferenceId` byte/string ordering before encoding.

All lengths count UTF-8 bytes, not Unicode scalar values or graphemes.

## Reservation descriptor commitment v1

Domain prefix, including the terminal NUL byte:

`MYCELIX_FINANCE_RESERVATION_DESCRIPTOR_V1\0`

The prefix is followed by these fields in exactly this order:

1. commitment profile revision — `u16`; v1 requires `1`.
2. reservation ID — reference.
3. request ID — reference.
4. Finance domain — reference.
5. issuer authority — reference. This identifies the exact authority that the
   runtime adapter must bind to the Holochain action author or equivalent issuer.
6. issuer profile ID — reference.
7. issuer profile revision — `u64`; must be non-zero.
8. issuer profile digest — `Digest32`.
9. Finance policy sequence — `u64`; must be non-zero.
10. Finance policy digest — `Digest32`.
11. subject — reference.
12. asset identifier — UTF-8 string.
13. atomic units — `u64`.
14. effect class — reference.
15. Action Contract semantic ID — reference.
16. Action Contract digest — `Digest32`.
17. Decision Capsule ID — reference.
18. Decision Capsule digest — `Digest32`.
19. authorized intent ID — reference.
20. intent digest — `Digest32`.
21. authority lease ID — reference.
22. authority epoch — `u64`.
23. fencing token — `u64`.
24. aggregate-policy keys — canonical ordered reference set.
25. idempotency key — reference.
26. required finality-profile ID — reference.
27. required finality-profile revision — `u64`.
28. required finality-profile digest — `Digest32`.
29. issued-at Unix milliseconds — `u64`.
30. expires-at Unix milliseconds — `u64`.

The reservation descriptor commitment is:

`SHA256(canonical_descriptor_bytes)`

Mutable lifecycle state is intentionally excluded from the descriptor
commitment.

### Issuer and policy meaning

`issuer authority` and the issuer-profile fields are semantic identifiers, not
proof by themselves. FIN-ECO-004B must map the actual Holochain action author to
the expected issuer authority and validate the issuer profile/policy frontier.

The Finance policy sequence/digest binds the reservation to the exact policy
frontier under which it was issued. Whether a later policy revision invalidates
an already-issued reservation is a runtime policy question; implementations must
not silently substitute another policy frontier while preserving the same
reservation commitment.

## Reservation state commitment v1

Domain prefix, including the terminal NUL byte:

`MYCELIX_FINANCE_RESERVATION_STATE_V1\0`

The prefix is followed by:

1. state commitment profile revision — `u16`; v1 requires `1`.
2. reservation descriptor commitment — 32 raw bytes.
3. transition sequence — `u64`.
4. lifecycle encoding, defined below.

Lifecycle tags:

- `0x00 Active`: tag only.
- `0x01 Consumed`: tag, execution-attempt reference, idempotency reference,
  consumed-at Unix milliseconds (`u64`).
- `0x02 Released`: tag, evidence reference, released-at Unix milliseconds
  (`u64`).
- `0x03 Revoked`: tag, evidence reference, revoked-at Unix milliseconds
  (`u64`).
- `0x04 Expired`: tag, expired-at Unix milliseconds (`u64`).

The state commitment is:

`SHA256(canonical_state_bytes)`

Sequence `1` is the initial `Active` state. Every successful non-idempotent
transition increments the sequence exactly once. A same-reservation,
same-attempt, same-idempotency consumption replay returns the existing consumed
state and does not increment the sequence.

## v1 golden vector

The machine-readable fixture is checked in at:

`test-vectors/reservation-v1.json`

For that fixture:

- canonical descriptor length: `621` bytes.
- descriptor commitment:
  `9fca16ee26def839f6d98d8b413c126a1fdd1267f8d6c065c7515941737538cd`.
- initial Active state commitment at sequence 1:
  `03e797473a7cf8cb8b1ce4fdd6170363c135b28765abcb553eb76fcd7c7599bf`.
- Consumed state commitment at sequence 2 for `attempt:1`, idempotency key
  `payment:idem:1`, consumed at Unix millisecond `400`:
  `5b2aedc5f5fd483a1ff2053057871ddf7be1ce30c3a52aac41c70ab472df2a1d`.

An implementation that does not reproduce these bytes/hashes is not v1
compatible.

## Nonclaims

Matching a commitment proves deterministic representation/equality only. It
does not prove that Finance issued the reservation, that a Holochain record is
current/final, that capacity was actually reserved, or that a transition was
atomically persisted. FIN-ECO-004B is responsible for binding these semantics to
runtime provenance and concurrency control.