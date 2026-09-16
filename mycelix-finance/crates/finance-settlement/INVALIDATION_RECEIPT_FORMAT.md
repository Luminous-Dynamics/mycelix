# FIN-ECO-002A Settlement Invalidation Receipt v1

This document defines the language-neutral canonical byte profile for the audit-complete settlement invalidation receipt introduced by the FIN-ECO-002A receipt-completeness successor.

It does **not** define provider truth, legal reversal, refund behavior, Business failure, or trusted current time. It commits the exact invalidation theorem that the Finance settlement kernel derived plus the temporal evaluation-context identity under which that theorem was evaluated.

## Domain

The canonical byte sequence begins with the ASCII domain prefix including its terminal NUL byte:

```text
MYCELIX_FINANCE_SETTLEMENT_INVALIDATION_RECEIPT_V1\0
```

Changing any field meaning, ordering, or encoding requires a new domain/profile revision.

## Primitive encodings

The receipt reuses the FIN-ECO-002A canonical primitive rules:

```text
u8       = 1 byte
u16      = 2 bytes unsigned big-endian
u32      = 4 bytes unsigned big-endian
u64      = 8 bytes unsigned big-endian
Digest32 = 32 raw bytes
reference = u32 byte_length || UTF-8 bytes
reference set = u32 count || lexicographically ordered canonical references
```

JSON, Rust enum discriminants, debug rendering, locale formatting, native-endian integers, and floating-point values never enter canonical bytes.

## Canonical field order

After the domain prefix, encode exactly:

1. settlement commitment profile revision — `u16`; v1 = `1`;
2. settlement subject ID — reference;
3. reserved/financial-effect commitment — raw `Digest32`;
4. prior finality-profile ID — reference;
5. prior finality-profile semantic revision — `u64`;
6. prior finality-profile canonical digest — raw `Digest32`;
7. operation ID — reference;
8. prior qualified operation revision — `u64`;
9. invalidating observation ID — reference;
10. invalidating observation canonical commitment — raw `Digest32`;
11. evaluation-context canonical commitment — raw `Digest32`;
12. evaluation-context class tag — `u8`:
    - `0x00` DeterministicSupplied
    - `0x01` HistoricalReplay;
13. invalidation evaluation time in Unix milliseconds — `u64`;
14. exact invalidating evidence-ID set — canonical ordered reference set.

Then:

```text
invalidation_receipt_commitment = SHA256(canonical_receipt_bytes)
```

## Why both context commitment and class are present

The evaluation-context commitment already binds the class and temporal profile. The class is also included explicitly in the receipt to keep the receipt self-describing at the semantic boundary where live/historical distinctions matter.

This redundancy is intentional and deterministic.

```text
same millisecond
+ different evaluation-context class
=> different evaluation-context commitment
=> different invalidation receipt commitment
```

A timestamp alone is never sufficient temporal identity.

## Observation and evidence identity

The invalidating observation commitment transitively binds:

- exact subject;
- exact financial-effect commitment;
- execution attempt;
- rail/network;
- operation ID + revision;
- exact AssetAmount;
- state;
- observed time;
- canonical invalidating evidence commitments.

Each evidence commitment binds its exact observation ID and operation revision. The receipt therefore does not duplicate all observation fields; it binds the already-canonical observation commitment plus the human/audit-relevant observation ID and evidence-ID set.

## Golden vector

`test-vectors/invalidation-receipt-v1.json` is normative for v1 interoperability.

The current vector has:

```text
canonical receipt length = 314 bytes
receipt SHA-256 = cdc9cc53d2273c63c962c4695da6ea245b144d5f0fb747a10ec90bc578c0f001
```

The fixture also includes the independently derived revision-2 evidence commitment, invalidating reversal observation commitment, and evaluation-context commitment used by the receipt.

Rust qualification must load the checked-in vector. An independent non-Rust oracle should reconstruct every byte/hash without calling the production canonicalization functions.

## Versioning law

```text
change field meaning
or field order
or primitive encoding
or context-class tag meaning
=> new receipt domain/profile revision
```

Old receipt bytes must never be reinterpreted under new semantics.

## Nonclaims

A valid invalidation receipt proves only that the FIN-ECO settlement invalidation theorem was derived for the exact prior profile/effect/operation and exact temporal evaluation context represented by the receipt.

It does not itself:

- authenticate the external settlement provider;
- prove source-authority independence;
- establish trusted current time;
- reverse a payment;
- issue a refund;
- mutate accounting state;
- determine commercial/legal failure;
- resolve a dispute;
- determine remediation.

Those are separate theorems and authorities.
