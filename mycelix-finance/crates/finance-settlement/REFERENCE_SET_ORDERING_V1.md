# FIN-ECO-002A canonical reference-set ordering clarification v1

This document is a normative clarification of the existing FIN-ECO-002A v1 Rust behavior. It does not introduce a new settlement commitment profile and does not change any existing registered v1 fixture bytes.

## Normative rule

For every v1 **ordered reference set**:

1. treat each reference as its exact accepted UTF-8 byte sequence;
2. deduplicate exact byte-identical reference values;
3. sort the unique values in ascending lexicographic order of the **raw UTF-8 bytes of the reference value**;
4. encode the unique-count as unsigned big-endian `u32`;
5. for each value in that order, encode `u32 byte_length || raw_utf8_bytes`.

The length prefix is added **after ordering**. It does not participate in the sort key.

Therefore implementations MUST NOT sort the complete length-prefixed encodings.

They also MUST NOT use locale collation, case-insensitive comparison, Unicode normalization, case folding, display rendering, JSON serialization, or host-specific string collation as the sort key.

For v1, byte-distinct accepted references remain distinct protocol references. This document does not introduce Unicode normalization.

## Example that distinguishes the rules

Given the physical input values:

```text
b
z:long
aa
a
b
```

exact duplicate removal and raw-UTF-8 lexical ordering produce:

```text
a
aa
b
z:long
```

The ordered reference-set bytes are therefore:

```text
00 00 00 04
00 00 00 01 61
00 00 00 02 61 61
00 00 00 01 62
00 00 00 06 7a 3a 6c 6f 6e 67
```

Sorting the complete encoded references would incorrectly place `b` before `aa` because their length prefixes differ.

## Digest sets

For every v1 ordered `Digest32` set:

1. deduplicate exact 32-byte values;
2. sort ascending lexicographically by the 32 raw bytes;
3. encode the count as unsigned big-endian `u32`;
4. append each 32-byte digest in that order.

## Conformance fixture

`test-vectors/reference-ordering-v1.json` binds a multi-element finality profile whose `required_evidence_kinds` set necessarily distinguishes raw-reference ordering from full-encoded-reference ordering.

The fixture is normative for this clarification. Rust tests must consume it, and strong qualification must reconstruct it independently without calling production Rust.

## Relationship to CANONICAL_FORMAT.md

Where `CANONICAL_FORMAT.md` says:

```text
ordered reference set: u32 count || sorted encoded references
```

the word `sorted` is governed by this clarification: references are sorted by their raw UTF-8 value bytes **before** each reference is length-prefix encoded.

This clarification preserves the current Rust `BTreeSet<ReferenceId>` v1 behavior and removes a cross-language ambiguity in the prose specification.

## Nonclaims

This clarification does not change source authority, finality thresholds, asset legitimacy, Unicode/confusable policy, legal discharge, or settlement semantics. FIN-ECO-001B owns future authoritative asset-ID hardening; FIN-ECO-002G owns future per-kind evidence quorum semantics.
