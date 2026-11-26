# Mycelix Canonical Frontier Encoding v1

Status: implemented by `canonical_owner_cut_entry_v1` and `canonical_knowledge_receipt_entry_v1` in the Rust frontier model. This document is the interoperability contract for independent implementations.

## Binary primitives

- All literal domain tags are exact byte strings ending in one NUL byte.
- Strings are UTF-8 encoded and prefixed by their byte length as an unsigned 32-bit big-endian integer (u32 BE).
- Unsigned sequence numbers, tree sizes, and gap endpoints are unsigned 64-bit big-endian integers (u64 BE).
- Collection counts are u32 BE.
- Digests are encoded as the ASCII string `sha256:` followed by 64 lower-case hexadecimal characters. The binary proof hashes themselves are 32 raw bytes and never the ASCII digest representation.
- Integer overflow or a string/count that cannot fit its length prefix is invalid.
- No JSON serializer, map ordering, locale, platform endianness, or implicit Unicode normalization participates in the encoding.

## Owner-cut leaf

Domain tag:

    MYCELIX-OWNER-CUT-V1\0

Fields in exact order:

1. verifier_ref: string
2. policy_epoch_ref: string
3. owner_ref: string
4. history_ref: string
5. sequence: u64 BE
6. tree_size: u64 BE
7. root_digest: string
8. gap_count: u32 BE
9. For each gap, in strictly increasing non-overlapping order:
   - first: u64 BE
   - last: u64 BE

Checkpoint inclusion metadata is not part of the leaf: it would be self-referential. The aggregate checkpoint root is the RFC 9162 Merkle Tree Hash over the ordered owner-cut leaf payloads. The implementation orders owner cuts by owner_ref and binds each cut to its leaf index.

## Knowledge-receipt leaf

Domain tag:

    MYCELIX-KNOWLEDGE-RECEIPT-V1\0

Fields in exact order:

1. receipt_id: string
2. verifier_ref: string
3. policy_epoch_ref: string
4. assertion_id: string
5. assertion_digest: string
6. owner_ref: string (the verifier's receipt-acceptance log owner)
7. owner_sequence: u64 BE
8. dependency_count: u32 BE
9. Dependencies sorted lexicographically by (owner_ref UTF-8 bytes, sequence):
   - dependency owner_ref: string
   - dependency sequence: u64 BE

The receipt_digest is `sha256:` plus the lower-case hex SHA-256 of these exact canonical bytes.

Receipt inclusion proof fields are excluded from the canonical receipt leaf to avoid self-reference. The canonical receipt bytes themselves are the RFC 9162 log entry. The Merkle leaf hash applies the additional RFC 9162 0x00 prefix.

Duplicate dependencies are rejected. Canonical dependency sorting makes input vector permutations encode identically.

## Owner sequence, gaps and leaf positions

The OwnerCut commits both:

- `sequence`: highest owner-local sequence represented by the cut;
- `tree_size`: number of committed receipt leaves.

A gap range [a,b] means owner-local sequence positions a through b have no receipt leaf in this owner history. The cut requires:

    tree_size + sum(gap.last - gap.first + 1) == sequence

The zero-based leaf index for a non-gap sequence s is:

    s - 1 - sum(length(gap) for gaps whose last < s)

A receipt whose sequence falls inside a declared gap has no leaf index and is Indeterminate. A receipt after prior gaps can still be cryptographically included at its derived leaf index. This encoding makes gaps explicit; it does not by itself prove that no external or unindexed receipt exists.

## Merkle profile

The frontier model uses the RFC 9162 SHA-256 Merkle Tree Hash:

- empty tree: SHA256(empty bytes);
- leaf: SHA256(0x00 || canonical entry bytes);
- node: SHA256(0x01 || left hash || right hash);
- tree shape for non-power-of-two leaf counts is the RFC 9162 largest-power-of-two split.

Proof paths are verified by `mycelix-rfc9162-proof-verifier`. Tree-head signatures, log identity, trusted policy epochs, gossip/witness policy and completeness proofs remain separate qualification steps.

Reference: https://www.rfc-editor.org/rfc/rfc9162.html