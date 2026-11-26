# Experimental Mycelix Sparse Assertion Index V1

Status: research prototype. Implementation: src/sparse_map.rs. Independent Go mirror: tools/economic/temporal-frontier-model/conformance/go/sparse.go.

**This is not RFC 9162 and is not currently a registered RFC 9942 VDS/VDP profile.** Do not describe it as an RFC 9162 or RFC 9942 non-inclusion proof.

## Why add a second proof structure?

An append-only Merkle log proves that a receipt/event was included at a log position and that a later tree extends an earlier prefix. It is not an efficient arbitrary-key index for answering whether one exact (verifier, policy, subject, claim, dimension) entry is absent.

A separate authenticated map supports bounded membership and non-membership for one exact key under one exact map root. They answer different questions:

| Structure | Proven property | Not established |
|---|---|---|
| RFC 9162 append-only log | An exact receipt is included at a leaf/index in a supplied root; a later tree extends an earlier tree | The index is complete; no other facts exist; the root is trusted |
| Experimental sparse map | One exact key maps to a value, or has an empty leaf, under a supplied sparse-map root | Every accepted receipt is indexed; verifier accepted the index; absence outside this root/scope |

Economic OS should eventually verify both structures, not replace the log with the map.

## V1 sparse-map hash profile

Keys and values are 32-byte digests. Keys are 256 bits, traversed most-significant-bit first. The domain is:

    MYCELIX-EXPERIMENTAL-SMT-SHA256-V1\0

SHA-256 uses explicit type tags:
- Present leaf: SHA256(0x10 || domain || key[32] || value[32])
- Empty leaf: SHA256(0x12 || domain)
- Branch: SHA256(0x11 || left[32] || right[32])

Empty-subtree defaults are calculated bottom-up. A proof has exactly 256 sibling hashes, ordered leaf-to-root. Verification is a fixed 256-branch fold. The baseline uncompressed proof is 256 × 32 = **8,192 bytes**, before framing or signature.

This profile is independent of RFC 9162's 0x00 / 0x01 leaf and node tags and CT tree-shape rules.

## Query key/value projection

The exact-assertion query key is SHA-256 over the domain tag MYCELIX-ASSERTION-INDEX-KEY-V1, followed by six UTF-8 strings (verifier, policy epoch, subject, claim, dimension, assertion_ref), each prefixed by a u32 big-endian byte length. This identifies one exact assertion reference, not every historical assertion for the logical claim.

The value is SHA-256 over MYCELIX-ASSERTION-INDEX-VALUE-V1, followed by the receipt digest string and owner reference (each u32 BE length-prefixed), then owner sequence as u64 BE.

The receipt digest must be lower-case sha256: plus 64 hex characters. Owner sequence must be non-zero. This value is an index pointer/commitment, not the receipt itself; the referenced receipt and its append-only inclusion proof must still be resolved and verified.

## Cross-language known-answer vectors

The Rust and Go encoders share these independent fixtures:

- Exact assertion key for verifier `did:mycelix:verifier:finance-v1`, policy `policy-epoch:1`, subject `subject:grant:42`, claim `claim:revoked`, dimension `VALID_AT`, assertion `assertion:42`: `6f02bb0fe2b2c6bd9b0c2efac3f74fdc0db676eb21ac5d5f42648428b3dc1817`.
- Index value for receipt digest `sha256:` followed by 64 lower-case `a` characters, owner `owner:verifier-log`, sequence 7: `d641a80447381fdce99b19eb3e9f4fc58000699bff05c81a3410cf174c5c2d7c`.
- Three-entry map root (keys 00/80/ff in the first byte, values A1/A2/A3 in the first byte): `ca399e0f53e5b7607402225d68a14e607cc7e7e488e44ccf8bde194e6386718f`.
- Non-membership path for the key whose first byte is 40: SHA-256 of the 8,192 concatenated sibling bytes is `dc05c01529111d01b12e06f0ddb2a9b22d78822352ba8425c6768e3e3548766f`.
- Empty-map root: `de3054ab8d693834a8b74a5f07765258eb95da1c61faaf16ce437659142ee250`.

These vectors freeze canonical scope, leaf/branch tags, key-bit order, proof-sibling order and the empty-subtree profile. They do not authenticate the map root or prove index completeness.

## Proof dispositions and completeness

The low-level verifier can establish only:
- **SparseMembershipProofValid**: the key/value leaf recomputes to the supplied root.
- **SparseNonMembershipProofValid**: the key's empty leaf recomputes to the supplied root.

These are properties of one map root. They are not AbsenceEstablishedForProfile. An Economic OS absence query must remain Indeterminate unless all higher-level obligations are separately established:

1. The map root is bound into an authenticated verifier checkpoint and exact policy epoch.
2. The checkpoint identifies the verifier, index profile/version, namespace and query projection.
3. The map source frontier is identified, including exact owner cuts and append-only receipt-log roots from which the index was materialized.
4. An independent replay/audit or qualified builder attestation establishes that the map is a complete projection of every in-scope accepted receipt through that source frontier. A zero counter or self-asserted complete=true flag is insufficient.
5. The source coverage universe is explicit. Unknown owners, omitted sources, gaps, partitions, unresolved forks, rejected/unparsed receipts and delayed index updates remain visible.
6. Source-frontier currentness is fresh and non-equivocating for the declared purpose.
7. The exact lookup key, dimension, scope, profile, checkpoint and verifier are carried in the query certificate.

Even then, the strongest result is bounded to non-membership in this exact profile, namespace, query projection and checkpoint. It is not universal non-existence and not a claim that the verifier never knew a fact outside that index.

The current V1 key is assertion-specific. A proof that assertion_ref A is not indexed does **not** prove that no other assertion for the same (subject, claim, dimension) exists or that no assertion was valid at business time V. A claim-level temporal lookup needs a separate authenticated bucket/range structure committing to the complete candidate assertion set and a verifier that checks the requested V against every in-scope candidate. That range/bucket proof and its coverage theorem are not implemented in V1.

## Fail-closed lifecycle

- Duplicate keys are rejected by insert; explicit updates create a new root.
- Any key, value, path or root mismatch fails closed.
- An absent key in a stale map is not a current absence result.
- Missing/unavailable map, unsupported profile, unanchored root, unknown coverage, unresolved fork or indexer disagreement is Indeterminate.
- Conflicting same-scope map roots at the same claimed checkpoint are retained as conflict evidence; do not select by timestamp or local preference.
- The sparse map is not a journal, payment authorization or external-effect gate, and does not replace the verifier's owner-local receipt log.

## Interoperability route

The long-term route is a registered VDS/VDP profile using RFC 9942 COSE Receipts where appropriate. RFC 9942 separates VDS implementation from receipt semantics and registers supported proof types; RFC 9162's registered SHA-256 profile covers append-only inclusion/consistency, not this custom sparse-map proof. Until registered and independently reviewed, this is an experimental Mycelix structure only.

Do not implement a second Transparency Service or witness quorum here. Compose with CIV-009 signed-head/key-lifecycle and #5942 split-view/witness work. Those layers are prototypes and must be independently qualified.

## V1 qualification tests

- Empty map and empty-map non-membership.
- Present and absent keys in the same root.
- First-bit, middle-bit and final-bit paths.
- Irregular map sizes and shared-prefix keys.
- Wrong key/value/root, changed sibling, missing/extra sibling.
- Duplicate insertion and explicit replacement.
- Query-key scoping by verifier, policy, subject, claim and dimension.
- Cross-language canonical key/value, map-root and proof-hash vectors.
- Proof size and generation/verification p50/p95/p99 by map size.
- Negative controls where the sparse root is valid but not bound to the source log frontier, or where the indexer omits a receipt.

## Claim ceiling

This prototype verifies that a sparse-map membership/non-membership path is mathematically consistent with a supplied root. It does not authenticate the root, establish index completeness, prove universal absence, establish source truth, grant authority or demonstrate production readiness.
