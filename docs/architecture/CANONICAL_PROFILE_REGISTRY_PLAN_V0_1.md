# Canonical Profile Registry Plan v0.1

Status: design note on the institutional-core branch; no wire format is changed by this document.

## Problem

Mycelix now has several legitimate uses of deterministic commitments, but they do not all represent the same semantic object and should not be forced into one byte format prematurely.

Current draft lineages include:

- reciprocal-accountability canonical commitments (SIF / accountability core);
- Content Fabric infrastructure IDs and payload commitments;
- governance execution action authorization;
- institutional authority/rulebook/action digests;
- multi-party workflow request/policy/credential digests.

The wrong consolidation strategy is to pick one new generic serializer and migrate everything simply to reduce the number of codecs. Different domains have different identity, ordering, compatibility, and external-standard requirements.

The correct first step is a **canonical profile registry**.

## Registry record

Each profile should publish at least:

- stable profile ID, e.g. `mycelix.accountability.receipt.v1`;
- semantic object being committed;
- owning crate/spec;
- schema version;
- digest algorithm and output length;
- domain separator / magic bytes;
- exact field ordering/framing rules;
- collection semantics (ordered list vs canonical set);
- optional-value encoding;
- string/Unicode rules;
- integer units/endian rules;
- whether floats are forbidden or normalized;
- maximum input sizes;
- at least one exact canonical-byte golden vector;
- expected digest for every vector;
- compatibility/upgrade rules;
- whether the profile is an internal identity, signature preimage, external-standard adapter, or transport-only encoding.

## Non-equivalence rule

Two different canonical profiles MUST NOT be treated as the same semantic identity merely because their source records happen to contain equivalent-looking fields.

Cross-profile relationships require an explicit verified alias/mapping record:

`source_profile + source_digest -> target_profile + target_digest + mapping_proof`

Learning a new alias must not mutate an existing immutable object's identity.

## Algorithm agility

The registry must identify the digest algorithm explicitly. It must not silently infer an algorithm from digest length.

Different profiles may legitimately use BLAKE3-256, SHA-256, or an external-standard commitment. Algorithm migration should create a new profile/version or an explicit verified alias; it must not reinterpret historical digests.

## External standards

A Mycelix canonical profile must not override an external standard's own canonicalization or proof rules.

Examples:

- W3C Verifiable Credentials / Data Integrity proof suites follow their selected standards;
- SCITT/COSE receipts follow the relevant CBOR/COSE profile;
- OCI/Nix content identities follow their native definitions;
- Mycelix may bind or map those identities, but should not relabel a proprietary byte encoding as the external standard's canonical representation.

## Conformance gate

Before a profile is used as a production authorization boundary:

1. Rust golden vectors pass.
2. At least one independent implementation (TypeScript or Python) reproduces the exact bytes and digest.
3. Mutating every authority-bearing field changes the digest.
4. Field reordering/collection-order tests match the declared semantics.
5. Domain/profile substitution changes or invalidates the digest.
6. Oversized/invalid inputs fail deterministically.
7. The upgrade story is documented before v1 is frozen.

## Near-term registry entries

The first inventory should cover:

1. Accountability receipt pre-attestation commitment from PR #28.
2. Accountability purpose and policy commitments from PR #28.
3. Content Fabric `StableIdV1` and `PayloadCommitmentV1` from PR #35.
4. Governance execution authorization digest from PR #32.
5. Future institutional-core authority/rulebook/action profiles from PR #31/#36.
6. Multiparty workflow request/policy/credential profiles from PR #34.

## Decision rule for convergence

Only replace two profile formats with one shared canonical codec when all of the following hold:

- they commit the same kind of semantic structure;
- the new profile preserves existing threat-model properties;
- cross-language vectors exist;
- migration/alias semantics are explicit;
- downstream signature/proof systems can migrate without ambiguous historical verification.

Until then, a small registry plus verified mapping is safer than format capture.
