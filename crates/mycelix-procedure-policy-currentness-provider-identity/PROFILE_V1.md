# Canonical Procedure-Policy Currentness-Provider Identity Profile v1

Profile identifier:

`mycelix-procedure-policy-currentness-provider-v1-blake3-framed-semantic`

Domain separator:

`mycelix/administrative-procedure/currentness-provider-policy/v1`

This document is normative. Implementations in Rust or any other language must produce identical bytes and BLAKE3-256 output for identical semantic policy content.

## 1. Hash primitive

Use BLAKE3 with 256-bit output.

Initialize a fresh hasher and append the raw UTF-8 bytes of the domain separator **without framing**.

Every subsequent field is framed as:

```text
u64_le(byte_length) || raw_bytes
```

No Unicode normalization, JSON serialization, MessagePack encoding, locale-aware ordering, platform-native integer encoding, or implicit string terminator is permitted.

## 2. Primitive encodings

Strings are raw UTF-8 bytes.

`Digest32` values are the exact 32 raw digest bytes.

Semantic integer counts are encoded as unsigned 64-bit little-endian integers and then framed as an 8-byte field.

Optional identifier values are encoded as:

```text
None: frame([0x00])
Some(x): frame([0x01]) || frame(utf8(x))
```

## 3. Top-level field order

After the unframed domain separator, append these framed fields in exactly this order:

1. profile identifier;
2. `ProcedurePolicyCurrentnessPolicy.protocol_version`;
3. target institution ID;
4. optional target jurisdiction using §2;
5. target rulebook ID;
6. target rulebook version;
7. target rulebook raw 32-byte digest;
8. procedure profile ID;
9. provider namespace;
10. provider-authority institution ID;
11. optional provider-authority jurisdiction using §2;
12. provider-authority rulebook ID;
13. provider-authority rulebook version;
14. provider-authority rulebook raw 32-byte digest;
15. required provider capability ID;
16. framed `u64_le(number_of_canonical_roles)`;
17. each canonical role as one framed UTF-8 string;
18. framed `u64_le(number_of_canonical_evidence_requirements)`;
19. each canonical evidence-requirement blob as one framed byte string.

No policy locator, publication reference, currentness claim, provider principal, provider grant ID, provider generation, provider state digest, currentness proof reference, or runtime timestamp is included. Those are evidence/currentness facts, not verifier-selection policy semantics.

## 4. Accepted-provider-role canonicalization

`accepted_provider_roles` has set semantics.

Before hashing:

1. validate every role ID;
2. reject duplicate role IDs according to the owning policy validator;
3. sort role strings by lexicographic order of their raw UTF-8 bytes.

Hash only this canonical order.

Changing vector order alone must not change identity.

## 5. Evidence-requirement canonicalization

Each `EvidenceRequirement` is converted to an inner canonical byte blob.

For one requirement:

1. validate `evidence_type`;
2. validate every accepted issuer ID;
3. reject duplicate accepted issuer IDs;
4. sort accepted issuer strings by lexicographic raw UTF-8 bytes;
5. append to an initially empty byte vector:
   - `frame(utf8(evidence_type))`;
   - `frame(u64_le(number_of_accepted_issuers))`;
   - each issuer as `frame(utf8(issuer_id))` in canonical order.

After every requirement has been encoded:

1. sort requirement blobs lexicographically by their raw bytes;
2. reject adjacent equal blobs after sorting, because they are semantically duplicate requirements;
3. hash the canonical count and each entire requirement blob as specified in top-level fields 18–19.

An empty accepted-issuer set retains the inherited meaning “issuer not constrained by this requirement.” It is encoded with issuer count zero.

## 6. Identity-bearing semantics

The following changes must change identity:

- target institution or jurisdiction;
- target rulebook ID, version, or digest;
- procedure profile;
- provider namespace;
- provider-authority institution or jurisdiction;
- provider-authority rulebook ID, version, or digest;
- required provider capability;
- accepted provider role set; or
- provider-authority evidence-requirement semantics.

The following changes alone must **not** change identity:

- accepted-role vector ordering;
- evidence-requirement vector ordering; or
- accepted-issuer ordering inside an evidence requirement.

## 7. Semantic and trust boundaries

This digest answers only:

> What exact policy selects the authority allowed to attest procedure-policy currentness?

It does not answer:

- whether an immutable record containing this policy is authentic;
- whether an institution adopted it;
- whether it remains current rather than revoked/superseded;
- whether a provider claim is authentic/current;
- which procedure policy is current; or
- whether any actor has administrative/effect authority.

Those require independent proof domains.

## 8. Versioning rule

Any change to:

- included fields;
- field order;
- framing;
- optional encoding;
- integer encoding;
- set canonicalization;
- duplicate policy;
- string-byte semantics;
- hash primitive; or
- domain separator

requires a new profile identifier and domain separator. Historical v1 identities must never be reinterpreted under new encoding rules.
