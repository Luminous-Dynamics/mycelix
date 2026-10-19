# Content Fabric v1 — ID Canonicalization

Status: frozen contract for CF-01 and later Content Fabric implementations.

## Purpose

Content Fabric uses two different kinds of cryptographic identity and they MUST remain distinct:

1. **Content digests** identify immutable bytes and are algorithm-tagged (`BLAKE3-256`, `SHA-256`, NAR hashes, etc.).
2. **Infrastructure record IDs** identify versioned Capability → Offering → Lease → Receipt records.

This document freezes the v1 rules for the second category.

## CF-ID-01 — Domain separation is mandatory

Every infrastructure ID MUST bind a short ASCII domain token before any record fields. v1 domains are:

- `capability`
- `offering`
- `lease`
- `receipt`

The same canonical fields under different domains MUST produce different IDs.

## CF-ID-02 — Schema version is part of identity

The infrastructure schema version MUST be included in the hash preimage. A future v2 record MUST NOT be reinterpreted as a v1 record with additional optional fields.

## CF-ID-03 — Fields are explicitly framed

Variable-length fields MUST be length-prefixed before hashing. Concatenating variable-length byte strings without framing is forbidden.

Implementations MUST agree on:

- byte order for integers (v1: big endian),
- field ordering,
- field-count encoding,
- variable-field length encoding,
- domain token,
- schema version.

## CF-ID-04 — Generic payloads enter IDs through commitments

Infrastructure envelopes MUST NOT hash arbitrary language-native or Serde/JSON serialization directly.

Instead, the domain owning a payload supplies:

- a canonical schema tag, and
- canonical payload bytes.

CF-01 hashes those into a `PayloadCommitmentV1`. Envelope IDs bind the schema tag and commitment digest, not an incidental serializer representation.

This keeps the infrastructure envelope reusable by Rust, Holochain, CLIs, services, and future non-Rust clients without making Rust struct layout or JSON map ordering part of consensus.

## CF-ID-05 — Canonical tokens are constrained

Schema tags, resource dimensions, and infrastructure ID domains use lowercase namespaced ASCII tokens. v1 tokens:

- start with `a-z`,
- are at most 64 bytes,
- may contain lowercase letters, digits, `.`, `_`, `/`, `-`, and `:`.

Examples:

- `storage/bytes`
- `bandwidth/bytes`
- `storage/capability-v1`
- `compute/gpu-millis`

## CF-ID-06 — Resource vectors are deterministically ordered

Resource vectors MUST have deterministic key order before hashing. v1 uses lexicographic ordering of canonical resource tokens.

Duplicate dimensions, zero amounts, empty vectors, and invalid tokens MUST be rejected.

A child allocation MUST be a component-wise subset of its parent allocation.

## CF-ID-07 — Parent identity is transitive evidence

An Offering ID binds its Capability ID.

A Lease ID binds its Offering ID.

A Receipt ID binds its Lease ID.

This does not make the parent authoritative by itself; it makes the lineage tamper-evident.

## CF-ID-08 — Stored IDs are always recomputable

Deserialization MUST NOT make an ID trustworthy merely because the wire record contains one.

Validators MUST recompute identity-bearing fields and reject a stored ID that differs from the canonical recomputation.

The same applies after mutation in memory: changing an identity-bearing field invalidates the previous ID.

## CF-ID-09 — Version upgrades never reinterpret old preimages

If these canonicalization rules change, introduce a new schema version and new preimage rules. Do not silently change v1 hashing behavior.

Published test vectors therefore become part of the compatibility surface.

## CF-ID-10 — Content identity remains algorithm-agile

The use of BLAKE3 for v1 infrastructure record IDs does **not** collapse Content Fabric content identity into BLAKE3.

Immutable content continues to use algorithm-tagged digests so OCI SHA-256, Iroh/BLAKE3, NAR semantic verification, and future algorithms can coexist without pretending to be the same identifier.

## v1 preimage shape

Conceptually:

```text
magic
|| framed(domain)
|| schema_version_be
|| field_count_be
|| framed(field_0)
|| framed(field_1)
|| ...
```

Exact executable behavior and compatibility fixtures live in `mycelix-infrastructure-types` starting with CF-01.

## Non-goals

This contract does not define:

- content blob digest formats,
- Holochain entry hashes,
- payment/settlement identifiers,
- Nix store-path identity,
- OCI digest syntax,
- transport chunk identity.

Those systems retain their own semantics and are bridged explicitly rather than conflated.