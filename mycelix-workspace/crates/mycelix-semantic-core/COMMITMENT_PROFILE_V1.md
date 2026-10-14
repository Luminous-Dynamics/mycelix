# MYC-SEM-001C-r2 — Semantic Commitment Profile v1

Status: repaired source specification for the first language-neutral Mycelix semantic envelope commitment profile.

This profile identifies semantic environments and semantic subjects. It does not establish truth, currentness, trust, authority, interoperability, or semantic equivalence.

## Repair note

The first MYC-SEM-001C candidate published incorrect golden-vector constants even though its Rust canonicalization procedure and written byte formula agreed. Independent reconstruction of the written formula exposed the mismatch before qualification completed. This r2 subject is rebuilt as a fresh direct child of MYC-SEM-001B; it does not inherit the invalid 001C candidate.

## Profile identity

```text
id       = mycelix-semantic-commitment/sha256-length-prefixed-be-v1
revision = 1
hash     = SHA-256
```

All unsigned integers are big-endian.

Text is:

```text
u32(byte_length) || UTF-8 bytes
```

`BoundedSemanticTextV1` limits semantic text to 256 UTF-8 bytes before canonicalization.

A profile reference is:

```text
text(profile_id)
|| u64(profile_revision)
|| digest[32]
```

The 32-byte profile digest is opaque to this profile.

## Environment commitment

```text
SHA256(
  "MYCELIX_SEMANTIC_ENVIRONMENT_V1\0"
  || u16(outer_commitment_profile_revision = 1)
  || profile(schema)
  || profile(interpretation)
  || profile(identity)
  || profile(authority)
  || profile(temporal)
  || profile(domain_canonicalization)
)
```

`domain_canonicalization` is a semantic profile *inside* the environment and is intentionally distinct from this outer commitment profile.

## Environment reference

An exact environment reference carries:

```text
u16(outer_commitment_profile_revision)
|| environment_commitment[32]
```

A reference is identity material only.

```text
environment reference exists
!= environment trusted
!= environment current
!= environment compatible with another environment
```

## Subject commitment

A semantic subject is identified under one exact environment:

```text
SHA256(
  "MYCELIX_SEMANTIC_SUBJECT_V1\0"
  || u16(outer_commitment_profile_revision = 1)
  || u16(environment_reference_profile_revision)
  || environment_commitment[32]
  || text(domain)
  || profile(subject_schema)
  || text(subject_id)
)
```

Therefore identical textual coordinates under a different semantic environment are a different semantic subject unless a later qualified migration/translation theorem establishes a relationship.

## Frozen independently reconstructed vectors

Test environment:

```text
schema                  schema/base                 rev 1 digest = 01 repeated 32 bytes
interpretation          interpretation/base         rev 2 digest = 02 repeated 32 bytes
identity                identity/base               rev 3 digest = 03 repeated 32 bytes
authority               authority/base              rev 4 digest = 04 repeated 32 bytes
temporal                temporal/base               rev 5 digest = 05 repeated 32 bytes
domain canonicalization canonical/domain-v1         rev 1 digest = 06 repeated 32 bytes
```

The canonical environment preimage is exactly 387 bytes.

Environment commitment:

```text
283f04d533916528a7054f9afe8958526cd3fdd94e29d9e990828af18dd12343
```

Subject:

```text
domain     = personal
schema     = schema/base rev 1 digest = 01 repeated 32 bytes
subject_id = did:mycelix:test/profile
```

The canonical subject preimage is exactly 159 bytes.

Subject commitment:

```text
5a88314b454bf23478c91af6adeff8a03bef0b8d4bd2f0c031b27321a50eb323
```

## Nonclaims

A correct commitment proves only deterministic binding to the canonical bytes. It does not prove profile authenticity, issuer trust, currentness, subject existence, global uniqueness, identity control, authority, cross-environment equivalence, or valid migration/translation.
