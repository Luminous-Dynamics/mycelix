# MYC-SEM-001C — Semantic Commitment Profile v1

Status: source specification for the first language-neutral Mycelix semantic envelope commitment profile.

This profile identifies semantic environments and semantic subjects. It does not establish truth, currentness, trust, authority, interoperability, or semantic equivalence.

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

`domain_canonicalization` is a semantic profile *inside* the environment.

It is intentionally distinct from this outer commitment profile:

```text
domain canonicalization semantics
!=
outer Mycelix environment commitment procedure
```

This prevents an environment from choosing an arbitrary profile label and thereby relabeling the fixed envelope hash procedure.

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

Therefore:

```text
same domain + same schema + same subject_id
under environment A
!=
same semantic subject under environment B
```

unless a later qualified migration/translation theorem establishes a relationship.

## Frozen independently reconstructed vectors

The vectors below were reconstructed from this byte specification independently of the Rust implementation.

Test environment:

```text
schema                  schema/base                 rev 1 digest = 01 repeated 32 bytes
interpretation          interpretation/base         rev 2 digest = 02 repeated 32 bytes
identity                identity/base               rev 3 digest = 03 repeated 32 bytes
authority               authority/base              rev 4 digest = 04 repeated 32 bytes
temporal                temporal/base               rev 5 digest = 05 repeated 32 bytes
domain canonicalization canonical/domain-v1         rev 1 digest = 06 repeated 32 bytes
```

Environment commitment:

```text
72cc14b9f38fc9cb917da7e98d777443d980ad28ca12231cfec0c92919d09d03
```

Subject:

```text
domain     = personal
schema     = schema/base rev 1
subject_id = did:mycelix:test/profile
```

Subject commitment:

```text
d78cf4c70f6dffef2285188f98a556c232c67f689300a0acd3d8df21a8c647bf
```

## Nonclaims

A correct commitment proves only deterministic binding to the canonical bytes.

It does not prove:

- the referenced profile is authentic;
- the profile issuer is trusted;
- the environment is current;
- the subject exists in the world;
- the subject identifier is globally unique;
- an identity controls the subject;
- anyone has authority over the subject;
- another environment is equivalent;
- translation or migration is valid.
