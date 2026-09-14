# Constitutional Trust-Root Identity v0.1

Status: **normative semantic identity profile**

GOVSYS-003A defines the first non-recursive building block beneath institutional policy currentness: a canonical identity for the exact constitutional trust-root commitment an institution intends to provision.

This is a direct child of GOVSYS-002. It deliberately remains language-neutral and runtime-free so the constitutional lineage does not import later authority/runtime ancestry implicitly.

## Governing theorem

```text
constitutional trust-root semantics
        -> canonical root identity
```

and explicitly:

```text
canonical root identity
!= root provenance
!= institutional adoption
!= current root
!= legal legitimacy
!= provider-policy currentness
!= domain-policy currentness
!= actor authority
!= external-effect authority
```

A root identity says exactly **what would be trusted if a separately qualified bootstrap/provenance theorem establishes that root**. It does not establish that trust by itself.

This preserves GOVSYS-002 PI-003: authority and evidence may not self-justify cyclically.

## Profile identifiers

Protocol version:

`mycelix-constitutional-trust-root-v0.1`

Identity profile:

`mycelix-constitutional-trust-root-v1-sha256-framed-semantic`

Unframed domain separator bytes:

`mycelix/public-institution/constitutional-trust-root/v1`

Digest algorithm: SHA-256.

SHA-256 is used here because GOVSYS-003A is an external constitutional/interoperability profile with a zero-dependency independent oracle. The resulting 32-byte identity can later be adapted to internal `Digest32` representations after explicit ancestry convergence.

## Semantic commitment

One root commitment contains, in fixed semantic order:

1. exact protocol version;
2. exact institution identifier;
3. optional exact jurisdiction identifier;
4. exact constitutional/root rulebook identifier, version, and 32-byte content digest;
5. root generation;
6. optional predecessor root digest;
7. exact bootstrap mode;
8. exact bootstrap verification/provisioning profile;
9. a canonical set of policy semantic-identity profiles the root may authorize;
10. a canonical set of exact policy-registry namespaces the root may authorize;
11. exact provider-authority institution;
12. optional exact provider-authority jurisdiction;
13. exact provider-authority rulebook identifier, version, and digest;
14. a canonical set of provider capabilities the root may authorize;
15. root validity start;
16. optional root expiry;
17. exact rotation mode; and
18. optional exact rotation profile.

This profile intentionally authorizes **exact namespaces**, not string prefixes or wildcard patterns. Namespace-family semantics require a later explicit profile rather than accidental prefix authority.

## Bootstrap modes

Exactly three v0.1 bootstrap-mode strings are registered:

- `pinned-constitutional-commitment`
- `genesis-governance-decision`
- `external-institutional-credential`

The accompanying `bootstrap_profile` identifies the exact external verification/provisioning semantics expected by the later provenance theorem.

Changing bootstrap mode or bootstrap profile changes the canonical root identity.

GOVSYS-003A does not verify any of these modes. In particular, a caller cannot make a root trusted merely by setting `bootstrap_mode = pinned-constitutional-commitment`.

## Generation and predecessor semantics

Generation `0` is a genesis/root commitment and MUST have no predecessor digest.

Every generation greater than `0` MUST carry one non-zero 32-byte predecessor root digest.

This establishes an unambiguous semantic hook for a later succession theorem, but GOVSYS-003A does not prove that the predecessor was current, authentic, or authorized to rotate.

```text
generation link encoded
!= predecessor authority verified
```

## Rotation modes

Exactly two v0.1 rotation-mode strings are registered:

- `immutable`
- `predecessor-authorized`

`immutable` MUST have no rotation profile.

`predecessor-authorized` MUST carry a non-empty rotation profile naming the exact future transition semantics.

A successor root's own rotation mode does not authorize the transition into that successor. Root-C must evaluate the predecessor root's frozen rotation rules.

## Validity semantics

`valid_from_ms` is part of root semantics.

`expires_at_ms` is optional. When present it MUST be strictly greater than `valid_from_ms`.

GOVSYS-003A has no ambient clock and does not prove that the root is currently live. It only commits the intended validity interval.

## Canonical sets

The following are semantic sets:

- authorized policy identity profiles;
- authorized policy-registry namespaces; and
- allowed provider capabilities.

Each member is encoded as raw UTF-8 bytes after validation. Duplicate members are invalid. Members are sorted by raw UTF-8 byte sequence before hashing. Input order is therefore non-semantic.

Empty sets are permitted. This allows a constitutional root to intentionally authorize no policy profiles, namespaces, or provider capabilities without inventing a separate shutdown semantic.

## String validity

Strings are hashed as their exact UTF-8 bytes. No Unicode normalization, case folding, trimming, URI rewriting, identifier aliasing, or locale transformation is performed.

Semantic strings MUST:

- be non-empty after trimming for validation purposes;
- contain no ASCII control characters;
- fit their profile-defined byte bound; and
- otherwise retain their exact supplied bytes.

A producer that wants normalized institutional identifiers must do so in the owning identifier profile before constructing this root commitment.

## Framing primitive

For byte string `x`:

```text
frame(x) = u64_le(len(x)) || x
```

Unsigned integers are encoded as 8 little-endian bytes and then framed:

```text
frame_u64(n) = frame(u64_le(n))
```

Optional text/digest/u64 values use a framed one-byte presence tag:

```text
None    = frame(0x00)
Some(x) = frame(0x01) || encoded(x)
```

where `encoded(x)` is `frame(x)` for text/digest and `frame_u64(x)` for u64.

A rulebook is encoded as:

```text
frame(rulebook_id)
|| frame(rulebook_version)
|| frame(raw_32_byte_rulebook_digest)
```

A canonical string set is encoded as:

```text
frame_u64(member_count)
|| frame(member_0)
|| ...
|| frame(member_n)
```

with members sorted by raw UTF-8 bytes.

## Canonical byte sequence

The SHA-256 input is exactly:

```text
DOMAIN_UNFRAMED
|| frame(IDENTITY_PROFILE)
|| frame(protocol_version)
|| frame(institution_id)
|| optional_text(jurisdiction_id)
|| rulebook(constitutional_rulebook)
|| frame_u64(generation)
|| optional_digest(predecessor_root_digest)
|| frame(bootstrap_mode)
|| frame(bootstrap_profile)
|| canonical_set(authorized_policy_profiles)
|| canonical_set(authorized_policy_registry_namespaces)
|| frame(provider_authority_institution_id)
|| optional_text(provider_authority_jurisdiction_id)
|| rulebook(provider_authority_rulebook)
|| canonical_set(allowed_provider_capabilities)
|| frame_u64(valid_from_ms)
|| optional_u64(expires_at_ms)
|| frame(rotation_mode)
|| optional_text(rotation_profile)
```

No JSON, CBOR, MessagePack, Rust struct layout, Holochain encoding, map iteration order, or language-specific serialization participates in the digest.

## Golden vector

The checked-in GOVSYS-003A vector is normative for v0.1 and has expected digest:

`bb622a0322a30c314db6fe545c3b4cd933c6d9c3159f8b36d5e5a79a2e08f5c2`

The independent Python oracle recomputes this value using only the Python standard library.

## Relationship to qualified policy layers

Qualified #815 gives a stable semantic identity to one procedure-policy currentness-provider selection policy.

Qualified #812 gives a reusable evidence waist for immutable policy-record verification plus institutional adoption evidence.

Neither artifact is imported into this constitutional branch. After explicit ancestry convergence, a later adapter may require that a provider-policy identity/profile and namespace are members of this exact root commitment and then consume independently qualified #812 evidence.

The intended future composition is:

```text
GOVSYS-003A canonical root identity
        ↓
GOVSYS-003B independently qualified bootstrap/provenance
        ↓
GOVSYS-003C predecessor-authorized succession/currentness
        ↓
root-authorized provider-policy currentness
        ↓
qualified provider-selection policy
        ↓
domain policy currentness
```

## Explicit nonclaims

GOVSYS-003A does **not** establish:

- root-key ownership or signature validity;
- bootstrap provenance;
- institutional or democratic adoption;
- legal validity or legitimacy;
- currentness or non-revocation;
- predecessor authorization;
- successful root rotation;
- policy-record authenticity;
- provider-policy currentness;
- procedure/review policy currentness;
- administrative competence;
- judicial competence;
- execution authority; or
- external-effect authority.

The network remains infrastructure for institutions. It is not the sovereign.
