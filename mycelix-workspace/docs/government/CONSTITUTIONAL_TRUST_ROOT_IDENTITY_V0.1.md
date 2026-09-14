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
9. a canonical set of exact **authorized policy scopes**;
10. root validity start;
11. optional root expiry;
12. exact rotation mode; and
13. optional exact rotation profile.

Each authorized policy scope is one indivisible tuple:

```text
(
  policy_identity_profile,
  exact_policy_registry_namespace,
  provider_authority_institution,
  optional_provider_authority_jurisdiction,
  exact_provider_authority_rulebook,
  required_provider_capability
)
```

A later provider-policy adapter must exact-match **one whole tuple**. It may not independently choose a profile from one scope, a namespace from another, and a capability or provider authority from a third.

This prevents a constitutional confused-deputy / Cartesian-product failure mode.

```text
{profile A, profile B}
+ {namespace X, namespace Y}
+ {capability P, capability Q}

MUST NOT imply

all eight profile × namespace × capability combinations
```

Only the exact checked-in scope tuples are authorized by Root-A semantics.

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

## Canonical authorized policy-scope set

`authorized_policy_scopes` is a semantic set of bound authorization tuples.

Every scope contains exactly:

- `policy_identity_profile`;
- `policy_registry_namespace`;
- `provider_authority_institution_id`;
- optional `provider_authority_jurisdiction_id`;
- `provider_authority_rulebook { id, version, digest_hex }`; and
- `required_provider_capability`.

Each scope is independently validated and encoded. Duplicate encoded scopes are invalid. Encoded scope byte strings are sorted lexicographically before hashing. Input scope order is therefore non-semantic.

The scope set may be empty. An empty scope set means the constitutional root intentionally authorizes no policy-currentness provider scope; it does not require a separate shutdown flag.

The v0.1 profile does not infer capability inheritance, namespace prefixing, institution hierarchy, jurisdiction subsumption, role equivalence, or rulebook equivalence. Any such semantic expansion requires another explicitly qualified profile.

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

One authorized policy scope is encoded as:

```text
frame(policy_identity_profile)
|| frame(policy_registry_namespace)
|| frame(provider_authority_institution_id)
|| optional_text(provider_authority_jurisdiction_id)
|| rulebook(provider_authority_rulebook)
|| frame(required_provider_capability)
```

The canonical scope set is encoded as:

```text
frame_u64(scope_count)
|| encoded_scope_0
|| ...
|| encoded_scope_n
```

where the complete encoded scope byte strings are sorted lexicographically. Duplicate complete encoded scopes are invalid.

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
|| canonical_policy_scope_set(authorized_policy_scopes)
|| frame_u64(valid_from_ms)
|| optional_u64(expires_at_ms)
|| frame(rotation_mode)
|| optional_text(rotation_profile)
```

No JSON, CBOR, MessagePack, Rust struct layout, Holochain encoding, map iteration order, or language-specific serialization participates in the digest.

## Golden vector

The checked-in GOVSYS-003A vector is normative for v0.1 and has expected digest:

`9c5d1a27ccb89e6bdea6788c21f811a956be7b631765539640c79f44a5a4daf7`

The independent Python oracle recomputes this value using only the Python standard library.

The vector contains two separate policy scopes using the same city-clerk authority rulebook but different exact `(profile, namespace, capability)` bindings. Reordering those scopes is non-semantic; swapping capabilities between them changes the root identity.

## Relationship to qualified policy layers

Qualified #815 gives a stable semantic identity to one procedure-policy currentness-provider selection policy.

Qualified #812 gives a reusable evidence waist for immutable policy-record verification plus institutional adoption evidence.

Neither artifact is imported into this constitutional branch. After explicit ancestry convergence, a later adapter may require that one qualified provider policy exact-match one complete `authorized_policy_scopes` tuple and then consume independently qualified #812 evidence.

For a procedure-policy provider, the adapter should separately require the provider-policy target institution/jurisdiction to match the constitutional root institution/jurisdiction. Root-A v0.1 does not turn one provider scope into cross-institution authority.

The intended future composition is:

```text
GOVSYS-003A canonical root identity
        ↓
GOVSYS-003B independently qualified bootstrap/provenance
        ↓
GOVSYS-003C predecessor-authorized succession/currentness
        ↓
exact authorized-policy-scope tuple match
        + qualified provider-policy identity
        + qualified policy record/adoption evidence
        ↓
root-authorized provider-policy currentness
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
