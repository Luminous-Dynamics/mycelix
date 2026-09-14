# Constitutional Trust-Root Identity v0.1

Status: **normative semantic identity profile**

GOVSYS-003A defines the language-neutral semantic identity of the constitutional trust-root commitment used to terminate institutional policy-currentness recursion. It is a direct child of GOVSYS-002 and deliberately introduces no runtime or authority surface.

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

A root identity states exactly what would be trusted **if** an independent bootstrap/provenance theorem establishes that root. It does not establish trust by itself. This preserves GOVSYS-002 PI-003: authority and evidence may not self-justify cyclically.

The network remains infrastructure for institutions. It is not the sovereign.

## Registered identifiers

Protocol version:

`mycelix-constitutional-trust-root-v0.1`

Identity profile:

`mycelix-constitutional-trust-root-v1-sha256-framed-semantic`

Unframed domain separator:

`mycelix/public-institution/constitutional-trust-root/v1`

Digest algorithm: SHA-256.

SHA-256 is used for this external constitutional/interoperability profile so an independent zero-dependency oracle can implement the profile. A later typed adapter may map the resulting raw 32 bytes into an internal digest type only after explicit ancestry convergence.

## Root semantic object

One root contains exactly:

1. `protocol_version`;
2. `institution_id`;
3. optional `jurisdiction_id`;
4. `constitutional_rulebook { id, version, digest_hex }`;
5. `generation`;
6. optional `predecessor_root_digest_hex`;
7. `bootstrap_mode`;
8. `bootstrap_profile`;
9. `authoritative_root_source_ref`;
10. `root_coverage_profile`;
11. canonical set `authorized_policy_scopes`;
12. `valid_from_ms`;
13. optional `expires_at_ms`;
14. `rotation_mode`; and
15. optional `rotation_profile`.

Unknown or omitted fields are invalid at the normative vector/oracle boundary.

## Authoritative root source is part of Root-A

Closed-world currentness cannot be derived from a locally supplied transition prefix. Therefore Root-A commits the exact logical source from which root-lineage coverage/current-head evidence must later be obtained:

`authoritative_root_source_ref`

It also commits the exact semantic profile used to interpret a root-source coverage/head receipt:

`root_coverage_profile`

These are constitutional semantics, not Root-D caller configuration.

```text
caller-selected root registry
!= authoritative constitutional root source
```

Root-B's external pin anchors these values indirectly by pinning the complete Root-A digest. Root-D must exact-match its coverage evidence to both values before any current-root claim is possible.

Changing either source or coverage profile changes the Root-A identity and therefore requires predecessor-authorized constitutional succession or a separately qualified recovery/migration theorem. A currentness provider cannot silently redirect the constitutional source.

## Authorized policy scopes are bound tuples

Each `authorized_policy_scopes` member contains exactly:

```text
(
  policy_identity_profile,
  policy_registry_namespace,
  provider_authority_institution_id,
  optional provider_authority_jurisdiction_id,
  provider_authority_rulebook { id, version, digest },
  required_provider_capability
)
```

A future provider-policy adapter must exact-match **one complete tuple**.

The root must never authorize independent profile, namespace, authority, or capability sets whose Cartesian product could be interpreted as authority. A profile from one scope cannot be combined with a namespace, provider authority, rulebook, or capability from another scope.

### One provider scope per `(profile, namespace)` in v0.1

Within one root, the pair:

```text
(policy_identity_profile, policy_registry_namespace)
```

MUST be unique.

Two distinct provider-authority tuples for the same pair are rejected even if both tuples are individually valid. This prevents caller choice, arrival order, or an implicit `OR` from deciding which provider owns one closed-world policy namespace.

Multi-provider quorum, threshold, federation, failover, or diversity semantics require a later explicit profile. They are not inferred by duplicating a v0.1 key.

### Exact namespace semantics only

Namespaces are exact UTF-8 values. v0.1 grants no prefix, wildcard, subtree, suffix, URI-normalization, or family semantics.

The profile likewise infers no capability inheritance, jurisdiction subsumption, institution hierarchy, role equivalence, or rulebook equivalence.

### Constitutional self-authorization is forbidden

No policy scope may use Root-A's own identity profile:

`mycelix-constitutional-trust-root-v1-sha256-framed-semantic`

as `policy_identity_profile`.

```text
constitutional root identity
    cannot authorize
ordinary policy-currentness machinery
    to declare constitutional root identity current
```

Root provenance/currentness belongs to GOVSYS-003B/003C/003D.

### Resource bound

A root may contain at most **1,024** authorized policy scopes. The empty set is valid and means the root intentionally authorizes no ordinary policy-currentness scope.

## Bootstrap modes

Exactly three v0.1 bootstrap modes are registered:

- `pinned-constitutional-commitment`;
- `genesis-governance-decision`; and
- `external-institutional-credential`.

`bootstrap_profile` names the exact verification/provisioning semantics expected by Root-B. Merely declaring a bootstrap mode does not verify it.

Root-B must obtain its expectation/evidence through a trust mechanism independent from the ordinary policy-currentness machinery being bootstrapped.

For generation greater than zero, normal succession does not interpret `bootstrap_mode`/`bootstrap_profile` as a fresh trust anchor. A later Root-C normal-rotation profile should require them to remain equal to the predecessor lineage unless a separately qualified recovery/reprovisioning theorem explicitly authorizes a new anchor.

## Generation and predecessor

Generation `0` MUST have no predecessor digest.

Every generation greater than `0` MUST carry one non-zero 32-byte predecessor root digest.

```text
generation link encoded
!= predecessor authority verified
```

Root-A commits the succession hook only. Root-C must prove predecessor authorization and rooted lineage; Root-D must separately prove which covered lineage endpoint is current.

## Rotation modes

Exactly two v0.1 modes are registered:

- `immutable` — `rotation_profile` MUST be absent;
- `predecessor-authorized` — `rotation_profile` MUST be present and valid.

A successor root's own rotation declaration never authorizes the transition into that successor. The predecessor's frozen transition theorem controls that transition.

## Validity interval

`valid_from_ms` is a `u64` semantic field.

`expires_at_ms` is optional. When present it MUST be a `u64` strictly greater than `valid_from_ms`.

Root-A has no ambient clock and establishes no live/current root claim.

## Deterministic text validity

Text is hashed as exact UTF-8 bytes. No Unicode normalization, case folding, locale transformation, trimming, URI rewriting, or alias resolution participates in identity.

To keep validation language-neutral, semantic text MUST:

- encode to at least one UTF-8 byte;
- fit the field byte bound;
- contain no ASCII control byte `0x00..0x1f` or `0x7f`; and
- not begin or end with ASCII space `0x20`.

Non-ASCII whitespace is not implicitly stripped. If supplied, its exact UTF-8 bytes are semantic.

Field bounds:

- ordinary institutional/capability IDs: 512 bytes;
- policy/bootstrap/rotation/coverage profiles: 256 bytes;
- policy-registry namespace or authoritative root-source reference: 1,024 bytes;
- rulebook version: 128 bytes.

A producer that needs stronger identifier syntax or normalization must apply the owning identifier profile before constructing Root-A.

## Digest representation

Every rulebook/predecessor digest is semantically a raw non-zero 32-byte value.

The JSON oracle/vector transport represents those bytes as exactly 64 hexadecimal characters. Hexadecimal letter case is representation-only: upper- and lower-case encodings of the same 32 bytes have identical Root-A identity.

The final Root-A SHA-256 digest is likewise a raw 32-byte identity; hexadecimal is only its test-vector representation.

## Framing

For bytes `x`:

```text
frame(x) = u64_le(len(x)) || x
```

For unsigned integer `n`:

```text
frame_u64(n) = frame(u64_le(n))
```

Optional text/digest/u64 values are encoded with a framed one-byte presence tag:

```text
None    = frame(0x00)
Some(x) = frame(0x01) || encoded(x)
```

A rulebook is:

```text
frame(rulebook_id)
|| frame(rulebook_version)
|| frame(raw_32_byte_rulebook_digest)
```

One policy scope is:

```text
frame(policy_identity_profile)
|| frame(policy_registry_namespace)
|| frame(provider_authority_institution_id)
|| optional_text(provider_authority_jurisdiction_id)
|| rulebook(provider_authority_rulebook)
|| frame(required_provider_capability)
```

`authorized_policy_scopes` is encoded as:

```text
frame_u64(scope_count)
|| encoded_scope_0
|| ...
|| encoded_scope_n
```

where complete encoded scopes are sorted lexicographically. Duplicate complete scopes are invalid. In addition, duplicate `(policy_identity_profile, policy_registry_namespace)` keys are invalid in v0.1.

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
|| frame(authoritative_root_source_ref)
|| frame(root_coverage_profile)
|| canonical_policy_scope_set(authorized_policy_scopes)
|| frame_u64(valid_from_ms)
|| optional_u64(expires_at_ms)
|| frame(rotation_mode)
|| optional_text(rotation_profile)
```

No JSON serialization, object/map iteration order, CBOR, MessagePack, Rust layout, Holochain serialization, or language-specific object representation participates in the digest.

## Normative golden vector

The checked-in v0.1 vector has canonical identity:

`fb91b8d0abca410985f88c1763ccd943036c105a6cd5cee75ba4e4c8565753dc`

The standard-library Python oracle independently recomputes the value and verifies the profile's adversarial corpus.

The vector wrapper itself has exactly three members:

- `profile`;
- `expected_digest_hex`; and
- `root`.

Extra wrapper metadata is rejected so a consumer cannot accidentally treat unauthenticated side metadata as part of the normative vector contract.

## Relationship to Root-D and qualified policy layers

Root-D must require coverage/head evidence to exact-match both `authoritative_root_source_ref` and `root_coverage_profile` from the current Root-A lineage. It must not accept a caller-selected root source.

Qualified #815 gives stable semantic identity to a procedure-policy currentness-provider selection policy. Qualified #812 gives a reusable record/adoption evidence waist. Neither is imported into this constitutional branch.

After explicit convergence, a downstream provider-policy adapter should require all of the following:

1. a Root-D-qualified current constitutional root;
2. provider-policy target institution/jurisdiction exactly matching the constitutional root scope;
3. the qualified provider policy exact-matching one complete authorized policy tuple; and
4. independently qualified #812 record/adoption evidence for the exact provider-policy identity.

```text
GOVSYS-003A root identity
        ↓
GOVSYS-003B independent genesis provenance
        ↓
GOVSYS-003C predecessor-authorized rooted lineage
        ↓
GOVSYS-003D exact-source covered current head
        ↓
exact authorized policy-scope tuple
+ qualified provider-policy identity
+ qualified record/adoption evidence
        ↓
root-authorized provider-policy currentness
        ↓
domain policy currentness
```

## Explicit nonclaims

GOVSYS-003A does **not** establish:

- root-key ownership or signature validity;
- bootstrap provenance;
- institutional/democratic adoption;
- legal validity or legitimacy;
- authoritative source availability/authenticity;
- source coverage or current head;
- root currentness or non-revocation;
- predecessor authorization;
- successful root rotation;
- policy-record authenticity;
- provider-policy currentness;
- administrative or review-policy currentness;
- administrative competence;
- judicial competence;
- execution authority; or
- external-effect authority.

The network remains infrastructure for institutions. It is not the sovereign.
