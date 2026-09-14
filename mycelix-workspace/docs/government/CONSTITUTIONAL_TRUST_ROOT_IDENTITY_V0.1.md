# Constitutional Trust-Root Identity v0.1

Status: **normative semantic identity profile**

GOVSYS-003A defines the language-neutral semantic identity of the constitutional trust-root commitment used to terminate institutional policy-currentness recursion. It is a direct child of GOVSYS-002 and deliberately introduces no runtime or consequential authority surface.

## Governing theorem

```text
constitutional trust-root semantics
        -> canonical Root-A identity
```

and explicitly:

```text
canonical Root-A identity
!= bootstrap provenance
!= transition authorization
!= authoritative source coverage
!= current root
!= legal legitimacy
!= provider-policy currentness
!= domain-policy currentness
!= actor authority
!= external-effect authority
```

A Root-A identity states exactly what would be trusted **if** an independent bootstrap/provenance theorem establishes that root. It does not establish trust by itself. This preserves GOVSYS-002 PI-003: authority and evidence may not self-justify cyclically.

The network remains infrastructure for institutions. It is not the sovereign.

## Registered Root-A identity

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
11. `root_source_verification_profile`;
12. `root_source_anchor_digest_hex`;
13. canonical set `authorized_policy_scopes`;
14. `valid_from_ms`;
15. optional `expires_at_ms`;
16. `rotation_mode`;
17. optional `rotation_profile`; and
18. optional `rotation_authority_anchor_digest_hex`.

Unknown or omitted fields are invalid at the normative vector/oracle boundary.

## Separate constitutional trust roles

Root-A deliberately carries **two different verifier-anchor commitments**. They represent different powers and MUST NOT be silently treated as interchangeable.

```text
root_source_anchor_digest
        -> authenticate closed-world constitutional-root source coverage

rotation_authority_anchor_digest
        -> authenticate predecessor-authorized constitutional-root transitions
```

Therefore:

```text
source-verification authority
!= constitutional rotation authority
```

Possession, validity, currentness, or compromise of one anchor says nothing about the other unless a later explicitly named profile intentionally cross-authorizes them.

v0.1 defines no such cross-authorization.

## Constitutional root-source trust descriptor

Closed-world currentness cannot be derived from a locally supplied transition prefix. Root-A therefore commits the complete trust descriptor Root-D must later use:

```text
(
  authoritative_root_source_ref,
  root_coverage_profile,
  root_source_verification_profile,
  root_source_anchor_digest
)
```

`authoritative_root_source_ref` names the exact logical source whose covered effective head may define constitutional currentness.

`root_coverage_profile` names the exact semantic contract for proving source coverage/head completeness.

`root_source_verification_profile` names the exact verification semantics used to authenticate a source snapshot or coverage receipt.

`root_source_anchor_digest` is a non-zero raw 32-byte commitment to the exact verification anchor required by that profile: for example a pinned verification-key commitment, threshold-policy commitment, trust-bundle commitment, or another profile-defined verification configuration.

The verification profile owns interpretation of those anchor bytes.

```text
right source name
+ caller-selected verifier/key/config
!= qualified constitutional root coverage
```

Root-B's external pin anchors all four values indirectly by pinning the complete Root-A digest. Root-D MUST exact-match independently verified coverage to this descriptor before any current-root claim is possible.

### Derived source-descriptor identity

For domain-neutral lineage/current-head composition, the four source fields have a separately registered derived identity.

Profile:

`mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic`

Unframed domain separator:

`mycelix/public-institution/constitutional-root-source-descriptor/v1`

Transcript:

```text
SOURCE_DESCRIPTOR_DOMAIN_UNFRAMED
|| frame(SOURCE_DESCRIPTOR_PROFILE)
|| frame(authoritative_root_source_ref)
|| frame(root_coverage_profile)
|| frame(root_source_verification_profile)
|| frame(raw_32_byte_root_source_anchor_digest)
```

For the normative v0.1 fixture:

`f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126`

This is a projection over fields already committed by Root-A. It is not inserted back into the Root-A transcript and creates no recursive commitment.

Its exact subset semantics are:

- changing source ref changes the descriptor;
- changing coverage profile changes the descriptor;
- changing source-verification profile changes the descriptor;
- changing source-anchor bytes changes the descriptor;
- hexadecimal case of identical source-anchor bytes does not change the descriptor;
- changing the rotation-authority anchor does **not** change the source descriptor; and
- changing unrelated constitutional/policy fields does not change the source descriptor.

This gives CORE-LINEAGE-001 / #837 one opaque profiled adapter key without teaching the generic kernel civic source/verifier semantics.

```text
source-descriptor identity
!= complete Root-A identity
!= source authenticity
!= source coverage
!= currentness
```

## Constitutional rotation-authority descriptor

A root that permits a successor must also commit the exact verifier/key/quorum configuration authorized to approve that successor.

For `rotation_mode = predecessor-authorized`, Root-A commits:

```text
(
  rotation_profile,
  rotation_authority_anchor_digest
)
```

`rotation_profile` names the exact transition-authorization semantics.

`rotation_authority_anchor_digest` is a non-zero raw 32-byte commitment interpreted only by that profile. It may commit an exact signer key, threshold keyset, quorum policy, ceremony configuration, or another profile-defined authorization configuration.

The predecessor's pair controls the transition **out of that predecessor and into its successor**.

```text
predecessor rotation profile + predecessor rotation authority anchor
        -> verifier contract for P -> S

successor rotation declaration
        -> only future S -> next transitions
```

A successor never authorizes its own admission.

### Derived rotation-authority identity

Root-C may consume one opaque identity over the predecessor-owned transition verifier configuration.

Profile:

`mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic`

Unframed domain separator:

`mycelix/public-institution/constitutional-root-rotation-authority/v1`

For a predecessor-authorized root, transcript:

```text
ROTATION_AUTHORITY_DOMAIN_UNFRAMED
|| frame(ROTATION_AUTHORITY_PROFILE)
|| frame(rotation_profile)
|| frame(raw_32_byte_rotation_authority_anchor_digest)
```

For the normative v0.1 fixture:

`cf86718a53410b1e5e38cf5c552b3e4448772da97533fbd08c7b9bbfbee909ae`

For an immutable root there is no positive rotation-authority descriptor because the root cannot authorize a normal successor.

Like the source descriptor, this is a projection over already-committed Root-A fields and is not inserted back into the Root-A transcript.

The oracle freezes role separation:

- changing the rotation profile or rotation-authority anchor changes the rotation-authority identity;
- changing only the source-verification anchor does not change the rotation-authority identity;
- changing only the rotation-authority anchor does not change the source descriptor; and
- hex case is representation-only for identical raw anchor bytes.

```text
rotation-authority identity
!= transition signature/proof validity
!= transition authorization by itself
!= currentness
```

A Root-C verifier still has to establish verifier origin and execute the exact rotation profile against the exact predecessor/successor transition statement.

## Rotation modes

Exactly two v0.1 rotation modes are registered.

### `immutable`

An immutable root MUST have:

```text
rotation_profile = None
rotation_authority_anchor_digest = None
```

Either field being present is invalid.

### `predecessor-authorized`

A predecessor-authorized root MUST have:

```text
rotation_profile = non-empty registered profile
rotation_authority_anchor_digest = non-zero 32-byte commitment
```

Missing either field is invalid.

The current fixture uses:

`rotation_profile = constitutional-root-rotation-v1`

That profile will be given executable transition semantics in Root-C. Merely naming the profile or carrying its anchor does not verify a transition.

## Normal rotation v1 direction

The intended `constitutional-root-rotation-v1` rule preserves, unless a separately qualified migration/recovery theorem exists:

- institution ID;
- jurisdiction ID;
- Root-A identity profile/protocol family;
- bootstrap mode/profile;
- authoritative root source ref;
- root coverage profile; and
- root source verification profile.

A predecessor-authorized successor may change, when the exact transition verifier binds the full successor Root-A digest:

- constitutional rulebook identity/version/content;
- ordinary authorized policy scopes;
- successor expiry;
- successor rotation mode/profile/rotation-authority anchor; and
- source-verification anchor.

Allowing source-anchor rotation is intentional key/config rotation. It changes the derived source descriptor and becomes effective only with the successor. It does not imply that the source-verification anchor authorized the transition.

Likewise, changing the successor rotation-authority anchor changes who may authorize the **next** transition, not who authorized the current one.

## Authorized ordinary policy scopes are bound tuples

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

The root must never authorize independent profile, namespace, authority, rulebook, or capability sets whose Cartesian product could be interpreted as authority.

### One provider scope per `(profile, namespace)` in v0.1

Within one root, `(policy_identity_profile, policy_registry_namespace)` MUST be unique.

Two different provider-authority tuples for the same key are rejected. Multi-provider quorum, threshold, federation, failover, or diversity semantics require a later explicit profile.

### Exact namespace semantics only

Namespaces are exact UTF-8 values. v0.1 grants no prefix, wildcard, subtree, suffix, URI-normalization, or family semantics.

The profile likewise infers no capability inheritance, jurisdiction subsumption, institution hierarchy, role equivalence, or rulebook equivalence.

### Constitutional self-authorization is forbidden

No ordinary policy scope may use Root-A's own identity profile as `policy_identity_profile`.

```text
constitutional root identity
    cannot authorize
ordinary policy-currentness machinery
    to declare constitutional root identity current
```

Root provenance/currentness belongs only to GOVSYS-003B/C/D.

### Resource bound

A root may contain at most **1,024** authorized policy scopes. The empty set is valid and means the root intentionally authorizes no ordinary policy-currentness scope.

## Bootstrap modes

Exactly three v0.1 bootstrap modes are registered:

- `pinned-constitutional-commitment`;
- `genesis-governance-decision`; and
- `external-institutional-credential`.

`bootstrap_profile` names the exact verification/provisioning semantics expected by Root-B. Merely declaring a mode does not verify it.

Root-B must obtain its expectation/evidence through a trust mechanism independent from ordinary policy-currentness machinery.

For generation greater than zero, normal succession does not interpret `bootstrap_mode`/`bootstrap_profile` as a fresh trust anchor. They remain predecessor-lineage semantics unless a separately qualified recovery/reprovisioning theorem explicitly authorizes a new bootstrap assumption.

## Generation and predecessor

Generation `0` MUST have no predecessor digest.

Every generation greater than `0` MUST carry one non-zero 32-byte predecessor Root-A digest.

```text
generation link encoded
!= predecessor authority verified
```

Root-A commits the succession hook only. Root-C must prove predecessor authorization and rooted lineage; Root-D must separately prove which covered lineage endpoint is current.

## Validity interval

`valid_from_ms` is a `u64` semantic field.

`expires_at_ms` is optional. When present it MUST be a `u64` strictly greater than `valid_from_ms`.

Root-A has no ambient clock and establishes no live/current root claim.

## Deterministic text validity

Text is hashed as exact UTF-8 bytes. No Unicode normalization, case folding, locale transformation, trimming, URI rewriting, or alias resolution participates in identity.

Semantic text MUST:

- encode to at least one UTF-8 byte;
- fit the field byte bound;
- contain no ASCII control byte `0x00..0x1f` or `0x7f`; and
- not begin or end with ASCII space `0x20`.

Non-ASCII whitespace is not implicitly stripped. If supplied, its exact UTF-8 bytes are semantic.

Field bounds:

- ordinary institutional/capability IDs: 512 bytes;
- policy/bootstrap/rotation/coverage/verification profiles: 256 bytes;
- policy-registry namespace or authoritative root-source reference: 1,024 bytes;
- rulebook version: 128 bytes.

A producer that needs stronger identifier syntax or normalization must apply the owning identifier profile before constructing Root-A.

## Digest representation

Every rulebook, predecessor, source-verification anchor, and rotation-authority anchor digest is semantically a raw non-zero 32-byte value whenever present.

The JSON oracle/vector transport represents these bytes as exactly 64 hexadecimal characters. Hexadecimal letter case is representation-only.

The final Root-A SHA-256 digest and both derived descriptor digests are likewise raw 32-byte identities; hexadecimal is only test-vector transport.

## Framing

For bytes `x`:

```text
frame(x) = u64_le(len(x)) || x
```

For unsigned integer `n`:

```text
frame_u64(n) = frame(u64_le(n))
```

Optional text/digest/u64 values use a framed one-byte presence tag:

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

where complete encoded scopes are sorted lexicographically. Duplicate complete scopes are invalid. Duplicate `(policy_identity_profile, policy_registry_namespace)` keys are additionally invalid in v0.1.

## Canonical Root-A byte sequence

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
|| frame(root_source_verification_profile)
|| frame(raw_32_byte_root_source_anchor_digest)
|| canonical_policy_scope_set(authorized_policy_scopes)
|| frame_u64(valid_from_ms)
|| optional_u64(expires_at_ms)
|| frame(rotation_mode)
|| optional_text(rotation_profile)
|| optional_digest(rotation_authority_anchor_digest)
```

No JSON serialization, map iteration order, CBOR, MessagePack, Rust layout, Holochain serialization, or language-specific object representation participates in the digest.

## Normative golden vectors

For the checked-in v0.1 fixture:

Root-A identity:

`c3f9ba9b323f20c2d2ebd597e424857e8f3459d31393886fd6a7f835a6a93d6d`

Source-descriptor identity:

`f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126`

Rotation-authority identity:

`cf86718a53410b1e5e38cf5c552b3e4448772da97533fbd08c7b9bbfbee909ae`

The standard-library Python oracle independently recomputes all three and executes the adversarial corpus.

The Root-A vector wrapper itself remains exactly:

- `profile`;
- `expected_digest_hex`; and
- `root`.

Derived descriptor values are not injected as unauthenticated side metadata into that wrapper; qualification independently recomputes them from the normative root fields.

## Relationship to Root-B/C/D and CORE-LINEAGE

```text
Root-A exact semantic identity
        ↓
Root-B external generation-zero provenance
        ↓
Root-C predecessor-authorized transition verifier
        + predecessor rotation-authority identity
        ↓
CORE-LINEAGE-001 rooted-lineage structure
        ↓
Root-D exact endpoint source descriptor
        + independently authenticated closed-world coverage
        ↓
CORE-LINEAGE-001 covered-current-head structure
        ↓
QualifiedCurrentConstitutionalRoot
```

Root-C must not use the source-verification anchor as transition authority. Root-D must not use the rotation-authority anchor as source-currentness authority.

Qualified #815 gives stable semantic identity to procedure-policy currentness-provider selection policy. Qualified #812 gives a reusable record/adoption evidence waist. Neither is imported into this Root-A branch.

Only after Root-D may a downstream provider-policy adapter exact-match a complete Root-A ordinary policy tuple and qualified record/adoption evidence.

## Explicit nonclaims

GOVSYS-003A does **not** establish:

- root-key ownership or signature validity;
- bootstrap provenance;
- institutional/democratic adoption;
- legal validity or legitimacy;
- rotation-authority verifier origin or correctness;
- predecessor transition authorization;
- successful root rotation;
- authoritative source availability/authenticity;
- source coverage or coverage-verifier origin;
- current effective head;
- root currentness or non-revocation;
- policy-record authenticity;
- provider-policy currentness;
- administrative or review-policy currentness;
- administrative competence;
- judicial competence;
- execution authority; or
- external-effect authority.

The network remains infrastructure for institutions. It is not the sovereign.
