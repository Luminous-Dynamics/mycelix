# GOVSYS-003C-V — Constitutional Root Transition Verifier v0.1

## Status

This document freezes the first executable predecessor-owned transition-verifier profile above qualified GOVSYS-003A. It is a verification theorem for one exact constitutional transition candidate; it is not a current-root theorem and it grants no actor or effect authority.

```text
stored transition candidate
!= predecessor authorization
!= rooted lineage
!= current constitutional root
```

## Qualified parent

The implementation is a direct child of qualified Root-A exact head:

`235834fc726c1a79467e6f910a2cf39fb9abd2e3`

Root-A hosted qualification: PASS, run `34836891820`.

This verifier reuses the exact qualified Root-A oracle and independently recomputes complete Root-A, source-descriptor and rotation-authority identities for both predecessor and successor.

## Trust direction

For predecessor `P` and successor candidate `S`:

```text
exact P Root-A
+ exact P rotation-authority commitment
+ exact canonical transition P -> S
+ exact verifier material matching P's committed anchor
        ↓
execute P.rotation_profile
        ↓
VerifiedRootTransitionEvidence
```

`S` never authorizes its own admission. Its rotation declaration can govern only a later `S -> N` transition.

## Separate source and rotation powers

Root-A deliberately commits independent trust roles:

```text
root_source_anchor_digest
        -> source/coverage verification

rotation_authority_anchor_digest
        -> constitutional transition authorization
```

The source anchor cannot substitute for the rotation anchor. A valid rotation signature cannot establish source coverage or currentness.

## Reference authorization material profile

Profile:

`mycelix-constitutional-root-rotation-authority-ed25519-single-v1`

Suite:

`ed25519-spki-v1`

Canonical material contains exactly one DER SubjectPublicKeyInfo encoding of an Ed25519 public verification key. The material commitment is:

```text
SHA-256(
    "mycelix/public-institution/constitutional-root-rotation-authority-material/ed25519-single/v1"
    || frame(material_profile)
    || frame(suite)
    || frame(canonical_spki_der)
)
```

The commitment must exact-match `P.rotation_authority_anchor_digest_hex` before signature verification is attempted.

The public conformance key is not a production authority key. No private signing key is checked into the repository.

The original GOVSYS-003A golden root uses the placeholder anchor `44…44` to qualify identity semantics. This transition fixture instead constructs a second valid Root-A predecessor whose anchor is the actual commitment of the checked-in public verifier material. It does not modify or supersede the qualified Root-A golden fixture.

## Stable transition profile

Profile:

`mycelix-constitutional-root-transition-v1-sha256-framed-semantic`

Domain:

`mycelix/public-institution/constitutional-root-transition/v1`

The stable candidate commits:

- predecessor Root-A identity and generation;
- predecessor source-descriptor identity;
- predecessor rotation-authority identity;
- successor Root-A identity and generation;
- successor source-descriptor identity;
- successor rotation-authority identity or explicit immutable absence;
- `authorized_at_ms`;
- `effective_at_ms`; and
- a non-zero bounded replay nonce.

The stable transition identity is SHA-256 over that domain-separated framed transcript.

Dynamic verifier invocation evidence is deliberately excluded:

```text
same semantic transition
+ refreshed verifier execution
        -> same stable transition identity
```

Signature bytes, proof references, verification timestamps and evidence leases remain audit/verifier evidence outside CORE-LINEAGE Stage 1.

## Normal rotation relation

`constitutional-root-rotation-v1` requires:

- `P.rotation_mode == predecessor-authorized`;
- exact predecessor rotation profile `constitutional-root-rotation-v1`;
- checked `S.generation == P.generation + 1`;
- `S.predecessor_root_digest == digest(P)`;
- unchanged protocol family, institution and jurisdiction;
- unchanged bootstrap mode/profile;
- unchanged authoritative root source ref;
- unchanged root coverage profile;
- unchanged root source-verification profile;
- authorization no earlier than predecessor validity;
- `effective_at_ms >= authorized_at_ms`;
- authorization/effect strictly before predecessor expiry when expiry exists;
- `S.valid_from_ms == effective_at_ms`; and
- structurally valid successor Root-A semantics.

The exact successor Root-A may change its constitutional rulebook, ordinary authorized policy scopes, expiry, source-verification anchor, rotation mode/profile and future rotation-authority anchor.

Changing the source-verification anchor changes the successor source-descriptor identity. This is allowed only because the exact successor Root-A identity binds that change; it does not let the old source verifier establish successor currentness.

## Conformance fixture

The checked-in vector freezes these independently reproducible values:

```text
predecessor verifier-material commitment
32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f

predecessor Root-A
b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6

predecessor source descriptor
f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126

predecessor rotation-authority identity
db3bf05b04063ea2e9626e9ea63def4920f205cb36c43d64a046fd04e87b8304

successor Root-A
c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963

successor source descriptor
3f867aab08093b08f8f4088a540e8cffa85dc25ab7c7cfb34ac65c8bc071a3fe

successor future verifier-material commitment
f6833e11f4317ff680813e6e1d9e1189b4cbb1abb44a67f4b7fec3897427435f

successor rotation-authority identity
ee35844f067306e66ec8fcb40442b3564e9f7f5bd824a3e2a71265ea318aebea

stable transition identity
d71e66425effe3668790f552409aeba2005cca3c50ccb23cbd1d5a32dd060a24
```

The signature in the vector is a conformance proof over the exact frozen candidate transcript. CI verifies it through both the dependency-free Python verifier and OpenSSL.

## Replay discipline

One verifier invocation can prove that a nonce is non-zero, bounded and signed as part of a candidate. It cannot prove global uniqueness.

Root-C aggregation therefore owns:

```text
same predecessor Root-A
+ same replay nonce
+ different stable transition identity
        -> ReplayNonceConflict
```

Exact duplicate evidence for one identical transition may normalize. Replay collision across different transitions fails closed.

## Fork discipline

Two distinct successors may both carry cryptographically valid predecessor authorization. That remains explicit fork evidence.

The verifier does not choose a winner using timestamp, arrival order, key identity, lexical digest order, reputation, stake, Phi or caller ordering. Root-C/CORE-LINEAGE must surface the conflict.

## Hosted qualification

The dedicated exact-head workflow requires:

- literal PR-head checkout with persisted credentials disabled;
- exact qualified Root-A ancestry and exact parent oracle/vector blob identities;
- exact four-file child review surface;
- no runtime, Holochain, persistence or effect-authority additions;
- no checked-in private key material;
- execution of the exact qualified Root-A parent oracle;
- independent material/candidate transcript recomputation outside the verifier implementation;
- independent OpenSSL Ed25519 verification over a separately reconstructed transcript;
- Python syntax compilation;
- dependency-free verifier golden/adversarial corpus; and
- immutable checkout postflight.

Diagnostic executable lanes may report independently, but the final gate fails closed unless every required lane succeeds.

## Nonclaims

This theorem proves only that one exact Root-A successor transition was authorized by verifier material committed by its exact predecessor under the frozen profile.

It does **not** prove current root, source coverage, legal legitimacy, democratic mandate, ordinary policy currentness, administrative authority, execution authority or external-effect authority.

The network remains infrastructure for institutions. It is not the sovereign.
