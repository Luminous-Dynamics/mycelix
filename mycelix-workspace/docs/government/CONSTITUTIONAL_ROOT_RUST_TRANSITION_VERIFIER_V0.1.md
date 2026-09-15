# GOVSYS-003C-VR — Rust predecessor transition verifier v0.1

## Purpose

This tranche ports the already-qualified GOVSYS-003C-V predecessor-owned transition authorization theorem into a small production-oriented Rust boundary.

It is a child of the hosted-qualified #878 selective convergence, so both of its theorem families are real Git ancestors:

- byte-exact Rust Root-A semantics (#862); and
- predecessor-owned transition verification (#844).

## Authority direction

The verifier accepts:

- a locally qualified predecessor `QualifiedConstitutionalRootA`;
- a locally qualified successor `QualifiedConstitutionalRootA`;
- an untrusted signed transition candidate;
- untrusted canonical Ed25519 SPKI material; and
- an untrusted signature proof.

It then independently:

1. checks predecessor-owned normal-rotation semantics;
2. recomputes every predecessor/successor Root-A, source and rotation identity;
3. exact-rebinds the asserted candidate to those locally recomputed facts;
4. canonicalizes the signed transition transcript;
5. recomputes the exact authorization-material commitment and requires equality with the predecessor Root-A rotation anchor;
6. applies the stricter #839 Ed25519 encoding/subgroup rules; and
7. calls `ed25519-dalek::VerifyingKey::verify_strict` only after those wrapper checks pass.

A successor key cannot authorize its own admission. Root-source verification material cannot substitute for constitutional rotation authority.

## Cryptographic parity

The production wrapper intentionally enforces a stricter language than `verify_strict()` alone.

For both public key point **A** and signature point **R** it requires:

- successful Edwards decompression;
- exact recompression equality, rejecting noncanonical encodings; and
- `curve25519-dalek::EdwardsPoint::is_torsion_free()`.

It also requires a canonical scalar **S** via `Scalar::from_canonical_bytes` before calling `verify_strict()`.

This preserves the acceptance boundary of the qualified #839 reference theorem instead of silently broadening it to the default library acceptance language.

The accepted SPKI representation is exactly:

```text
302a300506032b6570032100 || 32-byte Ed25519 key
```

No PEM, raw-key alias, alternate DER shape, suite alias, or caller-selected profile is accepted by v0.1.

## Stable transition identity

Profile:

`mycelix-constitutional-root-transition-v1-sha256-framed-semantic`

The stable identity is SHA-256 over the exact #839 framed candidate transcript. Signature bytes and proof digest are retained only as dynamic/audit provenance and do not alter stable constitutional history.

## Positive token

Success returns `VerifiedRootTransition` with private fields and no serialization/deserialization path. Only local cryptographic verification can construct it.

It exposes read-only access to exact transition, predecessor/successor, source, rotation, material, proof, timing and replay facts.

It explicitly returns:

```text
grants_currentness = false
grants_effect_authority = false
```

## Exact conformance fixture

The Rust tests use the exact already-qualified #839 public vector:

- predecessor Root-A `b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6`;
- successor Root-A `c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963`;
- material commitment `32d29fa9f5a28f5ef5ca3f298902add00f1ddd79007b729d0c12eae77358367f`; and
- transition identity `d71e66425effe3668790f552409aeba2005cca3c50ccb23cbd1d5a32dd060a24`.

The tests additionally deny signature mutation, candidate rebinding, successor-key substitution, zero replay nonce, and a non-prime-order signature point.

The hosted workflow separately reruns the inherited Python #839 theorem and independent OpenSSL verification over the same frozen public material/signature.

## Dependency boundary

The semantic candidate exact-pins its direct cryptographic dependencies:

- `ed25519-dalek = 3.0.0`;
- `curve25519-dalek = 5.0.0`; and
- `sha2 = 0.11.0`.

A follow-on qualification child should freeze the exact transitive Cargo graph with a reviewed lock and `--locked`, following CORE-LINEAGE #860. Semantic conformance and dependency-resolution evidence remain separate claims.

## Nonclaims

```text
VerifiedRootTransition
!= replay-set uniqueness
!= rooted-lineage completeness
!= complete global history
!= current constitutional root
!= source coverage/currentness
!= ordinary policy authority
!= administrative/judicial/execution authority
!= external-effect authority
```

Replay-set closure belongs to Root-C aggregation. Closed-world current-head promotion belongs to Root-D.

The network remains infrastructure for institutions. It is not the sovereign.
