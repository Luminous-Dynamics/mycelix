# mycelix-reader-enrollment

`mycelix-reader-enrollment` is CF-07C1: an explicit, deterministic directory from an exact hybrid peer public-key pair to Mycelix reader identity facts.

It is deliberately **not authentication**.

## Boundary

```text
explicit enrollment policy
        |
        +-- Ed25519 public key
        +-- ML-DSA-65 public key
        +-- PartyIdV1
        +-- groups
        |
        v
ReaderEnrollmentV1
        |
        v
ReaderEnrollmentRegistryV1
```

The registry answers only:

> If policy enrolled this exact hybrid key pair, which Mycelix principal and groups did it assign?

It does **not** answer:

> Did the current network peer authenticate this key pair?

That proof belongs to a later Xenia live-connection adapter.

## Exact hybrid matching

The credential is the exact pair:

```text
Ed25519 / RFC 8032 / 32-byte verifying key
        +
ML-DSA-65 / FIPS 204 / 1952-byte verifying key
```

Both keys are included in the versioned stable commitment.

Matching only Ed25519 is intentionally insufficient. A peer holding one enrolled classical key must not be able to pair it with an unrelated self-generated ML-DSA key and inherit an enrolled reader identity.

The model validates the fixed ML-DSA-65 representation length but does not claim that arbitrary bytes are a cryptographically valid public key. Xenia's handshake is responsible for cryptographic parsing and proof-of-possession.

## No derived PartyId shortcut

The hybrid credential commitment is **not** a Mycelix principal.

CF-07C1 never defines:

```text
PartyId = hash(public key)
```

Instead, enrollment explicitly binds:

```text
exact hybrid credential -> PartyIdV1 + groups
```

This allows identity policy, account recovery, key rotation, organizational identities, and future credential suites to remain independent from one cryptographic key representation.

## Stable commitments

Three domain-separated commitments are used:

```text
content-fabric/xenia-reader-credential@1
content-fabric/reader-enrollment@1
content-fabric/reader-enrollment-registry@1
```

The credential commitment includes explicit Ed25519 and ML-DSA-65 suite labels plus exact public-key bytes.

The enrollment commitment includes:

- credential commitment;
- explicit non-zero `PartyIdV1`; and
- canonical sorted/deduplicated non-zero group IDs.

The registry commitment includes every enrollment ID in credential-ID sort order. Therefore registry identity is independent of input insertion order.

## Rotation and ambiguity

Multiple different credentials may map to the same PartyId. This supports explicitly controlled key-rotation overlap.

The same exact hybrid credential may appear only once in one registry snapshot. Duplicate enrollment fails closed even if the duplicate would carry identical fields.

Changing the principal or group snapshot changes the enrollment ID and therefore the registry ID.

## Future Xenia composition

A later authenticated request boundary should require all of:

```text
Xenia AuthenticatedPeerHandshakeV1
        +
ReaderEnrollmentRegistryV1 exact pair match
        +
live transport ownership bound to the same handshake generation
        |
        v
request-scoped RemoteReaderV1
```

CF-07C1 alone must never mint request authority from caller-supplied keys.

## Non-claims

This crate does not:

- run a handshake;
- verify Ed25519 or ML-DSA signatures;
- prove private-key possession;
- prove connection liveness;
- bind a request to a transport generation;
- construct a network listener;
- grant content exposure;
- sign Nix metadata; or
- alter Nix's own software trust decision.
