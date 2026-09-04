# mycelix-xenia-reader-bridge

CF-07C2 binds Xenia's sealed authenticated application-channel evidence to one explicitly pinned Mycelix CF-07C1 reader-enrollment snapshot.

The boundary is:

```text
Xenia OpenedPeerApplicationPayloadV1
        |
        +-- exact Content Fabric application payload type (0xCF)
        +-- authenticated Ed25519 + ML-DSA-65 peer keys
        +-- same-carrier / AEAD / replay evidence already sealed by Xenia
        |
        v
pinned ReaderEnrollmentRegistryV1 commitment
        |
        +-- exact hybrid-key lookup
        +-- explicit PartyIdV1 + canonical groups
        |
        v
EnrollmentBoundOpenedPayloadV1
        |
        +-- plaintext sealed from public API
        +-- RemoteReaderV1 sealed from public API
        +-- audit commitments remain observable
        |
        v
future exact typed request parser in this crate
```

This crate deliberately does **not**:

- authenticate a raw public-key pair;
- accept a caller-provided PartyId or group assertion;
- derive PartyId from a key hash;
- accept an unpinned enrollment registry silently;
- expose the bound `RemoteReaderV1` before request-schema validation;
- expose authenticated plaintext before request-schema validation;
- parse or validate the Content Fabric request payload yet;
- authorize a Nix store object;
- expose a listener; or
- decide Nix signature/trusted-key policy.

The Xenia dependency is pinned to exact review commit:

```text
2125d0db2a5b7a995d129646e851adacb1c9f1fa
```

from Xenia PR #281. Updating that cryptographic-evidence dependency is an explicit review event rather than a floating branch dependency.

## Content Fabric payload domain

CF-07C2 reserves Xenia application payload byte `0xCF` for the Content Fabric reader channel.

That byte is only the Xenia AEAD/replay stream domain. The plaintext still requires a separate versioned Content Fabric request schema before any resource lookup is allowed.

## Registry pinning

`bind_xenia_opened_reader_v1(...)` requires both:

- the immutable `ReaderEnrollmentRegistryV1`; and
- the `StableIdV1` registry commitment expected by higher-level policy/configuration.

The call fails if the two differ.

This does not make the expected ID self-authenticating. A deployment still needs a trustworthy source for the pinned commitment. It does make accidental or malicious substitution of a different registry snapshot explicit at this boundary.

## Output non-claims

`EnrollmentBoundOpenedPayloadV1` internally contains AEAD-authenticated/replay-accepted bytes plus the reader identity resulting from the exact pinned enrollment.

It intentionally has no `Debug`, `Clone`, `Copy`, or serialization implementation. In CF-07C2, both plaintext and `RemoteReaderV1` access are crate-private. Public callers can inspect only non-sensitive audit commitments and generation metadata.

It is **not** yet a validated Content Fabric request. CF-07C3 should live in this crate, consume the bound value under one exact request schema, and only then release a public typed operation together with its already-bound reader identity for CF-07A serving-snapshot lookup.
