# mycelix-xenia-reader-bridge

CF-07C2 binds Xenia's sealed authenticated application-channel evidence to one explicitly pinned Mycelix CF-07C1 reader-enrollment snapshot. CF-07C3 then consumes that sealed reader+plaintext binding through one exact bounded request schema and a current CF-07A serving snapshot.

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
        |
        v
exact 40-byte CF-07C3 request parse
        |
        +-- family magic MCFR
        +-- schema = 1
        +-- operation = NarInfo | Nar
        +-- reserved byte = 0
        +-- exact 32-byte Nix store hash
        |
        v
RemoteServingSnapshotV1::entry_for_reader_at(...)
        |
        v
AuthorizedNixReadV1
        |
        +-- exact operation
        +-- exact authorized store entry internally
        +-- exact serving snapshot/projection/policy commitments
        +-- exclusive serve-until deadline
        +-- enrollment/Xenia audit chain
```

This crate deliberately does **not**:

- authenticate a raw public-key pair;
- accept a caller-provided PartyId or group assertion;
- derive PartyId from a key hash;
- accept an unpinned enrollment registry silently;
- expose the bound `RemoteReaderV1` before request authorization;
- expose authenticated plaintext before request authorization;
- accept variable-length/trailing request bytes;
- accept unknown request operations or non-zero reserved semantics;
- allow request plaintext to choose the serving-evaluation time;
- expose a generic authorized cache-entry accessor that erases NarInfo-vs-Nar authority;
- expose representation facts without a current-time deadline check;
- expose a listener; or
- decide Nix signature/trusted-key policy.

The Xenia dependency is pinned to exact review commit:

```text
2125d0db2a5b7a995d129646e851adacb1c9f1fa
```

from Xenia PR #281. Updating that cryptographic-evidence dependency is an explicit review event rather than a floating branch dependency.

## Content Fabric payload domain

CF-07C2 reserves Xenia application payload byte `0xCF` for the Content Fabric reader channel.

That byte is only the Xenia AEAD/replay stream domain. CF-07C3 defines the independent inner request schema.

## CF-07C3 request wire format

The v1 request is exactly 40 bytes:

```text
0..4   "MCFR"
4..6   schema version, u16 big-endian = 1
6      operation: 1 = NarInfo, 2 = Nar
7      reserved = 0
8..40  exact 32-byte Nix store hash text
```

`MCFR` is stable family framing; the separate u16 field owns schema versioning.

No identity, group, enrollment, registry, serving snapshot, timestamp, path suffix, arbitrary URL, or other authority assertion appears in request bytes.

The decoder rejects short or long messages, wrong magic, unsupported schema versions, unknown operations, non-zero reserved bits, and store hashes rejected by the existing exact `NixStoreHashV1` parser.

## Registry pinning

`bind_xenia_opened_reader_v1(...)` requires both the immutable `ReaderEnrollmentRegistryV1` and the `StableIdV1` registry commitment expected by higher-level policy/configuration. The call fails if they differ.

This does not make the expected ID self-authenticating. A deployment still needs a trustworthy source for the pinned commitment. It makes substitution of another registry snapshot explicit at this boundary.

## Serving authority

`authorize_bound_nix_read_v1(...)` consumes `EnrollmentBoundOpenedPayloadV1`; callers cannot retain a second copy because that evidence is non-Clone.

The trusted server runtime supplies `request_time_unix_ms`. CF-07C3 parses the request and immediately calls `RemoteServingSnapshotV1::entry_for_reader_at(store_hash, reader, request_time_unix_ms)` using the still-sealed enrolled reader.

Unauthorized and absent objects both remain `Ok(None)`, preserving CF-07A's non-enumerating behavior.

On success, `AuthorizedNixReadV1` binds the operation to the exact authorized entry, the serving snapshot/projection/policy commitments, the trusted authorization time, and the snapshot's exclusive `serve_until_unix_ms` horizon. There is no public generic `entry()` accessor:

- `render_narinfo_at(now)` returns metadata only for a `NarInfo` authorization that is still within the serving horizon;
- `nar_sha256_digest_at(now)` and `nar_size_at(now)` return raw-NAR retrieval facts only for a `Nar` authorization that is still within the serving horizon.

A metadata authorization therefore cannot be reused as a raw-NAR capability, and an expired authorization cannot yield representation facts through the ordinary public API.

## Remaining boundary

CF-07C3 authorizes *which immutable Nix object representation may be read and until when*. A later serving adapter still must use the Nar digest/size to obtain CF-03 verified bytes, must preserve the authorized operation, must recheck the deadline during long streams, and must keep Nix artifact signature/trusted-key policy independent of transport/cache authorization.
