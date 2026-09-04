# mycelix-xenia-reader-bridge

CF-07C2 binds Xenia's sealed authenticated application-channel evidence to one explicitly pinned Mycelix CF-07C1 reader-enrollment snapshot. CF-07C3 consumes that sealed reader+plaintext binding through one exact bounded request schema and a current CF-07A serving snapshot. CF-07C4 then consumes only the resulting `AuthorizedNixReadV1` and prepares the exact representation through CF-03 verified local CAS reads.

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
        v
RemoteServingSnapshotV1::entry_for_reader_at(...)
        |
        v
AuthorizedNixReadV1
        |
        +-- exact operation/store entry
        +-- exact serving snapshot/projection/policy commitments
        +-- exclusive serve-until deadline
        |
        v
prepare_authorized_nix_read_v1(...)
        |
        +-- NarInfo -> one-shot deadline-bound writer
        |
        `-- Nar -> exact SHA-256 + size
                    -> CF-03 LocalCasV1::open_verified
                    -> same verified/rewound file handle
                    -> sequential deadline-bound reader
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
- expose a generic authorized cache-entry accessor;
- accept a second digest, size, path, store hash, operation, or deadline after `AuthorizedNixReadV1` exists;
- expose the verified raw `File`, `Seek`, or `into_file` from the NAR serving path;
- expose a reusable NarInfo `String`/`Vec` body;
- expose a listener; or
- decide Nix signature/trusted-key policy.

The Xenia dependency remains pinned to exact review commit:

```text
2125d0db2a5b7a995d129646e851adacb1c9f1fa
```

from Xenia PR #281.

## CF-07C3 request schema

The v1 request is exactly 40 bytes:

```text
0..4   "MCFR"
4..6   schema version, u16 big-endian = 1
6      operation: 1 = NarInfo, 2 = Nar
7      reserved = 0
8..40  exact 32-byte Nix store hash text
```

`MCFR` is stable family framing; the separate u16 field owns schema versioning. Identity, groups, enrollment, registry, serving snapshot, timestamps, arbitrary URL/path, and artifact-trust assertions do not appear in request bytes.

## CF-07C4 verified representation preparation

`prepare_authorized_nix_read_v1(authorized, cas)` consumes the non-Clone CF-07C3 authorization. There is no second resource argument.

For `NarInfo`, the already-authorized cache entry is rendered internally into `AuthorizedNarInfoV1`. The body remains private; `write_to(...)` consumes the authority and emits at most 4 KiB per write while checking both the original Unix `serve_until_unix_ms` and a private monotonic deadline before and after every chunk.

For `Nar`, CF-07C4 reads the exact authorized SHA-256 and size from `AuthorizedNixReadV1`, constructs:

```text
BlobDescriptorV1 {
    digest = sha256:<authorized digest>,
    size_bytes = <authorized NAR size>,
    media_type = None,
}
```

and calls CF-03 `LocalCasV1::open_verified(&descriptor)`.

CF-03 validates the exact size, opens with its no-follow final-file rules, hashes the same opened handle, verifies the digest, rewinds it, and returns that handle. CF-07C4 does not reconstruct or reopen a filesystem path afterward.

Because full-file verification can take time, CF-07C4 rechecks the serving horizon *after* CF-03 verification before releasing a reader.

## Dual-clock stream expiry

Prepared streams carry both:

- the original exclusive Unix `serve_until_unix_ms`; and
- a private monotonic `Instant` derived conservatively from the remaining authority horizon.

Every bounded NAR read and NarInfo write checks both clocks. This gives fail-closed behavior for either direction of wall-clock movement:

- wall-clock rollback cannot extend authority beyond the monotonic deadline;
- wall-clock jump forward expires authority immediately through the Unix check.

If raw-NAR expiry occurs while one file read is in progress, the just-read caller-buffer region is zeroed and the stream terminalizes before returning an error. NAR reads are capped at 64 KiB per call, are sequential only, and terminalize on expiry, clock failure, underlying I/O error, or unexpected early EOF.

There is deliberately no `Seek`, clone, or raw-file extraction path.

## Audit-only output

Prepared representations expose `PreparedNixReadAuditV1`, which is cloneable because it is intentionally non-authorizing. It contains operation/store-hash and snapshot/enrollment/Xenia generation commitments, but no reader principal/groups, CAS path, NAR digest, raw NAR size, file handle, or body bytes.

Cloning audit facts therefore does not clone resource-serving authority.

## Remaining boundary

CF-07C4 provides a verified local representation reader/writer, not a network listener. A future HTTP/Xenia response adapter should consume these one-shot/streaming types directly rather than accepting a new resource identifier. It must preserve deadline errors as terminal response failures and keep Nix artifact signature/trusted-key policy independent of cache disclosure authority.
