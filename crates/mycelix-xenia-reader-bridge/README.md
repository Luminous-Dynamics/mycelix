# mycelix-xenia-reader-bridge

This crate implements the CF-07C2 → CF-07C5 authenticated Content Fabric reader chain.

```text
Xenia hybrid authentication
        |
        v
same owned carrier
        |
        v
AEAD + replay application channel (0xCF)
        |
        v
pinned CF-07C1 hybrid reader enrollment
        |
        v
EnrollmentBoundOpenedPayloadV1
        |
        +-- plaintext sealed from public API
        +-- RemoteReaderV1 sealed from public API
        |
        v
exact 40-byte MCFR request
        |
        v
current CF-07A serving lookup
        |
        v
AuthorizedNixReadV1
        |
        +-- exact operation/store object
        +-- exact serving snapshot/policy commitments
        +-- exclusive serve-until deadline
        +-- request transcript/context/credential lineage
        |
        v
exact outbound Xenia generation/credential rebind
        |
        v
CF-07C4 verified representation preparation
        |
        +-- NarInfo -> one-shot bounded writer
        |
        `-- Nar -> exact authorized SHA-256 + size
                    -> CF-03 LocalCasV1::open_verified
                    -> same verified/rewound file handle
                    -> sequential bounded reader
        |
        v
CF-07C5 MCFS response stream
        |
        +-- serial Data frames
        +-- same operation + store hash on every frame
        +-- deadline-bounded authenticated sends
        `-- channel returned only after valid End frame
```

The Xenia dependency is pinned to exact review commit:

```text
2125d0db2a5b7a995d129646e851adacb1c9f1fa
```

from Xenia PR #281. CI resolves Cargo metadata and checks that exact source revision.

## Request domain and schema

Xenia application payload byte `0xCF` is reserved for Content Fabric reader traffic. The cryptographic payload domain is not itself the request schema.

The inner CF-07C3 request is exactly 40 bytes:

```text
0..4   "MCFR"
4..6   schema version, u16 big-endian = 1
6      operation: 1 = NarInfo, 2 = Nar
7      reserved = 0
8..40  exact 32-byte Nix store hash text
```

No PartyId, groups, enrollment, registry ID, serving snapshot, timestamp, arbitrary path/URL, or artifact trust assertion comes from request plaintext.

## Enrollment and serving authority remain sealed

CF-07C2 maps only the exact authenticated Ed25519 + ML-DSA-65 key pair through one explicitly pinned reader-enrollment registry. It never hashes a peer key into a PartyId and never accepts a request-supplied principal/group assertion.

The resulting `RemoteReaderV1` stays crate-private through request parsing. CF-07C3 immediately evaluates that sealed reader against `RemoteServingSnapshotV1::entry_for_reader_at(...)` at a trusted server-supplied time.

Unauthorized and absent objects remain indistinguishable.

`AuthorizedNixReadV1` is the first public resource-operation authority. It binds operation, exact store entry, serving snapshot/projection/policy commitments, enrollment/Xenia audit lineage, authorization time, and the exclusive `serve_until_unix_ms` horizon.

There is no generic public cache-entry accessor and no public reader identity extraction.

## CF-07C4 verified representation boundary

`prepare_authorized_nix_read_v1(authorized, cas)` consumes CF-07C3 authority and accepts no second digest, size, store hash, path, URL, operation, or deadline.

For `Nar`, C4 derives the exact SHA-256 + size only from the authorization, constructs one `BlobDescriptorV1`, and calls CF-03 `LocalCasV1::open_verified`.

CF-03 validates the opened file size, hashes the same opened handle, requires the exact digest, rewinds it, and returns that handle. C4 never reopens a path afterward.

The raw-NAR reader is sequential, at most 64 KiB per read, non-Clone, non-Seek, and has no raw-file extraction surface. NarInfo remains private behind a consuming one-shot writer.

C4 enforces both the original Unix serving deadline and a private monotonic deadline on every bounded read/write.

## CF-07C5 authenticated response stream

`send_authorized_nix_read_over_xenia_v1(...)` consumes:

- one `AuthenticatedPeerApplicationChannelV1<T>`;
- one `AuthorizedNixReadV1`; and
- an `Arc<LocalCasV1>`.

It returns the Xenia channel only if the complete response succeeds.

Before CAS verification or any response send, C5 requires the outbound channel to match the request authorization on:

- exact handshake transcript hash;
- exact negotiated context commitment; and
- exact hybrid credential commitment recomputed from the outbound channel's sealed Ed25519 + ML-DSA-65 authenticated keys.

This closes response redirection to a different authenticated peer/channel generation. A single-key match is insufficient.

The low-level frame sender remains hidden in a private module; the generation/credential-bound wrapper is the only exported C5 send API.

Before potentially expensive CAS verification, C5 derives an additional conservative monotonic deadline from the C3 serving horizon. It then runs C4 preparation and carries only the resulting representation through the same authenticated `0xCF` channel.

The response frame family uses `MCFS` and schema `1`. Every fixed 60-byte header repeats the operation and exact 32-byte store hash, then carries a u64 response sequence, u32 payload length, and u64 final-byte count field.

Data payloads are at most 64 KiB. A response is complete only after an authenticated End frame whose total equals all Data bytes.

The public API consumes the channel by value, structurally preventing two C5 responses from interleaving during delivery. The v1 server contract should keep one outstanding Content Fabric request per channel until that response either returns the channel or destroys it on failure.

## Bounded metadata production

C4's NarInfo writer is synchronous and one-shot. C5 runs it in a blocking task behind a Tokio channel of depth `1`.

Because C4 emits no more than 4 KiB per writer call, at most one metadata chunk can wait ahead of the authenticated carrier. C5 does not create a second full NarInfo buffer before sending.

## Deadline semantics

C4 and C5 enforce different, complementary boundaries.

C4 guarantees it releases no new representation bytes after its own serving authority expires.

C5 additionally checks the earlier conservative monotonic deadline plus the original Unix deadline before and after each response-frame send, and wraps the Xenia send future in a timeout at that deadline.

If a send times out, fails, or completes too late, the channel is not returned.

This does **not** claim that bytes already accepted by an OS/kernel/network buffer can be recalled after the deadline. Expiry cannot retroactively revoke bytes committed below the application layer.

If the outer async send future is cancelled while CF-03 preparation or a bounded blocking read is still running, the Xenia channel drops with the future. Detached blocking work may finish locally, but it no longer has a transport path through C5.

## Explicit non-claims

This crate does **not**:

- expose a listener or bind address;
- authenticate raw caller assertions;
- derive PartyId from key hashes;
- accept a caller-provided principal/group assertion;
- expose the enrolled `RemoteReaderV1` before resource authorization;
- accept a second resource identity after `AuthorizedNixReadV1` exists;
- expose a raw verified `File`, `Seek`, or reusable NarInfo body;
- add a public generic transport/sink abstraction;
- mint reader enrollments or remote-exposure grants;
- refresh serving snapshots;
- fetch missing content remotely;
- multiplex multiple outstanding v1 requests on one channel;
- implement long-lived Xenia rekey;
- sign Nix metadata;
- decide Nix `trusted-public-keys`; or
- prove that remote receipt occurred before the serving deadline.

Nix software trust remains independent of Content Fabric transport/disclosure authority.

## Normative contracts

- `docs/content-fabric/XENIA_READER_BRIDGE_V1.md`
- `docs/content-fabric/XENIA_TYPED_READER_REQUEST_V1.md`
- `docs/content-fabric/VERIFIED_READ_SERVING_V1.md`
- `docs/content-fabric/XENIA_RESPONSE_STREAM_V1.md`
