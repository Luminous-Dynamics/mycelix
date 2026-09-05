# mycelix-xenia-reader-bridge

This crate implements the CF-07C2 → CF-07C6 authenticated Content Fabric reader chain.

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
        +-- exclusive server serve-until deadline
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
        +-- deadline-aware Xenia send after AEAD seal
        `-- channel returned only after valid End frame

client side, same 0xCF channel:

MCFR request sent before one caller-owned deadline
        |
        v
CF-07C6 XeniaNixResponseReceiverV1
        |
        +-- Xenia deadline-owned receive
        +-- same channel + AEAD + replay
        +-- exact MCFS framing/sequence/hash/op checks
        +-- NarInfo <= 1 MiB
        +-- raw NAR == predeclared NarSize
        `-- channel recoverable only after valid End
```

The Xenia dependency is pinned to exact review commit:

```text
68cce019d472e00570b488599c97c62144aa30a0
```

from Xenia PR #302, which is stacked on PR #300's deadline-aware authenticated send and PR #281's sealed authenticated application channel. CI resolves Cargo metadata and checks that exact source revision.

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

The resulting `RemoteReaderV1` stays crate-private through request parsing. CF-07C3 immediately evaluates that sealed reader against `RemoteServingSnapshotV1::entry_for_reader_at(...)` at trusted server time.

Unauthorized and absent objects remain indistinguishable.

`AuthorizedNixReadV1` is the first public server-side resource-operation authority. It binds operation, exact store entry, serving snapshot/projection/policy commitments, enrollment/Xenia audit lineage, authorization time, and the exclusive `serve_until_unix_ms` horizon.

There is no generic public cache-entry accessor and no public reader identity extraction.

## CF-07C4 verified representation boundary

`prepare_authorized_nix_read_v1(authorized, cas)` consumes CF-07C3 authority and accepts no second digest, size, store hash, path, URL, operation, or deadline.

For raw NAR, C4 derives the exact SHA-256 + size only from the authorization and calls CF-03 `LocalCasV1::open_verified`. CF-03 hashes the same opened handle it returns. C4 never reopens a path afterward.

The raw-NAR reader is sequential, at most 64 KiB per read, non-Clone, non-Seek, and has no raw-file extraction surface. NarInfo remains private behind a consuming one-shot writer.

C4 enforces both the original Unix serving deadline and a private monotonic deadline on every bounded read/write.

## CF-07C5 authenticated response stream

`send_authorized_nix_read_over_xenia_v1(...)` returns the Xenia channel only if the complete response succeeds.

Before CAS verification or response send, C5 requires the outbound channel to match the request authorization on exact handshake transcript, negotiated context, and hybrid credential commitment. This closes response redirection to another valid authenticated peer/channel.

MCFS schema v1 repeats the exact operation and 32-byte store hash on every independently AEAD-protected frame, uses exact u64 sequencing, carries at most 64 KiB per Data frame, and completes only with an empty End frame committing the exact accumulated byte count.

C5 passes its conservative monotonic deadline into Xenia PR #300's `send_payload_before_deadline(...)`, which checks before seal, after seal/before carrier I/O, around carrier I/O, and after completion. Failed/timed-out/late channels are not returned.

## CF-07C6 deadline-owned response receive

C6 is the reciprocal client-side serial boundary.

The NarInfo and raw-NAR request functions now require one caller-owned `std::time::Instant` deadline. The MCFR request is sent with Xenia's deadline-aware send, and the same deadline is stored in `XeniaNixResponseReceiverV1` for every MCFS receive until End.

Before each receive, C6 removes the authenticated channel from receiver state and calls Xenia PR #302's consuming `recv_opened_payload_before_deadline_v1(...)`. The channel is reinserted only after both Xenia receive/open and MCFS structural validation succeed.

Therefore timeout, caller cancellation, carrier/domain/AEAD/replay failure, or malformed MCFS data leaves no recoverable channel. `finish()` returns the channel only after a valid End frame.

C6 validates exact MCFS magic/schema/kind, request operation, request store hash, sequence, declared length, frame ceiling, Data/End field rules, cumulative byte arithmetic, and End total.

NarInfo is bounded to 1 MiB. Raw NAR requires a non-zero expected NarSize from previously validated metadata and requires End to equal that exact size.

`ReceivedXeniaNixResponseChunkV1` is transient in-progress data, not completion authority. `CompletedXeniaNixResponseV1` exists only after valid End and proves framing/size completion only—not digest or software trust.

## Deadline semantics

Server disclosure authority and client round-trip lifetime remain separate:

- CF-07A/C4/C5 enforce the server's disclosure horizon;
- CF-07C6 enforces a caller-owned monotonic request/response deadline;
- neither deadline comes from untrusted response plaintext.

Expiry or cancellation cannot retroactively recall bytes already committed below the application layer. The invariant is instead that a failed/late path cannot return a healthy authority-bearing channel for reuse.

## Explicit non-claims

This crate does **not**:

- expose a listener or bind address;
- authenticate raw caller assertions;
- derive PartyId from key hashes;
- accept caller-provided principal/group assertions;
- expose the enrolled `RemoteReaderV1` before resource authorization;
- accept a second resource identity after `AuthorizedNixReadV1` exists;
- expose a raw verified `File`, `Seek`, or reusable NarInfo body;
- add a public generic transport/sink abstraction;
- mint reader enrollments or remote-exposure grants;
- refresh serving snapshots;
- fetch missing content remotely;
- multiplex multiple outstanding v1 requests on one channel;
- promote partial C6 chunks into CAS;
- prove raw NAR digest merely from MCFS completion;
- implement long-lived Xenia rekey;
- sign Nix metadata;
- decide Nix `trusted-public-keys`; or
- prove remote receipt occurred before the server serving deadline.

Nix software trust remains independent of Content Fabric transport/disclosure authority.

## Normative contracts

- `docs/content-fabric/XENIA_READER_BRIDGE_V1.md`
- `docs/content-fabric/XENIA_TYPED_READER_REQUEST_V1.md`
- `docs/content-fabric/VERIFIED_READ_SERVING_V1.md`
- `docs/content-fabric/XENIA_RESPONSE_STREAM_V1.md`
- `docs/content-fabric/XENIA_RESPONSE_RECEIVE_V1.md`
