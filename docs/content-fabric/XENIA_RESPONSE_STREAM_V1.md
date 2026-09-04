# Mycelix Content Fabric — Xenia Response Stream v0.1

Status: CF-07C5 executable draft stacked on CF-07C4.

## Purpose

CF-07C5 carries one already-authorized Content Fabric Nix representation through the exact authenticated Xenia application channel that owns the same peer/session generation as the request.

The boundary is:

```text
Xenia hybrid authentication
        ↓
same owned carrier
        ↓
AEAD + replay application channel (0xCF)
        ↓
pinned reader enrollment
        ↓
exact MCFR request
        ↓
CF-07A serving authorization
        ↓
CF-07C3 AuthorizedNixReadV1
        ↓
exact outbound transcript/context/credential rebind
        ↓
CF-07C5 derives conservative delivery deadline
        ↓
CF-07C4 verified representation preparation
        ↓
MCFS response data frames
        ↓
MCFS End frame
        ↓
channel returned only on complete success
```

The public send API consumes both the authenticated application channel and `AuthorizedNixReadV1`. It returns the channel only after the response is complete.

Any binding, preparation, deadline, read, frame, seal, or carrier error consumes/drops the channel. A partially transmitted response therefore cannot be followed by ordinary channel reuse through this API.

## Exact outbound channel binding

A valid CF-07C3 authorization is not sufficient by itself to choose an arbitrary authenticated destination.

Before CF-03 verification or response transmission, the only exported C5 send path checks that the supplied outbound `AuthenticatedPeerApplicationChannelV1<T>` matches the request lineage on all of:

1. exact hybrid-handshake transcript hash;
2. exact negotiated authenticated context commitment; and
3. exact CF-07C1 hybrid credential commitment recomputed from the channel's sealed authenticated Ed25519 + ML-DSA-65 public keys.

The channel therefore cannot be swapped for another peer's otherwise-valid authenticated channel after object authorization.

The low-level frame sender is kept inside a private crate module and is not re-exported as a public bypass.

The credential check remains hybrid. A matching Ed25519 key with a different ML-DSA-65 key fails, and the inverse fails as well.

The transcript comparison binds delivery to the exact authenticated handshake generation rather than merely to a long-lived application identity.

## Exact Xenia domain

The supplied `AuthenticatedPeerApplicationChannelV1<T>` must already be pinned to Content Fabric application payload byte:

```text
0xCF
```

A channel for any other Xenia application domain is rejected before CF-03 verification or response transmission.

CF-07C5 does not accept a generic public transport/sink trait. The production entry point accepts the concrete sealed Xenia application-channel type. A private sink abstraction exists only to test local frame/deadline logic without creating a forgeable production delivery path.

## Conservative deadline derivation

Before potentially expensive CF-03 verification, CF-07C5 reads the `serve_until_unix_ms` carried by CF-07C3 authority and converts the remaining Unix horizon into a private monotonic deadline.

This happens **before** calling CF-07C4 preparation. The C5 deadline is therefore no later than C4's later preparation deadline under an honest clock and remains conservative if the wall clock rolls backward during verification.

Every response-frame send checks:

- the private monotonic deadline; and
- current Unix time against the original exclusive `serve_until_unix_ms`.

The Xenia `send_payload(...)` future is additionally wrapped in a Tokio timeout ending at the same monotonic deadline.

If that timeout fires, C5 returns an error and does not return the channel. Dropping the future may leave partial carrier framing or bytes already accepted by an operating-system/network buffer, so the channel is treated as unusable rather than resumed.

A send that completes but is observed to have crossed the deadline also causes C5 to fail and drop the channel.

## Precise delivery guarantee

CF-07C5 guarantees:

1. outbound delivery is bound back to the same authenticated transcript/context/hybrid credential as the request authority;
2. no new response frame is started after serving authority is observed expired;
3. each frame-send future is bounded by the conservative monotonic authority deadline;
4. a timed-out/failed/late send makes the authority-bearing channel unavailable for reuse through this API; and
5. only a complete response ending in a valid `End` frame returns the channel to the caller.

CF-07C5 does **not** claim that bytes already handed to an OS/kernel/network buffer before a deadline cannot arrive at the peer afterward. Expiry cannot retroactively revoke bytes already committed below the application layer.

This is the strongest guarantee available without moving the authorization clock inside the remote peer or transport implementation itself.

## Response frame schema

Every response is a serial sequence of independently AEAD-protected Xenia application payloads.

Each plaintext response frame begins with the fixed 60-byte header:

```text
offset  size  field
0       4     magic = ASCII "MCFS"
4       2     schema_version = u16 big-endian, exactly 1
6       1     frame_kind = 1 Data | 2 End
7       1     operation = 1 NarInfo | 2 Nar
8       32    exact Nix store hash ASCII text
40      8     application sequence = u64 big-endian
48      4     payload length = u32 big-endian
52      8     total transmitted bytes = u64 big-endian
60      ...   payload bytes
```

For `Data`:

- sequence starts at `0` and increments by exactly one;
- payload length must be non-zero in normal production output and no greater than 64 KiB;
- `total transmitted bytes` is `0`.

For `End`:

- payload length is exactly `0`;
- sequence is the next sequence after the final Data frame; and
- `total transmitted bytes` is the exact accumulated representation byte count.

The response family magic is `MCFS`; request-family magic remains independently `MCFR`.

## Receiver requirements

A receiver must reject the response if any of these occur:

- wrong magic or schema;
- unknown frame kind or operation;
- operation changes during a response;
- store hash changes during a response;
- sequence is missing, duplicated, reordered, or overflows;
- a Data payload length does not match its frame bytes;
- a frame exceeds the frozen payload ceiling;
- an End frame contains payload bytes;
- End total does not equal accumulated Data bytes; or
- the authenticated channel closes/terminalizes before a valid End frame.

Xenia AEAD/replay validation remains mandatory before interpreting any MCFS plaintext frame.

## Serial request/response ownership

The public sender consumes `AuthenticatedPeerApplicationChannelV1<T>` by value.

That means one channel cannot be used concurrently for two C5 responses through the safe public API. The channel is returned only after End succeeds, structurally serializing complete responses.

The v1 server protocol should maintain **one outstanding Content Fabric request per application channel**: after receiving one request, it should not receive a second request on that channel until the first response returns the channel successfully. A failure destroys the channel.

This keeps `(operation, store hash, ordered frame sequence)` sufficient for the in-flight response without introducing a client-controlled correlation/authority identifier in v1.

A later multiplexed protocol would require its own explicitly authenticated request/response correlation field and concurrency rules rather than silently weakening this serial contract.

## NarInfo streaming

CF-07C4 keeps NarInfo bytes private behind a one-shot synchronous writer.

CF-07C5 runs that writer in a blocking task and feeds it into a Tokio bounded channel of depth **1**. Because C4 emits at most 4 KiB per write, at most one such metadata chunk can wait ahead of the authenticated carrier.

C5 therefore does not first materialize a second full NarInfo body before sending.

If C5 stops consuming because the Xenia send path fails/expires, the bounded producer sees a closed channel and fails rather than continuing to accumulate metadata.

## Raw NAR streaming

CF-07C5 receives `AuthorizedNarReaderV1` only from CF-07C4.

The reader already owns the exact CF-03 verified file handle and exposes sequential reads capped at 64 KiB.

C5 performs one bounded blocking read at a time, rechecks its own earlier conservative deadline, then immediately constructs and sends one Data frame. It never accepts a second digest/path/store-hash selector.

The End frame's byte count must equal the exact initial verified remaining-byte count or the stream fails.

## Async cancellation and background work

C5 owns the Xenia channel inside the returned future.

If the outer future is cancelled during CF-03 preparation, a detached blocking verification task may finish locally, but the Xenia channel is dropped with the cancelled future and the prepared representation has no transport path.

If cancellation/failure occurs during NarInfo production or a NAR read, any blocking task may finish a bounded local chunk, but the channel is dropped and that local result is not sent through C5 afterward.

This preserves confidentiality even though Rust/Tokio cannot forcibly abort an already-running `spawn_blocking` closure.

## Failure semantics

No application-level error frame is sent after a response stream has begun.

Any failure means the channel is not returned. A receiver that does not obtain a valid End frame must discard the incomplete response.

This avoids creating a protocol in which a partially delivered NAR could later be misinterpreted as successful because an error message happened to arrive on the same stream.

## Nix trust remains independent

C5 proves authenticated-generation binding and controlled delivery of the representation that Content Fabric authorized.

It does not decide whether Nix should trust or install that representation. Nix NarHash/signature/trusted-public-key policy remains an independent final authority.

## Explicit non-claims

CF-07C5 does **not**:

- expose a listener or choose a bind address;
- authenticate raw caller assertions;
- mint reader enrollment;
- mint exposure grants;
- authorize another object;
- fetch missing content remotely;
- compress or rechunk below the fixed response-frame layer;
- add a public generic sink interface;
- implement long-lived Xenia rekey;
- multiplex more than one outstanding Content Fabric request on a channel; or
- prove remote receipt occurred before the serving deadline.

## Qualification expectations

The focused C5 lane must run:

- Rust 1.96 rustfmt/check/strict Clippy/tests/rustdoc;
- Rust 1.94 MSRV check/tests;
- exact Xenia PR #281 dependency-source verification;
- focused `delivery::tests` for transcript/context/hybrid-credential rebind;
- focused `response::tests` for framing/deadline behavior;
- the complete bridge tests;
- CF-07C4 serving tests; and
- CF-03 verified-CAS tests.

Keep this tranche draft until exact-head GitHub Actions execute successfully.
