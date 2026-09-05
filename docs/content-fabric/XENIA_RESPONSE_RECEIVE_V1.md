# Mycelix Content Fabric — Xenia Response Receive v0.1

Status: CF-07C6 executable draft stacked on CF-07C5 / PR #107.

## Purpose

CF-07C6 closes the client-side half of the serial Content Fabric Xenia exchange.
It sends one exact `MCFR` request and validates the matching `MCFS` response on
the **same authenticated application channel** under one caller-owned monotonic
round-trip deadline.

```text
AuthenticatedPeerApplicationChannelV1<T> / payload domain 0xCF
        |
        +-- send MCFR with Xenia deadline-aware send
        |
        v
XeniaNixResponseReceiverV1<T>
        |
        +-- owns exact same channel
        +-- owns exact request operation + store hash
        +-- owns one monotonic round-trip deadline
        |
        +-- each receive moves channel out of receiver
        +-- Xenia deadline-owned receive consumes channel
        +-- AEAD + replay + domain validation
        +-- MCFS structural validation
        |
        +-- success: channel reinserted, Data or End event released
        |
        `-- any timeout/cancellation/Xenia/parser failure:
              no channel is recoverable
```

## Xenia dependency

CF-07C6 pins the exact Xenia deadline-receive successor / PR #302 head:

```text
68cce019d472e00570b488599c97c62144aa30a0
```

That revision inherits PR #300's `send_payload_before_deadline(...)` and adds
`recv_opened_payload_before_deadline_v1(...)`.

The receive primitive consumes `AuthenticatedPeerApplicationChannelV1<T>` by
value and returns the channel only after a safe successful receive/open. A
receive timeout or failure returns no channel. Caller-side cancellation of the
receive future likewise drops the moved channel.

## One deadline for one serial round trip

The request APIs require `std::time::Instant`:

```rust
send_nix_narinfo_request_over_xenia_v1(channel, store_hash, deadline)

send_nix_nar_request_over_xenia_v1(
    channel,
    store_hash,
    expected_nar_size,
    deadline,
)
```

The same monotonic deadline applies to:

1. sending the MCFR request; and
2. every MCFS response-frame receive until End.

The deadline is an operational client round-trip bound. It is not the server's
CF-07A disclosure horizon and is not derived from untrusted response bytes.

## Cancellation and framing safety

Before every response await, `XeniaNixResponseReceiverV1` removes its channel
from internal state and transfers ownership to Xenia's deadline-owned receive.

The channel is reinserted only after:

1. same-carrier receive succeeds;
2. Xenia application-domain check succeeds;
3. AEAD verification succeeds;
4. replay acceptance succeeds; and
5. the MCFS frame passes structural validation.

Therefore:

- receive timeout cannot return a partially consumed channel;
- caller cancellation while awaiting receive leaves the receiver without a
  channel;
- malformed MCFS plaintext drops the otherwise-valid authenticated channel; and
- `finish()` cannot recover a channel from a terminal/incomplete receiver.

The design intentionally chooses availability loss over framing/replay ambiguity.

## MCFS validation

Every independently AEAD-protected MCFS plaintext frame must have the fixed
60-byte v1 header:

```text
offset  size  field
0       4     magic = "MCFS"
4       2     schema = u16 big-endian, exactly 1
6       1     frame kind = 1 Data | 2 End
7       1     operation = 1 NarInfo | 2 Nar
8       32    exact requested Nix store hash
40      8     exact u64 sequence
48      4     payload length u32
52      8     End total byte count u64
60      ...   payload
```

Validation is closed-world:

- exact magic/schema/kind/operation;
- operation cannot change from the request;
- store hash cannot change from the request;
- sequence starts at zero and increments exactly once per Data frame;
- declared payload length must equal actual frame payload bytes;
- Data payload must be non-empty and no larger than 64 KiB;
- Data `total_bytes` must be zero;
- End must contain no payload;
- End total must equal accumulated Data bytes;
- byte and sequence arithmetic are checked; and
- no frame is accepted after End.

## Response bounds

### NarInfo

NarInfo has no prior exact response-size commitment in MCFR v1, so the receiver
applies a hard 1 MiB ceiling:

```text
MAX_NARINFO_RESPONSE_BYTES_V1 = 1 MiB
```

This bound is a resource-safety limit, not a claim that all valid NarInfo must
approach that size.

### Raw NAR

A raw-NAR request requires a non-zero `expected_nar_size` obtained from
previously validated metadata before the request is sent.

That size is both:

- the maximum number of Data bytes accepted; and
- the exact total required by End.

Receiving exactly the expected size still does **not** prove the NAR digest.
Digest verification remains a subsequent boundary.

## Data events are not completion authority

`ReceivedXeniaNixResponseChunkV1` contains authenticated, structurally valid Data
bytes from an in-progress response. It deliberately has no `Debug` or `Clone`.

A Data event is **not** proof that the representation completed. Consumers must
not promote staged content merely because one or more Data events were valid.

`CompletedXeniaNixResponseV1` is created only after a valid End frame. It is
cloneable audit evidence containing only:

- operation;
- exact requested store hash;
- accumulated total bytes; and
- Data-frame count.

It does not contain or prove a NAR digest, Nix signature, trusted-key decision,
or install authorization.

## Channel recovery

The authenticated application channel is recoverable only through:

```rust
receiver.finish()
```

after End has completed successfully.

Calling `finish()` early fails and drops the channel. Any receiver/Xenia failure
also prevents recovery.

This preserves the CF-07C5 serial contract: one outstanding Content Fabric
request/response exchange per channel.

## Nix trust remains independent

CF-07C6 proves authenticated-channel transport and closed-world response framing.
It does not decide whether the returned bytes should be trusted by Nix.

For raw NAR, the next ingest boundary must verify the expected NAR digest before
promotion. For NarInfo, signature/trusted-public-key policy remains a separate
Nix authority decision.

## Explicit non-claims

CF-07C6 does **not**:

- expose a listener or bind address;
- trust response bytes merely because Xenia authenticated the peer;
- derive expected NarSize from the raw NAR response itself;
- prove a raw NAR digest;
- promote partial response chunks into CAS;
- decide Nix signatures or trusted keys;
- multiplex multiple outstanding v1 requests on one channel;
- recover a channel after timeout, cancellation, malformed frame, or failed End;
- solve long-lived Xenia rekey; or
- claim the client deadline is the server disclosure-policy deadline.

## Qualification expectations

The focused C6 lane must run:

- Rust 1.96 rustfmt/check/strict Clippy/tests/rustdoc;
- Rust 1.94 MSRV check/tests;
- exact Xenia PR #302 dependency-source verification;
- focused `response_receive::tests`;
- C5 delivery/response requalification;
- the complete bridge test suite; and
- CF-03 verified-CAS tests.

Keep this tranche draft until exact-head GitHub Actions execute successfully.
