# WEB-LOCATOR-001B1 — Admission Receipt Canonicalization Corpus v0.1

## Status

Frozen architecture/test-vector subject for #2705.

This tranche defines **audit evidence and canonical commitment semantics only**.
It creates no DNS, socket, TLS, HTTP, browser, EPI, Xenia, or consequential action authority.

Exact parent product subject:

```text
WEB-LOCATOR-001B0 / PR #2700
d61ebdd2097dd1986c27c4bf6b7311bb01f91a74
```

Exact parent admission corpus:

```text
WEB-TARGET-ADMISSION-TEST-001R / PR #2693
53ee3f818984fb110ef0ab697956f98fb4fbacc7
```

## Core theorem

```text
admission decision
!= execution handoff
!= audit receipt
!= cryptographic commitment
!= globally consumed attempt
```

A receipt is deliberately cloneable evidence data.
It MUST NOT be accepted by the future connector as a network capability.

```text
cloneable receipt
!= replayable network permission
```

## Why freeze canonical bytes before implementation

The receipt will eventually be consumed by multiple implementations:

- Mycelix Rust;
- Xenia detached attestation;
- independent qualification code;
- archive/export tooling;
- possibly SCITT/transparency adapters.

Therefore no implementation is allowed to define the transcript accidentally through:

- Rust struct layout;
- serde field order;
- JSON object ordering;
- platform endianness;
- `HashMap` iteration;
- debug/display formatting;
- implicit Unicode normalization;
- implicit IP text formatting.

The canonical transcript in this subject is the protocol authority.

## Protocol identifiers

```text
receipt schema:
mycelix:web-target-admission-receipt:v1

admission profile:
mycelix:ordinary-public-target-admission:v2

commitment profile:
mycelix:web-target-admission-commitment:sha256:v1

authority scope:
audit-evidence-only
```

## Binary framing

All integers use network/big-endian order.

A TLV field is:

```text
u16 tag
|| u32 value_length
|| value[value_length]
```

A list is:

```text
u16 item_count
|| repeated (
     u32 item_length
     || item[item_length]
   )
```

The transcript starts with the exact ASCII bytes:

```text
mycelix-web-target-admission-receipt-v1\0
```

followed by:

```text
u16 version = 1
```

and then top-level TLVs in strictly increasing tag order.

No unknown, duplicate, missing, or reordered top-level tag is canonical under V1.

## Top-level tags

| Tag | Meaning |
|---:|---|
| 1 | receipt schema ID |
| 2 | exact B0 product head |
| 3 | exact repaired admission-corpus head |
| 4 | admission profile ID |
| 5 | origin authority projection |
| 6 | optional initial-domain-policy evidence |
| 7 | ordered CNAME-domain-policy evidence list |
| 8 | resolution observation |
| 9 | ordered endpoint-policy assessment list |
| 10 | deterministic endpoint-set projection |
| 11 | acquisition attempt ID |
| 12 | admission disposition |
| 13 | authority scope |

Disposition V1:

```text
0x01 = AdmittedUnderProfile
```

B1 currently freezes positive receipts only.
A later refusal-receipt profile must use a new schema/version rather than
silently overloading this one.

## Evidence reference encoding

```text
tag 1 = profile ID bytes
tag 2 = exact subject reference bytes
```

No normalization is performed.

```text
same visible label
+ different subject bytes
!= same evidence reference
```

## Origin encoding

Origin is a nested TLV object:

```text
1 locator evidence reference
2 scheme
3 typed host
4 effective port
5 exact HTTP origin-form target bytes
6 userinfo-present bit
```

Scheme V1:

```text
0x01 = HTTPS
```

Host kinds:

```text
0x01 = DNS A-label name
0x02 = IPv4
0x03 = IPv6
```

DNS names are encoded as the exact normalized ASCII bytes supplied by the
admission model.

IPv4 and IPv6 are encoded as raw network octets, never text.

The HTTP target is committed exactly.

```text
/path?a=1&a=2
!= /path?a=2&a=1
```

and:

```text
committing exact request target
!= safe to print target in logs
```

## Optional initial domain policy

Tag 6 contains:

```text
0x00
```

for absence, or:

```text
0x01
|| u32 encoded_policy_length
|| encoded_policy
```

for presence.

This avoids using an empty object to mean two things.

## Domain-policy encoding

```text
1 evidence reference
2 exact normalized DNS name
3 state
```

State V1:

```text
0x01 = Eligible
0x02 = Refused
```

A positive receipt produced from B0 must contain only eligible policy records,
but the state remains encoded explicitly so the transcript is self-describing.

## Resolution encoding

Kind:

```text
0x01 = DirectIpLiteral
0x02 = ObservedDnsResolution
```

For an observed DNS resolution:

```text
1 kind
2 resolution evidence reference
3 exact resolution-lineage ID
4 exact query name
5 resolution status
6 ordered CNAME-name list
7 ordered endpoint-observation list
```

Resolution status values are frozen in the fixture generator and must not be
reordered by a Rust enum discriminant.

For a positive V2 admission the status is:

```text
0x01 = Complete
```

Resolver endpoint order is preserved because it is an observation.

```text
[A, B]
!= observed as [B, A]
```

for the full receipt transcript.

## IP encoding

```text
IPv4 = 0x04 || four raw octets
IPv6 = 0x06 || sixteen raw octets
```

Text representations are not canonical input.

Therefore:

```text
2001:4860:4860::8888
```

and any equivalent textual spelling encode to the same raw address bytes only
after an upstream qualified parser has established the address value.

## Endpoint-policy assessment encoding

```text
1 endpoint-policy evidence reference
2 typed raw IP address
3 endpoint-policy state
4 optional IPv6-allocation evidence reference
5 optional IPv6 envelope bytes
```

For IPv4, tags 4 and 5 have zero-length values.

For IPv6 under the current V2 profile they bind:

```text
mycelix:iana-ipv6-address-space:2025-10-23:v1
2000::/3
```

as already required by B0.

## Ordered endpoint observation vs finite-set identity

The receipt binds two different concepts.

### Observed order

The resolution and assessment fields preserve exact endpoint observation order.

### Set projection

Top-level tag 10 contains:

```text
unique canonical IP encodings
-> lexicographic byte sort
-> canonical list framing
```

This means two otherwise identical receipts with:

```text
[A, B]
```

versus:

```text
[B, A]
```

have:

```text
different full transcript
different full admission commitment
same endpoint-set projection
```

The fixture includes this exact metamorphic pair.

This distinction prevents either mistake:

```text
resolver ordering == endpoint-set identity
```

or:

```text
sorting endpoint set may erase the observed resolver order
```

## Commitment profile

The full canonical transcript is not itself the digest input without framing.

The V1 commitment is exactly:

```text
SHA-256(
  ASCII("mycelix-web-target-admission-commitment-sha256-v1\0")
  || u64be(canonical_transcript_length)
  || canonical_transcript
)
```

The length binding prevents concatenation ambiguity and the commitment-specific
domain separator prevents cross-protocol hash reuse.

```text
same SHA-256 primitive
!= same protocol commitment
```

## Cryptographic authority ceiling

A matching commitment proves only:

```text
same canonical receipt bytes under this profile
```

It does NOT prove:

```text
B0 admission logic was correct
upstream parser was qualified
DNS was authentic
endpoint was reachable
TLS identity was valid
source was authentic
content was true
attempt was globally unique
attempt was consumed
```

Later layering remains:

```text
Mycelix canonical receipt
        ↓
SHA-256 admission commitment
        ↓
optional Xenia detached attestation
        ↓
optional transparency / SCITT registration
```

and:

```text
signature verifies
!= admission correct
!= source truthful
```

## Privacy boundary

Canonical transcripts deliberately bind sensitive exact data such as:

- target path/query;
- evidence subject references;
- endpoint addresses;
- attempt IDs;
- resolution lineage.

Therefore:

```text
canonical transcript
!= safe ordinary log representation
```

Implementations MUST provide redacted ordinary `Debug`/display behavior.

The SHA-256 commitment is also **not a secrecy primitive**. Low-entropy fields
can be guessed offline. Private investigation capsules may later require
encryption/access-control around the receipt.

## Frozen vectors

`WEB_TARGET_ADMISSION_RECEIPT_001_V0_1.json` contains four vectors:

1. one DNS origin with one IPv4 endpoint;
2. dual-stack DNS observation in order `[IPv4, IPv6]`;
3. the exact same semantic evidence in order `[IPv6, IPv4]`;
4. one direct IPv6 origin.

The order-mutation pair differs only in ordered endpoint observation/assessment
order.

Expected relation:

```text
full_commitment(order A)
!= full_commitment(order B)

endpoint_set_projection(order A)
== endpoint_set_projection(order B)
```

## Golden commitment values

```text
domain-v4-single
5ddc613701686b553aba946159cea1a3d87363d297bec9ddce7b8fe081bb702e

domain-dual-order-a
4b34ca1819ae4b1fa08436c74097f06db75289ed1af6a7687134030581529d1d

domain-dual-order-b
b6c4b4365dc48cf5246c7f4e03824fa3feeffa4b1177c94c6e143f74e26800d3

direct-ipv6
3cf8e709be7c75e3f529a1e2054f6667669521fcccbe8d74222a27240f3e04b0
```

These values were independently generated from the written canonical protocol.
A Rust implementation must reproduce them exactly rather than generating new
"expected" values from itself.

## Required implementation properties

A future B1 source implementation must:

1. produce byte-for-byte identical transcripts for all frozen vectors;
2. produce the exact SHA-256 values above;
3. keep observed endpoint ordering and set identity separate;
4. bind the exact B0 head and repaired admission-corpus head;
5. retain every policy/evidence reference used by positive admission;
6. expose a cloneable audit receipt but no network entrypoint;
7. avoid serde/JSON ordering as canonicalization authority;
8. avoid platform-dependent integer or IP formatting;
9. reject impossible/ambiguous transcript construction;
10. provide redacted ordinary diagnostics;
11. keep the receipt distinct from `ConnectorHandoffV2`;
12. make Xenia attestation an optional later layer.

## Qualification requirements

A never-merge qualifier should independently reconstruct every transcript in a
second implementation, preferably Python or another non-Rust implementation,
and verify:

```text
exact bytes
exact transcript length
exact endpoint-set projection
exact SHA-256 commitment
mutation sensitivity
redaction behavior
dependency/API authority ceiling
```

A Rust implementation matching authored Rust tests alone is insufficient.

## Nonclaims

This subject establishes no runtime PASS and no network authority.

```text
frozen vector
!= implementation executed
!= implementation qualified
!= admission correct
!= network authorized
!= source true
```
