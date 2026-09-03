# Mycelix Content Fabric — Iroh Transport v1

Status: draft implementation contract for CF-04.

## Purpose

CF-04 transports already-defined Content Fabric blobs over Iroh. It MUST NOT
change global content identity, storage policy, publication provenance, lease
authority, or software-admission authority.

ALPN is the exact byte string:

`mycelix/content/1`

## Request frame

Every request is exactly 44 bytes:

| Offset | Length | Field |
| --- | ---: | --- |
| 0 | 8 | ASCII magic `MYCFGET1` |
| 8 | 1 | schema version = `1` |
| 9 | 1 | operation = `1` (`GET`) |
| 10 | 1 | digest algorithm (`1` = BLAKE3-256, `2` = SHA-256) |
| 11 | 1 | flags/reserved = `0` |
| 12 | 32 | digest bytes |

Unknown versions, operations, algorithms, or non-zero reserved fields MUST fail
closed.

The numeric digest tags are wire values; they are not derived from Rust enum
ordering.

## Response header

Every response starts with exactly 24 bytes:

| Offset | Length | Field |
| --- | ---: | --- |
| 0 | 8 | ASCII magic `MYCFRES1` |
| 8 | 1 | schema version = `1` |
| 9 | 1 | status |
| 10 | 6 | reserved = all zero |
| 16 | 8 | payload size, unsigned big-endian |

Statuses:

- `0` — OK
- `1` — NOT_FOUND
- `2` — BUSY
- `3` — INTEGRITY_FAILURE
- `4` — PROTOCOL_ERROR
- `5` — INTERNAL_ERROR

For every non-OK status, payload size MUST be zero and no content payload follows.
For OK, exactly `payload size` bytes follow and the stream then ends.
Trailing bytes are a protocol violation.

## Integrity

A provider MUST fully verify the requested CAS blob before sending the response
payload.

A client MUST know the expected `BlobDescriptorV1` before accepting a transfer.
It MUST reject:

- a response size different from the expected size;
- a payload shorter or longer than the declared size;
- trailing payload bytes;
- a final digest different from the expected algorithm-tagged digest.

The transport-level Iroh endpoint identity identifies the peer connection. It is
not content integrity and is not sufficient to authorize installation or trust
software.

## Why v1 has no remote ranges

Content Fabric supports BLAKE3-256 and SHA-256 exact-byte identities. Neither the
v1 object model nor this transport defines a digest-algorithm-neutral Merkle proof
for arbitrary byte ranges.

Therefore CF-04 v1 only transfers complete blobs. Local adapters may serve ranges
from a blob after the local CAS has verified the complete object.

A later range protocol MUST define authenticated range proofs explicitly rather
than treating a transport implementation's chunk tree as global object identity.

## Flow control and resource bounds

Providers MUST cap concurrent verified transfers and SHOULD shed excess demand
rather than permit an unbounded waiter queue.

Clients MUST configure a non-zero maximum blob size and MUST reject responses over
that bound.

Temporary download directories MUST be caller-selected real directories; symlink
directories are rejected by the reference Rust adapter.

## CAS handoff

The reference client returns a verified temporary download. Importing that file
into CF-03 uses the ordinary `LocalCasV1::put()` path.

This intentionally re-verifies and copies the file in v1. A future optimized
handoff MAY remove the duplicate I/O only if it preserves the same reservation,
fsync, immutable-file, digest-verification, and atomic-promotion invariants.
