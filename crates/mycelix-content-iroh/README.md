# mycelix-content-iroh

CF-04 transport adapter for Mycelix Content Fabric.

This crate uses **Iroh as transport only**. It does not replace Content Fabric
identity with an Iroh-specific hash, does not create leases, and does not make
transport peers software trust roots.

## v1 protocol

ALPN: `mycelix/content/1`

The provider supports a single operation: fetch one complete immutable blob by
`ContentDigestV1`.

Whole-blob transfer is deliberate. A full BLAKE3-256 or SHA-256 digest can be
verified uniformly by every client. Arbitrary remote byte ranges are not exposed
until Content Fabric has a digest-algorithm-neutral authenticated range-proof
contract.

## Provider path

1. Parse the fixed-width request frame.
2. Shed excess work with a bounded transfer semaphore.
3. Ask CF-03 to `open_verified_digest()`.
4. Send the fixed response header.
5. Stream the exact verified file handle.
6. Finish the QUIC stream and wait only a bounded interval for acknowledgement.

The provider never sends bytes from an unverified CAS path.

## Client path

1. Require an expected `BlobDescriptorV1`.
2. Enforce a configured maximum blob size before connecting.
3. Request the algorithm-tagged digest.
4. Require the provider's declared size to match the expected descriptor.
5. Receive into a temporary file while hashing.
6. Require exact payload length and EOF.
7. Compare the computed digest with the expected digest.
8. Return `VerifiedDownloadV1` only after complete verification.

`VerifiedDownloadV1::import_into_cas()` deliberately calls CF-03 `put()` instead
of bypassing the CAS promotion path. v1 therefore pays an extra local read/copy
in exchange for preserving quota, fsync, atomic-promotion, and digest invariants.

## Non-goals

- transport-defined global identity
- remote byte-range delivery
- multipart/range proofs
- provider discovery
- Holochain advertisements
- lease creation or settlement
- cache eviction/deletion
- authenticated administrative mutation

Those belong to later Content Fabric layers.
