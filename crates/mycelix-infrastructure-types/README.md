# mycelix-infrastructure-types

Transport-neutral v1 envelopes for infrastructure services.

The crate deliberately models a small lifecycle:

`Capability -> Offering -> Lease -> Receipt`

It is designed to be reused by storage, bandwidth, relay, compute, GPU, and edge-cache services without coupling those domains to Holochain, Marketplace, Finance, Iroh, HTTP, or an async runtime.

## Invariants

- Stable IDs are BLAKE3 domain-separated and versioned.
- Generic payloads enter envelope IDs only through an explicit schema-tagged commitment to canonical bytes.
- Resource dimensions are namespaced tokens stored in a deterministically ordered vector.
- Offers cannot exceed capabilities.
- Leases cannot exceed or outlive offers.
- Receipts cannot claim more than the lease and must identify the same provider/consumer.
- Every child explicitly references its parent ID.
- Mutation of identity-bearing fields changes the recomputed ID.

This crate implements the CF-01 contract described in `docs/content-fabric/`.
