# Mycelix Content Fabric v1 Architecture Contract

Status: frozen design contract for CF-00. This document defines boundaries for the first implementation tranche; it does not grant runtime authority or claim production readiness.

## Goal

Build one content fabric that can support edge caching, Nix binary-cache delivery, OCI distribution, durable storage and later private backup without coupling global object identity to any one transport, marketplace, Holochain schema, or optimizer.

## Layering

Bytes -> BlobDigest -> ObjectManifest -> PublicationRecord -> StorageIntent -> PlacementProposal -> PlacementLease -> ReplicaObservation -> ServiceReceipt

Each transition adds semantics without redefining immutable byte identity.

### Byte identity

A blob is immutable bytes identified by an algorithm-tagged digest. v1 requires algorithm agility; BLAKE3-256 and SHA-256 are the initial algorithms. A blob may carry verified digest aliases when the same byte sequence must interoperate with different ecosystems.

### Object composition

ObjectManifest composes one or more blobs and describes object-level media information. It does not contain publisher identity, placement policy, commercial terms, or transport chunk graphs.

### Publication and provenance

PublicationRecord binds an object manifest to logical naming, versioning and publisher provenance. Identical bytes may therefore deduplicate while provenance remains distinct.

### Storage intent and placement

StorageIntent expresses required availability and policy. Hard requirements are filtered before any soft optimization. PlacementProposal is advisory. PlacementLease is the authorized reservation/contract. ReplicaObservation records observed state. ServiceReceipt records delivered service.

## Data plane / coordination plane split

Bulk bytes MUST remain outside the Holochain DHT. Holochain coordinates provider capabilities, offerings, advertisements, leases, observations and receipts. Storage/edge nodes hold bytes and serve them directly.

The transport layer is an adapter. HTTP/local transport and Iroh may be used without becoming permanent Mycelix identity formats. Transport-specific chunking MUST NOT define global object identity.

## Primary components

- `mycelix-infrastructure-types`: generic Capability -> Offering -> Lease -> Receipt envelopes.
- `mycelix-content-core`: pure Rust content vocabulary; no Holochain, Iroh, async runtime or marketplace dependency.
- `mycelix-content-node`: local CAS, verified ingest/retrieval, ranges, quota, eviction, health and metrics.
- Content Fabric hApp: coordination only.
- Symthaea Content Planner: deterministic/advisory planning first; no direct authority.
- Nix adapter: normal HTTP binary-cache facade backed by the fabric.
- Later: durability controller, Marketplace/Finance integration, OCI facade, private backup, erasure coding.

## v0.1 implementation milestone: Edge Seed

The first usable milestone is CF-00 through CF-07:

1. architecture/threat/wire/authority freeze;
2. infrastructure envelopes;
3. pure content types;
4. local verified CAS;
5. transport adapter with Iroh + HTTP/local fallback;
6. Holochain coordination;
7. authority-free Symthaea planner;
8. standard Nix binary-cache facade.

Exit evidence should include algorithm-agile immutable identity, object manifests, hard placement policy, verified local CAS, HTTP range retrieval, provider advertisement, deterministic placement proposals, three-node replication, preserved Nix trust/admission, failure/recovery scenarios and reproducible deployment.

## Interoperability rule

Existing clients should not need to understand Mycelix when a standard facade exists. Nix should see a binary cache. OCI clients should later see a normal `/v2/` registry API. Content integrity remains independently verifiable by clients.

## Non-goals for the first milestone

- universal application-level chunk graphs;
- decentralized S3 compatibility;
- erasure-coded archival storage;
- proof-of-storage cryptography;
- payment/settlement as a prerequisite for technical delivery;
- direct optimizer enforcement;
- treating cache/provider identity as a software trust root.
