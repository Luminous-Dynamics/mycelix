# mycelix-content-core

Pure Rust vocabulary for Mycelix Content Fabric CF-02.

The crate separates:

`Blob bytes -> ObjectManifest -> PublicationRecord -> StorageIntent`

It intentionally does not contain provider discovery, Holochain records, Iroh/HTTP transport, planner authority, marketplace terms, or billing.

## Design rules

- Blob digests are algorithm-tagged; v1 supports BLAKE3-256 and SHA-256.
- A blob digest identifies exact bytes only.
- Cross-protocol digest/semantic mappings are not part of immutable blob identity in v1.
- Object manifests are ordered compositions and validate stored totals and IDs after deserialization.
- Publisher/provenance metadata is separate from object identity, preserving byte deduplication across publishers.
- Hard placement requirements are distinct from optimization preferences.
- Jurisdiction allow/deny rules are hard constraints.
- Failure-domain requirements are explicit per dimension (operator, machine, site, ASN, region, jurisdiction, power domain) rather than a vague replica score.
- `PrivateBackup` requires client-side encryption.

This crate depends only on low-level cryptographic/serialization libraries and CF-01 infrastructure primitives.
