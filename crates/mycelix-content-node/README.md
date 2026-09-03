# mycelix-content-node

CF-03 local storage engine for **Mycelix Content Fabric**.

This crate is intentionally narrower than a network node. It establishes the crash-safe content-addressed storage boundary that later HTTP, Iroh, Nix, OCI, and Holochain adapters must use.

## Guarantees in v0.1

- exact-byte BLAKE3-256 and SHA-256 verification using `mycelix-content-core` identities
- quota reservation before ingest
- sender input bounded to expected size + 1 byte
- same-filesystem staging
- file `sync_all()` before promotion
- read-only immutable final files
- no-clobber promotion into digest-derived paths
- final-directory and staging-directory fsync after promotion on Unix
- visible bytes are charged to quota immediately after successful promotion, even if a later directory fsync reports an error
- single-process ownership of a CAS root through an exclusive lock
- interrupted staging files are removed at startup
- startup rejects symlinks, writable immutable files, malformed digest names, and unexpected tree entries
- verified reads return the same file handle that was hashed and rewound
- full-store audit re-hashes every stored blob
- quota usage is reconstructed from immutable files after restart

## Deliberately absent

CF-03 does **not** provide:

- HTTP endpoints
- Iroh/QUIC transport
- Holochain coordination
- Marketplace or Finance behavior
- Symthaea placement authority
- eviction or deletion APIs

Those are separate authority and protocol layers. In particular, deletion/eviction is not added until the durability/placement authorization model can distinguish expendable cache replicas from durable data.

## Ingest contract

The caller supplies a `BlobDescriptorV1` and a `Read` stream. The CAS:

1. checks for an already-present, independently verified immutable blob;
2. reserves the declared byte size against quota;
3. streams into a temporary file under `staging/` while hashing;
4. reads at most the declared length plus one byte;
5. rejects truncation, oversize streams, or digest mismatch;
6. fsyncs the staged bytes;
7. marks the file read-only;
8. promotes without clobbering an existing digest path;
9. moves the reserved bytes into used accounting immediately;
10. fsyncs the final algorithm directory and staging directory on Unix.

A failed ingest cannot make a partial blob addressable.

## Validation

Run:

```bash
cargo fmt --manifest-path crates/mycelix-content-node/Cargo.toml -- --check
cargo clippy --manifest-path crates/mycelix-content-node/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path crates/mycelix-content-node/Cargo.toml --all-targets
```
