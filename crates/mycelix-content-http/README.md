# mycelix-content-http

CF-03A read-only HTTP adapter for **Mycelix Content Fabric**.

This crate sits above `mycelix-content-node`. It translates conventional local HTTP reads into the CAS' verified digest-open operation without gaining storage mutation or deletion authority.

## v0.1 boundary

The server is **loopback-only**. `HttpConfigV1::validate()` rejects every non-loopback bind address rather than allowing an accidental `0.0.0.0` exposure.

Exposed routes:

```text
GET  /v1/health
GET  /v1/capacity
GET  /v1/blobs/:algorithm/:digest
HEAD /v1/blobs/:algorithm/:digest
```

There is deliberately no HTTP `PUT`, `POST`, `PATCH`, or `DELETE` storage route in this tranche.

## Blob reads

For every GET or HEAD, the adapter:

1. validates the algorithm tag and canonical lowercase 64-hex digest;
2. obtains a bounded verification slot;
3. runs the synchronous CAS verification on a blocking worker;
4. receives the exact file handle that the CAS re-hashed and rewound;
5. streams that verified handle directly to the client.

The adapter never reconstructs a raw filesystem path and never reopens a blob after the CAS has verified it.

## HTTP behavior

Full immutable responses include:

- `Content-Length`
- `Accept-Ranges: bytes`
- strong content-address-derived `ETag`
- `Cache-Control: public, max-age=31536000, immutable`
- `Content-Type: application/octet-stream`
- `X-Content-Type-Options: nosniff`

GET supports one RFC-style byte range in v0.1:

- closed: `bytes=10-99`
- open-ended: `bytes=10-`
- suffix: `bytes=-90`

Multi-range requests are intentionally rejected with `416` rather than silently implementing multipart semantics incorrectly. Unsatisfiable responses include `Content-Range: bytes */<size>`.

## Concurrency

Full-file verification is intentionally completed before any response body is sent. A semaphore bounds concurrent blocking verification work so a local client cannot create unlimited hashing jobs.

## Authority boundary

This adapter may:

- verify/read immutable blobs;
- report CAS capacity;
- report basic health.

It may not:

- ingest content;
- remove or evict content;
- create leases;
- advertise replicas;
- pay providers;
- change placement policy;
- expose the service beyond loopback in v0.1.

Network-visible serving and authenticated mutation require their own explicit authority design rather than a command-line flag on this local facade.

## Validation

```bash
cargo fmt --manifest-path crates/mycelix-content-http/Cargo.toml -- --check
cargo clippy --manifest-path crates/mycelix-content-http/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path crates/mycelix-content-http/Cargo.toml --all-targets
```
