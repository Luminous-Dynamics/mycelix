# mycelix-nix-remote-http

`mycelix-nix-remote-http` is the CF-07B public-audience HTTP router core for Content Fabric Nix delivery.

It is deliberately **not a network listener**. The crate returns an Axum `Router` and has no bind address, TLS configuration, socket API, principal/group authentication, grant signer, raw exposure catalog, or mutable storage surface.

## Required input

The router accepts only a CF-07A `RemoteServingSnapshotV1`:

```text
CF-07 verified CAS
        +
CF-07A RemoteServingSnapshotV1
        +
trusted server-side request clock
        |
        v
CF-07B public_router(...)
        |
        v
Axum Router
```

It cannot be constructed from:

- `NixCacheCatalogV1`;
- `RemoteExposureSnapshotV1`; or
- a bind address.

This preserves the distinction between local cache exposure, diagnostic policy state, and strict time-bounded remote publication authority.

## Public-only audience

CF-07B v0.1 evaluates every request as `RemoteReaderV1::anonymous()`.

Therefore:

- `Public` exposure may be served;
- `AuthenticatedPrincipal` exposure remains invisible; and
- `AuthenticatedGroup` exposure remains invisible.

Restricted-but-existing and absent entries both return `404` before any CAS lookup. A later authenticated adapter must be a separate reviewed tranche that derives reader identity from an actually authenticated transport such as Xenia/mTLS; CF-07B does not provide a generic caller-asserted identity resolver.

## Time-bounded authority

Every request obtains server-controlled Unix time through `RemoteClockV1`.

The clock contract explicitly forbids deriving authorization time from untrusted request headers or query parameters.

The router also keeps a process-local atomic high-watermark of the greatest Unix timestamp observed. If a later clock reading is lower than that watermark, the request fails closed. Thus an NTP/manual wall-clock rollback cannot resurrect an already-observed expired serving snapshot within the lifetime of a router instance.

This does not create trusted time across process restarts. Startup time remains part of the host/deployment trust boundary and must be qualified separately before a real remote listener is enabled.

Content requests use CF-07A's time-bearing lookup before storage access. NAR and `.narinfo` requests then perform full CF-03 verification and check the clock **again immediately before response commitment**.

This closes the verification TOCTOU window:

```text
authorized before verification
        +
large/slow full-file hash
        +
authority expires during hashing
        x
response starts after expiry
```

If authority is stale, not yet valid, the clock regresses/fails, or the snapshot otherwise needs refresh, the router returns:

```text
503 Service Unavailable
Retry-After: 1
Cache-Control: no-store
```

The serving deadline is a request/response-admission boundary. CF-07B does not attempt to claw back bytes from an HTTP response that was already admitted while authority was valid.

## Authorization before storage

For `.narinfo` and NAR paths:

```text
parse store hash
      |
      v
RemoteServingSnapshotV1 + anonymous reader + trusted current time
      |
      +-- not authorized / absent --> 404
      |
      v
CF-03 verified CAS open
      |
      v
recheck trusted serving authority
      |
      v
response
```

Thus restricted content does not trigger a CAS existence check through this public router.

## Verified bytes

Authorized NAR reads use CF-03 `open_verified_digest()` through bounded `spawn_blocking` admission.

CF-07B:

- re-hashes the complete SHA-256 NAR before response commitment;
- checks the opened file size against `NarSize`;
- streams the same verified/rewound file handle; and
- never reconstructs or reopens a filesystem path after verification.

`.narinfo` is also withheld unless its referenced NAR verifies locally at request time. If the exposure is authorized but the local NAR is unavailable, the router returns `503` rather than falsely representing the store path as absent.

## HTTP cache semantics

Every CF-07B response is `Cache-Control: no-store`, including:

- successful NARs;
- successful `.narinfo`;
- `nix-cache-info`;
- authorization/absence `404` responses;
- stale-authority/storage `503` responses;
- integrity `500` responses;
- unsupported-method `405` responses; and
- unmatched-route `404` responses.

This differs intentionally from loopback CF-07's immutable-cache headers. A shared HTTP intermediary must not silently extend time-bounded remote publication authority beyond `RemoteServingSnapshotV1::serve_until_unix_ms()`.

The NAR remains immutable and still carries a strong digest-derived ETag; `no-store` is about remote publication authority, not content mutability.

## Routes

CF-07B mirrors the stock-Nix read surface:

```text
GET/HEAD /nix-cache-info
GET/HEAD /<store-hash>.narinfo
GET/HEAD /nar/<store-hash>-<nar-sha256>.nar
```

HEAD preserves representation `Content-Length` while returning no body. Mutation methods are not registered.

## Error semantics

- unauthorized or absent object -> `404`;
- malformed path/hash -> `404`;
- stale/not-yet-valid serving snapshot -> `503` + `Retry-After: 1`;
- trusted clock failure or rollback -> `503` + `Retry-After: 1`;
- verification admission saturated -> `503` + `Retry-After: 1`;
- local storage unavailable -> `503` + `Retry-After: 1`;
- local immutable/digest integrity failure -> `500`;
- unsupported method -> `405`;
- all responses -> `Cache-Control: no-store`.

## Deferred

CF-07B does not implement:

- a listener or bind address;
- TLS/mTLS termination;
- Xenia authentication;
- authenticated principal/group readers;
- DNS/certificate policy;
- persistent/cross-restart secure-time watermarking;
- grant or revocation verification/minting;
- dynamic serving-snapshot refresh;
- remote fill;
- eviction/deletion;
- lease, settlement, or payment authority.

Those boundaries should remain separate so adding network reachability cannot silently add identity, publication, storage, or software-trust authority.
