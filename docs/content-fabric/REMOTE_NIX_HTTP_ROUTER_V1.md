# Mycelix Content Fabric — Remote Nix HTTP Router v0.1

Status: CF-07B executable draft.

## Purpose

CF-07B is the first HTTP adapter allowed to consume CF-07A's strict remote-serving authority artifact.

It deliberately stops **before** network binding and authenticated principal/group serving.

The central rule is:

> Remote HTTP delivery may consume strict time-bounded authority, but it may not manufacture authority, identity, or network exposure.

## Dependency boundary

```text
CF-03 verified CAS
        |
        +-------------------+
                            |
CF-07 local Nix metadata    |
        |                   |
        v                   |
CF-07A strict remote exposure projection
        |
        v
RemoteServingSnapshotV1
        +
trusted server-side clock
        |
        v
CF-07B public_router(...)
        |
        v
Axum Router only
```

The router constructor MUST NOT accept:

- `NixCacheCatalogV1`;
- `RemoteExposureSnapshotV1`;
- raw grant/revocation evidence;
- an asserted principal/group identity; or
- a socket/bind address.

## Public-audience-only v0.1

Every content request is evaluated as `RemoteReaderV1::anonymous()`.

Consequently:

```text
Public                        -> eligible for serving
AuthenticatedPrincipal(...)   -> externally indistinguishable from absent
AuthenticatedGroup(...)       -> externally indistinguishable from absent
```

This is intentional. A generic caller-supplied identity resolver would create a path for unauthenticated code to claim authenticated reader identity.

Authenticated reader support must arrive behind a separate adapter that derives principal/group facts from a reviewed authenticated transport boundary.

## Trusted request time

The router requires `Arc<dyn RemoteClockV1>` and asks it on every request.

The clock is authority-bearing infrastructure. Implementations MUST derive time from server-controlled state and MUST NOT consume client-provided time as authorization truth.

Clock failure is fail-closed.

### Process-local monotonic watermark

Wall clocks can move backward. CF-07B therefore records the greatest Unix timestamp observed by a router instance in a shared atomic high-watermark.

For every clock read:

```text
now = trusted clock
previous = greatest timestamp already observed

now < previous  -> fail closed
now >= previous -> advance/retain watermark and continue
```

This prevents an already-observed expired serving snapshot from becoming valid again because NTP/manual correction moved the process wall clock backward.

This watermark is process-local. A restart loses its in-memory history, so trustworthy startup/boot time remains a deployment prerequisite for a future real listener. Cross-restart secure-time persistence is explicitly outside CF-07B v0.1.

## Request admission and verification TOCTOU

For NAR and `.narinfo` reads, CF-07B performs two time checks:

```text
current strict serving snapshot?
        |
        v
entry authorized for anonymous reader?
        |
        v
full CF-03 verified CAS re-hash
        |
        v
trusted clock still monotonic + serving snapshot still valid?
        |
        v
commit HTTP response
```

The second check is required because full-file verification may be non-trivial for large NARs. A request that was valid before verification MUST NOT begin its response after `serve_until_unix_ms`.

CF-07B treats the serving horizon as **response-admission authority**. It does not attempt to terminate a response body that was already admitted while authority was valid. Mid-transfer revocation cannot undo bytes already disclosed and forcibly truncating a valid NAR creates a different failure mode; later transport-specific designs may add stronger interruption semantics if justified.

## Authorization before CAS lookup

Remote authorization MUST precede local storage lookup.

Restricted-but-existing and absent entries both return `404` without consulting CAS existence.

This follows the same existence-privacy principle as CF-04.

Only an authorized public entry may enter the verified storage path.

If an authorized publication references a NAR that is locally unavailable, the result is `503`, not `404`: remote exposure exists but the replica is unavailable.

## Verified-on-read contract

For authorized content, CF-07B preserves the CF-03/CF-07 verification contract:

- bounded verification concurrency;
- fail-fast saturation rather than an unbounded waiter queue;
- synchronous full-file hashing occurs in `spawn_blocking`;
- `open_verified_digest()` supplies the verified/rewound file handle;
- opened-file size must equal the publication `NarSize`;
- the exact verified handle is streamed; and
- `.narinfo` is not emitted if its NAR cannot be verified locally.

CAS integrity failures remain distinct from ordinary storage unavailability.

## HTTP response surface

Routes:

```text
GET/HEAD /nix-cache-info
GET/HEAD /<store-hash>.narinfo
GET/HEAD /nar/<store-hash>-<nar-sha256>.nar
```

HEAD preserves representation metadata such as `Content-Length` while returning an empty body.

No mutation method is registered.

### Status model

- malformed path/hash -> `404`;
- unauthorized public-reader view -> `404`;
- absent entry -> `404`;
- stale/not-yet-valid serving authority -> `503`;
- trusted clock failure or rollback -> `503`;
- verification admission saturation -> `503`;
- authorized publication with local storage unavailable -> `503`;
- local immutable/digest integrity failure -> `500`;
- unsupported method -> `405`.

Retryable `503` responses include `Retry-After: 1`.

## No intermediary extension of authority

All CF-07B responses MUST carry:

```text
Cache-Control: no-store
```

This includes success, `404`, `405`, `500`, and `503` responses, including router/method fallbacks.

Loopback CF-07 may use long-lived immutable caching because its authority is local exposure. CF-07B must not copy those headers: a reverse proxy/shared cache retaining an object beyond the serving horizon would become an implicit publication authority outside CF-07A.

`no-store` does not claim that bytes already delivered to a public client can later be revoked. It prevents CF-07B from delegating future serving to an intermediary cache without an explicit authority design.

## Nix trust remains independent

CF-07B renders the same Nix publication metadata admitted by CF-07 and never signs software.

Existing `Sig:` and `CA:` facts remain part of the exact CF-07A exposure object commitment. Nix clients retain their normal software trust decision.

Remote publication authority and software-install trust are therefore distinct:

```text
CF-07A/07B says: may these bytes/metadata be remotely disclosed now?

Nix says: may this store object be accepted as trusted software?
```

## No listener in CF-07B

The public API returns an Axum `Router` only.

There is no:

- `serve()`;
- `TcpListener`;
- `SocketAddr`;
- `bind_addr`; or
- TLS configuration.

A later listener adapter must make network reachability explicit and separately review DNS/certificate/listener policy.

## Tests required for v0.1

The CF-07B test surface pins:

- public NAR + `.narinfo` before deadline;
- HEAD representation lengths with empty bodies;
- restricted-existing and absent both return `404`;
- wrong NAR URL hash returns `404`;
- stale serving snapshot returns `503` before storage access;
- crossing the deadline and then rolling the wall clock backward does not resurrect authority;
- trusted clock failure returns `503`;
- authority expiry during full verification returns `503` before response commitment;
- an authorized publication with missing local NAR returns `503`;
- mutation methods remain `405`;
- unmatched paths remain `404`;
- success and error/fallback responses are `Cache-Control: no-store`; and
- invalid verification concurrency fails closed.

## Deferred

- non-loopback listener;
- TLS/mTLS termination;
- Xenia identity extraction;
- authenticated principal/group routing;
- DNS/certificate policy;
- persistent/cross-restart secure-time watermarking;
- dynamic serving-snapshot refresh;
- connection-level revocation/stream interruption;
- remote fill;
- eviction/deletion;
- Holochain authority evidence collection;
- settlement/lease/payment coupling.
