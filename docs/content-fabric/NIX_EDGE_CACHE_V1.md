# Mycelix Content Fabric — Nix Edge Cache v0.1 Contract

Status: CF-07 executable draft, refining the CF-00 design contract.

## Adoption principle

Do not modify Nix. Expose an ordinary HTTP binary cache backed by Content Fabric. A normal Nix client should not need to know that Mycelix exists.

The cache is a delivery facade, not a software trust root, build authority, signing authority, placement authority, lease authority, or deletion authority.

```text
CF-03 CAS
  verifies exact stored/downloaded bytes
        |
        v
CF-07 Nix facade
  publishes only explicitly admitted Nix metadata + raw NARs
        |
        v
Nix client
  applies store-path/content-address/signature trust policy
```

Control of an edge cache therefore does not by itself authorize arbitrary software for installation.

## v0.1 HTTP surface

CF-07 serves only:

- `GET/HEAD /nix-cache-info`
- `GET/HEAD /<store-hash>.narinfo`
- `GET/HEAD /nar/<store-hash>-<nar-sha256>.nar`

No PUT, POST, PATCH, DELETE, eviction, signing, build, lease, settlement, Holochain-write, or planner endpoint exists.

`nix-cache-info` advertises:

```text
StoreDir: /nix/store
WantMassQuery: 0
Priority: <configured priority>
```

`WantMassQuery` is false because CF-07 v0.1 does not implement a bulk validity-query endpoint.

## Explicit exposure catalog

A blob being present in CF-03 is insufficient to expose it through Nix routes.

Every reachable Nix store object must have an explicit `NixCacheEntryV1` in `NixCacheCatalogV1`. The facade never derives this catalog by enumerating CAS paths.

The catalog is a **local exposure configuration**, not cryptographic or governance proof that the content is globally public or authorized for disclosure. CF-07 v0.1 deliberately does not mint a publication permit, inspect Holochain policy, or claim that catalog membership establishes public-content status.

This distinction is contained in v0.1 by the loopback-only server. Any future non-loopback/LAN/public adapter MUST place a separate publication/read-authorization boundary in front of catalog exposure and MUST NOT reinterpret `NixCacheCatalogV1` membership as authorization evidence.

This prevents the Nix facade from becoming a digest-enumeration surface for private, restricted, or unrelated Content Fabric objects while avoiding a new hidden authority plane inside the compatibility crate.

## Raw NAR profile

CF-07 v0.1 deliberately supports one NAR transport profile:

- raw/uncompressed NAR;
- SHA-256 exact-byte CAS identity;
- `Compression: none`;
- `FileHash == NarHash`;
- `FileSize == NarSize`.

The Nix hash text is the canonical Nix-base32 encoding of the exact 32 SHA-256 digest bytes.

The cache-local URL is:

```text
nar/<store-hash>-<nix-base32-sha256>.nar
```

Transport filename and compression are not global Content Fabric object identity.

## Verified-on-read advertisement

CF-07 must call CF-03 `open_verified_digest()` before returning either:

- `.narinfo`; or
- NAR bytes.

The verified file size must equal the admitted `NarSize`.

Consequences:

- unknown catalog entry -> `404`;
- known entry whose local bytes are missing/unavailable -> `503`;
- known entry whose immutable bytes fail verification -> `500`;
- verification-slot saturation -> `503` with `Retry-After: 1`.

A catalog record must not cause a broken local replica to be advertised as a valid cache hit.

## Nix signatures and install trust

CF-07 accepts no cache private key and exposes no signing API.

An admitted entry may contain zero or more existing Nix `Sig:` values. CF-07:

- validates them only enough to prevent malformed or line-injection output;
- sorts/deduplicates them for deterministic rendering;
- re-emits them;
- does not claim that a signature is trusted or cryptographically valid.

Nix clients retain their ordinary `trusted-public-keys`, `require-sigs`, content-address, and store-trust decisions.

Operators MUST NOT configure a CF-07 endpoint as `trusted=true` merely because it is a Mycelix node. Cache reachability is not software authority.

`Who delivered these bytes?` and `Am I authorized to install these bytes?` remain distinct questions.

## Store metadata

`NixCacheEntryV1` binds:

- full `/nix/store/...` path;
- 32-character store hash;
- exact SHA-256 NAR digest;
- exact NAR size;
- canonical references;
- optional deriver;
- zero or more existing signatures;
- optional content-address field.

References are rendered as store-path base names. Missing deriver renders as `unknown-deriver`.

CF-07 does not recompute arbitrary derivation/store-path semantics from build inputs. The Nix client remains responsible for interpreting `.narinfo` as Nix metadata and enforcing its own trust rules.

## Stock Nix 2.35.2 qualification

HTTP-level tests are not sufficient to claim Nix compatibility. CF-07 therefore has a separate stock-client qualification workflow using upstream Nix 2.35.2.

The fixture MUST be an ordinary **input-addressed** derivation output rather than a content-addressed object, because the qualification is intended to exercise Nix signature authority rather than a content-addressed signature exemption.

The qualification fixture MUST contain no runtime store references. This keeps the proof focused on one object and makes the exact signature fingerprint depend on:

- the input-addressed store path;
- the NAR hash;
- the NAR size; and
- an empty reference set.

Nix itself performs the authoritative fixture operations:

1. build the store object;
2. generate an Ed25519 binary-cache secret/public key pair;
3. sign the store path;
4. verify that signature locally;
5. dump the canonical raw NAR.

A qualification-only CF-07 example process receives only:

- the store path;
- the raw NAR file;
- the already-created Nix `Sig:` string; and
- a loopback port.

It does **not** receive the Nix secret key. It hashes/imports the NAR through CF-03, constructs the validated local catalog entry, and serves the normal CF-07 routes.

The client proof then uses two independent, initially empty local chroot stores whose logical store directory remains `/nix/store`:

### Negative trust case

The first destination is configured with an unrelated generated public key while `require-sigs = true`.

The copy MUST fail and the target object MUST remain absent.

The NAR bytes and the signature served by CF-07 are otherwise unchanged.

### Positive trust case

A second empty destination is configured with the public key corresponding to the signature carried in the same CF-07 `.narinfo`.

The copy MUST succeed. The restored object MUST then pass `nix store verify` with that public key and its file contents MUST match the original fixture.

The only trust input changed between the negative and positive cases is the destination Nix client's trusted public key.

This is the acceptance criterion for the v0.1 claim:

> CF-07 can deliver a valid input-addressed Nix store object without becoming the authority that decides whether the object is trusted.

The qualification does not establish LAN/public publication authority, remote-fill authorization, cache signing authority, or production deployment readiness.

## Retrieval flow

The full Edge Seed target remains:

1. Nix requests a store path through the ordinary substituter interface.
2. The local/LAN edge checks its explicit Nix exposure catalog and verified CAS.
3. On miss, a later fill adapter resolves an immutable Fabric object/provider set.
4. Content is fetched over an available transport and cryptographically verified.
5. Only verified bytes are promoted to the local CAS.
6. A separate publication/read-authority layer decides whether that store object may enter the externally exposed Nix catalog.
7. CF-07 presents normal narinfo/NAR responses for the resulting local exposure configuration.
8. Nix independently performs its existing trust/admission verification.

Steps 3–6 are intentionally outside the first CF-07 HTTP crate; v0.1 has no network mutation/admin API and does not establish publication authority by itself.

## Network exposure

The reference server binds only to loopback. A non-loopback `SocketAddr` is rejected during configuration validation.

LAN/public edge serving is a separate future boundary requiring explicit network exposure and authentication/reverse-proxy review plus publication/read authorization. CF-07 does not weaken CF-03A's loopback containment merely to add Nix compatibility.

## Failure behavior

- Remote/provider failure must not corrupt an existing local cache entry.
- Partial downloads must never become addressable.
- Already admitted local immutable content does not require Holochain for each read.
- A modified/corrupt local NAR must fail CF-03 verification before CF-07 serves it.
- Nix still independently verifies its own NAR/store/signature semantics.
- A catalog entry with locally unavailable bytes produces transient unavailability rather than a false permanent absence.

## Determinism

For a fixed catalog entry, CF-07 deterministically renders:

- NAR URL;
- Nix-base32 hash strings;
- reference ordering;
- signature ordering;
- `.narinfo` field ordering.

No system clock, Holochain arrival order, Symthaea output, cost score, reputation, or marketplace state influences the wire representation.

## Edge Seed demo target

The larger milestone demonstration should prove normal Nix interoperability across multiple geographically separated Fabric nodes, including first-request remote fill, second-machine local cache hit, continued operation after the original source disconnects, and later replication to a newly available node based on an advisory placement proposal.

The first CF-07 qualification is narrower: a stock Nix client must accept the generated `nix-cache-info` / `.narinfo` / raw NAR surface with the matching trusted key and reject the exact same cache when only an unrelated public key is trusted.

## Deferred

- compressed NAR variants;
- dynamic upstream-cache proxying;
- authenticated ingest/admin endpoints;
- Nix cache signing keys;
- LAN/public binding;
- build logs;
- NAR listings;
- build-trace v2;
- dynamic Holochain catalog projection;
- OCI routes;
- cache eviction/deletion;
- lease/payment coupling.

A later durability controller may decide which replicas are expendable, but CF-07 itself has no deletion primitive.
