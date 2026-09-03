# mycelix-nix-cache

`mycelix-nix-cache` is the CF-07 stock-Nix compatibility facade for Content Fabric.

It serves a conventional HTTP binary-cache surface while keeping software trust in Nix rather than in the cache node.

## v0.1 routes

- `GET/HEAD /nix-cache-info`
- `GET/HEAD /<store-hash>.narinfo`
- `GET/HEAD /nar/<store-hash>-<nar-sha256>.nar`

There are no mutation routes.

## Trust boundary

The cache is not a Nix signing authority.

- It accepts no secret signing key.
- It never creates `Sig:` lines.
- Existing admitted `Sig:` lines are preserved after line-safe syntax validation.
- Nix clients remain responsible for trusted-public-key / content-address acceptance policy.
- CAS presence does not imply Nix exposure: only catalog-admitted entries are reachable.
- Catalog membership is local exposure configuration, not proof that content is globally/publicly authorized for disclosure.

A provider that can serve bytes therefore cannot bless arbitrary software merely by controlling the cache.

## Raw NAR profile

CF-07 v0.1 deliberately supports only uncompressed SHA-256 NARs:

- `Compression: none`
- `FileHash == NarHash`
- `FileSize == NarSize`
- both hashes are the Nix-base32 rendering of the exact CF-03 SHA-256 CAS digest.

This avoids making compression format or transport chunking part of global content identity.

## Verified-on-read metadata

A catalog entry is not advertised merely because metadata exists in memory. Before returning either `.narinfo` or NAR bytes, the facade calls CF-03 `open_verified_digest()` and requires the verified file size to match the admitted NAR size.

Consequences:

- unknown store path -> `404`;
- catalog entry whose bytes are currently unavailable -> `503`;
- digest/immutable-tree corruption -> `500`;
- verification saturation -> `503` with `Retry-After: 1`.

This prevents a corrupt or missing local replica from being represented as a valid cache hit.

## Stock Nix qualification

The separate `Content Fabric Stock Nix Qualification` workflow exercises the facade with upstream Nix 2.35.2 rather than only HTTP-level unit tests.

The fixture is a normal input-addressed Nix derivation output with zero runtime references. Nix itself:

1. builds the store object;
2. generates an Ed25519 binary-cache keypair;
3. signs the store path;
4. dumps the exact canonical NAR.

The qualification-only example server imports that NAR into CF-03, preserves the Nix-generated signature in one validated `NixCacheEntryV1`, and exposes CF-07 on loopback.

Two fresh local chroot destination stores are then used:

- a store configured with an unrelated public key MUST reject the copy;
- a different empty store configured with the matching public key MUST accept it.

The accepted destination is verified again with `nix store verify` and its restored contents are checked exactly.

The server never receives the signing secret. Changing only the client-side trusted public key is what changes rejection into acceptance.

## Exposure

The reference server remains loopback-only, matching CF-03A. LAN/public exposure is intentionally deferred to a separately reviewed authenticated edge-network boundary rather than being enabled by `0.0.0.0` configuration.

## Deferred

- compressed NAR variants;
- ingest/admin HTTP APIs;
- cache signing keys;
- remote/LAN exposure;
- dynamic Holochain catalog projection;
- build logs and `nar/*.ls` indexes;
- build-trace v2;
- OCI compatibility;
- deletion/eviction authority.
